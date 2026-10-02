import argparse
import json
import re
from pathlib import Path
from xml.sax.saxutils import quoteattr

import numpy as np
from scipy.spatial.transform import Rotation


def read_extrinsics(filename):
    if filename.suffix.lower() == '.json':
        return json.loads(filename.read_text(encoding='utf-8'))
    if filename.suffix.lower() in ['.yaml', '.yml']:
        try:
            import yaml
        except ImportError as error:
            raise ValueError('YAML input requires PyYAML; use JSON or an existing environment with PyYAML') from error
        return yaml.safe_load(filename.read_text(encoding='utf-8'))
    raise ValueError('Extrinsics must be a JSON or YAML file')


def frame_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_./-]*', value):
        raise ValueError(f'Invalid frame ID: {value!r}; use a nonempty ASCII frame without a leading slash or whitespace')
    return value


def vector(values, size, label):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{label}: require {size} finite numeric values')
    return result


def render_launch(configuration, argument_style='positional'):
    if not isinstance(configuration, dict):
        raise ValueError('Extrinsics must be an object with reference_frame, length_unit and sensors')
    if configuration.get('length_unit') != 'meter':
        raise ValueError('Require length_unit: meter; no implicit unit conversion')
    if argument_style not in ['positional', 'flags']:
        raise ValueError('Argument style must be positional or flags')
    reference = frame_id(configuration['reference_frame'])
    sensors = configuration['sensors']
    if not isinstance(sensors, dict) or not sensors:
        raise ValueError('Require a nonempty sensors object')
    frames = set()
    names = set()
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', '<launch>', '']
    for sensor in sensors.values():
        child = frame_id(sensor['source_frame'])
        if sensor['target_frame'] != reference:
            raise ValueError(f'{child}: target_frame must equal reference_frame; rebase transforms explicitly')
        if child in frames:
            raise ValueError(f'Duplicate source_frame: {child}')
        frames.add(child)
        translation = vector(sensor['translation_m'], 3, f'{child}/translation_m')
        quaternion = vector(sensor['quaternion_xyzw'], 4, f'{child}/quaternion_xyzw')
        if abs(np.linalg.norm(quaternion) - 1) > 1e-6:
            raise ValueError(f'{child}: quaternion_xyzw must be unit length')
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        matrix[:3, 3] = translation
        if 'T_reference_from_sensor' in sensor:
            declared = np.asarray(sensor['T_reference_from_sensor'], dtype=float)
            if (declared.shape != (4, 4) or not np.isfinite(declared).all() or
                    not np.allclose(declared, matrix, atol=1e-6, rtol=0)):
                raise ValueError(f'{child}: T_reference_from_sensor disagrees with translation/quaternion')
        if child == reference:
            if not np.allclose(matrix, np.eye(4), atol=1e-8, rtol=0):
                raise ValueError('Reference self-transform must be identity')
            continue
        name = re.sub(r'[^A-Za-z0-9_]', '_', f'{reference}_{child}_tf')
        if name[0].isdigit():
            name = 'tf_' + name
        if name in names:
            raise ValueError(f'Frame names produce a duplicate ROS node name: {name}')
        names.add(name)
        values = [format(value, '.15f') for value in np.concatenate([translation, quaternion])]
        if argument_style == 'positional':
            arguments = values + [reference, child]
        else:
            arguments = []
            for flag, value in zip(['--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'], values):
                arguments.extend([flag, value])
            arguments.extend(['--frame-id', reference, '--child-frame-id', child])
        lines.extend([f'  <node pkg="tf2_ros" exec="static_transform_publisher" name={quoteattr(name)} args={quoteattr(" ".join(arguments))}/>', ''])
    if not names:
        raise ValueError('No non-reference transforms to export')
    lines.append('</launch>')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description='Export existing multi-LiDAR extrinsics as ROS 2 static TF launch XML; does not run ROS nodes.')
    parser.add_argument('--extrinsics', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--argument-style', choices=['positional', 'flags'], default='positional')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    try:
        if (args.output.resolve() == args.extrinsics.resolve() or
                (args.output.exists() and args.output.samefile(args.extrinsics))):
            raise ValueError('Output cannot overwrite input extrinsics')
        if args.output.exists() and not args.overwrite:
            raise ValueError('Output exists; choose a new path or explicitly use --overwrite')
        content = render_launch(read_extrinsics(args.extrinsics), args.argument_style)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('w' if args.overwrite else 'x', encoding='utf-8') as handle:
            handle.write(content)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    print(f'Wrote {args.output}; static TF configuration only, no nodes started')


if __name__ == '__main__':
    main()
