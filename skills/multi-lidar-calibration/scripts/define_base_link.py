import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from export_tf_launch import read_extrinsics
from sensor_layout import sensor_frames


def normalized(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if vector.shape != (3,) or not np.isfinite(vector).all() or length < 1e-8:
        raise ValueError('Cannot determine a stable direction from a finite nonzero 3-vector')
    return vector / length


def rebase(configuration, base_from_reference):
    result = copy.deepcopy(configuration)
    result['reference_frame'] = 'base_link'
    result['rpy_convention'] = 'degrees; Rz(yaw) Ry(pitch) Rx(roll)'
    for sensor in result['sensors'].values():
        if sensor['source_frame'] == 'base_link':
            raise ValueError('base_link must be a virtual vehicle frame, not a physical sensor frame')
        matrix = base_from_reference @ np.asarray(sensor['T_reference_from_sensor'], dtype=float)
        rotation = Rotation.from_matrix(matrix[:3, :3])
        sensor['target_frame'] = 'base_link'
        sensor['T_reference_from_sensor'] = matrix.tolist()
        sensor['translation_m'] = matrix[:3, 3].tolist()
        sensor['quaternion_xyzw'] = rotation.as_quat(canonical=True).tolist()
        sensor['rpy_xyz_deg'] = rotation.as_euler('xyz', degrees=True).tolist()
    sensor_frames(result)
    return result


def define_base_link(configuration, layout):
    frames = sensor_frames(configuration)
    if configuration['reference_frame'] == 'base_link':
        raise ValueError('Use original sensor-relative extrinsics to define a center, or use the axis-conversion mode')
    front = layout['front_sensors']
    rear = layout['rear_sensors']
    if not isinstance(front, list) or not isinstance(rear, list) or len(front) != 2 or len(rear) != 2:
        raise ValueError('Explicitly select two front and two rear main sensors')
    corners = front + rear
    if any(not isinstance(name, str) or name not in frames for name in corners) or len(set(corners)) != 4:
        raise ValueError('Main sensors must be four distinct existing sensor IDs')
    pairs = layout['diagonal_pairs']
    if (not isinstance(pairs, list) or len(pairs) != 2 or
            any(not isinstance(pair, list) or len(pair) != 2 for pair in pairs)):
        raise ValueError('Require two explicit diagonal pairs')
    flattened = [name for pair in pairs for name in pair]
    if sorted(flattened) != sorted(corners) or any(sum(name in front for name in pair) != 1 for pair in pairs):
        raise ValueError('Diagonal pairs must cover every main sensor once and each connect front to rear')
    up_hint = normalized(layout['up_hint_in_reference'])
    up_vectors = np.asarray([frames[name]['rotation'][:, 2] for name in corners])
    agreement = up_vectors @ up_hint
    if np.any(np.abs(agreement) < 1e-3):
        raise ValueError('Up hint is nearly perpendicular to a main sensor Z axis; confirm mounting')
    up_signs = np.where(agreement >= 0, 1, -1)
    up = normalized(np.mean(up_vectors * up_signs[:, None], axis=0))
    forward_signs = np.array([1, 1, -1, -1])
    forward_vectors = np.asarray([frames[name]['rotation'][:, 0] for name in corners]) * forward_signs[:, None]
    forward_vectors -= (forward_vectors @ up)[:, None] * up
    forward = normalized(np.mean([normalized(vector) for vector in forward_vectors], axis=0))
    left = normalized(np.cross(up, forward))
    forward = normalized(np.cross(left, up))
    midpoints = np.asarray([(frames[first]['origin'] + frames[second]['origin']) / 2 for first, second in pairs])
    original_from_base = np.eye(4)
    original_from_base[:3, :3] = np.column_stack([forward, left, up])
    original_from_base[:3, 3] = midpoints.mean(axis=0)
    base_from_original = np.linalg.inv(original_from_base)
    result = rebase(configuration, base_from_original)
    result['frame_definition_status'] = 'user-defined installation-layout approximation; not surveyed chassis/CAD truth'
    result['base_link_definition'] = {
        'axis_convention': {'x': 'front', 'y': 'left', 'z': 'up'},
        'original_reference_frame': configuration['reference_frame'],
        'center_sensors': corners, 'forward_sensors': corners, 'up_and_height_sensors': corners,
        'excluded_from_definition': [name for name in frames if name not in corners],
        'diagonal_pairs': pairs, 'diagonal_midpoints_in_original_reference_m': midpoints.tolist(),
        'up_hint_in_original_reference': up_hint.tolist(),
        'up_signs': dict(zip(corners, up_signs.tolist())),
        'forward_signs': dict(zip(corners, forward_signs.tolist())),
        'center_rule': 'Mean of two diagonal midpoints, including height; do not impose bilateral symmetry',
        'up_rule': 'Normalized mean of selected sensor Z axes after alignment with confirmed up hint',
        'forward_rule': 'Equal mean of individually normalized front +X and rear -X projected perpendicular to up',
        'symmetry_policy': 'Do not modify relative calibrated positions or rotations to force symmetry or parallel axes',
        'T_original_reference_from_base_link': original_from_base.tolist(),
        'T_base_link_from_original_reference': base_from_original.tolist(),
    }
    return result


def convert_right_forward_to_front_left(configuration):
    sensor_frames(configuration)
    definition = configuration.get('base_link_definition', {})
    if configuration['reference_frame'] != 'base_link' or definition.get('axis_convention') != {'x': 'right', 'y': 'front', 'z': 'up'}:
        raise ValueError('Axis conversion requires an existing base_link explicitly defined as X right / Y front / Z up')
    old_from_new = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    result = rebase(configuration, old_from_new.T)
    updated = result['base_link_definition']
    updated['axis_convention'] = {'x': 'front', 'y': 'left', 'z': 'up'}
    forward_key = 'T_original_reference_from_base_link'
    inverse_key = 'T_base_link_from_original_reference'
    if forward_key in definition or inverse_key in definition:
        original_from_old = np.asarray(definition[forward_key], dtype=float)
        old_from_original = np.asarray(definition[inverse_key], dtype=float)
        if original_from_old.shape != (4, 4) or old_from_original.shape != (4, 4):
            raise ValueError('Historical reference transforms must be 4x4')
        rotation = original_from_old[:3, :3]
        if (not np.isfinite(original_from_old).all() or not np.isfinite(old_from_original).all() or
                not np.allclose(original_from_old[3], [0, 0, 0, 1], atol=1e-8, rtol=0) or
                not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7, rtol=0) or
                abs(np.linalg.det(rotation) - 1) > 1e-7 or
                not np.allclose(original_from_old @ old_from_original, np.eye(4), atol=1e-8, rtol=0)):
            raise ValueError('Historical reference transforms are not consistent rigid inverses')
        updated[forward_key] = (original_from_old @ old_from_new).tolist()
        updated[inverse_key] = (old_from_new.T @ old_from_original).tolist()
    updated.setdefault('basis_changes', []).append({'T_old_base_from_new_base': old_from_new.tolist(),
                                                   'coordinate_rule': 'x_new=y_old; y_new=-x_old; z_new=z_old',
                                                   'origin_and_physical_up_unchanged': True})
    return result


def main():
    parser = argparse.ArgumentParser(description='Define a four-main-LiDAR vehicle center or convert an existing right/forward/up base to front/left/up.')
    parser.add_argument('--extrinsics', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--layout', type=Path)
    mode.add_argument('--right-forward-to-front-left', action='store_true')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    try:
        if args.output.suffix.lower() != '.json':
            raise ValueError('Output must be JSON')
        inputs = [args.extrinsics] + ([args.layout] if args.layout else [])
        if any(args.output.resolve() == path.resolve() or (args.output.exists() and args.output.samefile(path)) for path in inputs):
            raise ValueError('Output cannot overwrite input extrinsics or layout')
        if args.output.exists() and not args.overwrite:
            raise ValueError('Output exists; choose a different path or explicitly use --overwrite')
        configuration = read_extrinsics(args.extrinsics)
        if args.layout:
            result = define_base_link(configuration, json.loads(args.layout.read_text()))
            result['base_link_definition']['source_extrinsics_sha256'] = hashlib.sha256(args.extrinsics.read_bytes()).hexdigest()
        else:
            result = convert_right_forward_to_front_left(configuration)
        result['base_link_processing_input'] = {'sha256': hashlib.sha256(args.extrinsics.read_bytes()).hexdigest(),
                                                'reference_frame': configuration['reference_frame']}
        content = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('w' if args.overwrite else 'x', encoding='utf-8') as handle:
            handle.write(content)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    print(f'Wrote {args.output}; base_link X front / Y left / Z up; sensor-relative transforms unchanged')


if __name__ == '__main__':
    main()
