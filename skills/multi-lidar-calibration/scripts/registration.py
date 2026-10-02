import argparse
import itertools
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def read_dataset(filename):
    filename = filename.resolve()
    configuration = json.loads(filename.read_text())
    sensors = configuration['sensors']
    reference = configuration['reference']
    if len(sensors) < 2 or reference not in sensors:
        raise ValueError('Require at least two sensors and an explicitly selected reference')
    seen = {}
    paths = {}
    for sensor, splits in sensors.items():
        paths[sensor] = {}
        if not isinstance(splits.get('train'), list) or not splits['train']:
            raise ValueError(f'{sensor}: require a nonempty train list')
        for split in ['train', 'validation', 'test']:
            entries = splits.get(split, [])
            if not isinstance(entries, list):
                raise ValueError(f'{sensor}/{split}: expected a list')
            paths[sensor][split] = []
            for value in entries:
                if value is None:
                    if split == 'train':
                        raise ValueError('Null entries are only allowed in held-out frame groups')
                    paths[sensor][split].append(None)
                    continue
                path = (filename.parent / value).resolve()
                if path in seen:
                    raise ValueError(f'Reused frame path: {path}; {seen[path]} and {sensor}/{split}')
                if not path.is_file() or path.suffix not in ['.npy', '.npz']:
                    raise ValueError(f'Expected an existing NPY/NPZ file: {path}')
                seen[path] = f'{sensor}/{split}'
                paths[sensor][split].append(path)
    return reference, paths, set(seen) | {filename}


def read_poses(filename, reference, sensors):
    document = json.loads(filename.read_text())
    if 'poses' in document:
        if document.get('reference', reference) != reference:
            raise ValueError('Pose reference does not match dataset reference')
        document = document['poses']
    if set(document) != set(sensors):
        raise ValueError('Pose IDs must exactly match sensor IDs; no unknown pose may default to identity')
    poses = {}
    for sensor, values in document.items():
        matrix = np.asarray(values, dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError(f'{sensor}: expected a finite 4x4 transform')
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
            raise ValueError(f'{sensor}: invalid homogeneous row')
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or abs(np.linalg.det(rotation) - 1) > 1e-6:
            raise ValueError(f'{sensor}: rotation must be in SO(3), not a reflection or scale')
        poses[sensor] = matrix
    if not np.allclose(poses[reference], np.eye(4), atol=1e-8, rtol=0):
        raise ValueError('Reference pose must be identity; explicitly rebase all poses first')
    return poses


def read_points(filename, minimum, maximum):
    if filename.suffix == '.npz':
        with np.load(filename, allow_pickle=False) as data:
            points = np.asarray(data['xyz'], dtype=float)
    else:
        points = np.asarray(np.load(filename, allow_pickle=False), dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f'{filename}: expected Nx3 XYZ, in meters')
    points = points[np.isfinite(points).all(axis=1)]
    distance = np.linalg.norm(points, axis=1)
    return points[(distance > minimum) & (distance < maximum)]


def planar_cloud(points, voxel, neighbors=30, radius=3.0):
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    points = np.asarray(cloud.voxel_down_sample(voxel).points)
    if len(points) < neighbors:
        return np.empty((0, 3)), np.empty((0, 3))
    distances, indices = cKDTree(points).query(points, k=neighbors, workers=1)
    samples = points[indices]
    centered = samples - samples.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered) / neighbors
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    mask = ((eigenvalues[:, 0] < 0.08 * eigenvalues[:, 1]) &
            (eigenvalues[:, 1] > 0.025 * eigenvalues[:, 2]) & (distances[:, -1] < radius))
    return points[mask], eigenvectors[mask, :, 0]


def world_cloud(cloud, matrix):
    points, normals = cloud
    return points @ matrix[:3, :3].T + matrix[:3, 3], normals @ matrix[:3, :3].T


def matches(source, target, maximum, angle_deg, residual_fraction=None):
    first, first_normals = source
    second, second_normals = target
    if not len(first) or not len(second):
        return np.empty((0, 3)), np.empty((0, 3)), np.empty(0), np.empty(0)
    distances, indices = cKDTree(second).query(first, workers=1)
    normals = second_normals[indices]
    residuals = np.einsum('ij,ij->i', first - second[indices], normals)
    agreement = np.abs(np.einsum('ij,ij->i', first_normals, normals))
    mask = (distances < maximum) & (agreement > np.cos(np.deg2rad(angle_deg)))
    if residual_fraction is not None:
        mask &= np.abs(residuals) < maximum * residual_fraction
    return first[mask], normals[mask], residuals[mask], distances[mask]


def plane_jacobian(points, normals):
    return np.column_stack([np.cross(points, normals), normals])


def information_eigenvalues(hessian, centers):
    basis = np.eye(len(hessian))
    for index, center in enumerate(centers):
        start = index * 6
        basis[start + 3:start + 6, start:start + 3] = np.cross(center, np.eye(3)).T
    centered = basis.T @ hessian @ basis
    block_diagonal = np.diag(centered).reshape(-1, 3).mean(axis=1).repeat(3)
    scale = np.sqrt(np.maximum(block_diagonal, 1e-15))
    return np.linalg.eigvalsh(centered / scale[:, None] / scale[None, :])


def connected(sensors, edges, reference):
    reached = {reference}
    for iteration in range(len(sensors)):
        for first, second in edges:
            if first in reached or second in reached:
                reached.update([first, second])
    return reached == set(sensors)


def distribution(values):
    if not len(values):
        return {'count': 0, 'median_m': None, 'p90_m': None, 'p95_m': None, 'rms_m': None}
    return {'count': len(values), 'median_m': float(np.median(values)),
            'p90_m': float(np.percentile(values, 90)), 'p95_m': float(np.percentile(values, 95)),
            'rms_m': float(np.sqrt(np.mean(values ** 2)))}


def refine(args, reference, paths, poses):
    sensors = list(paths)
    movable = [sensor for sensor in sensors if sensor != reference]
    blocks = {sensor: index * 6 for index, sensor in enumerate(movable)}
    clouds = {}
    for sensor in sensors:
        points = np.concatenate([read_points(path, args.min_range, args.max_range) for path in paths[sensor]['train']])
        clouds[sensor] = planar_cloud(points, args.voxel, radius=args.normal_radius)
        if len(clouds[sensor][0]) < args.min_matches:
            raise ValueError(f'{sensor}: too few noncollinear planar points for refinement')
    history = []
    converged = False
    final_geometry = None
    for iteration in range(args.iterations + 1):
        final_check = converged or iteration == args.iterations
        maximum = args.distance if final_check else max(args.distance, 0.6 if iteration < 5 else (0.4 if iteration < 10 else args.distance))
        world = {sensor: world_cloud(clouds[sensor], poses[sensor]) for sensor in sensors}
        hessian = np.zeros((6 * len(movable), 6 * len(movable)))
        gradient = np.zeros(6 * len(movable))
        edges = []
        pair_metrics = []
        for source, target in itertools.permutations(sensors, 2):
            points, normals, residuals, distances = matches(world[source], world[target], maximum,
                                                          args.angle, args.residual_fraction)
            if len(residuals) < args.min_matches:
                continue
            edges.append((source, target))
            pair_metrics.append({'source': source, 'target': target, 'residual': distribution(np.abs(residuals))})
            if len(residuals) > args.max_matches:
                indices = np.linspace(0, len(residuals) - 1, args.max_matches).astype(int)
                points, normals, residuals = points[indices], normals[indices], residuals[indices]
            weight = np.minimum(1, args.huber / np.maximum(np.abs(residuals), 1e-12))
            weight *= min(1, 1200 / len(residuals))
            jacobian = np.zeros((len(points), len(gradient)))
            local = plane_jacobian(points, normals)
            if source in blocks:
                start = blocks[source]
                jacobian[:, start:start + 6] = local
            if target in blocks:
                start = blocks[target]
                jacobian[:, start:start + 6] = -local
            hessian += (jacobian.T * weight) @ jacobian
            gradient += jacobian.T @ (weight * residuals)
        if not connected(sensors, edges, reference):
            raise ValueError(f'Iteration {iteration}: disconnected overlap graph; no complete extrinsics written')
        eigenvalues = information_eigenvalues(hessian, [poses[sensor][:3, 3] for sensor in movable])
        information_ratio = float(eigenvalues[0] / max(eigenvalues[-1], 1e-15))
        if eigenvalues[0] <= 1e-10 or information_ratio <= args.min_information_ratio:
            raise ValueError(f'Iteration {iteration}: underconstrained or weak geometry before damping; information ratio {information_ratio:.3g}')
        if final_check:
            final_geometry = {'distance_m': maximum, 'information_eigenvalues': eigenvalues.tolist(),
                              'information_ratio': information_ratio, 'pairs': pair_metrics}
            print(f'final_geometry edges={len(edges)} information_ratio={information_ratio:.6g}', flush=True)
            break
        step = -np.linalg.solve(hessian + np.eye(len(gradient)) * 1e-3, gradient)
        origin_speed = []
        rotation_speed = []
        for sensor, start in blocks.items():
            update = step[start:start + 6]
            origin_speed.append(np.linalg.norm(np.cross(update[:3], poses[sensor][:3, 3]) + update[3:]))
            rotation_speed.append(np.linalg.norm(update[:3]))
        scale = min(1.0, 0.35 / max(max(origin_speed), 1e-12), np.deg2rad(3) / max(max(rotation_speed), 1e-12))
        movements = []
        for sensor, start in blocks.items():
            update = step[start:start + 6] * scale
            delta = np.eye(4)
            delta[:3, :3] = Rotation.from_rotvec(update[:3]).as_matrix()
            delta[:3, 3] = update[3:]
            candidate = delta @ poses[sensor]
            movements.append(np.linalg.norm(candidate[:3, 3] - poses[sensor][:3, 3]))
            poses[sensor] = candidate
        history.append({'iteration': iteration, 'distance_m': maximum, 'step_scale': scale,
                        'max_origin_motion_m': float(max(movements)),
                        'max_rotation_motion_rad': float(max(rotation_speed) * scale),
                        'information_eigenvalues': eigenvalues.tolist(),
                        'information_ratio': information_ratio, 'pairs': pair_metrics})
        print(f'iteration={iteration} edges={len(edges)} origin_step_m={max(movements):.6g}', flush=True)
        if iteration >= 10 and max(movements) < 1e-5 and max(rotation_speed) * scale < 1e-6:
            converged = True
    return {'status': 'local_refinement_only', 'reference': reference,
            'convention': 'p_reference = R @ p_sensor + t', 'converged': converged,
            'information_normalization': 'sensor-origin twists; common scale within each rotation/translation triplet',
            'poses': {sensor: matrix.tolist() for sensor, matrix in poses.items()},
            'history': history, 'final_geometry': final_geometry}


def evaluate(args, reference, paths, poses):
    lengths = {len(splits[args.split]) for splits in paths.values()}
    if len(lengths) != 1 or next(iter(lengths)) == 0:
        raise ValueError('Held-out lists must have equal positive lengths; use null for missing sensor frames')
    groups = []
    all_residuals = {sensor: [] for sensor in paths}
    for index in range(next(iter(lengths))):
        world = {}
        for sensor, splits in paths.items():
            path = splits[args.split][index]
            if path is not None:
                points = read_points(path, args.min_range, args.max_range)
                world[sensor] = world_cloud(planar_cloud(points, args.voxel, radius=args.normal_radius), poses[sensor])
        pairs = []
        edges = []
        for source, target in itertools.permutations(world, 2):
            points, normals, residuals, distances = matches(world[source], world[target], args.distance, args.angle)
            source_count = len(world[source][0])
            pairs.append({'source': source, 'target': target, 'source_planar_points': source_count,
                          'overlap_fraction': len(residuals) / source_count if source_count else 0.0,
                          'point_to_plane': distribution(np.abs(residuals)), 'nearest_neighbor': distribution(distances),
                          'within_5cm_fraction': float(np.mean(np.abs(residuals) < 0.05)) if len(residuals) else None})
            if len(residuals):
                all_residuals[source].append(residuals)
            if len(residuals) >= args.min_matches:
                edges.append((source, target))
        groups.append({'index': index, 'available_sensors': list(world),
                       'missing_sensors': sorted(set(paths) - set(world)),
                       'full_overlap_graph_connected': connected(paths, edges, reference), 'pairs': pairs})
    return {'status': 'descriptive_consistency_only', 'reference': reference, 'split': args.split,
            'residual_trimming': False, 'warning': 'Geometrically gated shared surfaces only; not absolute extrinsic accuracy.',
            'groups': groups, 'sensors': {sensor: distribution(np.abs(np.concatenate(parts))) if parts else distribution([])
                                         for sensor, parts in all_residuals.items()}}


def main():
    parser = argparse.ArgumentParser(description='Local multi-LiDAR refinement or held-out consistency evaluation; requires trustworthy initial poses.')
    parser.add_argument('mode', choices=['refine', 'evaluate'])
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--overwrite', action='store_true')
    parser.add_argument('--split', choices=['validation', 'test'], default='validation')
    parser.add_argument('--voxel', type=float, default=0.10)
    parser.add_argument('--normal-radius', type=float, default=3.0)
    parser.add_argument('--min-range', type=float, default=0.3)
    parser.add_argument('--max-range', type=float, default=150.0)
    parser.add_argument('--distance', type=float)
    parser.add_argument('--angle', type=float, default=25.0)
    parser.add_argument('--huber', type=float, default=0.015)
    parser.add_argument('--residual-fraction', type=float, default=0.4)
    parser.add_argument('--iterations', type=int, default=45)
    parser.add_argument('--min-matches', type=int, default=80)
    parser.add_argument('--max-matches', type=int, default=6000)
    parser.add_argument('--min-information-ratio', type=float, default=1e-4)
    args = parser.parse_args()
    args.distance = args.distance if args.distance is not None else (0.25 if args.mode == 'refine' else 0.6)
    if not np.isfinite([args.voxel, args.normal_radius, args.min_range, args.max_range,
                        args.distance, args.angle, args.huber, args.residual_fraction,
                        args.min_information_ratio]).all():
        parser.error('All numeric thresholds must be finite')
    if min(args.voxel, args.normal_radius, args.distance, args.huber, args.residual_fraction) <= 0:
        parser.error('Voxel, radius, distance, Huber and residual fraction must be positive')
    if not 0 <= args.min_range < args.max_range or not 0 < args.angle < 90:
        parser.error('Require 0 <= min_range < max_range and 0 < angle < 90')
    if not 0 < args.min_information_ratio < 1:
        parser.error('Require 0 < min_information_ratio < 1')
    if args.iterations < 1 or args.min_matches < 6 or args.max_matches < args.min_matches:
        parser.error('Require positive iterations and 6 <= min_matches <= max_matches')
    reference, paths, inputs = read_dataset(args.config)
    inputs.add(args.poses.resolve())
    if args.output.resolve() in inputs or (args.output.exists() and any(args.output.samefile(path) for path in inputs)):
        parser.error('Output cannot overwrite an input')
    if args.output.exists() and not args.overwrite:
        parser.error('Output exists; select a new output or explicitly use --overwrite')
    poses = read_poses(args.poses, reference, paths)
    report = refine(args, reference, paths, poses) if args.mode == 'refine' else evaluate(args, reference, paths, poses)
    report['parameters'] = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w' if args.overwrite else 'x') as handle:
        handle.write(json.dumps(report, indent=2, allow_nan=False))
    print(f'Wrote {args.output}; status={report["status"]}')


if __name__ == '__main__':
    main()
