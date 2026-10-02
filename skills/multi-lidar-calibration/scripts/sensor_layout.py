import argparse
import html
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation

from export_tf_launch import frame_id, read_extrinsics


SENSOR_COLORS = {
    'up_front': '#e41a1c', 'up_rear': '#377eb8', 'left_front': '#4daf4a', 'right_front': '#984ea3',
    'left_rear': '#ff7f00', 'right_rear': '#00b8ba', 'left_front_bp': '#e377c2', 'right_rear_bp': '#795548',
}
AXES = [('X', '#ff5555'), ('Y', '#46d879'), ('Z', '#559dff')]
CAMERAS = {
    '3D': {'eye': {'x': 1.35, 'y': -1.85, 'z': 1.2}, 'up': {'x': 0, 'y': 0, 'z': 1}},
    'XY': {'eye': {'x': 0, 'y': 0, 'z': 2.5}, 'up': {'x': 0, 'y': 1, 'z': 0}},
    'XZ': {'eye': {'x': 0, 'y': -2.5, 'z': 0}, 'up': {'x': 0, 'y': 0, 'z': 1}},
    'YZ': {'eye': {'x': 2.5, 'y': 0, 'z': 0}, 'up': {'x': 0, 'y': 0, 'z': 1}},
}


def base_directions(configuration):
    convention = configuration.get('base_link_definition', {}).get('axis_convention', {})
    labels = {'front': '前', 'back': '后', 'left': '左', 'right': '右', 'up': '上', 'down': '下'}
    return [labels.get(convention.get(coordinate), '未指定') for coordinate in 'xyz']


def sensor_frames(configuration):
    if configuration['length_unit'] != 'meter':
        raise ValueError('Expected extrinsics in meters')
    reference = frame_id(configuration['reference_frame'])
    frames = {}
    source_frames = set()
    for name, sensor in configuration['sensors'].items():
        if not isinstance(name, str) or not name or name == 'base_link':
            raise ValueError('Require nonempty sensor IDs distinct from the virtual base_link')
        frame_id(sensor['source_frame'])
        matrix = np.asarray(sensor['T_reference_from_sensor'], dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError(f'{name}: expected a finite 4x4 transform')
        rotation = matrix[:3, :3]
        if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0) or
                not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7, rtol=0) or
                abs(np.linalg.det(rotation) - 1) > 1e-7):
            raise ValueError(f'{name}: invalid rigid transform')
        if sensor['target_frame'] != reference or sensor['source_frame'] in source_frames:
            raise ValueError(f'{name}: inconsistent reference or duplicate source frame')
        if sensor['source_frame'] == reference and not np.allclose(matrix, np.eye(4), atol=1e-8, rtol=0):
            raise ValueError('Reference transform must be identity')
        if 'translation_m' in sensor:
            translation = np.asarray(sensor['translation_m'], dtype=float)
            if translation.shape != (3,) or not np.allclose(translation, matrix[:3, 3], atol=1e-7, rtol=0):
                raise ValueError(f'{name}: translation disagrees with matrix')
        if 'quaternion_xyzw' in sensor:
            quaternion = np.asarray(sensor['quaternion_xyzw'], dtype=float)
            if (quaternion.shape != (4,) or not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1) > 1e-7 or
                    not np.allclose(Rotation.from_quat(quaternion).as_matrix(), rotation, atol=1e-7, rtol=0)):
                raise ValueError(f'{name}: quaternion disagrees with matrix')
        source_frames.add(sensor['source_frame'])
        frames[name] = {'origin': matrix[:3, 3], 'rotation': rotation, 'frame': sensor['source_frame']}
    if not frames:
        raise ValueError('No sensors to display')
    return frames


def arrow_vertices(origin, rotation, axis_index, length):
    tip = origin + length * rotation[:, axis_index]
    base = tip - 0.22 * length * rotation[:, axis_index]
    angles = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    circle = (np.cos(angles)[:, None] * rotation[:, (axis_index + 1) % 3] +
              np.sin(angles)[:, None] * rotation[:, (axis_index + 2) % 3])
    return np.vstack([tip, base + 0.065 * length * circle, base])


def scene_ranges(frames, names, length):
    points = np.concatenate([np.vstack([frames[name]['origin'],
                            frames[name]['origin'] + frames[name].get('axis_length', length) * frames[name]['rotation'].T])
                             for name in names])
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    longest_axis = max(frames[name].get('axis_length', length) for name in names)
    padding = np.maximum((upper - lower) * 0.12, longest_axis * 0.8)
    return {f'scene.{coordinate}axis.range': [float(lower[index] - padding[index]),
                                           float(upper[index] + padding[index])]
            for index, coordinate in enumerate('xyz')}


def build_figure(configuration, axis_length=0.8, base_axis_length=2.0):
    if not np.isfinite([axis_length, base_axis_length]).all() or min(axis_length, base_axis_length) <= 0:
        raise ValueError('Axis length must be finite and positive')
    frames = sensor_frames(configuration)
    has_base = configuration['reference_frame'] == 'base_link'
    direction_labels = base_directions(configuration)
    if has_base:
        if 'base_link' in frames:
            raise ValueError('base_link is the vehicle frame, not a sensor ID')
        frames = {'base_link': {'origin': np.zeros(3), 'rotation': np.eye(3), 'frame': 'base_link',
                                'axis_length': base_axis_length, 'is_base': True}, **frames}
    palette = list(SENSOR_COLORS.values())
    figure = go.Figure()
    for sensor_index, (name, frame) in enumerate(frames.items()):
        origin, rotation = frame['origin'], frame['rotation']
        is_base = frame.get('is_base', False)
        length = frame.get('axis_length', axis_length)
        label = '<b>base_link · 车辆中心</b>' if is_base else html.escape(name)
        color = '#ffd166' if is_base else SENSOR_COLORS.get(name, palette[sensor_index % len(palette)])
        reference_label = '（自定义车体坐标系）' if is_base else (
            '（参考雷达）' if frame['frame'] == configuration['reference_frame'] else '')
        figure.add_trace(go.Scatter3d(
            x=[origin[0]], y=[origin[1]], z=[origin[2]], mode='markers+text',
            name='base_link · 车辆中心' if is_base else name, legendgroup=name, showlegend=True, text=[label],
            textposition='bottom center' if name.endswith('_bp') else 'top center',
            textfont={'size': 20 if is_base else 13, 'color': '#ffd166' if is_base else '#f0f4fa'},
            marker={'size': 12 if is_base else 6, 'symbol': 'diamond' if is_base else 'circle',
                    'color': color, 'line': {'color': '#ffffff' if is_base else '#e5edf7', 'width': 2 if is_base else 1}},
            hovertemplate=f'<b>{label}{reference_label}</b><br>{html.escape(frame["frame"])}'
                          '<br>位置：(%{x:.4f}, %{y:.4f}, %{z:.4f}) m<extra></extra>',
            meta={'sensor': name, 'kind': 'origin'}))
        for axis_index, (axis_name, axis_color) in enumerate(AXES):
            direction = rotation[:, axis_index]
            tip = origin + length * direction
            axis_label = f'+{axis_name} · {direction_labels[axis_index]}' if is_base else axis_name
            direction_text = ', '.join(f'{value:.5f}' for value in direction)
            figure.add_trace(go.Scatter3d(
                x=[origin[0], tip[0]], y=[origin[1], tip[1]], z=[origin[2], tip[2]],
                mode='lines+text', line={'color': axis_color, 'width': 12 if is_base else 6},
                text=['', axis_label], textfont={'size': 20 if is_base else 14, 'color': axis_color}, textposition='top center',
                name=f'{name} +{axis_name}', legendgroup=name, showlegend=False,
                hovertemplate=f'<b>{label} / +{axis_name}</b><br>方向（参考系）：[{direction_text}]'
                              '<br>位置：(%{x:.4f}, %{y:.4f}, %{z:.4f}) m<extra></extra>',
                meta={'sensor': name, 'kind': 'axis', 'axis': axis_name}))
            vertices = arrow_vertices(origin, rotation, axis_index, length)
            ring = np.arange(1, 13)
            next_ring = np.roll(ring, -1)
            figure.add_trace(go.Mesh3d(
                x=vertices[:, 0].tolist(), y=vertices[:, 1].tolist(), z=vertices[:, 2].tolist(),
                i=[0] * 12 + [13] * 12, j=np.concatenate([ring, next_ring]).tolist(),
                k=np.concatenate([next_ring, ring]).tolist(), color=axis_color,
                flatshading=True, hoverinfo='skip', name=f'{name} +{axis_name} arrow',
                legendgroup=name, showlegend=False,
                meta={'sensor': name, 'kind': 'arrowhead', 'axis': axis_name}))
    groups = [('全部雷达 + base_link' if has_base else '全部雷达', list(frames))]
    for label, suffix in [('前端雷达', 'front'), ('后端雷达', 'rear')]:
        names = [name for name in frames if suffix in name]
        if names:
            groups.append((label, names))
    groups.extend((name, [name]) for name in frames)
    selection_buttons = []
    for label, names in groups:
        layout = scene_ranges(frames, names, axis_length)
        layout['scene.camera'] = CAMERAS['3D']
        selection_buttons.append({'label': label, 'method': 'update',
                                  'args': [{'visible': [trace.legendgroup in names for trace in figure.data]}, layout]})
    view_buttons = [{'label': label, 'method': 'relayout', 'args': [{'scene.camera': camera}]}
                    for label, camera in CAMERAS.items()]
    figure.update_layout(
        template='plotly_dark', paper_bgcolor='#101723', plot_bgcolor='#101723',
        margin={'l': 10, 'r': 200, 'b': 10, 't': 65}, font={'family': 'Arial, sans-serif'},
        scene={'aspectmode': 'data', 'camera': CAMERAS['3D'], 'dragmode': 'orbit',
               'xaxis': {'title': f'base_link X / {direction_labels[0]} (m)' if has_base else 'Reference X (m)', 'backgroundcolor': '#152031', 'gridcolor': '#334257'},
               'yaxis': {'title': f'base_link Y / {direction_labels[1]} (m)' if has_base else 'Reference Y (m)', 'backgroundcolor': '#152031', 'gridcolor': '#334257'},
               'zaxis': {'title': f'base_link Z / {direction_labels[2]} (m)' if has_base else 'Reference Z (m)', 'backgroundcolor': '#152031', 'gridcolor': '#334257'}},
        legend={'title': {'text': '坐标系开关' if has_base else '雷达开关'}, 'x': 1.02, 'y': 1, 'groupclick': 'togglegroup', 'itemsizing': 'constant'},
        updatemenus=[
            {'buttons': selection_buttons, 'x': 0, 'y': 1.09, 'xanchor': 'left',
             'bgcolor': '#24344b', 'bordercolor': '#49617e', 'font': {'color': '#ffffff'}},
            {'buttons': view_buttons, 'type': 'buttons', 'direction': 'right', 'x': 0.35, 'y': 1.09,
             'xanchor': 'left', 'bgcolor': '#24344b', 'bordercolor': '#49617e', 'font': {'color': '#ffffff'}}])
    figure.update_layout(scene_ranges(frames, list(frames), axis_length))
    return figure


def render_layout(configuration, output, axis_length=0.8, base_axis_length=2.0, overwrite=False):
    figure = build_figure(configuration, axis_length, base_axis_length)
    chart = figure.to_html(full_html=False, include_plotlyjs=True, div_id='sensor-layout', default_height='82vh',
                           config={'scrollZoom': True, 'displaylogo': False, 'responsive': True,
                                   'toImageButtonOptions': {'filename': 'sensor_layout_3d', 'scale': 2}})
    reference = html.escape(configuration['reference_frame'])
    has_base = configuration['reference_frame'] == 'base_link'
    direction_labels = base_directions(configuration)
    base_note = (f'<p style="color:#ffd166"><b>金色菱形为 base_link 车辆中心：X 朝{direction_labels[0]}、Y 朝{direction_labels[1]}、Z 朝{direction_labels[2]}。</b>'
                 f'车体坐标轴加粗，长度 {base_axis_length:g} m；雷达坐标轴长度 {axis_length:g} m。</p>') if has_base else ''
    forward_sensors = configuration.get('base_link_definition', {}).get('forward_sensors', [])
    if has_base and forward_sensors:
        basis = html.escape(', '.join(forward_sensors))
        base_note += (f'<p class="note">车头朝向基准：<code>{basis}</code> 的前雷达 +X、后雷达 −X；'
                      '不使用安装点连线推算朝向，不强行调整各雷达安装姿态。</p>')
    frame_note = ('所有坐标已换算到自定义 base_link；中心和朝向按已确认的雷达布局规则定义，非 CAD/测量真值，'
                  '保留各雷达实际安装高差。') if has_base else '这是雷达间相对外参，不是车体 base_link 坐标系。'
    notices = [configuration.get('status'), configuration.get('frame_definition_status')]
    warnings = configuration.get('warnings', [])
    notices.extend(warnings if isinstance(warnings, list) else [warnings])
    warning_note = ''.join(f'<p class="note">{html.escape(str(notice))}</p>' for notice in notices if notice)
    document = f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>多激光雷达 · 雷达坐标系 3D 布局</title>
  <style>
    body {{ margin: 0; background: #101723; color: #dce5f2; font-family: Arial, sans-serif; }}
    header {{ padding: 18px 24px 0; }}
    h1 {{ font-size: 22px; margin: 0 0 10px; }}
    p {{ margin: 7px 0; font-size: 14px; line-height: 1.5; }}
    .note {{ color: #9fb0c7; }}
    #sensor-layout {{ min-height: 560px; }}
    code {{ color: #d8e9ff; }}
  </style>
</head>
<body>
  <header>
    <h1>多激光雷达 · 各雷达自身 XYZ 坐标轴</h1>
    <p><b style="color:#ff5555">红色 +X</b> · <b style="color:#46d879">绿色 +Y</b> ·
       <b style="color:#559dff">蓝色 +Z</b> ｜ 统一参考系：<code>{reference}</code> ｜ 单位：米</p>
    <p>拖动旋转、滚轮缩放；左上选择单个雷达或前后区域放大；右侧图例可开关整组坐标轴。</p>
    {base_note}
    {warning_note}
    <p class="note">雷达箭头从各自真实原点出发，长度统一为 {axis_length:g} m，仅表示方向，不代表雷达尺寸。
       XYZ 等比例显示；{frame_note}仅显示安装布局，不叠加点云。</p>
  </header>
  {chart}
</body>
</html>
'''
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w' if overwrite else 'x', encoding='utf-8') as handle:
        handle.write(document)


def main():
    parser = argparse.ArgumentParser(description='Render sensor-local XYZ axes in the common reference frame; no calibration or point-cloud changes.')
    parser.add_argument('extrinsics', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--axis-length', type=float, default=0.8)
    parser.add_argument('--base-axis-length', type=float, default=2.0)
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    try:
        if (args.output.resolve() == args.extrinsics.resolve() or args.output.suffix.lower() != '.html' or
                (args.output.exists() and args.output.samefile(args.extrinsics))):
            raise ValueError('Output must be an HTML file distinct from the input extrinsics')
        if args.output.exists() and not args.overwrite:
            raise ValueError('Output exists; choose a different path or explicitly use --overwrite')
        configuration = read_extrinsics(args.extrinsics)
        render_layout(configuration, args.output, args.axis_length, args.base_axis_length, overwrite=args.overwrite)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    print(f'Wrote {args.output}; {len(configuration["sensors"])} sensors, 3 axes per sensor')


if __name__ == '__main__':
    main()
