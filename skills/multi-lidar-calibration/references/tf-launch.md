# ROS 2 静态 TF launch XML 导出

用户要求 XML/静态 TF 文件时使用 `scripts/export_tf_launch.py`，不必重新标定。只读取用户选定、已核验方向的外参；不套用示例车辆的数值，也不将局部优化候选自动视为通过验收。

## 输入契约

支持 JSON 或 YAML。使用本流程交付的 `extrinsics.json`/`extrinsics.yaml`，不是仅含矩阵的 `poses.json` 或 `registration.py` 的原始 refine 报告。若只有矩阵，先明确真实 frame_id、参考系和米制，再按约定转换为以下结构；不能把传感器简称猜成 ROS frame_id。

```json
{
  "reference_frame": "lidar_front",
  "length_unit": "meter",
  "sensors": {
    "rear": {
      "source_frame": "lidar_rear",
      "target_frame": "lidar_front",
      "translation_m": [-5.0, 0.0, 0.0],
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
    }
  }
}
```

数值仅演示结构，不是默认外参。矩阵方向为 `p_reference = R @ p_sensor + t`；XML 的 parent=reference、child=source_frame，不能取逆。参考传感器条目可省略；若存在则必须为单位变换，自动跳过自环。

脚本要求有限的三维平移、单位 `quaternion_xyzw`，各路 target 与 reference 一致且 source_frame 唯一；可选的 `T_reference_from_sensor` 若存在，必须与平移/四元数一致。frame_id 支持 ASCII 字母、数字、下划线、点、连字符及非开头的斜杠，不接受空白或 launch 替换表达式。节点名中的斜杠等符号会转为下划线；产生重名时停止，不修改 frame_id。

## 运行与格式

依赖 Python、NumPy、SciPy；YAML 输入另需 PyYAML。不依赖 Open3D 或 ROS，也不自动安装软件。

```bash
"$PYTHON" "$SKILL_DIR/scripts/export_tf_launch.py" \
  --extrinsics calibration/output/extrinsics.json \
  --output multi_lidar_tf.launch.xml
```

`SKILL_DIR` 为 skill 所在目录，`PYTHON` 为已有适用解释器。输入路径和输出文件名按用户指定，所有输出仍放在允许目录。

默认输出与用户示例相同的格式：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <node pkg="tf2_ros" exec="static_transform_publisher" name="lidar_front_lidar_rear_tf" args="-5 0 0 0 0 0 1 lidar_front lidar_rear"/>
</launch>
```

位置参数严格为 `x y z qx qy qz qw parent child`，不要混用欧拉角次序。生成时保留15位小数。发布前核对本机 publisher 的参数解析；某些版本会对旧式语法提示弃用。需要新式参数时添加 `--argument-style flags`，输出 `--x … --qx … --qw … --frame-id … --child-frame-id …`，不改变变换。

默认拒绝覆盖已有输出；用户明确要求更新后可加 `--overwrite`，但仍禁止覆盖输入外参。输出前先校验所有条目，失败时不写半份 XML。

## 交付检查与边界

- 重新解析 XML，核对节点数、唯一 child/name、参考系、四元数和矩阵一致性。八路雷达含一路参考时应为七个节点；若参考已换成独立的 `base_link`，则为八个节点，包括原参考雷达。不发布参考系自环。
- 有 ROS 环境时，用 `ros2 launch ./multi_lidar_tf.launch.xml --show-args` 验证解析；不执行普通 launch 或启动 publisher 进行导出测试。
- 不自动补 `base_link`。用户提供车体外参，或按 [车体坐标系说明](base-link.md) 确认安装布局定义后，先得到 `T_base_link_from_reference`，统一换基所有雷达；此脚本可直接从新 JSON 输出以 `base_link` 为参考的星形树，不负责推断车体中心。
- 若用户明确选择保留原雷达树，也可另加 `base_link → 原参考雷达` 的一条连接；不要再同时输出 base 到各雷达的直接边。两种拓扑择一，避免重复 parent、重复发布或 TF 环路。
- 原标定外参、原 XML 示例和原始数据不因导出被改写；现网已有同一 child 的 TF 发布者应在实际部署前核对。生成文件不等于获准部署，也不意味着对外参精度作额外保证。
