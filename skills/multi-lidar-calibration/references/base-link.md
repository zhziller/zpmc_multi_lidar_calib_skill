# 自定义 base_link、坐标换基与 3D 布局

适用于已有可信雷达间外参，用户要求车辆中心、车体轴向或坐标轴网页的任务。这是坐标系定义与刚体换基，不是重新标定，也不能提升已有外参的物理精度。

## 定义前确认

- 明确原点、正前方、上方、单位和各轴约定；车体坐标系固定于车体，不随倒车改变，不动态对齐世界重力。
- 区分实测/CAD 车体外参与用户授权的安装布局近似。没有测量基准时，须声明中心和朝向是约定，不是车辆几何中心的测量真值。
- 分别记录用于中心、前向、上向/高度的传感器集合。排除某雷达参与定义，不等于删除它的外参或不再导出它。
- 若仅将已有轴向换成另一约定，保持已确认原点与不变轴；不要顺带重新估计中心、拟合点云或强制安装对称。

## 四路主周扫的安装规则

以下是一种可选的四主雷达布局定义；使用前须由用户确认传感器映射与安装方向，不能按名称猜测。

- 仅配置中的四路主雷达参与中心、朝向与高度；其他 BP、上方或辅助雷达均不参与，但全部保留在输出中。
- 左前与右后、右前与左后分别中心对称；左前/右前、左后/右后并不要求左右镜像对称。不要将四个点强行改成矩形或移动到对称位置。
- 四路主雷达的本地 X 轴基本平行：前雷达 +X 朝前，后雷达 +X 朝后。这里是安装先验，不要求各路偏角严格为零。
- 脚本构造 **X朝前、Y朝左、Z朝上**。如果现有 base 为 X右/Y前/Z上，则使用独立的换轴模式，而不是重新估计中心。

### 从原始相对外参构造车体系

记每路变换为 `T_reference_from_sensor`，满足 `p_reference = rotation @ p_sensor + position`。`rotation` 的三列分别是该雷达本地 +X、+Y、+Z 在参考系中的方向。以下所有向量均在原参考系表达。

1. 分别计算左前/右后、右前/左后的三维中点，取两个中点的平均为 `center`，等价于四路位置的均值；包括高度，不添加任意地面高度。保留两中点及其差异作为诊断，不强行令它们重合。
2. 将四路本地 +Z 按已确认的上方向统一符号，平均后归一化得到 `up`。配置 `up_hint_in_reference` 必须给出已确认的原参考系上方向，不能默认参考 +Z 朝上。统一符号只用于估计 `up`，不翻转雷达自身姿态。
3. 取前两路的本地 +X、后两路的本地 −X。逐个投影到垂直于 `up` 的平面并归一化，再等权平均、归一化得到 `forward`。使用轴方向而不是前后安装点中线：错位安装时后者会引入错误航向。
4. 取 `left = normalize(cross(up, forward))`，再以 `forward = normalize(cross(left, up))` 正交化。组成右手系 `rotation_reference_from_base = column_stack([forward, left, up])`。
5. 构造 `T_reference_from_base_link`，其平移为 `center`；求逆得到 `T_base_link_from_reference`。对全部雷达执行：

```text
T_base_link_from_sensor = T_base_link_from_reference @ T_reference_from_sensor
```

若方向不明确、投影接近零、平均方向抵消或结果非刚体变换，停止并核对安装条件，不能用单位向量或强制调平掩盖问题。小的安装偏角与高差保留；四路在新系中的平均 Z 为零，并不代表每路 Z 都为零。平均的安装 Z 方向也不等同于精密测量的重力方向。

## 仅切换轴向：X右/Y前 → X前/Y左

原点和物理 +Z 不变。新基底相对于旧基底绕原 +Z **+90°**，但点坐标要按逆旋转换算；不能给所有雷达的欧拉 yaw 简单加 90°。

```text
T_old_base_from_new_base =
[[ 0, -1, 0, 0],
 [ 1,  0, 0, 0],
 [ 0,  0, 1, 0],
 [ 0,  0, 0, 1]]

T_new_base_from_old_base = inverse(T_old_base_from_new_base)
T_new_base_from_sensor = T_new_base_from_old_base @ T_old_base_from_sensor
T_reference_from_new_base = T_reference_from_old_base @ T_old_base_from_new_base

x_new = y_old
y_new = -x_old
z_new = z_old
```

同时更新平移、旋转矩阵、四元数和可选 RPY；不要只改 frame 名称、网页标签或 XML 中的角度。四元数采用 `xyzw`；由新旋转矩阵重新生成，比较旋转时允许四元数正负等价。历史参考变换元数据也必须同步更新，不能留成旧坐标约定。

## 外参、网页与 XML 同源

新文件保留 `length_unit: meter`、真实 `source_frame`，并设置 `reference_frame` 和各路 `target_frame` 为 `base_link`。将新矩阵、平移、四元数及带单位/约定的可选 RPY 保持一致。不覆盖原始标定文件。

在 `base_link_definition` 中记录可复现依据：

- `axis_convention`，本例为 `{"x": "front", "y": "left", "z": "up"}`。
- `original_reference_frame`、`center_sensors`、`forward_sensors`、`up_and_height_sensors` 与 `excluded_from_definition`。
- 对角配对、原系中的中点、方向符号和中心/高度/朝向算法；说明未施加左右镜像或强制平行约束。
- `T_original_reference_from_base_link` 及其逆 `T_base_link_from_original_reference`，原始输入 SHA-256；如从已有 base 文件切换，也保留该输入的来源及换基关系。
- 状态注明“用户定义的安装布局近似，非测量/CAD 真值”。

### 3D 坐标轴网页

- 以统一外参绘制每路真实原点和本地 XYZ 轴：第 `axis_index` 个正轴端点为 `position + axis_length * rotation[:, axis_index]`，不要误用行向量或逆矩阵。
- 红 X、绿 Y、蓝 Z；XYZ 等物理比例、单位米，保留安装高差/倾斜。箭头长度仅示意方向，不是传感器尺寸。
- 在 `base_link` 参考系中额外绘制原点为零、旋转为单位阵的虚拟车体系，用大标记、粗长轴和醒目文字区别于真实雷达。它不算一个额外雷达，也不生成 TF 自环。
- 轴标签从 `axis_convention` 读取，不能硬编码旧的“X右/Y前”；定义不明时标注未指定，不能猜测。
- HTML 内嵌 JS，支持离线旋转、缩放和按雷达开关/聚焦；标题说明是安装布局还是点云拼接。仅画布局不能宣称已经检查点云拼接。
- 用户只要 3D 时不再生成投影图；用户负责可视化检查时仍做数值/结构校验，但不擅自启动浏览器或声称已视觉验收。

### TF 导出

读取 [XML 导出说明](tf-launch.md)，对已换基的新 JSON 使用 `scripts/export_tf_launch.py`。参考为 `base_link` 且八路均为真实雷达时，应得到 **8 条直接变换**，包括此前作为参考的左前雷达。它不同于原雷达参考系的七条边；无需另插入 `base_link` 自环或重复的参考连接。

新旧 TF 文件不能同时用于发布同一组 child。只生成/校验文件，不启动节点；最终 frame 名称与现网发布者在部署前另行核对。

## 数值与结构验收

- 检查所有矩阵有限、齐次末行、旋转正交及 `det=+1`，四元数归一化并与矩阵一致；两方向参考变换互逆。
- 对每对雷达比较换基前后的 `inverse(T_reference_from_first) @ T_reference_from_second`，应在浮点容差内不变；这是换基正确性，不是新一次外参精度验证。
- 四路定义时检查对角中点、平均高度和前/后朝向；扰动 BP/上方雷达不能改变 base 的定义，但这些雷达仍应保留在所有新产物里。
- 对纯 90° 轴向切换，核对原系中的原点与物理 +Z 不变，以及 `x_new=y_old`、`y_new=-x_old`、`z_new=z_old`；全部旋转也须符合统一左乘。
- 核对 JSON、XML 和网页的坐标/旋转同源；解析 XML 检查节点数、唯一 child/name 和重建矩阵。网页的轴端点应对应矩阵列，标签应对应当前约定。
- 核对原始外参与旧部署文件的哈希未变；区分“数值检查通过”和“用户视觉检查通过”。

可移植脚本与配置示例见下节；不依赖特定工作区、车辆尺寸或真实 frame_id。

## 已打包的脚本

`scripts/define_base_link.py` 和 `scripts/sensor_layout.py` 与本 skill 一起分发。依赖 Python 3.10+、NumPy、SciPy、Plotly；YAML 输入另需 PyYAML，版本范围见 skill 内 `requirements.txt`。无需 ROS 或 Open3D，不自动安装依赖。

定义中心前创建 JSON 布局，必须使用输入外参中的传感器 ID，不是自行猜测 frame_id：

```json
{
  "front_sensors": ["front_left", "front_right"],
  "rear_sensors": ["rear_left", "rear_right"],
  "diagonal_pairs": [["front_left", "rear_right"], ["front_right", "rear_left"]],
  "up_hint_in_reference": [0, 0, 1]
}
```

这些 ID 与上方向只是结构示例；确认实际安装后再使用。当前中心定义模式要求恰好两前两后四路主雷达，每路恰好出现在一组前后对角配对中。输入可以包含任意数量的其他雷达，它们不参与定义但会一起换基。输入结构同 [XML 外参契约](tf-launch.md)，且每路必须提供 `T_reference_from_sensor`；不是配准脚本原始的 `poses` 报告。

```bash
"$PYTHON" "$SKILL_DIR/scripts/define_base_link.py" \
  --extrinsics work/extrinsics.json --layout work/vehicle_layout.json \
  --output work/extrinsics_base_link.json

"$PYTHON" "$SKILL_DIR/scripts/sensor_layout.py" \
  work/extrinsics_base_link.json --output work/sensor_layout.html \
  --axis-length 0.8 --base-axis-length 2.0

"$PYTHON" "$SKILL_DIR/scripts/export_tf_launch.py" \
  --extrinsics work/extrinsics_base_link.json --output work/multi_lidar_tf.launch.xml
```

只切换已有 X右/Y前/Z上坐标系，保留原点与上方向：

```bash
"$PYTHON" "$SKILL_DIR/scripts/define_base_link.py" \
  --extrinsics work/extrinsics_base_rfu.json --right-forward-to-front-left \
  --output work/extrinsics_base_flu.json
```

换轴模式要求输入 `reference_frame=base_link`，且 `base_link_definition.axis_convention` 明确为 `{"x":"right","y":"front","z":"up"}`，防止重复旋转或方向误用。已提供的原参考正反变换会一起更新；原 `status`、`warnings` 和传感器诊断保留，换基不将候选升级为验收通过。输入 SHA-256 记录在 `base_link_processing_input`。

默认拒绝覆盖输出；用户允许更新时才加 `--overwrite`。始终禁止覆盖输入外参或布局配置，包括软/硬链接指向同一文件的情况。网页状态/警告按输入显示；生成页面不自动打开浏览器。
