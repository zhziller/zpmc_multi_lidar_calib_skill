# 配准脚本输入与用法

`scripts/registration.py` 是可搬迁的局部联合优化/评估工具，不依赖案例目录或固定雷达名。依赖 Python 3.10+、NumPy、SciPy、Open3D；不自行联网安装依赖或改动环境。

## 输入

1. 每个源帧保存为 NPY 的 `N×3` 数组，或包含 `xyz` 的 NPZ。单位为米，数据位于各自传感器坐标系。ROS 解码、PCD/PLY 转换及时间同步分组由前置步骤完成。
2. JSON 配置示例：

```json
{
  "reference": "front",
  "sensors": {
    "front": {
      "train": ["clouds/front_000.npz", "clouds/front_001.npz"],
      "validation": ["clouds/front_010.npz"],
      "test": ["clouds/front_020.npz"]
    },
    "rear": {
      "train": ["clouds/rear_000.npz", "clouds/rear_001.npz"],
      "validation": ["clouds/rear_010.npz"],
      "test": ["clouds/rear_020.npz"]
    }
  }
}
```

文件路径相对于配置文件解析；也可用绝对路径。传感器 ID 和数量任意，至少两路。每路训练数组不能为空。评估时选定 split 的各路数组必须等长，同一数组下标必须是事先配好的时间组；缺帧用 `null`，并在另外的源数据 manifest 中保存真实时间与消息 ID。脚本不会自动推断这些文件是否时间对齐。

脚本拒绝在配置中重复使用同一个解析后的文件路径，包括跨 split/跨传感器。它不比较文件内容哈希，不能发现“同一源帧另存为不同文件”造成的数据泄漏；提取阶段应使用源 bag、topic、ID/时间戳校验 split。

3. 初值文件是 `{ "front": [[...4×4...]], "rear": [[...4×4...]] }`。每个配置中的传感器必须提供有限的正确 SE(3) 矩阵，表示从该传感器到 reference 的变换。参考必须为单位阵。也可以传入脚本先前生成的包含 `reference` 和 `poses` 的 refine 输出。

不能从参考系变了、矩阵取逆错误、未知传感器默认为单位阵的输入开始优化。脚本会拒绝非法刚体矩阵，但无法识别“数值合法、物理方向错误”的初值。

## 运行

使用运行时提供的 skill 路径定位脚本。下面的 `SKILL_DIR` 指向包含 `SKILL.md` 的目录；`PYTHON` 指向已有适用解释器，`work/` 是允许写入的目录。运行前先按数据检查参考设置工作区内的缓存/临时目录，不要依赖系统默认位置。

```bash
"$PYTHON" "$SKILL_DIR/scripts/registration.py" refine \
  --config work/dataset.json --poses work/initial_poses.json \
  --output work/refined.json --voxel 0.10 --distance 0.25 \
  --huber 0.015 --iterations 45

"$PYTHON" "$SKILL_DIR/scripts/registration.py" evaluate \
  --config work/dataset.json --poses work/refined.json \
  --split validation --output work/validation.json \
  --voxel 0.10 --distance 0.60
```

只有参数冻结后，才把第二条命令的 split 改为 `test`，写入另一个报告文件。输出已存在时默认拒绝覆盖；显式使用 `--overwrite` 也不能覆盖配置、点云或输入位姿。

## 参数与产物

- `refine` 只加载 train 帧；按传感器合并，估计局部面，逐对建立双向共视对应。固定 reference，并在未加阻尼的信息矩阵上检查秩；断开的图或几何欠约束时抛错，不输出完整外参。
- 信息矩阵先转换到各传感器原点处的扰动，再分别对每组三维旋转/平移使用统一尺度归一化，保留方向间的强弱差异。`--min-information-ratio` 默认 `1e-4`，最小/最大特征值比不足时停止；这是保守的局部几何检查，不是精度保证。不要为了让单平面数据通过而盲目降低阈值。
- 距离阈值前5轮至少0.6 m，随后5轮至少0.4 m，之后为 `--distance`。这是近似初值附近的局部优化，不能修复任意错误的全局初始化。
- `--angle` 控制法向夹角；`--normal-radius` 限制邻域支撑；`--min-range`/`--max-range` 在传感器本地坐标中裁量程。邻域默认30点，平面性阈值见脚本，需要时根据采样特征适配。
- `--residual-fraction` 仅训练阶段使用，剔除绝对残差大于当前距离阈值乘该比例的对应；`evaluate` 不使用这项残差剔除。
- `--min-matches` 控制优化边和评估连通性所需点数。评估即使某边不满足该数，也保留其实际统计，零对应报告 count=0/null；不可把零值当作零误差。
- 输出 refine JSON 包含 `poses`、收敛标记、阈值、信息特征值和逐轮逐对指标。`history` 属于每次更新前；输出前以最终位姿和 `--distance` 重建对应并重新检查几何，记录为 `final_geometry`。`converged=false` 表示达到迭代上限，不等于任务成功；`converged=true` 也只是局部数值收敛。
- evaluate 输出是 `descriptive_consistency_only`，不计算可观测性或认证数据独立性，没有统一的“标定精度合格”判定。即使图连通、平面误差很小，也需要按流程另查来源和弱方向，并结合覆盖率、缺路、误差尾部和实际用途判断。

此配准脚本不生成可视化或部署文件，不提供绝对精度/时间偏差/车体坐标。可视化按 [验证与交付](validation-delivery.md) 处理；已有外参转静态 TF XML 时使用单独的 [XML 导出脚本](tf-launch.md)，不需要重新配准。
