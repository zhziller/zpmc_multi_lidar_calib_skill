# 数据完整性、解码与环境

## 环境与写入边界

先尝试已有 ROS Python 和数值环境；解析 ROS 消息的解释器可以与配准环境分开，用 NPY/NPZ 作为中间格式。不要为了一个包升级系统 ROS、全局 NumPy 或用户已有 conda 环境。

若需要安装，在允许的工作目录中创建命名清晰的独立环境。执行安装或绘图前设置 `TMPDIR`、`PIP_CACHE_DIR`、`XDG_CACHE_HOME`、`MPLCONFIGDIR`、`ROS_LOG_DIR`；使用 conda 时同时设置 `CONDA_PKGS_DIRS`、`CONDA_ENVS_PATH`。先创建这些目录，避免默认落到用户目录或系统临时目录。无 GPU 需求时可考虑 CPU 版 Open3D；包名/版本以实际可用性为准，不盲目使用旧案例的锁定版本。

数据包体积较大时先元数据巡检，再抽样，最后有界全量检查。限制线程数，保存真实运行的会话 ID；观察超时不等于任务停止。

## ROS 2 SQLite bag

使用 `sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)` 只读打开已停止录制的 bag。若录制仍在进行，应先由用户提供一致性快照；不要悄悄忽略 WAL。

1. 查看 metadata、实际文件列表、话题表和消息表结构，逐文件统计。支持分卷，不假设只有一个 DB3。
2. 运行 `PRAGMA quick_check`；必要时检查 `integrity_check`。保存异常摘要，避免把上千行错误直接刷屏。
3. 分别检查只读元信息查询和实际读取 BLOB。只读取 `length(data)` 成功不代表整个消息能读出；“索引坏”与“负载读失败”需要证据区分。
4. 顺序遍历遇坏页失败后，可用主键范围进行有界逐条查询。主键不保证从 1 开始或连续；元数据计数不能直接等同于最大 ID。使用可读索引、查询到的上下界或明确声明的候选区间，记录扫描覆盖范围和未覆盖的不确定性。
5. 对可读 BLOB 真正反序列化，保存原 bag/分卷、消息 ID、话题、record/header 时间戳、源文件哈希和错误类型。

ROS 1 bag、MCAP、PCD/PLY 则使用对应读取器；不能对非 SQLite 格式照搬数据库修复。SQLite 恢复或 reindex 只能在经允许的副本上进行。本流程不自动修复原包。

## PointCloud2 解码

- 从 `fields`、`datatype`、`count`、`offset` 和 `point_step` 构造 dtype；使用 `is_bigendian` 决定字节序。
- 按 `(height, width)` 与 `(row_step, point_step)` 步长访问，支持行填充；验证 `len(data) == height * row_step` 和 `row_step >= width * point_step`。
- 不假设 width 是水平点数、height 是线数，也不把 16/32 线、26 字节布局、1800 行写死。
- 分开统计 NaN、Inf、全零/近零点、超量程点和有效 XYZ；保留 ring/intensity/逐点时间字段的原始含义。不要猜测未知字段单位或 ring 与物理线束的精确对应。
- 无回波和遮挡可能形成大量 NaN；仅凭比例不能判断硬件故障。`is_dense` 也不能替代实际检查。

提取产物建议：每条可用消息一个 NPZ，至少 `xyz`，可附 intensity/ring/timestamp；独立 manifest 保存传感器名、frame_id、消息 ID、时间戳、split、同步组、文件和点数。量程裁剪在传感器自己的坐标系内进行并记录阈值。

## 时间与静态条件

分别检查 record 时间、header 时间和逐点时间：间隔、倒退、重复、扫描时长，以及 header 接近扫描开始还是结束。时间单位不能由字段名推断，先比较量级和变化。

按照时间窗口匹配各路最近扫描，报告窗口和最大差；只有足够规律的同频数据才适合近似槽位分组。缺失/不可读帧导致的间隔不能直接报告为录制丢帧。

同一路在早晚帧之间做局部距离或小位姿检查，识别车辆移动、动态物体和振动。对静态假设不成立的数据，改用运动补偿/动态标定或请求重新采集；不能叠帧后把运动重影误判为外参。

不要靠静态数据声称已标定硬件时间偏差。多个扫描 header 相差很小，也不能证明逐点时钟无固定偏置。
