# G1 本地动作播放手册：CSV、Kimodo NPZ 与 SMPL PT


更新日期：2026-11-13。现场操作者已确认 CSV、Kimodo NPZ，以及 **GENMO 生成的 SMPL PT 经本体转换后播放** 三条链路均在 G1 上真机验证成功。此记录针对当前候选源码，不表示 v0.1.0 预编译包已包含这些功能。PT 上传、独立 CPU 导入环境及完整命令见 [SMPL PT 本体播放](SMPL_PT_PLAYBACK_zh.md)。

使用 [README](../README.md) 的联网安装入口并启用 `playback`。虚拟环境、模型、引擎、凭据及动作文件由部署机本地管理。

以下命令在 G1 上执行：

```bash
cd ~/OnboardMimic
```

## 1. 准备 CSV 动作

已有正确的 SONIC CSV 文件夹可跳过转换。文件夹须在运行时的 `--motion-root` 下；当前目录为 `var/motions`。播放器需要 `joint_pos.csv`、`joint_vel.csv`、`body_quat.csv`，均带表头，50 Hz，29 关节为 SONIC 交错参考顺序，根四元数为 wxyz。

GMR PKL 在端侧离线转换，输入为可信 G1 GMR 生成的文件。输出目录必须尚不存在，重转时换新名称：

```bash
.venv/bin/python -m onboardmimic.backends.sonic.motion_import gmr \
  var/gmr/wave.pkl var/motions/wave_reimport \
  --input-joint-order mujoco

.venv/bin/python -m onboardmimic.backends.sonic.csv_player \
  var/motions/wave_reimport --fps 50 --joint-order sonic_reference
```

`mujoco` 表示 GMR 的 `qpos[7:]`／硬件分组顺序，导入器执行 GMR→SONIC 交错重排；`isaaclab` 仅表示输入已经是交错参考顺序。源帧率读取 PKL 的 `fps`，导入时重采样到 50 Hz，根旋转 xyzw→wxyz。`import.json` 应记录 `joint_mapping=gmr_to_sonic_reference_v2`。不要使用修复前跳过重排的旧 CSV；更新代码不会自动改写旧动作。

已有动作时先核对源 SHA-256，再对照转换后的关节顺序及时间轴。

## 2. 准备 Kimodo NPZ 动作

主机先将 Kimodo 原始输出转换为 AMASS NPZ；端侧要求 `pose_body (T,63)`、`root_orient (T,3)` axis-angle 及真实源帧率。含 `local_rot_mats/posed_joints` 的原始 Kimodo NPZ 不能直接作为此输入。

在 G1 将 AMASS 导入为内部 SMPL 缓存。输出 NPZ 必须尚不存在：

```bash
.venv/bin/python -m onboardmimic.backends.sonic.motion_import kimodo \
  var/kimodo/propose_amass.npz var/motions/kimodo_propose_new.npz

.venv/bin/python -m onboardmimic.backends.sonic.smpl_player \
  var/motions/kimodo_propose_new.npz
```

导入器读取 `mocap_frame_rate`，兼容 `mocap_framerate`／`fps`。元数据缺失或冲突时，明确提供真实源帧率，例如在导入命令末尾加 `--fps 30`；它不是播放速度设置。导入生成 50 Hz 缓存，端侧不需要 Torch 或 Kimodo 模型。原 AMASS NPZ 和内部缓存虽然都是 `.npz`，只有导入后的缓存可供 `playback.py load` 使用。

转换代码或映射变化后旧缓存可能报 `Stale SMPL cache`；从原 AMASS 重新导入到新路径，不修改指纹绕过检查。当前只控制 29 自由度身体，附加的手指、脸、形状及 `trans` 不用于此身体播放链路。

## 2.1 准备 GENMO SMPL PT 动作

先将主机生成的 `smpl_params.pt` 上传到 G1 的 `var/smpl/`。本体已安装独立 `.venv_import`，在策略停止时执行；输出名称必须尚不存在：

```bash
.venv_import/bin/python -m onboardmimic.backends.sonic.motion_import smpl \
  var/smpl/wave.pt var/motions/genmo_wave_new.npz \
  --group body_params_global --fps 50

.venv/bin/python -m onboardmimic.backends.sonic.smpl_player \
  var/motions/genmo_wave_new.npz
```

`--fps 50` 复现原 PT 发布命令的时序；按原视频速度播放时填写真实源帧率，导入器统一生成 50 Hz 缓存。导入在本体 CPU 上完成，后续播放器通过本机 ZMQ 向 SONIC 供帧，主机无需持续运行发布器。PT 和 Kimodo 共用内部 SMPL 缓存格式及以下播放操作。

## 3. 在策略停止时加载一种动作

先查看状态，加载要求 `state=stopped/error` 且 `pids=[]`：

```bash
.venv/bin/python scripts/playback.py status
```

CSV 示例（加载已生成的修正版）：

```bash
.venv/bin/python scripts/playback.py load wave_g1_fixed_20261113 \
  --fps 50 --joint-order sonic_reference
```

或者加载刚导入的 Kimodo 缓存：

```bash
.venv/bin/python scripts/playback.py load kimodo_propose_new.npz
```

GENMO PT 导入缓存则使用：

```bash
.venv/bin/python scripts/playback.py load genmo_wave_new.npz
```

动作参数是相对 `motion-root` 的路径，不能写绝对路径。加载不会连接服务、准备姿态、启动策略或播放。换动作前先结束当前策略，再加载。

## 4. 人工启动并播放

人员在场、机器人可靠支撑且现场急停可用时，在控制台逐步执行 **连接服务 → 准备姿态 → 启动控制器**；每一步等待完成后再继续。若服务已连接，也可逐条使用 CLI：

```bash
.venv/bin/python scripts/control.py prepare
.venv/bin/python scripts/playback.py status
```

确认 `state=prepared` 和现场姿态后再启动：

```bash
.venv/bin/python scripts/control.py start
.venv/bin/python scripts/playback.py status
```

确认 `state=active`、Planner 已站稳且反馈正常后，显式开始播放：

```bash
.venv/bin/python scripts/playback.py play
.venv/bin/python scripts/playback.py status
```

CSV 播放状态为 `input_source=file_joint`、`reference_representation=joint`，后端为 streamed／encoder 0；Kimodo 与 GENMO PT 缓存均为 `file_smpl`、`smpl`，后端为 streamed／encoder 2。等待 `mode_switch_pending=false`，再核对播放位置递增。单次播放，不自动循环；正常结束请求回到零速 Planner。

## 5. 暂停、恢复、停止与急停

以下是独立操作，按需要单独执行：

```bash
.venv/bin/python scripts/playback.py pause
.venv/bin/python scripts/playback.py resume
.venv/bin/python scripts/playback.py stop
```

暂停冻结文件时间并持续发布当前参考；恢复从暂停位置继续，不补播暂停期间的时间。`stop` 停止文件并请求零速 Planner，策略仍可能运行；用 `status` 确认切换完成。

结束策略或发生异常时使用原急停入口：

```bash
.venv/bin/python scripts/control.py estop
```

急停会终止策略，不保证站立，需保持现场支撑。服务或机器人重启后不会自动恢复文件加载和播放。

## 6. 首帧检查与常见问题

`play` 默认等于 `play --entry-checks on`：检查首帧参考速度不超过 0.6 rad/s、根部倾斜差不超过 0.15 rad。当前实现允许首帧关节位置跳变，不添加自动姿态对齐或平滑过渡；`on` 也不会恢复位置偏差门槛。操作者需要按次跳过速度／倾斜门槛时可显式用 `play --entry-checks off`，但它不转换关节顺序，也不关闭人工启动、反馈及运行时保护。

- `Output already exists`：换新的输出目录／文件名。
- `请在控制器未接管时加载动作`：先结束策略并确认 PID 已退出，再加载。
- `请人工准备、启动策略并等待 Planner 站稳`：按第 4 节完成启动和状态确认。
- `Stale SMPL cache`：从原 AMASS／PT 重新导入缓存。
- 仿真姿态扭曲：检查输入来源和关节顺序；同一 PKL 的六 CSV 可用 `scripts/compare_motion_csv.py EXPECTED ACTUAL` 对比，不能以旧错误 CSV 为标准。
