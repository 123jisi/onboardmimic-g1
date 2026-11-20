# G1 上传 PT 后完全在本体播放

更新日期：2026-11-13。现场操作者已确认 **GENMO 生成 PT → 上传 G1 → 本体转换 → 本地播放器真机播放** 验证成功。此验收针对当前 G1 候选源码，不表示 v0.1.0 预编译包已包含该功能。

主机负责生成并上传 `smpl_params.pt`；本体使用独立 CPU Torch 环境读取 PT、生成内部缓存，由机载服务播放。导入与播放均不依赖主机持续推流，不需要在 G1 上安装 Genmo 或启动主机仿真。

实际链路：`G1 本地 PT → CPU 离线导入 → 本地 SMPL 缓存 → 机载播放器 → 本机 ZMQ pose v3 → SONIC encoder 2 → 本体控制`。PT 是 SMPL 人体参数，不是可直接给电机的关节指令；内部缓存保留人体姿态、关节和根旋转的转换结果。

## 已验证的 wave 示例

- 本体仓库：`~/OnboardMimic`。
- 原始 PT：`var/smpl/wave.pt`，与主机 `genmo-teleop/outputs/inference/wave/smpl_params.pt` 的 SHA256 一致。
- 缓存：`var/motions/wave_pt_50hz_20261113.npz`，选择 `body_params_global`，按原命令的 `--fps 50` 导入；166 帧，播放约 3.32 秒。
- 部署时已完成原始 PT 的本体转换与实际 API `load` 检查；随后操作者确认 GENMO PT 转换后在 G1 上真机播放成功。离线检查和现场播放是两份独立验证证据。

可以跳过上传和导入，直接按下方“启动与播放”操作。服务重启后需重新 load；不会自动播放。

## 上传新动作

在主机执行，替换部署用户名、地址、输入路径和动作名。

```bash
ssh DEPLOY_USER@G1_HOST 'mkdir -p ~/OnboardMimic/var/smpl'
scp /path/to/smpl_params.pt \
  DEPLOY_USER@G1_HOST:~/OnboardMimic/var/smpl/wave_new.pt
ssh DEPLOY_USER@G1_HOST
```

之后的命令均在 G1 执行：

```bash
cd ~/OnboardMimic
.venv/bin/python scripts/playback.py status
```

在策略停止时离线导入。输出名称必须尚不存在；更换 PT 后使用新输出名：

```bash
.venv_import/bin/python -m onboardmimic.backends.sonic.motion_import smpl \
  var/smpl/wave_new.pt var/motions/wave_pt_new.npz \
  --group body_params_global --fps 50

.venv/bin/python -m onboardmimic.backends.sonic.smpl_player \
  var/motions/wave_pt_new.npz

.venv/bin/python scripts/playback.py load wave_pt_new.npz
```

`--fps` 是输入时间轴的帧率：50 复现用户原发布器 `--fps 50 --stride 1` 的时序。若想按原始视频速度播放，需填写真实源帧率，例如 30，导入器会重采样到播放器使用的 50 Hz；不能因为播放器是 50 Hz 就把任意源文件都填成 50。该 PT 本身没有帧率字段。

支持顶层 SMPL 字段和 `body_params_global/body_params_incam` 嵌套字段，`auto` 优先 global。导入仅在 CPU 上运行；本体 `.venv_import` 用于 PT 导入，`.venv` 用于运控和播放。load 只读取缓存，不准备姿态或启动控制器。

## 启动与播放

先在现有控制台 `http://G1_HOST:8769` 执行“连接服务”。机器人有现场支撑、人员和急停就绪后，逐步执行，每一步观察状态后再继续。

对于本轮已经导入的 wave，如果服务重启或加载过其他动作，先在策略停止时重新加载：

```bash
.venv/bin/python scripts/playback.py load wave_pt_50hz_20261113.npz
```

若刚加载的是新动作，保留新动作即可。随后人工准备姿态：

```bash
.venv/bin/python scripts/control.py prepare
.venv/bin/python scripts/playback.py status
```

确认 `state=prepared`、姿态符合现场条件，再启动策略：

```bash
.venv/bin/python scripts/control.py start
.venv/bin/python scripts/playback.py status
```

确认 `state=active`、Planner 站稳、反馈正常，再播放：

```bash
.venv/bin/python scripts/playback.py play
.venv/bin/python scripts/playback.py status
```

等待 `mode_switch_pending=false`、`input_source=file_smpl`、`reference_representation=smpl`，观察播放位置递增。由本体 systemd 服务持续供帧；启动命令返回后可关闭 SSH 终端或主机，不需要主机维持 ZMQ、PICO 心跳或网页。执行单次播放，结束后请求零速 Planner；不自动循环或重启恢复。

按需要单独使用：

```bash
.venv/bin/python scripts/playback.py pause
.venv/bin/python scripts/playback.py resume
.venv/bin/python scripts/playback.py stop
```

`stop` 停止文件并回到 Planner，策略仍可能运行。结束策略用原有急停入口 `.venv/bin/python scripts/control.py estop`。首帧速度、倾斜及反馈检查沿用现有播放器，见[完整播放手册](MOTION_PLAYBACK_zh.md)。

## 在另一台同配置 G1 配置导入环境

本机已完成此步骤，无需重复。JP5 / Ubuntu 20.04 / ARM64 / Python 3.8 环境使用：

```bash
bash install_scripts/install_smpl_import_jp5.sh
```

也支持无网安装；当前本体 `var/smpl-import-wheels` 已保留全部依赖：

```bash
bash install_scripts/install_smpl_import_jp5.sh var/smpl-import-wheels
```

脚本创建独立 `.venv_import`，固定 NumPy 1.24.4、SciPy 1.10.1、CPU Torch 2.4.1 及依赖，校验 `weights_only=True` CPU 读取和 pip 依赖。它不升级运控环境，也不启动服务或机器人。版本及 wheel 来自 [PyPI 的 Torch 2.4.1 发布](https://pypi.org/project/torch/2.4.1/)。PT 导入使用 tensor/primitive loader，不回退执行任意 pickle；导入对象为本地生成的可信数据。

## 验证与源码管理

虚拟环境、模型、引擎、动作和凭据在目标机器本地管理。

本轮验证：G1 原始 wave PT→缓存→实际 API load 通过；主机 40 项回归通过；G1 独立导入环境 3 项 PT 测试、运控环境 37 项既有转换与播放回归通过。G1 生成缓存与用户实际使用的 `GR00T-WholeBodyControl/gear_sonic/scripts/publish_smpl_params.py` 逐帧比较：pose 和 joint_vel 相同，local joints 最大误差 2.27e-6 m，手腕 2.99e-8 rad，根旋转角差 5.01e-7 rad，均小于 1e-5。跨 SciPy 版本的四元数整体正负号按同一旋转处理。

主机离线重做差分：

```bash
/path/to/GR00T-WholeBodyControl/.venv_teleop/bin/python \
  scripts/check_smpl_pt_parity.py \
  --sonic /path/to/GR00T-WholeBodyControl \
  --input /path/to/smpl_params.pt \
  --group body_params_global --fps 50
```

该工具仅导入可信参考代码和数据，不调用参考发布器 main，不发送控制数据。`--cache` 可指定从 G1 取回的缓存，`--output-json` 可保存数值证据。

现场验收：2026-11-13，操作者明确反馈“通过 pt（GENMO 生成）转化再播放真机验证成功”。已确认上述文件链路可在当前 G1 上使用；此结论不扩展为任意生成动作、长期运行、异常分支或主机关机测试均已验证。后续新动作仍按本手册逐步加载和播放。
