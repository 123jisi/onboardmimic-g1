# OnboardMimic G1

OnboardMimic 是面向人形机器人的板载 S0 平台。本仓库面向宇树 G1 EDU，把 SONIC 控制器、动作导入与回放、遥操接收和桌面／手机控制台部署在机器人本机。机器人通过本机 ROS 2、CycloneDDS 和桥接服务与底层通信。

OnboardMimic G1 integrates NVIDIA SONIC, local motion playback and optional PICO, PN Link and BrainCo inputs on Unitree G1 EDU. Start with the [English source build guide](docs/SOURCE_BUILD.md), [architecture](docs/ARCHITECTURE.md) and [validation scope](docs/VALIDATION.md).

原项目由 LambdaHumanoid 开发，作者为 [Yiming Jiang](https://iridescentjiang.github.io/)、[Yilun Chen](https://yilunchen.com/about/)（项目负责人）及 [Codex](https://openai.com/codex/)。SONIC 控制器、模型及第三方组件的归属和使用条款见[项目来源](docs/PROJECT.md)和[许可证](LICENSE)。

## 功能

- **核心**提供 SONIC Planner、控制器生命周期、G1 原生通信桥、状态反馈、桌面与手机控制台。操作员从控制台显式连接服务、准备姿态和启动控制器。
- **`playback`** 启用本地动作库、CSV 回放、GMR PKL 导入和 GENMO SMPL PT、Kimodo AMASS NPZ 转换。PT 导入使用独立的 CPU Torch 环境，转换输出可以复用。[动作回放指南](docs/MOTION_PLAYBACK_zh.md) · [PT 导入指南](docs/SMPL_PT_PLAYBACK_zh.md)
- **`pico`** 运行机器人端的 XRoboToolkit 接收容器，提供身体跟随、三点跟随、手柄输入和上身冻结模式。头显需安装匹配的发送端，并能访问 G1。
- **`pnlink`** 用 ARM64 SDK 接收 PN Link 动捕数据，支持现场配置的有线或 Wi-Fi 链路。操作者从网页开启接收、校准和预览。
- **`brainco`** 构建独立手部适配器，依赖厂商现有的 `brainco_hand.service`。与 Pico 或 PN Link 配合使用时前者提供扳机输入，后者支持双手标定与跟随。

仓库中的 `onboardmimic/core` 负责运行时和控制权，`onboardmimic/backends/sonic` 存放 SONIC 适配层与原生构建代码，`onboardmimic/robots/unitree_g1` 包含 G1 传输和手部适配，`onboardmimic/inputs` 与 `onboardmimic/console` 分别提供接收器和控制台。安装入口位于 `scripts/`，离线回归位于 `tests/`。

## 环境要求

- 宇树 G1 EDU，29 自由度本体，Orin NX 16 GB（ARM64）。系统为 Ubuntu 20.04、系统 Python 3.8、JetPack 5.1.1 / L4T 35.3.1、CUDA 11.4 与 TensorRT 8.5.2，需有原厂内部 DDS 直连路由。
- 使用部署用户的终端和可用的 `systemctl --user` 会话。脚本按需调用 `sudo` 安装包及设置服务。核心安装至少预留 8 GiB，选择扩展时至少预留 12 GiB。
- G1 需能访问 Ubuntu、ROS、GitHub、PyPI、NuGet 和 Hugging Face。Pico 另需 Docker Hub 与 Ubuntu 容器软件源。固定版本的 SONIC 原始模型合计约 865 MB，构建会另外生成引擎与缓存。

## 从 GitHub 联网安装

在准备做首次部署的 G1 上，确认控制器和桥接服务已停止，以普通用户执行。

```bash
git clone https://github.com/123jisi/onboardmimic-g1.git OnboardMimic
cd OnboardMimic

python3 scripts/setup_g1_online.py --check
python3 scripts/setup_g1_online.py
```

`--check` 只检查平台、磁盘、内部路由和服务冲突。安装器配置签名验证的 APT 依赖，建立 `.venv` 和固定版本的 ROS 2 工作空间，编译 SONIC 策略程序和 G1 桥接，下载并核对 NVIDIA 模型，然后生成本机 TensorRT FP32 引擎，完成数值校验和缓存核对。校验通过后才安装服务并启动空闲 API 和网页。

默认网页仅监听回环。电脑上可执行 `ssh -N -L 8769:127.0.0.1:8769 DEPLOY_USER@G1_HOST`，访问 [电脑界面](http://127.0.0.1:8769/) 或 [移动界面](http://127.0.0.1:8769/remote)。需要手机直接访问时，首次安装给预检和安装命令都加 `--console-access lan`，随后在同一可信局域网打开 `http://G1_HOST:8769/remote`，把 `G1_HOST` 换成机器人实际管理地址。

### 可选功能

在首次预检和安装时使用同一组参数。以下命令选用回放与 Pico。

```bash
python3 scripts/setup_g1_online.py --check --features playback,pico
python3 scripts/setup_g1_online.py --features playback,pico
```

- **回放**将自行取得的原始动作放到 `var/gmr/`、`var/smpl/` 或 `var/kimodo/`，在网页按格式转换。动作库使用 `var/motions/`。从可信来源获取动作并核对格式、帧率和参考姿态。
- **Pico**安装器从 XRoboToolkit 官方发布下载经校验的 ARM64 服务和绑定，构建接收容器。按头显发送端要求将 TCP 目标指向 G1 的管理地址和端口 `63901`。需要外置追踪器时，在本机 `onboardmimic/inputs/pico/devices.json` 配置角色。
- **PN Link**把下方示例另存为本机文件并替换两个地址，使用 `--features pnlink --pn-config /path/to/receiver.json`。SDK 默认从 PN 固定版本获取，已有原始厂商文件可用 `--pn-sdk-source /path/to/sdk`。给 G1 的所选接口配置接收 IPv4，并把动捕盒的 UDP 目标设为该地址和 `local_port`。默认接收服务不开机启动；通过网页进入 PN Link 后手动启动接收。网络切换可先用 `python3 scripts/switch_pnlink_network.py status --ssid YOUR_SSID` 查看。
- **BrainCo**使用 `--features pico,brainco` 或 `--features pnlink,brainco`。保留厂商手部 DDS 驱动。默认下载固定 Unitree SDK2，也可选择 `--hand-sdk-root /path/to/compatible/sdk2`。适配器在安装后保持停止，由现场操作者按手部验收流程确认驱动与反馈后启用。

```json
{
  "local_ip": "192.0.2.10",
  "server_ip": "192.0.2.20",
  "local_port": 8002,
  "server_port": 8080,
  "auto_capture": false
}
```

### 日常维护与排查

- 查看空闲服务和日志，使用 `systemctl --user status onboardmimic onboardmimic-console` 及 `journalctl --user -u onboardmimic -u onboardmimic-console -n 80`。首次安装后网页应显示停止，控制器 PID 列表为空。控制、准备姿态和现场动作执行遵循[操作说明](docs/CONTROL_EXTENSIONS_zh.md)。
- 依赖下载或构建中断后，按终端报告修复对应阶段，再用相同参数重跑。引擎转换和数值检查日志位于 `onboardmimic/backends/sonic/deploy/onboard/logs/`。出现 ROS APT 签名错误时，检查时钟和配置的 ROS 源。发现已有自定义 systemd 服务时，先审查迁移方式，参见[升级说明](docs/UPGRADE_zh.md)。
- Git 跟踪源码、测试和文档。模型、引擎、厂商二进制、离线包、动作、标定、密钥和本机配置留在各部署环境。[上游 v0.1.0 预编译包](https://github.com/LambdaHumanoid/OnboardMimic/releases/tag/v0.1.0)对应上游旧版功能。本仓库以联网源码构建为主。

## 验收范围

- **现有 G1**已有现场功能验证。首发所用的主机源码基线与其 581 个受管文件逐文件比对 SHA-256 一致。
- **公开版本**包含脱敏配置、联网安装入口及文档。用干净 clone 进行隔离回归，检查安装参数、功能选择、依赖失败重试和空闲服务配置。
- **待实机复验**是本公开提交在备用同型号 G1 上的首次联网安装、TensorRT 现场构建、重启及空闲状态检查。具体边界见[验证范围](docs/VALIDATION.md)。
