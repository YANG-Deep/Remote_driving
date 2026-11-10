# CARLA 远程驾驶辅助演示

European HGV 的低速环视辅助：四路 RGB 相机、四个 CARLA 雷达、车身轨迹与周边净空提示。360° 环视由前、后、左、右四摄做地面投影拼接，不使用额外的车顶俯拍摄像头；中央货车为程序绘制的示意图。演示面向局部平坦路面，当前轨迹精度验证条件约为 5 km/h。

距离只来自 `sensor.other.radar`。`--validate` 使用独立场景射线做诊断，不补填雷达读数。数值表示**已检测点到车身包围盒投影矩形的最短水平距离**（本车型使用保守碰撞包络，车尾比名义包围盒额外包含 0.55 米），不是完整环境的保证净空。

四摄拼接的算法与代码调用链见：[四摄 360° 环视代码逻辑 README](docs/360-surround-view/README.md)。

## GitHub 仓库内容

仓库保留源代码、测试、依赖说明、标定参数与 README，以及小型的[在线测试报告](artifacts/live-final/report.md)和[结果摘要](artifacts/live-final/summary.json)。图片、视频、参考素材、原始运行数据、源码压缩备份和本地缓存由 `.gitignore` 排除，本机文件不会被删除。

文档中提到的 `ref_pic/`、`_debug_out/`、截图和遥测路径主要是本地输入/运行产物，不随仓库上传；可按启动或测试命令重新生成输出。文中的 `/home/yang/...` 是本机示例路径，在其他电脑上运行时请替换为实际安装路径。

## 启动流程

正常演示使用两个终端：**终端 1 运行 CARLA 服务端，终端 2 运行本项目的 HUD 程序**。如果服务端已经运行且能够连接，直接跳到第 3 步，不要重复启动。

### 1. 检查环境与目录

本机已配置以下路径和环境：

| 项目 | 本机配置 |
|---|---|
| 项目代码 | `/home/yang/AD_Repo/Remote_driving` |
| CARLA 安装目录 | `/home/yang/AD_Repo/Carla` |
| Conda 环境 | `carla` |
| Python | 3.11 |
| CARLA Python API 包 | 0.9.16 |
| 已验证的客户端/服务端构建 | `3b938d7` |
| 默认连接 | `127.0.0.1:2000` |
| 默认地图参数 | `Town10HD`，本机运行地图为 `Town10HD_Opt` |

在终端中检查 GPU、Conda 环境和 Python 依赖：

```bash
nvidia-smi
conda env list
conda run --no-capture-output -n carla python -c "import carla, cv2, numpy, pygame; print('Python dependencies OK')"
```

如果终端提示找不到 `conda`，先加载本机 Conda：

```bash
source /home/yang/anaconda3/etc/profile.d/conda.sh
```

本机已有可用环境，不需要每次启动都安装依赖。重新部署时参考 `requirements.txt`，并确保 CARLA Python API 与服务端构建匹配。

### 2. 启动 CARLA 服务端（终端 1）

```bash
cd /home/yang/AD_Repo/Carla
./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Low -carla-rpc-port=2000
```

参数含义：

- `-RenderOffScreen`：不打开 CARLA 的 3D 窗口，仍正常渲染车载相机。
- `-nosound`：关闭服务端声音。
- `-quality-level=Low`：使用低画质以降低渲染负载。
- `-carla-rpc-port=2000`：本机 RPC 端口。

**保持终端 1 和服务端进程运行。** 服务端启动需要一些时间；即使没有显示 3D 窗口，也不能据此判断启动失败。`-RenderOffScreen` 仍需要可用的 GPU/Vulkan 环境。

如需观察 CARLA 的 3D 窗口，可以省略 `-RenderOffScreen`。这不改变本项目四摄拼接的环视来源。

### 3. 确认服务端可连接（终端 2）

```bash
cd /home/yang/AD_Repo/Remote_driving
conda run --no-capture-output -n carla python -c "import carla; c=carla.Client('127.0.0.1',2000); c.set_timeout(5.0); print('Client:', c.get_client_version()); print('Server:', c.get_server_version()); print('Map:', c.get_world().get_map().name)"
```

本机预期能打印客户端/服务端构建 `3b938d7` 和地图名称。若报连接超时，先排查服务端，不要反复启动 HUD。

如需确认端口是否已被进程监听：

```bash
ss -ltnp 'sport = :2000'
```

### 4. 启动远程驾驶辅助界面（终端 2）

```bash
cd /home/yang/AD_Repo/Remote_driving
conda run --no-capture-output -n carla python main.py --save-frame --output artifacts/manual-demo
```

启动成功后会打开 pygame 窗口，控制台打印连接、地图、车辆生成和几何信息。程序自动完成：

1. 连接 CARLA，按需加载地图，启用 20 Hz 同步仿真。
2. 生成 European HGV，并等待悬架稳定。
3. 挂载前、后、左、右四路相机及四个雷达。
4. 等待四摄同帧图像，开始环视拼接、轨迹绘制和雷达测距显示。

界面布局为左侧 360° 环视、右上前/后主视图、右下左右视图。启动时为 D 挡，倒车切换到后视图。

默认窗口为 1920×1080。**先点击 HUD 窗口使其获得键盘焦点，再操作车辆。** 普通演示建议先保持约 5 km/h 以内；更高速度会提示超出已验证的轨迹速度范围。

### 5. 操作车辆

| 操作 | 按键 |
|---|---|
| 油门 | W / ↑ |
| 刹车 | S / ↓ / 空格 |
| 左右转向 | A、D / ←、→ |
| 前进、倒车切换 | R，基本停车后操作（速度不超过 0.8 km/h） |
| 退出 HUD | Esc，或关闭 HUD 窗口 |

松开方向键会回正。松开油门不等于立即停车，需要使用刹车。R 仅切换挡位，倒挡下仍使用 W / ↑ 给油。正转角表示右转。

### 6. 查看输出与正常退出

示例命令将结果写入 `artifacts/manual-demo/`：

| 文件 | 内容 |
|---|---|
| `setup.json` | 车辆几何、相机/雷达安装位姿、构建版本、解析覆盖信息 |
| `telemetry.jsonl` | 逐帧状态、相机帧号与年龄、雷达量测、处理耗时 |
| `cam_front.png`、`cam_rear.png`、`cam_left.png`、`cam_right.png` | 四路原始相机首组图像 |
| `bev.png` | 拼接后的环视图及叠加信息 |
| `hud.png` | 完整界面截图 |

PNG 截图需要 `--save-frame`；不加该参数仍会记录配置和遥测。同一输出目录重复运行会覆盖同名文件，保留旧结果时请换一个 `--output` 目录。

先在 HUD 中按 Esc 退出。程序会清理本次车辆与传感器，恢复世界设置和天气，**不会自动关闭 CARLA 服务端**。如需停止服务端，再到终端 1 按 Ctrl+C。地图切换本身不回滚。

## 其他运行方式

以下命令均在 `/home/yang/AD_Repo/Remote_driving` 执行。

### 不保存截图的日常演示

```bash
conda run --no-capture-output -n carla python main.py
```

### 开启独立射线诊断，并在 300 帧后自动退出

```bash
conda run --no-capture-output -n carla python main.py --validate --max-frames 300 --save-frame --output artifacts/diagnostic-demo
```

`--validate` 的参考距离只进入诊断记录，不填补界面雷达读数；它会增加耗时。`--max-frames` 按仿真循环计数，不能保证固定的实际运行秒数。

### 无桌面窗口的有限帧测试

```bash
SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy conda run --no-capture-output -n carla python main.py --max-frames 200 --save-frame --output artifacts/headless-demo
```

该方式不显示 HUD，也不适合手动键盘驾驶；用于自动运行和导出截图。CARLA 服务端仍需启动并具备正常渲染条件。

### 指定主机、端口或地图

```bash
conda run --no-capture-output -n carla python main.py --host 127.0.0.1 --port 2000 --map Town10HD
```

服务端修改 RPC 端口时，客户端 `--port` 也必须一致。连接远程主机时，两端需要允许对应的 CARLA 通信，且客户端与服务端构建应匹配。

查看全部启动参数：

```bash
conda run --no-capture-output -n carla python main.py --help
```

## 常见启动问题

| 现象 | 检查与处理 |
|---|---|
| `conda: command not found` | 加载前面的 `conda.sh`，再检查 `conda env list`。 |
| `No module named carla/cv2/pygame` | 确认使用 `carla` 环境，不要直接调用其他环境的 `python`。 |
| 连接 `127.0.0.1:2000` 超时 | 检查终端 1 的服务端进程和日志；运行第 3 步探测命令及 `ss`，核对主机和端口。 |
| 端口已占用 | 先确认是不是已有可连接的 CARLA；可复用时无需再次启动。使用其他端口时同步修改客户端参数。 |
| 无 GPU/Vulkan、`nvidia-smi` 失败 | 检查当前执行环境是否可以访问 GPU 设备；无窗口渲染仍依赖 GPU。 |
| 创建套接字报 `Operation not permitted` | 当前执行环境限制本机网络，需在允许 CARLA 网络通信及 GPU 访问的环境中运行。 |
| 启动脚本 `chmod` 报只读文件系统 | 检查 CARLA 安装目录的可写性；已有可执行权限时可直接调用下方的服务端二进制。 |
| 没有 HUD 窗口 | 检查是否使用了 `SDL_VIDEODRIVER=dummy`；手动驾驶应在正常桌面会话启动。 |
| 按键没有反应 | 点击 HUD 窗口获得焦点；确认不在 headless 模式。切换 R 前先刹停。 |
| 四摄首帧超时或画面卡住 | 检查服务端渲染状态，避免其他客户端同时推进/阻塞同步仿真。 |
| 客户端/服务端版本不同 | 核对第 3 步打印的构建标识；版本不匹配时先统一构建，不能把未加载本机标定文件误认为已标定。 |

已有可执行权限但启动脚本无法 `chmod` 时，可在终端 1 使用：

```bash
cd /home/yang/AD_Repo/Carla
./CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping CarlaUE4 -RenderOffScreen -nosound -quality-level=Low -carla-rpc-port=2000
```

请使用专用演示服务端。不要在同一个世界同时运行 `main.py`、在线验收脚本或其他会调用 `world.tick()` 的控制程序；它们会相互影响。测试命令见后面的“输出与验证”和“在线测试与标定”。

## 显示规则

- 数字：有效雷达量测，以 0.1 米显示；≤1 米橙红、1–2 米黄、2–4 米浅色。
- `--`：当前无有效回波，**不能解释为安全**。
- `N/A`：未获得量测或过期到无法显示距离。
- `*`：保留的过期量测；0.2 秒起标记，0.5 秒后取消数值。
- 斜纹：相机地面投影不可见区。车身示意覆盖区域不代表透明底盘图像。
- 影像超过 0.2 秒：暂停图像上的轨迹和障碍位置叠加，底栏仍可显示独立的实时雷达数字。

相机严格按 frame 编号配对；完整原始视场按比例显示，面板可能有少量比例留边。轨迹使用实际车身宽度与等效后轴的低速模型；BEV 阴影表示预测车身扫掠范围，不表示已确认无障碍。

## 输出与验证

`--output` 目录包含：

- `setup.json`：车辆几何、安装位姿、版本和四雷达解析水平视场覆盖。
- `telemetry.jsonl`：仿真帧、相机帧及年龄、雷达来源/状态/命中点、处理耗时；启用校验后包含参考误差和漏检。
- `--save-frame`：首组相机、BEV、HUD 截图。

独立场景验收（会创建并清理测试 actors，包含六种直行/转弯方向、八方向间距及双侧窄通道）：

```bash
conda run --no-capture-output -n carla python tools/validate_demo.py
```

状态 `measured` 仅表示拿到了结果；`criterion_pass` 才是对应数值条件。蓝图缺失、场景冲突或无回波都不能记作通过。街墙/路沿蓝图若不可用会明确记录；地面标记对齐需要查看导出的 `live-images`，不会自动判通过。平行靠近测试通过冻结车辆并逐步平移 2 米完成，只验证测距稳定性，不代表动力学验收。运动测试依赖开放路段，需检查背景碰撞影响。

当前离线 36 项测试通过，编译与 CLI 检查通过；独立审阅的三处运行时缺陷已有回归测试。完整记录见 `artifacts/verification.json`。

离线测试与预览不需要服务端：

```bash
conda run --no-capture-output -n carla python -m unittest discover -s tests -v
conda run --no-capture-output -n carla python tools/preview.py
```

`artifacts/preview/` 是明确标记的合成地面/模拟距离预览，不是 CARLA 实测。旧截图保留在 `artifacts/baseline/`。实施前源码备份在 `artifacts/baseline/source-before.tar.gz`。

## 在线测试与标定

已在本机 RTX A4000、CARLA 构建 `3b938d7` 上启动服务端并完成在线自动测试。最终原始数据与截图见 `artifacts/live-final/`，转角独立验证见 `artifacts/live-session/steering-validated-path.json`。汇总见 `artifacts/live-final/report.md`。

```bash
conda run --no-capture-output -n carla python tools/run_online_checks.py
conda run --no-capture-output -n carla python tools/validate_steering.py
```

测试覆盖八方向三种间距、真实车辆/行人/路锥、双侧窄通道、平行靠近、前进/倒车以及独立转角。大转角验证会逐帧比较车身四角，报告终点与全程两种偏差。测试工具会保留不通过、场景不支持和需要人工看图的状态，不能把整体脚本有输出当作全部通过。

已修正本机构建 `get_matrix()` 与实际 `Transform.transform()` 的旋转方向差异，统一使用前/右/上基向量构造矩阵。相机按 frame 配对；雷达位置标记会换算到相机采集时的车辆姿态，底栏数字保留当前雷达净空。

货车多后轴的转弯有侧向滑移。`calibration/european_hgv-3b938d7.json` 保存了独立训练所得的曲率和侧向滑移参数；只有车型和服务端构建均匹配才启用。训练与验证使用不同转角；短于 3 米的训练片段不会被算作 3 米验收通过。验证条件约为 5 km/h、平坦路面，不能推及全部车速、坡度或载荷；超过 5 km/h 界面会提示轨迹超出已验证速度范围。12 米预测显示范围不等于已验证 12 米误差。

可复现标定流程：先用 `tools/validate_steering.py --controls -.25 -.5 -.75 -1 .25 .5 .75 1 --output <训练文件>` 采集专用训练数据，再用 `tools/fit_motion_calibration.py <训练文件> --server-build 3b938d7 --output <标定文件>` 拟合。正式标定前应暂时移走现有标定文件，或保留它并确保训练仍以原始实际位姿/行程而非预测值拟合；验证数据不能回填为训练数据。更换构建或车型须重新验证。

## 已知限制

- 四个雷达的水平视场经实测调整为 170°，每个 12000 点/秒，仍不能保证无盲区。解析地面覆盖率不等于实际检测率。
- 车辆四角是曲面：包围盒角点间距与实际碰撞表面间距不同。Tesla 四角相对包围盒基准仍有约 0.4 米差距；这一项没有被记作通过，也不能声称雷达保证测到了完整物体的最近表面。
- 路锥等低矮/细小目标会受稀疏采样和高度筛选影响；真实路沿专项验收尚未完成，地图没有可直接生成的 wall/curb 蓝图时会记录 unavailable。箱体平面测试不是道路所有墙体/路沿测试的替代。
- 雷达不模拟完整真实毫米波多径/反射强度。单平面环视会拉伸立体物体，不承诺坡道上的精确贴地。没有自动制动和语音报警。
- 普通界面的 FPS 与开启独立射线诊断时的 FPS 必须分别看待，诊断会额外占用时间。
