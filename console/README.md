# G1-D 具身智能控制台

Unitree G1-D 双臂机器人的网页控制台：**示教航点 + 视觉标定 + LLM 任务执行**。
不用 ROS，直接用 `unitree_sdk2py` 走 DDS。

```
浏览器 ──HTTP/MJPEG──► console.py
                          │
                          ├─ console_core.py  100 Hz 控制环 + 安全层
                          ├─ collision.py     Pinocchio 自碰撞
                          ├─ calib_service.py ChArUco 标定 + 深度
                          └─ unitree_sdk2py ──► rt/lowcmd / rt/lowstate
```

## 功能

| 模块 | 内容 |
|---|---|
| 实时控制 | 100 Hz 循环，重力前馈（RNEA），梯形速度曲线 |
| 安全层 | 自碰撞检测、关节软限位、防饱和、堵转/振荡/力矩/温度/电压保护、急停 |
| 示教 | 航点记录（关节角 + 夹爪状态），轨迹串接回放 |
| 视觉 | 双目/腕部内参、双目外参、手眼标定（Park-Martin） |
| 深度 | 用标定板角点测双目深度精度，与 PnP 交叉验证 |
| LLM | gpt-6 调用工具：`goto_waypoints` / `jog_multi` / `gripper`；到位即判成功，不回问模型 |

## 目录

```
console/
├── console.py          Web 服务 + 页面
├── console_core.py     控制环、安全、标定接口、LLM agent
├── collision.py        自碰撞（Pinocchio + coal）
├── feedback.py         电机反馈（备用，默认复用后端订阅）
├── logger.py           20 Hz 数据记录 + 异常检测
├── calib_service.py    标定/深度的服务层
├── calib_page.py       标定页面
├── calib/              ChArUco、内参、双目、手眼、深度
├── models/             URDF（轮子改成 fixed，Pinocchio 才能用）
└── docs/               相机配置记录
```

## 依赖

- Python 3.10 + numpy 2.x + Pinocchio 4.1 + OpenCV 4.12（含 contrib/aruco）
- **unitree_robotskills** 的 `unitree_gpt6/g1d_draw`（提供 `G1DDSBackend` 与 .venv）

```bash
git clone https://github.com/unitreerobotics/unitree_robotskills /path/to/unitree_gpt6
# 目录布局：
#   /path/to/unitree_gpt6/g1d_draw/.venv/bin/python
#   console/run_console.sh   ← 与本仓库同级即可
```

若放在别处，用环境变量指定：

```bash
G1D_STACK=/path/to/unitree_gpt6 bash run_console.sh
```

## 运行

```bash
export ASTRA_API_KEY=...        # LLM 任务框需要
bash run_console.sh             # 真机
bash run_console.sh --sim       # 无硬件预览（只读，不发指令）
```

默认 <http://127.0.0.1:8090>，标定页 `/calib`。

## 接管流程（重要）

程序启动后**只读**，不发任何指令。点「接管手臂」才会：

1. 切到 debug 模式并**确认成功**（失败就放弃，不发指令）
2. 用当前姿态 acquire
3. 立刻开始以重力补偿保持

**顺序不能反**：先发指令再切模式会和机器人自带控制器抢电机（实测肩 roll 力矩震荡 7 N·m、关节咔咔响）。

## 标定

标定页 `/calib`，ChArUco 板（12×9，格 15 mm，标记 11.25 mm，DICT_5X5_250）：

- **内参**：四个相机（头部左右目、左右腕），1920×1080
- **双目**：同步图对 → 内参 + 外参（当前 RMS 0.48 px，基线 60.3 mm）
- **手眼**：eye-to-hand，板子装在夹爪上，Park-Martin 解 AX=XB（残差 1.65 mm / 0.28°）
- **深度测试**：用角点视差算深度，与 PnP 交叉验证

结果写入 `~/g1d_vision_data/`。

## 安全说明

- 软件急停 ≠ 硬件急停，**硬件急停优先**
- 自碰撞检测**不包含环境**（桌子、人）
- 关节力矩上限取自 URDF 的 `effort` 字段，超过 80% 立即停机
- 心跳看门狗默认关闭，可在界面开启
- 只在 debug 模式下有效；退出会把控制权交回机器人（`SelectMode("ai")`）

## 已知限制

- 单机械臂实例，多机型需要适配器抽象
- 视频走 MJPEG，并发上限 3 路
- 无自动避障、无力控闭环
