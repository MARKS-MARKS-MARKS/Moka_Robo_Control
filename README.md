# Moka Robot Control

Moka Robot Control 是一个面向 MR07S 六轴机械臂的实验控制项目。项目由三部分组成：

- C++ EtherCAT 实时控制程序：负责伺服上电、关节运动、TCP 直线运动、书写轨迹执行和夹爪 IO 输出。
- FastAPI 网页服务：负责提供 Web 控制台、WebSocket 指令转发、机器人状态转发、汉字笔画数据接口和相机视频流接口。
- Web 前端控制台：提供 3D 模型显示、关节控制、TCP 控制、TCP 示教再现、汉字书写、井字棋对弈、末端相机预览和夹爪控制。

仓库地址：

```text
https://github.com/MARKS-MARKS-MARKS/Moka_Robo_Control
```

## 目录结构

```text
.
├── README.md                    # 项目说明文档
├── moka.txt                     # 本机常用启动命令备忘
├── TCP_使用备忘录.md             # TCP 控制姿态和操作经验记录
├── wangye.cpp                   # C++ 主控制程序，EtherCAT、TCP 命令服务、状态服务
├── axis.cpp / axis.h            # 单轴对象和伺服相关封装
├── general_6s.cpp / general_6s.h# 六轴运动学、关节插补、直线插补
├── struct_define.h              # 机器人参数结构体定义
├── ecrt.h                       # IgH EtherCAT 用户态接口头文件
├── libethercat.a                # EtherCAT 静态库
├── eigen/                       # Eigen 数学库
├── app/
│   ├── app.py                   # FastAPI 服务入口，监听 9002
│   ├── robot_bridge.py          # Python 到 C++ 9000/9001 的 TCP 桥接
│   └── static/
│       ├── index.html           # Web 控制台
│       ├── MR07S.urdf           # 三维模型 URDF
│       ├── mr07s/meshes/        # 机械臂 STL 模型资源
│       ├── hiwin/meshs/         # 夹爪 STL 模型资源
│       └── vendor/              # 前端本地依赖资源
└── moka wangye/
    ├── data/                    # 汉字笔画 JSON 字库，供 /hanzi/{char} 查询
    └── ...                      # 历史/参考版本代码与资料
```

运行时会生成或使用以下文件，不建议提交到 GitHub：

```text
test.out
teach_path.txt
*.bak_codex
__pycache__/
*.pyc
```

## 硬件与端口

### EtherCAT 从站

当前 C++ 配置要求 EtherCAT 总线上有 7 个从站：

```text
0-5: Panasonic 伺服轴
6:   IO 模块
```

关键配置位于 `wangye.cpp`：

```cpp
#define SLAVE_NUM 7
#define VID_PID   0x00000922,0x00000a01
#define VID_PID2  0x00000c6d,0x00000001
#define GRIPPER_OPEN_IO 15
#define GRIPPER_CLOSE_IO 16
```

夹爪控制使用 IO15 和 IO16。当前 Web 端按钮已经按现场实际效果做了对调，用户点击“夹爪松开/夹爪夹紧”时会发送对应动作。

### 网络端口

```text
9000: C++ 指令接收端口，Python bridge 向这里发送 MOVEJ/MOVEL/WRITE/GRIPPER 等命令
9001: C++ 状态推送端口，Python bridge 从这里读取 6 轴关节角和 TCP 位姿
9002: FastAPI/Web 页面端口
```

## 环境依赖

### 系统依赖

- Linux
- IgH EtherCAT Master 1.6.0
- g++
- Python 3
- FastAPI / Uvicorn

### Python 依赖

基础网页控制：

```bash
python3 -m pip install fastapi uvicorn websockets
```

相机预览功能需要额外安装：

```bash
python3 -m pip install opencv-python depthai
```

如果不使用相机，缺少 `opencv-python` 或 `depthai` 不会影响关节、TCP、书写和夹爪控制；页面中的相机检测会提示不可用。

## 启动流程

### 1. 启动 EtherCAT Master

```bash
sudo /etc/init.d/ethercat start
sudo ethercat slaves
```

正常情况下应能看到 7 个从站。如果 `sudo ethercat slaves` 没有输出，优先检查：

- 机器人/驱动器是否上电
- EtherCAT 网线是否接到正确网口
- `/etc/ethercat.conf` 的 `MASTER0_DEVICE` 是否对应实际网卡
- 网口 `ip link` 是否处于 `LOWER_UP`

### 2. 编译并运行 C++ 底层

在项目根目录执行：

```bash
cd ~/moka
sudo g++ -o test.out wangye.cpp axis.cpp axis.h general_6s.cpp general_6s.h struct_define.h libethercat.a -lrt -lpthread -std=c++11
sudo ./test.out
```

正常启动后会看到类似输出：

```text
[底座] 机器人参数初始化完成！准备上电...
[网络] 指令接收端口 9000 启动监听...
[网络] 状态推送端口 9001 启动监听...
```

### 3. 启动 Web 服务

另开一个终端：

```bash
cd ~/moka/app
python3 app.py
```

浏览器打开：

```text
http://0.0.0.0:9002
```

如果是在同一台机器上访问，也可以用：

```text
http://127.0.0.1:9002
```

## Web 控制台功能

### 连接设备

打开页面后先点击“连接设备”。连接成功后页面会通过 WebSocket 接收机器人状态。

状态数据来自：

```text
C++ 9001 -> app/robot_bridge.py -> FastAPI WebSocket -> index.html
```

### 关节控制

关节控制发送命令：

```text
MOVEJ j1 j2 j3 j4 j5 j6
```

使用建议：

1. 先点击“回填当前关节角”
2. 小角度测试单轴运动
3. 确认实物方向和数值正常后再做较大运动

### TCP 空间控制

TCP 控制发送命令：

```text
MOVEL x y z rx ry rz
```

建议使用流程：

1. 点击“回填当前 TCP 位姿”
2. 只小幅修改 `X/Y/Z`
3. 姿态 `RX/RY/RZ` 不要随意改动
4. 点击“执行空间指令”

现场经验详见 `TCP_使用备忘录.md`。

### TCP 示教再现

Web 页面提供“TCP 示教再现”面板。该功能使用当前收到的 TCP 状态作为示教点，并同时记录界面中的夹爪开闭状态。

记录内容：

```text
P点 = [X, Y, Z, RX, RY, RZ] + 夹爪状态 + 备用时间(ms)
```

使用流程：

1. 点击“连接设备”，确认页面持续收到 TCP 状态
2. 使用“TCP 空间控制”或其他 TCP 示教方式移动机器人
3. 根据当前抓取步骤点击“夹爪松开”或“夹爪夹紧”
4. 点击“记录当前作业点”
5. 重复移动、夹爪动作、记录点位
6. 根据需要保留或调整“备用ms”
7. 点击“串联再现”

状态判停说明：

- 串联再现不再依赖人工设置的等待时间推进
- 再现到某个点时，先发送该点 `MOVEL`
- 前端每 2 秒读取一次关节状态
- 如果连续两次检测之间 6 个轴的最大变化不超过 `0.05deg`，认为机器人已停止
- 判定停止后，再执行该点保存的夹爪动作，然后进入下一个点

备用时间说明：

- 单位是毫秒，`2500` 表示 2.5 秒
- 当前允许范围是 `300-60000ms`，最长 60 秒
- 新记录的点会使用面板当前的“备用ms”作为保存值
- 该字段保留在 `teach_path.txt` 中，便于以后需要恢复固定等待或调试时使用

编辑与保存：

- 读取示教链后，可以继续在后面追加新的示教点
- 已记录的点可以直接编辑 `X/Y/Z/RX/RY/RZ`
- 已记录的点可以点击夹爪状态按钮切换“夹紧/松开”
- 页面会保存到浏览器本地存储，同时通过接口写入项目根目录：

```text
teach_path.txt
```

对应接口：

```text
GET  /teach-path
POST /teach-path
```

再现时前端会按顺序发送：

```text
MOVEL x y z rx ry rz
每 2 秒监测关节状态，判停后继续
GRIPPER OPEN/CLOSE
```

注意事项：

- 示教链会保存到当前浏览器的本地存储和项目根目录 `teach_path.txt`，刷新页面或重新打开页面会自动读取
- “保存示教链”会手动覆盖保存内容；记录、删除、清空、修改坐标、修改夹爪状态和修改备用时间也会自动保存
- 更换电脑、浏览器或清理浏览器数据后，本地保存的示教链不会自动跟随
- 状态判停依赖 9001 状态链路正常；如果页面状态离线，串联再现会停止
- 夹爪状态按当前 Web 端现场调换后的按钮语义记录，即页面显示“夹紧”就按夹紧再现

### 汉字书写

书写功能使用 `moka wangye/data/` 下的汉字 JSON 字库。FastAPI 提供接口：

```text
GET /hanzi/{char}
```

前端会读取汉字 `medians`，转换成机械臂书写航点，然后通过 WebSocket 发送：

```text
WRITE count lift x y z rx ry rz ...
```

书写参数说明：

```text
文字:   要写的汉字，建议一次 1-3 个
中心X:  书写区域中心 X
中心Y:  书写区域中心 Y
纸面Z:  笔尖接触纸面的 Z
抬笔:   笔画之间抬起高度
宽/高:  字体缩放尺寸
间距:   轨迹采样间距，数值越小点越密
RX/RY/RZ: 书写姿态
```

使用建议：

1. 先让机器人移动到纸面附近的安全高度
2. 点击“用当前 TCP 设置书写中心”
3. 点击“预览笔画”
4. 确认首段距离和纸面 Z 安全
5. 点击“开始书写”

底层限制：

```text
MAX_WRITE_POINTS = 1024
MAX_TRAJECTORY_QUEUE_VALUES = 2000000
```

如果出现“轨迹点超过上限”或“轨迹队列过长”，可以：

- 增大“间距”
- 减小“宽/高”
- 减少一次书写的汉字数量

### 井字棋对弈

Web 页面提供“井字棋对弈”面板。该功能复用现有 `write_path` 协议，不新增底层命令。

使用流程：

1. 点击“连接设备”
2. 设置井字棋纸面中心、纸面 Z、抬笔高度、棋盘尺寸和姿态
3. 可以点击“同步当前 TCP”快速回填当前 TCP 到纸面参数
4. 点击“绘制棋盘”，机器人会在纸面画 3x3 棋盘
5. 人类在网页棋盘点击一个格子，确认后机器人在该格写 `X`
6. 点击“机器人落子”，AI 会选择最优格子，确认后机器人在该格写 `O`
7. 重复直到分出胜负或平局

注意事项：

- 棋盘和棋子都通过 `WRITE count lift ...` 发送到底层
- 每次写棋盘、写 X、写 O 前都会弹出确认
- 页面中的人类落子也会让机器人写 `X`，用于保持纸面棋局和网页棋局一致
- 若轨迹点超过 1024，请增大“采样”或减小棋盘尺寸

### 夹爪控制

Web 页面提供：

```text
夹爪松开
夹爪夹紧
```

命令链路：

```text
index.html -> WebSocket -> app.py -> robot_bridge.py -> C++ 9000 -> IO15/IO16
```

当前 IO 逻辑：

```text
IO15 / IO16 用于夹爪开合
```

如果按钮动作和实物相反，优先在 Web 端对调按钮调用；当前版本已经按现场效果调换过。

### 末端相机

相机接口：

```text
GET /camera/status
GET /camera/stream
```

当前实现使用 DepthAI + OpenCV，输出 MJPEG 流。若检测失败：

- 检查相机 USB 连接
- 检查当前用户 USB 权限
- 检查是否安装 `depthai` 和 `opencv-python`
- 检查相机是否被其他程序占用

## C++ 指令列表

C++ 底层在 9000 端口接收纯文本命令。

```text
MOVEJ j1 j2 j3 j4 j5 j6
MOVEL x y z rx ry rz
WRITE count lift x y z rx ry rz ...
GRIPPER CLOSE
GRIPPER OPEN
IOSET channel value
IOCLEAR
ESTOP
```

状态端口 9001 输出 12 个浮点数：

```text
j1 j2 j3 j4 j5 j6 x y z rx ry rz
```

## 常见问题

### `sudo ethercat slaves` 没有输出

说明 master 没有扫描到从站。检查：

```bash
cat /etc/ethercat.conf | grep MASTER0_DEVICE
ip link
sudo /etc/init.d/ethercat restart
sudo ethercat slaves
```

如果配置里写了不存在的网卡，例如 `enp6s0`，需要改成实际连接 EtherCAT 的网口。

### `Failed to reserve master: No such device`

通常表示 EtherCAT master 没有正确绑定到设备，或 master 0 不存在。先确认：

```bash
sudo /etc/init.d/ethercat start
sudo ethercat master
sudo ethercat slaves
```

### 网页连接后状态不刷新

检查三个端口：

```bash
ss -ltnp | grep -E ':9000|:9001|:9002'
```

应至少看到：

```text
9000 C++ 指令端口
9001 C++ 状态端口
9002 Python Web 服务
```

也可以检查进程：

```bash
ps -ef | grep -E 'test.out|app.py' | grep -v grep
```

### 端口被占用

如果 `python3 app.py` 提示 `address already in use`：

```bash
ps -ef | grep app.py | grep -v grep
kill <pid>
```

如果 C++ 端口被占用：

```bash
sudo pkill test.out
```

### 书写不动

看 `sudo ./test.out` 终端输出：

```text
WRITE 接收点数=...
WRITE 已接受...
```

如果没有 `WRITE`，说明网页没有发到 9000。

如果出现拒绝信息，按提示处理：

- 点数超限：增大采样间距或减少汉字
- 队列过长：增大采样间距、减小字宽/字高
- 单段关节变化过大：先把机器人移动到更接近书写起点的位置
- 航点超出安全范围：检查 X/Y/Z 和纸面 Z

### 夹爪不动

确认：

1. EtherCAT 总线有第 7 个 IO 从站
2. `sudo ./test.out` 终端出现 `夹爪...指令已写入 IO`
3. IO15/IO16 接线和气源正常
4. 按钮动作是否与实物方向一致

## 上传 GitHub

当前 `~/moka` 目录本身不是标准 Git 仓库。推荐用临时克隆目录上传：

```bash
cd /tmp
rm -rf Moka_Robo_Control_upload
git clone https://github.com/MARKS-MARKS-MARKS/Moka_Robo_Control.git Moka_Robo_Control_upload
cd Moka_Robo_Control_upload
git rm -r .
rsync -a /home/seuauto/moka/ ./ \
  --exclude='.git' \
  --exclude='.agents' \
  --exclude='test.out' \
  --exclude='*.bak_codex' \
  --exclude='__pycache__/' \
  --exclude='*.pyc'
git add -A
git commit -m "Replace with current Moka robot control project"
git push origin main
```

不要把 GitHub token、密码、私钥写入任何项目文件。若误提交 token，必须删除文件中的 token，并用 `git commit --amend` 或历史清理方式移除后再推送。

## 安全提示

- 运行 `WRITE` 或 `MOVEL` 前确认机器人周围无人。
- 首次测试新轨迹时，降低速度、增大抬笔高度。
- TCP 姿态不要随意改动，避免腕部翻转。
- 急停后先确认伺服状态，再继续发送运动命令。
- 换机器人或换网口后，先确认 EtherCAT 从站列表和顺序。
