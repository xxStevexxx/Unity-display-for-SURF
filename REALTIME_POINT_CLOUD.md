# RealSense 实时三维点云

## 这一版实现了什么

此分支已包含 `tools/realsense_scene_publisher.py`。下载仓库后，在仓库根目录运行 `python3 tools/realsense_scene_publisher.py --sample-step 3 --max-points 12000 --publish-hz 5`。下方 `/mnt/d/Documents/SURF` 命令用于作者电脑现有的工作区副本，其他电脑请按实际克隆路径替换。

独立订阅 ROS `sensor_msgs/PointCloud2`，以真实颜色、米制比例显示相机当前看到的三维表面。原视角与夹爪视角都可观察，原有 V 键切换不变。最新帧替换上一帧，不积累拖影；断流超过两秒清空显示。

**这不是完整实体建模**：目前没有多帧融合、背面补全、网格重建、物体识别或碰撞体生成。不会改动现有方块、机械臂、抓取、温度与按键逻辑，也不会向真实机械臂发送命令。真实点云不自动带有温度信息。

## Unity 端

1. 停止 Play，重新加载 `Assets/Scenes/SampleScene.unity`，找到新增的 `RealSense Live Point Cloud`。其他场景可通过菜单 `Piper > RealSense > Add Live Point Cloud` 添加。
2. 当前默认 Topic 为 `/realsense/obstacle_points`、Expected Frame Id 为 `base_link`、Coordinates 为 `Ros Flu`，对应工作区已有的 `realsense_scene_publisher.py`。
3. Frame Origin 已指向当前场景的 `BaseLink_Frame`。它只指定 Unity 中的显示基准，不代表真实相机已经完成标定。默认启用 `Use Preview Offset`，仅把点云显示位置平移 `(1.1, 0.85, -0.35)` 米，避免当前未标定数据落到平台下面。Inspector 会显示 `PREVIEW OFFSET (not calibrated)`；这不是物体相对机械臂的真实位置，不能用于控制。完成相机外参和 Frame Origin 标定后，关闭此选项。
4. 通过现有 ROS Connection 设置连接 ROS-TCP-Endpoint 的电脑 IP 与端口（默认 10000）。不要更改现有机械臂话题。当前 Windows 工程已启用 ROS2，无需改动协议设置。
5. Play 后选中点云对象，在 Inspector 底部查看状态与点数。没有消息就不会显示假数据。

## 路线 A：使用已有 Python 发布脚本

### 这台 Windows 电脑的启动步骤

已确认 ROS2 Humble 安装在 WSL 的 `Ubuntu-22.04` 中，桥接工作区为 `/home/steve/ros2_ws`。普通 Windows PowerShell 的 `(base)` 环境不提供 `ros2` 命令，不需要因此重装 ROS2。

先在 PowerShell 进入 Ubuntu：

```powershell
wsl -d Ubuntu-22.04
```

进入 Ubuntu 后，在同一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/steve/ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

这个终端需保持运行。看到监听成功后，桥接才已启动；不要重复启动占用同一端口的第二个桥接进程。`0.0.0.0` 是服务端监听地址，不是 Unity 要填写的目标 IP。

相机发布程序要在第二个 Ubuntu 终端中运行，同样先加载上述两个 `setup.bash`，再执行：

```bash
cd /mnt/d/Documents/SURF
python3 realsense_scene_publisher.py --sample-step 3 --max-points 12000 --publish-hz 5
```

进入 Ubuntu 不代表 USB 相机已转交给 WSL。若出现 `RuntimeError: No device connected`，先保持 Ubuntu 窗口打开，在 Windows PowerShell 检查：

```powershell
usbipd list
```

找到 RealSense 对应的 BUSID。2026-09-17 本机 D435 的 BUSID 为 `1-20`，但换 USB 口后可能变化。若状态为 `Shared`，运行（替换成当前实际 BUSID）：

```powershell
usbipd attach --wsl --busid 1-20
```

若状态为 `Not shared`，需先在管理员 PowerShell 执行 `usbipd bind --busid 1-20`，再 attach。若已是 `Attached`，不用重复挂接。挂接时 Windows 程序不能同时使用该相机；需要还给 Windows 时运行 `usbipd detach --busid 1-20`。拔插相机或重启 WSL 后，可能需要重新 attach。

本机已实际验证 WSL 中 `pyrealsense2` 可识别 D435，并读取 424x240、15 FPS 配置下的深度和彩色帧；这不代表 ROS 到 Unity 的完整链路已完成验证。桥接启动成功也不代表已经收到相机点云。

参考：[Microsoft WSL USB 连接说明](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)。

### 其他已配置 ROS2 的电脑

在安装好 ROS2、`numpy`、`pyrealsense2` 且能访问相机的环境运行。先 source 你的 ROS2 和 ROS-TCP-Endpoint 工作区，然后启动桥接：

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

在另一个已经 source ROS2 的终端，进入包含 `realsense_scene_publisher.py` 的目录：

```bash
python3 realsense_scene_publisher.py --sample-step 3 --max-points 12000 --publish-hz 5
```

这是一组起步参数，不保证所有硬件都达到 5 Hz。不要同时让这个脚本和官方驱动占用同一台相机。不要加 `--publish-target`，这样该脚本只发点云，不写入现有目标坐标话题。

脚本原本会按 min/max depth 与 XYZ 范围裁剪数据；物体缺失时先检查这些参数。默认相机外参为零，只适合验证链路，不能直接用于真实机械臂定位。必须按实际安装标定 `--camera-x/y/z` 和旋转参数。

## 路线 B：使用官方 realsense-ros

ROS2 环境中启动 Endpoint，并单独运行官方相机节点：

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
ros2 topic list
ros2 topic echo /camera/camera/depth/color/points --field header --once
```

然后在 Unity 中停止 Play 再配置：

- Topic 改为实际点云话题，默认命名通常为 `/camera/camera/depth/color/points`。
- Expected Frame Id 必须填写上面 header 输出的 `frame_id`，不要根据话题名称猜测。
- 若消息在相机光学坐标系（X 右、Y 下、Z 前），Coordinates 选 `Optical`。
- 建立一个独立空物体作为 Frame Origin，其位置和朝向代表真实相机光学原点：Unity 本地 X 右、Y 上、Z 前。没有标定时只能做相机相对的独立预览，不能声称与机器人对齐。
- 当前模块不会自动读取 TF。若相机随夹爪运动，应在 ROS 端使用采集时间戳对应的 TF 把点云变换到固定 `base_link` 后发布，再选 `Ros Flu`，而不是直接跟随当前夹爪姿态。

官方高分辨率点云的数据量远大于路线 A。Unity 的 Max Points 只减少绘制点数，不减少网络带宽；卡顿时应在 ROS 端降频或降采样。

## 排查与限制

- `Waiting for ...`：检查 Endpoint、IP、端口、防火墙、ROS1/ROS2 协议、话题名与 ROS2 QoS。用 `ros2 topic hz` 确认发布；用 `ros2 topic info --verbose` 检查发布/订阅的 QoS 是否兼容。
- `Frame mismatch`：检查 header；不要仅修改期望名称来假装数据已完成坐标变换。
- 有点数但看不到：检查 Frame Origin、相机朝向/裁剪范围、相机 Culling Mask 与深度遮挡。点云使用正常深度测试，原场景平台可能挡住错误定位到平台下面的点云。2026-09-17 实测原始显示高度为约 -0.61 至 -0.17 米，整片低于平台表面 0 米；预览偏移用于显示排查，不应通过修改平台或关闭深度测试掩盖标定问题。第一帧有效点云会在 Console 输出点数及显示中心。
- 支持含行填充的 organized/unorganized 点云、大小端 FLOAT32 XYZ、打包 FLOAT32/UINT32 rgb/rgba；无颜色时显示灰色。忽略 NaN/Infinity；其他坐标字段类型会拒绝并提示。
- 默认上限 30000 点，10 Hz 更新，直径 8 mm；可在 Inspector 调节。显示点大小不是测量物体的实际厚度。
- 当前不会生成 Collider。要让真实场景参与碰撞，需要后续表面重建、去除机器人自身点云、动态物体分离及低频碰撞网格更新。
- 海底应用另需验证防水耐压、光学窗口折射、水体浑浊与量程；桌面 RealSense 演示不等于可直接水下使用。

## 参考

- [RealSense ROS 官方说明](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)
- [Unity ROS-TCP-Endpoint ROS2](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2)
- [ROS PointCloud2 消息定义](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)
