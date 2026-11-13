# MineGuard C1

MineGuard C1 is a four-wheel ROS 1 simulation robot with a torque-limited skid-steer drive, 3D LiDAR, IMU and RGB-D camera. You can spawn it in your own Gazebo world or start with Gazebo's built-in empty world. The documented target is Ubuntu 18.04, ROS Melodic and Gazebo Classic 9.

MineGuard C1 是一个可以放入自有 Gazebo 场景的 ROS 1 四轮小车包，含车体 URDF、外观网格、有限扭矩驱动、三维激光、IMU 和 RGB-D 相机。课程项目里的建图、导航与巡检集成方式见[架构说明](docs/architecture.md)。

## 编译 · Build

在已安装 ROS Melodic、Gazebo 9 和 `rosdep` 的 Ubuntu 环境中执行。以下命令假设使用一个新建的 catkin 工作区。

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/mineguard_ws/src
git clone https://github.com/junanxx9527-rgb/MineGuard.git ~/mineguard_ws/src/mineguard_robot
cd ~/mineguard_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 加入你的场景 · Spawn in your world

启动新场景，`world` 可填写你自己的 `.world` 绝对路径。默认使用 Gazebo 自带的空世界。

```bash
roslaunch mineguard_robot sim.launch
roslaunch mineguard_robot sim.launch world:=/absolute/path/to/your.world x:=1.0 y:=0.5 yaw:=0.0
```

如果 Gazebo 已经运行，在启动 Gazebo 的终端先加载本工作区的 `devel/setup.bash`，确保 Gazebo 进程能找到编译出的插件，并且世界启用了 `gazebo_ros_api_plugin`（例如使用 `gazebo_ros` 的启动入口）。然后在另一个加载过工作区的终端执行：

```bash
roslaunch mineguard_robot spawn.launch x:=1.0 y:=0.5 z:=0.03 yaw:=0.0
```

坐标单位为米，偏航角为弧度。选择无碰撞的出生点，并按场景地面高度调整 `z`。ROS 话题固定在 `/mineguard`，TF 帧名固定，因此默认同一 ROS Master 下运行一台 C1。修改 `model_name` 只改变 Gazebo 实体名称。

## 简单走车 · Send a velocity command

在有空地的仿真场景中，持续发送低速前进命令。用 Ctrl+C 结束发送，插件在约 0.4 仿真秒未收到命令后把目标速度归零。

```bash
rostopic pub -r 10 /mineguard/cmd_vel geometry_msgs/Twist '{linear: {x: 0.10}, angular: {z: 0.0}}'
```

| ROS 话题 | 用途 |
| --- | --- |
| `/mineguard/cmd_vel` | 车体线速度 `linear.x` 和转速 `angular.z` |
| `/mineguard/wheel_odom` | 由轮速积分得到的里程计 |
| `/mineguard/joint_states` | 车轮关节状态 |
| `/mineguard/lidar/points` | 三维点云，含 `ring` 与 `time` |
| `/mineguard/imu/data` | IMU |
| `/mineguard/front/image_raw`、`/mineguard/front/depth/image_raw` | RGB 与深度图像 |

SLAM、导航、避障和禁入任务可通过上表接口另行接入。直接向 `cmd_vel` 发送指令时，请在上层负责路径及避障。不同场景的地面摩擦、坡度和通行空间会影响结果。

## 验证范围 · Validation

已在 Ubuntu 18.04 / ROS Melodic / Gazebo Classic 9 的独立 catkin 工作区编译并安装。两个启动入口均在一个自建的简单场景通过，读回了车体移动、断流后的停稳、轮速里程计、关节状态、三维点云、IMU、深度图像和传感器 TF。RGB-D 相机需要可用的渲染环境，无窗口运行时可提供虚拟显示。测试场景为本地验收用，使用时请指定自己的场景。

源码与车身网格沿用原模型包声明的 [BSD-3-Clause](LICENSE)。Gazebo、ROS 和传感器插件由各自的软件包提供。
