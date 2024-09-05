# PanoSim How To

## 概述
这里是[PanoSim自动驾驶仿真平台]应用实例集。

## 目录
- 动力学
  - [蛇形工况](#todo)
  - [鱼钩工况](#todo)
  - [双移线工况](#todo)
- 主车控制信号
  - [驾驶员控制信号](https://github.com/liyanlee/PanoSim_How_To/tree/main/EgoControl/driver_signal)
  - [算法控制信号](https://github.com/liyanlee/PanoSim_How_To/tree/main/EgoControl/algorithm)
  - [期望速度控制信号](https://github.com/liyanlee/PanoSim_How_To/tree/main/EgoControl/expect_speed)
  - [期望轨迹控制信号](#todo:xDriver_path_input)
  - [加速度控制信号](#todo:xDriver_accel_input)
  - [前轮转角控制信号](#todo:xDriver_road_wheel_angle_input)
- 传感器
  - 感知器
    - [车道线感知器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/LaneInfoPerception)
    - [目标感知器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/ObjectPerception)
    - [交通灯感知器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/TrafficLightPerception)
    - [深度图感知器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/DepthmapPerception)
    - [分割图感知器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/SegmentationPerception)
    - [停车位感知器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/ParkingLotsPerception)
    - [自由行驶区域感知器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/FreeSpacePerception)
  - 相机
    - [单目相机传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Camera/MonoCamera)
    - [鱼眼相机传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Camera/FisheyeCamera)
  - 毫米波雷达
    - [毫米波雷达传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Radar/Radar)
    - [高精度毫米波雷达传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Radar/RadarHIFI)
  - 激光雷达
    - [机械式点云级激光雷达传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Lidar/SurroundLidarPointCloud)
    - [固态激光雷达传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Lidar/SolidStateLidarPointCloud)
  - 超声波雷达
    - [超声波雷达传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Ultrasonic/Ultrasonic)
    - [高精度超声波雷达传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Ultrasonic/UltrasonicHIFI)
  - 全球导航卫星系统
    - [全球导航卫星系统](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/GNSS/GNSS)
    - [高精度全球导航卫星系统](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/GNSS/GNSSHIFI)
  - IMU
    - [IMU传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/IMU)
  - User-defined
    - [事件相机传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/User-defined/EventCamera)
- 总线
  - 获取数据
    - [主车位姿、速度](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/ego)
    - [交通参与物类型、外形、位姿、速度](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/traffic)
    - [交通灯的方向、颜色、倒计时](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/traffic_light)
    - [主车所在车道、前方交叉口、SL坐标系位置](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/ego_traffic)
    - [主车信号(油门、刹车、方向盘等)](#todo:ego_driver)
  - 设置数据
    - [告警信息](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/warning)
    - [交通参与物高亮](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/traffic_object_highlight)
    - [动态天气光照](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/weather)
- 评估器
  - [碰撞评估器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/judge)
- 算法
- 联合仿真
  - [与Apollo联合仿真](https://github.com/liyanlee/PanoSim_Apollo_Bridge)
  - [与Autoware联合仿真](https://github.com/wobuzhuchele/PanoSim-Autoware)
  - [与Vissim联合仿真](https://github.com/liyanlee/PanoSim_Vissim_Bridge)

## 版权
许可证遵循 [Apache License 2.0协议]. 更多细节请访问 [LICENSE](https://github.com/liyanlee/PanoSim_How_To/blob/main/LICENSE.txt).
