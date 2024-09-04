# PanoSim How To

## 概述
这里是[PanoSim自动驾驶仿真平台]应用实例集。

## 目录
- 动力学
  - [蛇形工况](#todo)
  - [鱼钩工况](#todo)
  - [双移线工况](#todo)
- 主车控制
- 传感器
  - 感知
    - [车道线传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/LaneInfoPerception)
    - [目标感知传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/ObjectPerception)
    - [交通灯传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/TrafficLightPerception)
    - [深度图传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/DepthmapPerception)
    - [分割图传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/SegmentationPerception)
    - [停车位传感器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Sensor/Perception/ParkingLotsPerception)
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
  - [ego](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/ego)
  - [traffic](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/traffic)
  - [traffic_light](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/traffic_light)
  - [warning](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/warning)
  - [ego_traffic](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/ego_traffic)
  - [traffic_object_highlight](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/traffic_object_highlight)
  - [weather](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/weather)
- 评估器
  - [碰撞评估器](https://github.com/liyanlee/PanoSim_How_To/tree/main/Bus/judge)
- 算法
- 联合仿真
  - [与Apollo联合仿真](https://github.com/liyanlee/PanoSim_Apollo_Bridge)
  - [与Autoware联合仿真](https://github.com/wobuzhuchele/PanoSim-Autoware)
  - [与Vissim联合仿真](https://github.com/liyanlee/PanoSim_Vissim_Bridge)

## 版权
许可证遵循 [Apache License 2.0协议]. 更多细节请访问 [LICENSE](https://github.com/liyanlee/PanoSim_How_To/blob/main/LICENSE.txt).
