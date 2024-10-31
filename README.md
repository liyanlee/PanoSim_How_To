# PanoSim How To

## 概述
这里是[PanoSim自动驾驶仿真平台]应用实例集。

## 目录
- 动力学
  - [蛇形工况](#todo)
  - [鱼钩工况](#todo)
  - [双移线工况](#todo)
- 主车控制信号
  - [驾驶员控制信号](./EgoControl/driver_signal)
  - [算法控制信号](./EgoControl/algorithm)
  - [期望速度控制信号](./EgoControl/expect_speed)
  - [期望轨迹控制信号](./EgoControl/expect_trajectory)
  - [加速度控制信号](./EgoControl/acceleration)
  - [前轮转角控制信号](./EgoControl/front_wheel_angle)
- 使用传感器
  - 感知器
    - [车道线感知器](./Sensor/Perception/LaneInfoPerception)
    - [目标感知器](./Sensor/Perception/ObjectPerception)
    - [交通灯感知器](./Sensor/Perception/TrafficLightPerception)
    - [深度图感知器](./Sensor/Perception/DepthmapPerception)
    - [分割图感知器](./Sensor/Perception/SegmentationPerception)
    - [停车位感知器](./Sensor/Perception/ParkingLotsPerception)
    - [自由行驶区域感知器](./Sensor/Perception/FreeSpacePerception)
  - 相机
    - [单目相机传感器](./Sensor/Camera/MonoCamera)
    - [鱼眼相机传感器](./Sensor/Camera/FisheyeCamera)
  - 毫米波雷达
    - [毫米波雷达传感器](./Sensor/Radar/Radar)
    - [高精度毫米波雷达传感器](./Sensor/Radar/RadarHIFI)
  - 激光雷达
    - [机械式点云级激光雷达传感器](./Sensor/Lidar/SurroundLidarPointCloud)
    - [固态激光雷达传感器](./Sensor/Lidar/SolidStateLidarPointCloud)
  - 超声波雷达
    - [超声波雷达传感器](./Sensor/Ultrasonic/Ultrasonic)
    - [高精度超声波雷达传感器](./Sensor/Ultrasonic/UltrasonicHIFI)
  - 全球导航卫星系统
    - [全球导航卫星系统](./Sensor/GNSS/GNSS)
    - [高精度全球导航卫星系统](./Sensor/GNSS/GNSSHIFI)
  - IMU
    - [IMU传感器](./Sensor/IMU)
  - User-defined
    - [事件相机传感器](./Sensor/User-defined/EventCamera)
- 总线
  - 获取数据
    - [主车位姿、速度](./Bus/ego)
    - [交通参与物类型、外形、位姿、速度](./Bus/traffic)
    - [交通灯的方向、颜色、倒计时](./Bus/traffic_light)
    - [主车所在车道、前方交叉口、SL坐标系位置](./Bus/ego_traffic)
    - [主车信号(油门、刹车、方向盘等)](./Bus/ego_driver)
    - [交通标志](./Bus/traffic_sign)
  - 设置数据
    - [告警信息](./Bus/warning)
    - [交通参与物高亮](./Bus/traffic_object_highlight)
    - [动态天气光照](./Bus/weather)
    - [车灯控制](./Bus/vehicle_light)
- 算法
    - [AEB](./Algorithm/AEB)
    - [ACC](./Algorithm/ACC)
    - [LKA](./Algorithm/LKA)
    - [APA](./Algorithm/APA)
- 交通
  - [CutIn](./Traffic/CutIn)
  - [十字路口红绿灯](./Traffic/CrossroadTrafficLight)
- 定制插件
  - 同步阻塞运行模式
    - [同步保存图像](./Customize/SyncCaptureImage)
  - 传感器
    - [定位器](./Customize/Location)
    - [目标感知器](./Customize/ObjectPerception)
  - 评估器
    - [碰撞评估器](./Bus/judge)
- V2X
  - 一期预警实例
    - [EVW](./V2X/EVW)
	- [VRUCW](./V2X/VRUCW)
  - 车辆编队
    - [编队巡航](./V2X/Platoon/Platoon1)
    - [车辆组队](./V2X/Platoon/Platoon2)
- 联合仿真
  - [与Apollo联合仿真](https://github.com/liyanlee/PanoSim_Apollo_Bridge)
  - [与Autoware联合仿真](https://github.com/wobuzhuchele/PanoSim-Autoware)
  - [与Vissim联合仿真](https://github.com/liyanlee/PanoSim_Vissim_Bridge)

## 版权
许可证遵循 [Apache License 2.0协议]. 更多细节请访问 [LICENSE](./LICENSE.txt).
