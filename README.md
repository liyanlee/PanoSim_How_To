# PanoSim How To

## 概述
这里是[PanoSim自动驾驶仿真平台]应用实例集。

## 目录
- 一、API使用实例(API)
  - 1.1 主车控制信号(ego control signal)
    - [驾驶员控制信号(driver signal)](./EgoControl/driver_signal)
    - [算法控制信号(algorithm control signal)](./EgoControl/algorithm)
    - [期望速度控制信号(expect speed control signal)](./EgoControl/expect_speed)
    - [期望轨迹控制信号(expect trajectory control signal)](./EgoControl/expect_trajectory)
    - [加速度控制信号(acceleration control signal)](./EgoControl/acceleration)
    - [前轮转角控制信号(front wheel angle control signal)](./EgoControl/front_wheel_angle)
  - 1.2 使用传感器(using sensor)
    - 1.2.1 感知器(perceptron)
      - [车道线感知器(lane information perception)](./Sensor/Perception/LaneInfoPerception)
      - [目标感知器(object perception)](./Sensor/Perception/ObjectPerception)
      - [交通灯感知器(traffic light perception)](./Sensor/Perception/TrafficLightPerception)
      - [深度图感知器(depth map perception)](./Sensor/Perception/DepthmapPerception)
      - [分割图感知器(segmentation graph perception)](./Sensor/Perception/SegmentationPerception)
      - [停车位感知器(parking lots perception)](./Sensor/Perception/ParkingLotsPerception)
      - [自由行驶区域感知器(free space perception)](./Sensor/Perception/FreeSpacePerception)
    - 1.2.2 相机(camera)
      - [单目相机传感器(mono camera)](./Sensor/Camera/MonoCamera)
      - [鱼眼相机传感器(fisheye camera)](./Sensor/Camera/FisheyeCamera)
    - 1.2.3 毫米波雷达(radar)
      - [毫米波雷达传感器(radar)](./Sensor/Radar/Radar)
      - [高精度毫米波雷达传感器(HIFI radar)](./Sensor/Radar/RadarHIFI)
    - 1.2.4 激光雷达(lidar)
      - [机械式点云级激光雷达传感器(surround lidar point cloud)](./Sensor/Lidar/SurroundLidarPointCloud)
      - [固态激光雷达传感器(solid state lidar point cloud)](./Sensor/Lidar/SolidStateLidarPointCloud)
    - 1.2.5 超声波雷达(ultrasonic)
      - [超声波雷达传感器(ultrasonic)](./Sensor/Ultrasonic/Ultrasonic)
      - [高精度超声波雷达传感器(HIFI ultrasonic)](./Sensor/Ultrasonic/UltrasonicHIFI)
    - 1.2.6 全球导航卫星系统(gnss)
      - [全球导航卫星系统(gnss)](./Sensor/GNSS/GNSS)
      - [高精度全球导航卫星系统(HIFI gnss)](./Sensor/GNSS/GNSSHIFI)
    - 1.2.7 IMU
      - [IMU传感器(imu)](./Sensor/IMU)
    - 1.2.8 User-defined
      - [事件相机传感器(event camera)](./Sensor/User-defined/EventCamera)
	  - [智能车灯(Headlamp)](./Sensor/User-defined/Headlamp)
	  - [ARHUD(ARHUD)](./Sensor/User-defined/ARHUD)
  - 1.3 访问总线(access bus) 
    - 1.3.1 获取数据(get data)
      - [主车位姿、速度(ego: pose, speed)](./Bus/ego)
      - [交通参与物类型、外形、位姿、速度(traffic object: type, shape, pose, speed)](./Bus/traffic)
      - [交通灯的方向、颜色、倒计时(traffic light: direction, color, timer)](./Bus/traffic_light)
      - [主车所在车道、前方交叉口、SL坐标系位置(ego: lane, in front of junction, SL)](./Bus/ego_traffic)
      - [主车信号-油门、刹车、方向盘等(ego: throttle,brake,steer,mode,gear)](./Bus/ego_driver)
      - [交通标志(traffic sign)](./Bus/traffic_sign)
    - 1.3.2 设置数据(set data)
      - [动态天气光照(weather)](./Bus/weather)
      - [车灯控制(vehicle light)](./Bus/vehicle_light)
  - 1.4 创建交通场景(create traffic scenario)
    - [CutIn](./Traffic/CutIn)
    - [十字路口红绿灯(crossroad traffic light)](./Traffic/CrossroadTrafficLight)
    - [交通车灯(traffic vehicle light)](./Traffic/TrafficVehicleLight)
  - 1.5 可视化
    - [告警信息(warning information)](./Bus/warning)
    - [交通参与物高亮(highlight traffic object)](./Bus/traffic_object_highlight)
- 二、典型算法实例(typical algorithm)
  - 2.1 动力学测试典型工况(working conditions for dynamic testing)
    - [蛇形工况](#todo)
    - [鱼钩工况](#todo)
    - [双移线工况](#todo)
  - 2.2 视觉感知算法(perception)
    - 2D目标检测
      - [YOLO](./Customize/YOLO)
    - [车道线检测](#todo)
    - [3D目标识别](#todo)
  - 2.3 数据融合算法(fusion)
    - [感知数据融合](#todo)
    - [航迹推算](#todo)
  - 2.4 定位建图算法(positioning and mapping)
    - [组合导航定位](#todo)
    - [SLAM](#todo)
  - 2.5 决策规划算法(prediction and planning)
    - [全局轨迹规划](#todo)
    - [局部轨迹规划](#todo)
  - 2.6 轨迹跟随控制算法(trajectory follow)
    - [基于动力学模型的控制算法](#todo)
    - [基于人工智能的方法](#todo)
- 三、自动驾驶系统(autopilot)
  - 3.1 单车智能(automotive intelligence)
    - 3.1.1 ADAS
      - [AEB_Simulink](./Algorithm/AEB/AEB_Simulink)
      - [ACC_Simulink](./Algorithm/ACC/ACC_Simulink)
      - [LKA_Simulink](./Algorithm/LKA/LKA_Simulink)
      - [APA_Python](./Algorithm/APA/APA_Python)
    - 3.1.2 AD
      - [端到端闭环仿真](#todo)
  - 3.2 车路协同(V2X)
    - 3.2.1 V2X应用(using V2X)
      - 一期预警实例(early warning)
        - [EVW](./V2X/EVW)
        - [VRUCW](./V2X/VRUCW)
      - 车辆编队(vehicle platooning)
        - [编队巡航(cruise)](./V2X/Platoon/Platoon1)
        - [车辆组队(team)](./V2X/Platoon/Platoon2)
    - 3.2.2 车路协同感知(V2X perception)
- 四、评价器实例(evaluator)
  - 4.1 功能性评价(function)
  - 4.2 安全性评价(safety)
    - [碰撞评估器(collision detector)](./Bus/judge)
  - 4.3 舒适性评价(comfort)
  - 4.4 通行效率评价(efficiency)
  - 4.5 合规性评价(rule)
- 五、联合仿真(Co-simulation)
  - [与Apollo联合仿真(PanoSim & Apollo Co-simulation)](https://github.com/liyanlee/PanoSim_Apollo_Bridge)
  - [与Autoware联合仿真(PanoSim & Autoware Co-simulation)](https://github.com/wobuzhuchele/PanoSim-Autoware)
  - [与Vissim联合仿真(PanoSim & Vissim Co-simulation)](https://github.com/liyanlee/PanoSim_Vissim_Bridge)
- 六、定制(customization)
  - 6.1 同步阻塞运行模式(sync and block mode)
    - [同步保存图像(sync capture image)](./Customize/SyncCaptureImage)
  - 6.2 定制传感器(sensor)
    - [定位器(location)](./Customize/Location)
    - [目标感知器(object perception)](./Customize/ObjectPerception)
  - 6.3 定制外部调度器(external node)
    - [外部调度器(external node)](./Customize/ExternalNode)
  - 6.4 定制外部启动器(external starter)
    - [外部启动器(external starter)](./Customize/ExternalStarter)


## 版权
许可证遵循 [Apache License 2.0协议]. 更多细节请访问 [LICENSE](./LICENSE.txt).
