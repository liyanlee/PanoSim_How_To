# PanoSim 外部启动器

## 1. 概述

通常使用PanoExp启动仿真实验。在产品使用过程中, 用户希望使用自研软件控制仿真实验的启停。
在Windows平台, 可以使用各种常用编程语言(python/c++...), 控制仿真实验的启停。本实例使用python语言说明实现的方法。


## 2. 安装部署

### 2.1 下载外部启动器[ExternalStarter.py](ExternalStarter.py), 保存到本地任意路径

例如: D:/PanoSim5/PanoSim_How_To/Customize/ExternalStarter

### 2.2 下载[实验文件](./PanoSimDatabase)

> [!NOTE]
> 外部启动器可以启动任意实验(至少在PanoExp中成功启动过一次)。
> 这里的下载操作仅用于后续的说明。

### 2.3 查询本地对应目录

![image](../../Bus/ego/docs/images/folder.jpg)

### 2.4 复制文件到本地对应目录

### 2.5 重新启动PanoExp

## 3. 运行实验

```shell
conda activate py3.11.7
python D:/PanoSim5/PanoSim_How_To/Customize/ExternalStarter/ExternalStarter.py
```

## 4. 运行结果

实验启动后, 运行5秒, 自动停止实验。
