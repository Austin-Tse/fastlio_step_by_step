# IESKF "跑飞" 问题排查指南

## 已发现并修复的Bug

### 🔴 严重Bug 1: Q矩阵配置错误
**位置**: `ieskf.cpp` 构造函数

**问题**:
1. `cov_bias_gyroscope` 被错误地读取到了 `cov_gyroscope` 变量中
2. Q矩阵对角线元素设置错误：
   - `Q(3,3)` 块最后一个元素应该是 `cov_acceleration`，但错误写成了 `cov_gyroscope`
   - `Q(9,9)` 块最后一个元素应该是 `cov_bias_acceleration`，但错误写成了 `cov_acceleration`

**影响**: 过程噪声协方差矩阵错误会导致滤波器发散

**状态**: ✅ 已修复

---

### 🟡 严重Bug 2: IESKF update()函数被完全注释
**位置**: `ieskf.cpp` update函数

**问题**: 整个update函数实现都被注释掉，只返回true

**影响**: 系统只进行IMU预测，没有利用LiDAR观测进行状态校正，导致累积误差无法消除

**状态**: ⚠️ 需要手动启用update函数

---

### 🟡 可能的Bug 3: IMU scale重复应用
**位置**: `frontback_propagate.cpp`

**问题**: 
```cpp
auto acc_avr = 0.5 * (head.acceleration+tail.acceleration) * imu_scale;
```
如果IMU数据已经被缩放过，这里再乘以imu_scale可能导致重复缩放

**建议**: 确认imu_scale是否应该应用在这里

---

### 🟢 潜在问题 4: 外参未应用
**位置**: `frontend.cpp` track函数

**问题**: IMU到LiDAR的外参转换代码被注释掉
```cpp
// Eigen::Quaterniond lidar_q = x.rotation * extrin_r_q;
// Eigen::Vector3d lidar_p = x.rotation * extrin_t_v + x.position;
```

**影响**: 如果外参不是单位变换，会导致建图和定位不准确

---

## 已添加的调试功能

### 1. IMU预测监控 (`ieskf.cpp::predict()`)
- ✅ 检查dt是否合理 (0 < dt < 1.0)
- ✅ 检查IMU数据范数是否异常
- ✅ 检测NaN/Inf
- ✅ 监控协方差矩阵trace(P)是否发散 (>1000)
- ✅ 每100次打印一次状态信息

### 2. IMU初始化监控 (`frontend.cpp::initState()`)
- ✅ 显示初始化进度
- ✅ 输出关键参数：imu_scale, mean_acc, bg, gravity
- ✅ 检查imu_scale合理性 (期望值~1.0)
- ✅ 检查陀螺仪偏置大小

### 3. 帧处理监控 (`frontend.cpp::track()`)
- ✅ 显示帧号
- ✅ 输出更新前后的位置和速度
- ✅ 统计点云数量（原始/滤波后）
- ✅ 检测大的位置跳变 (>10m)

### 4. IMU传播监控 (`frontback_propagate.cpp::propagate()`)
- ✅ 显示IMU数量和时间跨度
- ✅ 检查每个IMU的dt是否合理
- ✅ 检查最后一次预测的dt

---

## 运行调试步骤

### Step 1: 编译代码
```bash
cd /home/mars_ugv/workspace/fastlio_step_by_step
catkin_make
```

### Step 2: 运行系统
```bash
source devel/setup.bash
roslaunch ieskf_slam aviz.launch
```

### Step 3: 播放bag包
```bash
# 在另一个终端
rosbag play dataset/cloud.bag
```

### Step 4: 观察输出

#### 正常情况应该看到：
```
========== IMU INITIALIZATION COMPLETE ==========
imu_scale: 0.95~1.05 之间
mean_acc: [x, y, z], norm: ~9.8
bg (gyro bias): 应该很小，< 0.1
gravity: [0, 0, -9.81] 或 [0, 0, 9.81]
=================================================

[PROPAGATE] IMU count: 10~50
[INFO] Predict #100 | pos: [小的数值] | vel_norm: 合理的速度
[FRAME #1] Before update - pos: [...]
After update - pos: [...]
```

#### 异常情况标志：
- ❌ `[ERROR] NaN/Inf detected`
- ❌ `[ERROR] Covariance diverging! trace(P) = 很大的数`
- ❌ `[WARN] Large position jump: >10 meters`
- ❌ `[WARN] IMU scale abnormal`
- ❌ `[WARN] Abnormal dt`
- ❌ 速度范数突然增大 (vel_norm > 50)
- ❌ 位置数值迅速增大

---

## 核心问题检查清单

### 1. 立即检查（最可能导致跑飞）
- [ ] **Q矩阵配置错误** - ✅ 已修复
- [ ] **update函数未启用** - ⚠️ 需要取消注释
- [ ] 协方差矩阵P是否发散
- [ ] IMU数据是否正常（无NaN/Inf）
- [ ] 时间戳dt是否正常

### 2. 重要检查
- [ ] IMU初始化是否成功
- [ ] imu_scale是否合理（~1.0）
- [ ] 重力方向是否正确
- [ ] 点云数据是否正常接收

### 3. 后续优化
- [ ] 外参是否正确应用
- [ ] IMU频率是否足够（>100Hz）
- [ ] 点云匹配是否有效

---

## 常见"跑飞"原因及解决方案

### 原因1: 协方差矩阵发散
**症状**: trace(P) 快速增大
**解决**: 
- 检查Q矩阵配置（已修复）
- 确保update函数正常工作
- 调整过程噪声参数

### 原因2: IMU数据异常
**症状**: 加速度或角速度有突变、NaN、Inf
**解决**:
- 检查传感器数据质量
- 添加数据滤波
- 检查IMU驱动

### 原因3: 时间同步问题
**症状**: dt异常（负数或很大）
**解决**:
- 检查时间戳计算
- 确保IMU和LiDAR时间对齐

### 原因4: 初始化失败
**症状**: imu_scale偏离1.0很多，重力方向错误
**解决**:
- 确保机器人静止初始化
- 收集更多IMU数据（增加初始化时间）

### 原因5: update函数未工作
**症状**: 位置持续漂移，没有LiDAR校正
**解决**:
- 取消注释update函数
- 检查点云匹配是否成功

---

## 下一步行动

### 立即行动（必须）:
1. ✅ 已修复Q矩阵bug
2. ⚠️ 取消注释并测试update函数
3. 运行系统，观察调试输出

### 根据调试输出判断:
- 如果初始化失败 → 检查IMU数据和初始化逻辑
- 如果trace(P)发散 → 调整噪声参数或检查predict逻辑
- 如果位置跳变 → 检查update函数和点云匹配
- 如果持续漂移 → 确保update函数在工作

### 优化建议:
1. 启用update函数后测试
2. 根据实际数据调整噪声参数
3. 验证外参是否需要应用
4. 检查imu_scale的使用位置

---

## 配置文件参数说明

`config/avia.yaml`:
```yaml
ieskf:
  cov_gyroscope: 0.1          # 陀螺仪测量噪声
  cov_acceleration: 0.1        # 加速度计测量噪声  
  cov_bias_acceleration: 0.0001  # 加速度计偏置随机游走
  cov_bias_gyroscope: 0.0001     # 陀螺仪偏置随机游走
```

如果系统发散，可以尝试：
- 减小 cov_gyroscope 和 cov_acceleration（更信任IMU）
- 增大 bias 相关参数（允许偏置变化更大）

---

## 联系与反馈

调试时请关注：
1. 第一次出现ERROR或WARN的位置
2. trace(P)的变化趋势
3. 位置和速度的数值变化

保存完整的终端输出，便于分析问题。
