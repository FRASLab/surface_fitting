# Surface Reconstructor & Inverse Kinematics

这是一个高性能的 C++ 库，旨在解决**点云曲面重建**与**机器人接触作业**中的运动学问题。它集成了基于 NURBS（非均匀有理 B 样条）的曲面拟合算法，并为搭载 5-DOF 机械臂的无人机系统（UAM）提供了逆运动学接口。

## 🌟 核心功能

* **NURBS 曲面拟合**：支持从 `.pcd` 文件或 `pcl::PointCloud` 实时输入点云，生成高精度的 NURBS 表现形式。
* **几何属性提取**：可计算曲面上任意参数  处的：
* 三维坐标（Position）
* 法向量（Normal）
* 一阶/二阶导数（Derivatives）
* 主曲率与高斯曲率（Curvature）


* **机器人逆运动学 (IK)**：专门针对 UAM 系统设计，将曲面切向/法向约束转化为机器人的关节配置空间。
* **数据导出**：支持将拟合后的曲面保存为工业标准的 **STL** 文件。
* **ROS 兼容**：项目结构兼容 ROS Noetic 环境，方便集成到机器人操作系统中。

---

## 🛠 依赖项目

在编译此项目之前，请确保系统中已安装以下库：

| 依赖名称 | 建议版本 | 说明 |
| --- | --- | --- |
| **CMake** | >= 3.0.2 | 构建系统 |
| **PCL** | >= 1.14 | 用于点云处理及 NURBS 算法库 |
| **Eigen** | 3.x | 矩阵与线性代数运算 |
| **VTK** | 默认版本 | PCL 可视化支持 |
| **ROS Noetic** | 可选 | 默认配置路径指向 `/opt/ros/noetic` |

---

## 🚀 快速入门

### 1. 编译安装

```bash
git clone <repository-url>
cd surface_contact
mkdir build && cd build
cmake ..
make

```

### 2. 代码示例

以下是如何加载点云并拟合曲面的简单示例（参考 `test_demo.cpp`）：

```cpp
#include "nurbs.h"
#include "invkin.h"

// 1. 初始化并加载点云
auto nurbs = new surface_reconstructor::Nurbs("path/to/your_cloud.pcd");

// 2. 执行曲面拟合
nurbs->fitSurface();

// 3. 获取曲面上 (0.5, 0.5) 处的空间点位和法向量
Eigen::Vector3d point, normal;
nurbs->getPos(0.5, 0.5, point);
nurbs->getNormal(0.5, 0.5, normal);

// 4. 计算 UAM 逆运动学
auto ik = new dynamic_planning::InvKin(nurbs);
ik->setLinkLength(1.0);
auto q_uam = ik->xToQ(0.5, 0.5); // 返回 [x, y, z, yaw, theta]

```

---

## 📂 模块说明

### `Nurbs` 类

核心拟合引擎。它包装了 `pcl::on_nurbs` 模块，提供了更友好的 API。通过 `setFittingParams` 可以调节拟合的平滑度（Smoothness）与权重（Weight）。

### `InvKin` 类

运动学转换器。它负责将曲面上的参数化坐标转换为机器人的实际位姿。

* `xToQs`: 将  转换为带法向的曲面配置。
* `qsToQ`: 计算最终的无人机基座位置与关节角度。
* `getJacobian`: 提供机械臂末端相对于基座的雅可比矩阵。

