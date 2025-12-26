echo "# SimpleCar MuJoCo Simulation" > README.md
echo "" >> README.md
echo "## 项目信息" >> README.md
echo "" >> README.md
echo "学号：232011083" >> README.md
echo "姓名：王敏杰" >> README.md
echo "班级：计科2302" >> README.md
echo "完成日期：2025年12月26号" >> README.md
echo "" >> README.md
echo "## 项目简介" >> README.md
echo "" >> README.md
echo "这是一个基于MuJoCo物理引擎的简单小车仿真项目。该项目模拟了一个简单的车体，展示了车速、转速（RPM）和油量的实时变化，并在仪表盘上进行可视化。仿真过程中，小车的速度、加速度和油耗会被动态计算，并通过仪表盘展示出来。" >> README.md
echo "" >> README.md
echo "## 特性" >> README.md
echo "- **车速与转速显示**：使用半圆形仪表盘显示当前车速（km/h）和转速（RPM）。" >> README.md
echo "- **油量跟踪**：根据油门输入，计算油量消耗并显示剩余油量的百分比。" >> README.md
echo "- **动态仿真**：小车在仿真环境中运动，且当小车接近目标时，目标位置会随机重置。" >> README.md
echo "- **可视化**：通过MuJoCo的绘制功能，实时展示小车的状态，包括速度、加速度、油耗等。" >> README.md
echo "" >> README.md
echo "## 环境要求" >> README.md
echo "- **MuJoCo**：物理引擎，仿真小车运动。" >> README.md
echo "- **C++ 编译器**：支持C++11及以上标准。" >> README.md
echo "- **其他依赖**：\`absl\`（Google的工具库）等。" >> README.md
echo "" >> README.md
echo "## 安装" >> README.md
echo "" >> README.md
echo "### 1. 安装MuJoCo" >> README.md
echo "请按照[MuJoCo官方文档](https://mujoco.org/)的说明安装MuJoCo物理引擎。" >> README.md
echo "" >> README.md
echo "### 2. 克隆仓库" >> README.md
echo '```bash' >> README.md
echo "git clone https://github.com/yourusername/simplecar-mujoco.git" >> README.md
echo "cd simplecar-mujoco" >> README.md
echo '```' >> README.md
echo "" >> README.md
echo "### 3. 编译项目" >> README.md
echo "确保你已经安装了支持C++11的编译器。然后运行以下命令来编译项目：" >> README.md
echo '```bash' >> README.md
echo "mkdir build" >> README.md
echo "cd build" >> README.md
echo "cmake .." >> README.md
echo "make" >> README.md
echo '```' >> README.md
echo "" >> README.md
echo "## 使用方法" >> README.md
echo "" >> README.md
echo "### 运行仿真" >> README.md
echo "编译完成后，可以使用以下命令启动仿真：" >> README.md
echo '```bash' >> README.md
echo "./simple_car_simulation" >> README.md
echo '```' >> README.md
echo "仿真运行后，控制台会显示小车的位置、速度、加速度和油量等信息。仪表盘会实时更新车速和转速，并通过进度条显示转速。" >> README.md
echo "" >> README.md
echo "### 仿真控制" >> README.md
echo "- **油门控制**：通过调整油门输入来控制小车的加速和油耗。" >> README.md
echo "- **目标位置**：小车会朝着目标位置运动，当接近目标时，目标会被随机重置。" >> README.md
echo "" >> README.md
echo "## 贡献" >> README.md
echo "如果你想为这个项目做出贡献，请提交Pull Request。我们欢迎任何形式的贡献，如bug修复、功能增强和文档改进。" >> README.md
echo "" >> README.md
echo "## 许可" >> README.md
echo "该项目使用 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) 开源协议。" >> README.md
