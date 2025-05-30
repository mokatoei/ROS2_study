### **ROS2 第一个程序：Hello World**
在 ROS2 中，编写程序通常是 **创建一个 ROS2 包（package）**，然后在其中编写 **节点（node）** 来运行。我们会：
1. 创建一个 ROS2 包  
2. 编写 **C++** 或 **Python** 节点  
3. 构建并运行节点  

---

## **1. 创建一个 ROS2 包**
在 ROS2 中，每个项目通常是一个 **包（package）**，包含代码、依赖项和配置文件。

### **1.1 进入工作空间**
在 ROS2 中，所有代码都应该放在 **工作空间（workspace）** 里，比如 `ros2_ws`：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### **1.2 创建 ROS2 包**
```bash
ros2 pkg create --build-type ament_cmake my_helloworld
```
> `my_helloworld` 是包名，你可以换成其他名字。  

📌 **解释**：
- `ros2 pkg create` → 创建 ROS2 包  
- `--build-type ament_cmake` → 指定 **CMake** 作为构建系统（C++用CMake，Python用`ament_python`）
- `my_helloworld` → 包名  

创建完成后，你会看到如下目录结构：
```
ros2_ws/src/my_helloworld
├── CMakeLists.txt  
├── package.xml
├── src/
└── include/
```
---

## **2. 编写 ROS2 节点**
### **2.1 编写 C++ 节点**
编辑 `src/hello_node.cpp`，输入以下代码：
```cpp
#include "rclcpp/rclcpp.hpp"

class HelloWorldNode : public rclcpp::Node {
public:
    HelloWorldNode() : Node("hello_node") {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS2!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldNode>());
    rclcpp::shutdown();
    return 0;
}
```
📌 **解释**：
- 继承 `rclcpp::Node` 创建节点  
- 在构造函数 `HelloWorldNode()` 里打印 `"Hello, ROS2!"`  
- `rclcpp::spin()` 让节点保持运行  

---

### **2.2 修改 CMakeLists.txt**
打开 `CMakeLists.txt`，找到 `# add_executable`，并修改：
```cmake
add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp)

install(TARGETS
  hello_node
  DESTINATION lib/${PROJECT_NAME}
)
```
📌 **解释**：
- `add_executable(hello_node src/hello_node.cpp)` → 定义可执行文件  
- `ament_target_dependencies(hello_node rclcpp)` → 链接 ROS2 库  
- `install(...)` → 安装二进制文件  

---

## **3. 构建和运行**
### **3.1 进入工作空间**
```bash
cd ~/ros2_ws
```

### **3.2 运行编译**
```bash
colcon build --packages-select my_helloworld
```
如果编译成功，你会看到：
```
[100%] Built target hello_node
```

### **3.3 运行节点**
首先，**设置环境变量**：
```bash
source install/setup.bash
```
然后运行：
```bash
ros2 run my_helloworld hello_node
```
你应该能看到：
```
[INFO] [hello_node]: Hello, ROS2!
```

---

## **4. Python 版 Hello World（可选）**
如果你更喜欢 Python，可以创建 `my_helloworld/hello_world.py`：
```python
import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS2!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### **添加 Python 节点到 `setup.py`**
在 `setup.py` 里找到 `entry_points`，添加：
```python
entry_points={
    'console_scripts': [
        'hello_node = my_helloworld.hello_world:main',
    ],
},
```
然后编译：
```bash
colcon build --packages-select my_helloworld
source install/setup.bash
ros2 run my_helloworld hello_node
```
你会看到：
```
[INFO] [hello_node]: Hello, ROS2!
```

---

## **总结**
✅ **创建 ROS2 包** → `ros2 pkg create`  
✅ **编写 C++ 或 Python 节点** → `rclcpp` 或 `rclpy`  
✅ **编译** → `colcon build`  
✅ **运行** → `ros2 run`  

你可以试试看！ 🚀