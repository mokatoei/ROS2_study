
##  `rclcpp::spin()` 

`rclcpp::spin()`是一个 **阻塞函数**，会使得程序持续运行并不断处理来自话题、服务、动作等的 **回调事件**，直到节点被显式地 `rclcpp::shutdown()` 或者接收到 Ctrl+C。

---

## 函数原型

```cpp
namespace rclcpp {
  void spin(const rclcpp::Node::SharedPtr & node);
}
```

---

## 参数

| 参数     | 类型                        | 说明         |
| ------ | ------------------------- | ---------- |
| `node` | `rclcpp::Node::SharedPtr` | 要执行回调的节点指针 |

你也可以传入 `rclcpp::LifecycleNode::SharedPtr`、`rclcpp::NodeBaseInterface::SharedPtr` 等变种，取决于你用的是哪种节点。

---

## 作用

`rclcpp::spin()` 的作用就是：

* 开始**事件循环**，不断检查是否有数据到达。
* 有数据时就会调用你注册的 **回调函数**（比如订阅话题的回调、定时器回调、服务请求处理等）。
* 没有数据就会阻塞等待，直到有事可做。
* 一直运行，直到：

  * 调用了 `rclcpp::shutdown()`；
  * 或者收到了 Ctrl+C（默认安装了 `SIGINT` 信号处理器）；
  * 或者被其他手段中断（如调用 `rclcpp::executor->cancel()`）。

---

## 示例代码

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("my_node");

    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        [](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received: '%s'", msg->data.c_str());
        }
    );

    rclcpp::spin(node);  // ✅ 保持运行直到 Ctrl+C 或 shutdown()

    rclcpp::shutdown();
    return 0;
}
```

---

#  常见扩展版本

##  `rclcpp::executors::SingleThreadedExecutor`

```cpp
rclcpp::executors::SingleThreadedExecutor exec;
exec.add_node(node);
exec.spin();
```
`rclcpp::executors::SingleThreadedExecutor` 是 ROS 2 中的一种执行器（Executor）实现，用于运行 **单线程事件循环**。它可以替代 `rclcpp::spin()` 函数，在需要更明确地控制节点执行流程或多节点管理时非常有用。

---

###  什么是 Executor（执行器）？

在 ROS 2 中，执行器（Executor）负责：

* 管理节点（Node）
* 监听话题、服务、动作、定时器等事件源
* 调用相应的 **回调函数**

可以把 Executor 理解为一个调度器或“主循环控制器”。

---

###  `SingleThreadedExecutor` 简介

```cpp
namespace rclcpp::executors {
    class SingleThreadedExecutor : public rclcpp::Executor {
        // ...
    };
}
```

* 它继承自 `rclcpp::Executor`
* 只使用**一个线程**来调度和执行所有回调
* 所有回调会**串行执行**，保证线程安全(所有任务一个接一个地执行，每次只执行一个任务，执行完一个才开始下一个。)
* 适合逻辑较简单的系统或你希望自己显式控制节点管理的情况

---

###  基本用法

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("my_node");

    auto sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10,
        [](const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(rclcpp::get_logger("my_node"), "Received: %s", msg->data.c_str());
        });

    // 创建执行器并添加节点
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();  // 阻塞式运行

    rclcpp::shutdown();
    return 0;
}
```

---

###  使用场景

| 场景         | 说明                                        |
| ---------- | ----------------------------------------- |
| 单个或多个节点管理  | 比如你需要用一个 Executor 同时 spin 多个节点            |
| 更好的结构化代码   | 相比直接调用 `rclcpp::spin()`，你可以显式控制节点添加、移除等逻辑 |
| 需要和多线程版本切换 | 比如开发时先用单线程调试，后续换成 `MultiThreadedExecutor` |

---

### 和 `rclcpp::spin()` 有什么区别？

| 特性    | `rclcpp::spin(node)` | `SingleThreadedExecutor`           |
| ----- | -------------------- | ---------------------------------- |
| 控制粒度  | 简单封装，适合单节点           | 更灵活，可以添加多个节点                       |
| 多线程支持 | ❌ 不支持                | ❌ 不支持，使用 MultiThreadedExecutor 才支持 |
| 使用难度  | 初学者推荐                | 进阶用法，更灵活                           |

---

##  `rclcpp::executors::MultiThreadedExecutor`

适合处理并发、多个定时器或服务的情况：

```cpp
rclcpp::executors::MultiThreadedExecutor exec;
exec.add_node(node);
exec.spin();
```
`rclcpp::executors::MultiThreadedExecutor` 是 ROS 2 中用于 **并行（并发）处理多个回调** 的执行器。与 `SingleThreadedExecutor` 串行执行不同，它可以在**多个线程中同时处理多个回调函数**，适合处理高频率消息、大量传感器数据等对实时性有要求的应用场景。

---

###  基本定义

```cpp
namespace rclcpp
{
namespace executors
{
class MultiThreadedExecutor : public rclcpp::Executor
```

这是一个 ROS 2 提供的执行器类，继承自 `rclcpp::Executor`。

---

###  工作原理

* 内部维护一个线程池；
* 当有多个回调待处理（如订阅、服务、定时器等）时，会自动分配线程并**并行处理多个回调**；
* 回调之间可以同时运行，适合多核 CPU 提高效率。

---

###  典型用途

* 同时处理多个高频话题；
* 服务响应与订阅消息同时进行；
* 多个定时器任务并发运行；
* 机器人需要同时处理多个传感器（激光雷达、IMU、摄像头等）数据时。

---

###  使用示例

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("multi_thread_node")
  {
    sub1_ = this->create_subscription<std_msgs::msg::String>(
      "/topic1", 10,
      [](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Topic1: '%s'", msg->data.c_str());
      });

    sub2_ = this->create_subscription<std_msgs::msg::String>(
      "/topic2", 10,
      [](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Topic2: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNode>();

  // 使用多线程执行器，默认线程数是系统核心数
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
```

---

###  可自定义线程数（如果你用的是 `rclcpp::ExecutorOptions`）

```cpp
rclcpp::ExecutorOptions options;
options.number_of_threads = 4;
rclcpp::executors::MultiThreadedExecutor executor(options);
```

---

### 注意事项

* 多线程执行器处理速度更快，但**可能带来线程安全问题**；
* 如果你在多个回调中**操作同一全局变量或资源**，请使用 `mutex` 锁保护；
* 并发调试比串行更复杂，因此一开始学习时建议先使用 `SingleThreadedExecutor`。

---

### 总结对比

| 特性   | SingleThreadedExecutor | MultiThreadedExecutor |
| ---- | ---------------------- | --------------------- |
| 回调处理 | 串行                     | 并行                    |
| 性能   | 较低                     | 高                     |
| 线程安全 | 高                      | 需要手动管理                |
| 适用场景 | 简单任务                   | 多数据源/高频数据             |

---

## `rclcpp::spin_some()` 
`rclcpp::spin_some()` 是 ROS 2 中用于处理**当前准备就绪的回调函数（callbacks）**的一种方式。与 `spin()` 持续阻塞不同，`spin_some()` 是**非阻塞、短时执行的**函数，适用于你希望在自己的主循环中控制 ROS 回调频率的场景。

当前准备就绪的回调函数 = 依次执行已经触发、等待处理的所有回调函数，而不是去等待新的事件。

---

###  函数定义

```cpp
void rclcpp::spin_some(std::shared_ptr<rclcpp::Node> node);
```

* **参数**：ROS 2 节点指针；
* **作用**：处理该节点上**所有就绪的回调函数**（例如消息到达、定时器触发等）；
* **返回**：无返回值；
* **行为**：执行完所有当前就绪的回调后立即返回。

---

### 适用场景

1. 自定义主循环逻辑（例如主线程中还有其他业务）；
2. 不希望阻塞，控制循环频率；
3. 控制处理回调的时间点；
4. 嵌入式系统或游戏主循环中集成 ROS。

---

## 🧪 示例代码

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("spin_some_example")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/topic", 10,
      [](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();

  rclcpp::Rate rate(10);  // 10Hz 主循环
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);  // 处理就绪回调
    // 主循环中的其他任务
    // ...
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
```

---

## 🔄 与 `spin()` 对比

| 特性     | `spin()`      | `spin_some()`    |
| ------ | ------------- | ---------------- |
| 是否阻塞   | 是，直到 shutdown | 否，执行当前所有可用回调立即返回 |
| 控制回调节奏 | 无法控制          | 可以结合主循环频率灵活控制    |
| 适用场景   | 简单节点          | 需要集成其他逻辑的复杂节点    |

---

如果你在写一个机器人总控制节点，需要轮询状态、周期性发布等，推荐使用 `spin_some()` + `rclcpp::Rate` 来更好地控制逻辑流程。

注意：
- spin_some() 不会阻塞，只处理当下已有的回调；

- 若频率太低，回调堆积；

- 若你希望“实时处理”，应使用 spin() 或 MultiThreadedExecutor::spin()；

- 若你希望“主动控制”，spin_some() 适合定时轮询、手动控制频率的场景。

---

## spin\_until\_future\_complete
`rclcpp::spin_until_future_complete()` 是 ROS 2 C++ 中的一个 **同步等待机制**，常用于等待异步操作（如 `async_send_request()`）的结果返回。

---

###  函数定义

```cpp
namespace rclcpp
{
  template<typename NodeT>
  rclcpp::FutureReturnCode spin_until_future_complete(
    std::shared_ptr<NodeT> node,
    rclcpp::Client<...>::SharedFuture future,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
}
```

---

###  参数解释

| 参数        | 类型                         | 含义                                    |
| --------- | -------------------------- | ------------------------------------- |
| `node`    | `std::shared_ptr<Node>`    | 你当前使用的 ROS 2 节点                       |
| `future`  | `std::shared_future<...>`  | 来自 `async_send_request()` 的 future 对象 |
| `timeout` | `std::chrono::nanoseconds` | 等待最大时长（默认-1表示一直等到完成）                  |

---

###  返回值

```cpp
enum class FutureReturnCode {
  SUCCESS,       // future 完成
  INTERRUPTED,   // ROS2 被 Ctrl+C 中断
  TIMEOUT        // 到达超时时间，future 仍未完成
};
```

---

### 使用场景

用于等待一个服务的响应，例如：

* 向某服务端发送请求；
* 主动等待这个请求有回应（带超时）；
* 根据是否超时做不同的处理。

---

## 示例代码：客户端请求 + spin\_until\_future\_complete

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("client_node");

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 3;
  request->b = 4;

  auto result_future = client->async_send_request(request);

  // 等待响应，超时时间 5 秒
  auto status = rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(5));

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result_future.get();
    RCLCPP_INFO(node->get_logger(), "Result: %ld", response->sum);
  } else if (status == rclcpp::FutureReturnCode::TIMEOUT) {
    RCLCPP_ERROR(node->get_logger(), "Service call timed out");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call interrupted");
  }

  rclcpp::shutdown();
  return 0;
}
```

---

### 总结

| 特点          | 说明                          |
| ----------- | --------------------------- |
| 等待机制        | 主动等待一个 future 完成            |
| 非阻塞 spin 替代 | 不使用 `spin()`，适合一次性等待        |
| 可设置超时       | 保证不会永久阻塞                    |
| 常用于         | 服务客户端（Service Client）请求响应等待 |

---


##  总结

| 点    | 说明                                                |
| ---- | ------------------------------------------------- |
| 本质   | 一个事件循环，持续处理回调函数                                   |
| 特性   | 阻塞、直到 shutdown 或 Ctrl+C                           |
| 使用场景 | 普通节点运行、服务/话题处理、动作处理等                              |
| 高级替代 | Executor、spin\_some、spin\_until\_future\_complete |

---

