# ROS2常见函数介绍
## CPP中的常见函数
### rclcpp::init函数
`rclcpp::init()` 是 ROS 2 C++ 编程中最基础也是最关键的初始化函数之一，**用于初始化整个 ROS 2 客户端运行环境**。任何 ROS 2 节点的运行都必须先调用它。

---

## 函数定义

```cpp
namespace rclcpp
{
  void init(int argc, char * argv[], InitOptions options = InitOptions());
}
```

---

## 参数详解

| 参数            | 类型                    | 说明                                  |
| ------------- | --------------------- | ----------------------------------- |
| `argc`        | `int`                 | 命令行参数数量（通常从 `main()` 传入）            |
| `argv`        | `char*[]`             | 命令行参数数组（同样从 `main()` 传入）            |
| `options`（可选） | `rclcpp::InitOptions` | ROS2 初始化选项，例如设置信号处理器、上下文等，通常使用默认值即可 |

> 👉 其实和传统 `main(int argc, char** argv)` 一样，把命令行参数交给 ROS 来解析它自己的参数（比如 `__node:=my_node`、`__log:=...`）

---

## 函数作用

* 初始化 ROS 2 底层通信框架（`rcl`）
* 注册信号处理器（比如 Ctrl+C 终止）
* 解析命令行参数（提取 ROS 2 自定义参数）
* 为后续的节点创建、话题订阅、发布等行为准备环境

---

## 常见使用场景

在 C++ 的 ROS2 程序中，必须首先调用它。以下是一个最小的完整例子：

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    // 初始化 ROS2 环境
    rclcpp::init(argc, argv);

    // 创建节点并运行
    auto node = rclcpp::Node::make_shared("my_node");
    rclcpp::spin(node);

    // 释放资源
    rclcpp::shutdown();
    return 0;
}
```

---

## 和 `rclcpp::shutdown()` 搭配使用

* `rclcpp::init()` 用于初始化
* `rclcpp::shutdown()` 用于清理资源（比如退出时终止所有节点）

---

## 注意事项

* 多次调用 `rclcpp::init()` 是 **无效的**，只能调用一次

* 在 ROS2 中，每个程序（`main()`）必须以 `init()` 开始，以 `shutdown()` 结束
* 如果不调用 `rclcpp::init()`，则 `Node` 创建和其他 ROS 功能都会失败

###  为什么说多次调用 `rclcpp::init()` 是**无效的**？

#### 原因：

`rclcpp::init()` 的**初始化操作是全局的**，它只会在程序中**第一次调用时生效**，后续调用将被忽略。

ROS 2 在内部维护了一个全局的上下文对象（默认是 `rclcpp::contexts::get_global_default_context()`），它的状态一旦被标记为“已初始化”，**再调用 `init()` 就不会做任何事情了**。

#### 举例说明：

```cpp
rclcpp::init(argc, argv); // 第一次调用，有效
rclcpp::init(argc, argv); // 第二次调用，**无效**
```



#### 多次调用可能会出现的问题

* **不会重新初始化**：上下文已经处于 initialized 状态，`rclcpp` 会忽略重复调用。
* **会误导开发者**：以为每次调用都能带来不同初始化效果，其实都被忽略了。
* **不允许 shutdown 后再 init**：一旦 `rclcpp::shutdown()` 被调用，整个运行时就结束了，不能再调用 `init()` 重启系统（除非使用手动上下文）。

---

## 高级用法 —— 使用自定义上下文 `rclcpp::Context`

希望对**多个上下文环境进行控制**，例如在测试、插件、并行运行多个 ROS 实例。

### 代码示例：

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    auto context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    context->init(argc, argv, init_options);  // 初始化该上下文

    auto node = std::make_shared<rclcpp::Node>("my_node", rclcpp::NodeOptions().context(context));
    rclcpp::spin(node);  // 使用该上下文的 node

    context->shutdown(); // 清理该上下文
    return 0;
}
```

---

## 应用场景

| 场景    | 说明                                  |
| ----- | ----------------------------------- |
| 多实例运行 | 你可以用多个 `Context` 同时运行多个 ROS2 节点，不干扰 |
| 插件管理  | 加载插件时单独维护一个上下文，卸载不影响主进程             |
| 测试框架  | 每个测试用例用独立上下文，确保环境干净                 |

---

## 注意事项

* 如果用自定义 `Context`，**所有节点必须显式指定 context**。
* 一旦使用了 `context->shutdown()`，就不能再使用这个上下文创建节点。

---

## 总结

| 场景   | 推荐做法                                             |
| ---- | ------------------------------------------------ |
| 一般项目 | 使用 `rclcpp::init()` + `rclcpp::shutdown()`，不多次调用 |
| 高级应用 | 用 `rclcpp::Context` 创建多个独立运行环境，例如在测试、插件、并行运行多个 ROS 实例。                   |

---

