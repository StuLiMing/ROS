## 0.1 常用网站

百科：https://wiki.ros.org/

索引站：https://index.ros.org/

问答：https://answers.ros.org/

讨论：https://discourse.ros.org/

官网：https://www.ros.org/

中文教程：https://book.guyuehome.com/

中文论坛：https://www.guyuehome.com/

## 0.2 安装

不能自动补全的话安装这个

```bash
sudo apt-get insall python3-argcomplete  
```

---

VScode 下会用到的扩展：`ROS`、 `Msg Language Support`、`URDF`

## 1 命令行操作

所有命令都以 `ros2` 开头。

`rqt_graph`：可视化界面绘制当前的节点结构（好像有点卡不太好使）

**node**

+ `ros2 node list `：打印当前系统的所有 node

+ `ros2 node info <node_name>`：打印 `node_name` 的信息

**topic**

+ `ros2 topic list`：打印当前系统的所有 topic

+ `ros2 topic info <topic_name> [--verbose]` ：打印 `topic_name` 的信息

  `--verbose` 会输出更详细的信息，包括 QoS 等。

+ `ros2 topic bw <topic_name>`：持续打印 `topic_name` 的带宽

+ `ros topic echo <topic_name>`：订阅 `topic_name` 传输的数据的内容

+ `ros2 topic pub <topic_name> <topic_type> "<date_name1>: <val1>[,...]"` ：发布一个话题： `topic_name`

**service**

+ `ros2 service list`：打印当前系统的所有 service

+ `ros2 service type <service_name>`：打印该服务的接口类型

+ `ros2 service call <service_name> <type> "[<arg_name1:val1>,...]"`：向某个服务发送请求

**interface**

+ `ros2 interface list`：列出所有的 interface
+ `ros2 interface show <interface_name>`：打印 interface_name 的具体定义
+ `ros2 interface package <package_name>`：列出 `package_name` 下的所有 interface 

**action**

+ `ros2 action list`：列出所有动作
+ `ros2 action info <action_name>`：列出动作的详细信息
+ `ros2 action send_goal <action_name> <action_datatype> "{<arg_name1>:cal1,[...]} [--feedback]"`：手动给动作设定一个 `goal` 。 `feed_back` 会打开实时反馈。

**param**

+ `ros2 param list`：列出每个节点的所有参数
+ `ros2 param describe <node_name> <param_name>`：列出某参数的具体信息
+ `ros2 param get <node_name> <param_name>`：获取某参数的具体值
+ `ros2 param set <node_name> <param_name> <val>`：设置参数的值

+ `ros2 parma dump <node_name>`：输出某个节点的所有参数。如果想保存这些参数，就重定向一下

+ `ros2 param load <node_name> <file_name>`：加载参数文件







`ros2 bag record <topic_name>`：录制一个 topic。录制完后会在当前工作目录下产生一个文件夹![image-20240227020433435](.\images\image-20240227020433435.png)

这个文件夹的名称为 `rosbag2_年_月_日_时_分_秒`

`ros2 bag play <bag_path>`：回放一个 topic 的录制结果。这里的 `bag_path` 是文件夹的路径。

`ros2 pkg create --build-type <build-type> <package_name>`：创建功能包，这里 `build-type` 可以取 `ament_cmake`（C++）、`ament_python` （python）等。

## 2 工作空间与功能包

**工作空间**：存放项目开放相关文件的文件夹；是开发过程的大本营

`src`：代码空间

`install`：安装空间

`build`：编译空间

`log`：日志空间

---

**功能包**，即 package。

C++ 版本的功能包会有 `package.xml` 和 `CMakeLists.txt`。

`package.xml` ：

1. package 的基本信息，比如名称版本号描述等。开发者可以手动填写。

2. 描述该功能包的依赖项

 `CMakeLists.txt`：

1. 设置编译规则，指导编译

Python 版本的功能包也会有 `package.xml` ，与 C++ 版本基本一致。Python版本的功能包还会有 `setup.cfg` 和 `setup.py`

`setup.py`

1. 和代码的定位有关

`setup.cfg`

1. 也没太懂

## 3 节点

+ 每个节点独立运行一个可执行文件，可使用不同的编程语言，比如C++、python。

+ 节点可以是分布式的
+ 通过节点名称进行管理

### 3.1 第一个节点 `node_helloworld`

```python
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import time

def main(args=None):                             # ROS2节点主入口main函数
    rclpy.init(args=args)                        # ROS2 Python接口初始化
    node = Node("node_helloworld")               # 创建ROS2节点对象并进行初始化
    
    while rclpy.ok():                            # ROS2系统是否正常运行
        node.get_logger().info("Hello World")    # ROS2日志输出
        time.sleep(0.5)                          # 休眠控制循环时间
    
    node.destroy_node()                          # 销毁节点对象    
    rclpy.shutdown()                             # 关闭ROS2 Python接口
```

+ `rclpy.init(args=args)` ：范式，初始化接口
+ `node=Node("node_helloworld") ` ：范式，创建节点对象。这里的 `node_helloworld` 是节点名称
+ `rclpy.ok()` 会检查 ros2 是否正常运行
+ `node.get_logger.info("Hello World")`：往当前节点的 `logger` 对象上打印信息
+ `node.destroy_node()`：范式：销毁节点
+ `rclpy.shutdown()`：范式：关闭 ros2 python 接口

每个 py 脚本都必须在该功能包下的 `setup.py` 中进行相应的配置

```python
    entry_points={
        'console_scripts': [
         'node_helloworld       = learning_node.node_helloworld:main',
         'node_helloworld_class = learning_node.node_helloworld_class:main',
         'node_object            = learning_node.node_object:main',
         'node_object_webcam     = learning_node.node_object_webcam:main',
        ],
    },
```

以 `node_helloworld` 为例，`learning_node.node_helloworld:main` 意思是 `node_helloworld` 以当前功能包下的 `learning_node` 文件夹下的 `node_helloworld.py` 的 `main` 函数作为入口。

运行该节点

```bash
ros2 run learning_node node_helloworld
```

---

python 下更推荐面向对象的实现方法

```python
class HelloWorldNode(Node):
    def __init__(self, name):
        super().__init__(name)                       # ROS2节点父类初始化
        while rclpy.ok():                            # ROS2系统是否正常运行
            self.get_logger().info("Hello World")    # ROS2日志输出
            time.sleep(0.5)                          # 休眠控制循环时间

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = HelloWorldNode("node_helloworld_class")   # 创建ROS2节点对象并进行初始化
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

### 3.2 常用方法

`rclpy.spin(node)`：该方法用会启动一个循环，这个循环会持续运行，直到节点被明确地关闭或被某个外部事件（例如Ctrl+C信号）中断。

## 4 话题

+ 话题的作用是实现 node A 到 node B 的数据传输。称 A 为**发布者**，B为**订阅者**

+ 每个话题都需要一个名字、特定的数据类型

+ 话题可以多对多

  <img src="C:\Users\lm\Desktop\机器人\ROS\images\image-20240228135221541.png" alt="image-20240228135221541" style="zoom:50%;" />

+ 异步通信，发布者并不知道订阅者是否收到数据

+ 单向

+ 数据在 node 间以**消息**的形式传递。`.msg` 文件定义通信的消息结构

### 4.1 第一个话题 `chatter`

运行示例：

<img src="C:\Users\lm\Desktop\机器人\ROS\images\image-20240228135449712.png" alt="image-20240228135449712" style="zoom:50%;" />

---

`topic_helloworld_pub.py`：

```python
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型

class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.pub = self.create_publisher(String, "chatter", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        
    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        msg = String()                                            # 创建一个String类型的消息对象
        msg.data = 'Hello World'                                  # 填充消息对象中的消息数据
        self.pub.publish(msg)                                     # 发布话题消息
        self.get_logger().info('Publishing: "%s"' % msg.data)     # 输出日志信息，提示已经完成话题发布
        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PublisherNode("topic_helloworld_pub")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

+ 和消息有关的东西定义在 `std_msgs` 里
+ `self.pub = self.create_publisher(String, "chatter", 10) `：创建发布者对象。这里的 `String` 是消息类型，`chatter` 是消息名称，`10` 是缓冲区大小。缓冲区溢出后会丢弃时间戳较老的数据帧。
+ `self.create_timer(0.5, self.timer_callback)` 是 `Node` 类的创建定时器的函数，`0.5` 即每 0.5 秒执行一次回调函数。
+ `msg = String()` ：实例化一个消息对象
+ `msg.data = 'Hello World'`：填充消息对象中的消息数据
+ ` self.pub.publish(msg) `：发布话题消息

---

`topic_helloworld_sub.py`：

```python
import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from std_msgs.msg import String                  # ROS2标准定义的String消息

class SubscriberNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.sub = self.create_subscription(\
            String, "chatter", self.listener_callback, 10)        # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

    def listener_callback(self, msg):                             # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('I heard: "%s"' % msg.data)        # 输出日志信息，提示订阅收到的话题消息
        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = SubscriberNode("topic_helloworld_sub")    # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

+ `self.create_subscription(String, "chatter", self.listener_callback, 10)`：与 `create_publisher` 一个很大的不同是这里可以传回调函数，定义每次收到消息时的动作。
+ 注意话题名 `chatter`,，订阅者通过话题名去定位发布者。

### 4.2 常用方法

**图像处理相关**

+ `from sensor_msgs.msg import Image  `：图像消息类型
+ `from cv_bridge import CvBridge`：ROS 与 OpenCV 图像转换类
+ `CvBridge().cv2_to_imgmsg(frame, 'bgr8')`：将 cv2 读入的图像转为信息的格式，这里参数 `bgr8` 意思是 cv2 图像的通道是 `BRG`，每个像素为 `8bit`，通过 `from cv_bridge import CvBridge ` 导入
+ `CvBridge().imgmsg_to_cv2(data, 'bgr8')`：将 msg 信息格式的图像转为 cv2 的格式。

## 5 服务

+ 服务也是一种通讯方法，可以实现类似于你问我答的通讯效果
+ 服务器唯一，客户端可以不唯一
+ `.srv` 文件定义请求和应答数据结构
+ 有点像计网学的 C/S 模型

### 5.1 第一个服务 `add_two_ints`

`service_adder_server`

```python
import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from learning_interface.srv import AddTwoInts    # 自定义的服务接口

class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)                                                             # ROS2节点父类初始化
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.adder_callback)    # 创建服务器对象（接口类型、服务名、服务器回调函数）

    def adder_callback(self, request, response):                                           # 创建回调函数，执行收到请求后对数据的处理
        response.sum = request.a + request.b                                               # 完成加法求和计算，将结果放到反馈的数据中
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))   # 输出日志信息，提示已经完成加法求和计算
        return response                                                                    # 反馈应答信息

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = adderServer("service_adder_server")       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

```

+ `self.create_service(AddTwoInts, 'add_two_ints', self.adder_callback)`：创建服务器对象，这里的 `AddTwoInts` 是自定义的服务接口，通过 `from learning_interface.srv import AddTwoInts` 导入，`adder_callback` 是收到客户端请求的回调函数
+ `adder_callback` 的编写是一个范式，底层已经帮我们实现好了，这里必须要传参 `request` 和 `response`，函数的最后要 `return response`

---

`service_adder_client`

```python
import sys
import rclpy                                                                      # ROS2 Python接口库
from rclpy.node   import Node                                                     # ROS2 节点类
from learning_interface.srv import AddTwoInts                                     # 自定义的服务接口

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                    # ROS2节点父类初始化
        self.client = self.create_client(AddTwoInts, 'add_two_ints')              # 创建服务客户端对象（服务接口类型，服务名）
        while not self.client.wait_for_service(timeout_sec=1.0):                  # 循环等待服务器端成功启动
            self.get_logger().info('service not available, waiting again...') 
        self.request = AddTwoInts.Request()                                       # 创建服务请求的数据对象
                    
    def send_request(self):                                                       # 创建一个发送服务请求的函数
        self.request.a = int(sys.argv[1])
        self.request.b = int(sys.argv[2])
        self.future = self.client.call_async(self.request)                        # 异步方式发送服务请求

def main(args=None):
    rclpy.init(args=args)                                                         # ROS2 Python接口初始化
    node = adderClient("service_adder_client")                                    # 创建ROS2节点对象并进行初始化
    node.send_request()                                                           # 发送服务请求
    
    while rclpy.ok():                                                             # ROS2系统正常运行
        rclpy.spin_once(node)                                                     # 循环执行一次节点

        if node.future.done():                                                    # 数据是否处理完成
            try:
                response = node.future.result()                                   # 接收服务器端的反馈数据
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(                                           # 将收到的反馈信息打印输出
                    'Result of add_two_ints: for %d + %d = %d' % 
                    (node.request.a, node.request.b, response.sum))
            break
            
    node.destroy_node()                                                           # 销毁节点对象
    rclpy.shutdown()                                                              # 关闭ROS2 Python接口
```

+ `self.create_client(AddTwoInts, 'add_two_ints')`：创建客户端对象。以下列举客户端对象的方法，略去 `self.client.`
  + `wait_for_service(timeout_sec=1.0)`：等待服务器开机。服务开机了就返回 `True`，否则返回 `False`。这里的 `timeout_sec` 应该是阻塞的时间。
  + `future = call_async(args)`：异步方式发送服务请求，这里异步的意思是不会阻塞。
    + `future.done()`：检查服务器是否响应。返回 `bool` 值。
    + `future.result()`：返回的结果

+ `AddTwoInts.Request() `：自定义的类型接口的实例化
+ `rclpy.spin_once(node) `：仅处理当前待处理的事件队列中的一个事件，然后返回。

### 5.2 `.srv` 文件的格式

`GetObjectPosition.srv`

```python
bool get      # 获取目标位置的指令
---
int32 x       # 目标的X坐标
int32 y       # 目标的Y坐标
```

+ 三个横杠分割，上方是客户端请求数据，下方是服务器端的响应数据

## 6 通信接口

+ 通信接口的定义是与语言无关的。

+ 三种接口定义文件的格式：

  <img src="C:\Users\lm\Desktop\机器人\ROS\images\image-20240228182220562.png" alt="image-20240228182220562"  />

  这里数据的不同组别都是用 `---` 分割的。

+ 服务的两个部分分别会被 `ros2` 封装到接口对象的 `Request` 和 `Response`；动作的三个部分会分别被封装到接口对象的 `Goal`，`Result`，`Feedback`

+ 系统中默认定义了一些接口，在 `/opt/ros/humble/share` 下

+ 接口定义文件的注释用 `#` 

+ 基本数据类型：

  + `bool`
  + `int32`，`int64`
  + `float32`
  + `string`

+ 接口定义文件需要在对应文件夹下的 `Cmake` 文件中列出

  ```cmake
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ObjectPosition.msg"
    "srv/AddTwoInts.srv"
    "srv/GetObjectPosition.srv"
    "action/MoveCircle.action"
   )
  ```

## 7 动作

+ 动作是完整行为的流程管理

+ 也使用 C/S 模型，服务器端唯一，客户端可以不唯一。这里客户端是发布命令的，服务器端是执行命令的。

+ 动作是一种应用层的通信机制，其底层是基于话题和服务实现的

  ![image-20240302193015296](C:\Users\lm\Desktop\机器人\ROS\images\image-20240302193015296.png)

+ 动作要执行（物理世界的）一段时间

### 7.1 第一个动作 `move_circle`

**运行示例**

这个动作模拟了 client 请求 server 机器人旋转一周的行为

![image-20240302194251427](C:\Users\lm\Desktop\机器人\ROS\images\image-20240302194251427.png)

**接口定义**

`MoveCircle.action`

```python
bool enable     # 定义动作的目标，表示动作开始的指令
---
bool finish     # 定义动作的结果，表示是否成功执行
---
int32 state     # 定义动作的反馈，表示当前执行到的位置
```

+ `enale=True` 表示旋转，`enable=False` 表示不旋转
+ `finisi=True` 表示运动成功，`finish=True` 表示运动不成功
+ `state` 表示当前状态的信息（角度）

**服务器端**

`action_move_server.py`

```python
import time
import rclpy                                      # ROS2 Python接口库
from rclpy.node   import Node                     # ROS2 节点类
from rclpy.action import ActionServer             # ROS2 动作服务器类
from learning_interface.action import MoveCircle  # 自定义的圆周运动接口

class MoveCircleActionServer(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化
        self._action_server = ActionServer(      # 创建动作服务器（接口类型、动作名、回调函数）
            self,
            MoveCircle,
            'move_circle',
            self.execute_callback)

    def execute_callback(self, goal_handle):            # 执行收到动作目标之后的处理函数
        self.get_logger().info('Moving circle...')
        feedback_msg = MoveCircle.Feedback()            # 创建一个动作反馈信息的消息

        for i in range(0, 360, 30):                     # 从0到360度，执行圆周运动，并周期反馈信息
            feedback_msg.state = i                      # 创建反馈信息，表示当前执行到的角度
            self.get_logger().info('Publishing feedback: %d' % feedback_msg.state)
            goal_handle.publish_feedback(feedback_msg)  # 发布反馈信息
            time.sleep(0.5)

        goal_handle.succeed()                           # 动作执行成功
        result = MoveCircle.Result()                    # 创建结果消息
        result.finish = True                            
        return result                                   # 反馈最终动作执行的结果

def main(args=None):                                       # ROS2节点主入口main函数
    rclpy.init(args=args)                                  # ROS2 Python接口初始化
    node = MoveCircleActionServer("action_move_server")    # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                       # 循环等待ROS2退出
    node.destroy_node()                                    # 销毁节点对象
    rclpy.shutdown()                                       # 关闭ROS2 Python接口
```

+ `self._action_server = ActionServer(self,MoveCircle,'move_circle',self.execute_callback)`。这里的 `ActionServer` 来自  `from rclpy.action import ActionServer `。这里第一个参数 `self` 是当前 `node`，第二个参数 `MoveCircle` 是接口定义，第三个参数 `move_circle` 是动作的名称。`self.execute_callback` 是回调函数名，当动作服务器接收到一个新的动作目标（goal）请求时回调函数会被执行。
+ 这里着重注意 `execute_callback` 的参数 `goal_handle`，这个参数是由 ROS2 动作服务器框架自动传递给回调函数的，代表当前正在处理的 goal。当客户端向动作服务器发送一个动作请求时，动作服务器会创建一个`goal_handle`对象，并将其作为参数传递给`execute_callback`函数。
  + `goal_handle.publish_feedback(feedback_msg)`：发布动作执行过程中的 feedback 给客户端。
  + `goal_handle.succeed()`：标记动作成功完成
  + 这里 `return result` 也是一个范式。

**客户端**

`action_move_client.py`

```python
import rclpy                                      # ROS2 Python接口库
from rclpy.node   import Node                     # ROS2 节点类
from rclpy.action import ActionClient             # ROS2 动作客户端类

from learning_interface.action import MoveCircle  # 自定义的圆周运动接口

class MoveCircleActionClient(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化
        self._action_client = ActionClient(      # 创建动作客户端（接口类型、动作名）
            self, MoveCircle, 'move_circle') 

    def send_goal(self, enable):                 # 创建一个发送动作目标的函数
        goal_msg = MoveCircle.Goal()             # 创建一个动作目标的消息
        goal_msg.enable = enable                 # 设置动作目标为使能，希望机器人开始运动

        self._action_client.wait_for_server()    # 等待动作的服务器端启动
        self._send_goal_future = self._action_client.send_goal_async(   # 异步方式发送动作的目标
            goal_msg,                                                   # 动作目标
            feedback_callback=self.feedback_callback)                   # 处理周期反馈消息的回调函数
                          
        self._send_goal_future.add_done_callback(self.goal_response_callback) # 设置一个服务器收到目标之后反馈时的回调函数

    def goal_response_callback(self, future):           # 创建一个服务器收到目标之后反馈时的回调函数
        goal_handle = future.result()                   # 接收动作的结果
        if not goal_handle.accepted:                    # 如果动作被拒绝执行
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')                            # 动作被顺利执行

        self._get_result_future = goal_handle.get_result_async()              # 异步获取动作最终执行的结果反馈
        self._get_result_future.add_done_callback(self.get_result_callback)   # 设置一个收到最终结果的回调函数 

    def get_result_callback(self, future):                                    # 创建一个收到最终结果的回调函数
        result = future.result().result                                       # 读取动作执行的结果
        self.get_logger().info('Result: {%d}' % result.finish)                # 日志输出执行结果

    def feedback_callback(self, feedback_msg):                                # 创建处理周期反馈消息的回调函数
        feedback = feedback_msg.feedback                                      # 读取反馈的数据
        self.get_logger().info('Received feedback: {%d}' % feedback.state) 

def main(args=None):                                       # ROS2节点主入口main函数
    rclpy.init(args=args)                                  # ROS2 Python接口初始化
    node = MoveCircleActionClient("action_move_client")    # 创建ROS2节点对象并进行初始化
    node.send_goal(True)                                   # 发送动作目标
    rclpy.spin(node)                                       # 循环等待ROS2退出
    node.destroy_node()                                    # 销毁节点对象
    rclpy.shutdown()                                       # 关闭ROS2 Python接口

```

+ `self._action_client = ActionClient(self, MoveCircle, 'move_circle') `：创建 action 的客户端，和服务器端类似，但是不需要回调函数。
  + `wait_for_server()` 和 service 的 `wait_for_server()` 类似
  + `send_goal_async(goal_msg, feedback_callback=self.feedback_callback)      `：异步地发送 `goal_msg`，异步即不阻塞。这里的 `feedback_callback` 是收到 feed_back 时触发的回调函数。当 `feedback_callback` 被触发时， ros2 自动创建一个 `feedback_msg` 对象传入，该对象的 `feedback` 属性即符合接口定义的 feedback 数据。
    + `add_done_callback()` 为 `future` 对象添加服务器响应 goal 时的回调函数
+ `future.result() ` 是 goal 被服务器接收后服务器返回的结果
  + `accepted` 表示服务器接受动作
  + `get_result_async() `：异步地等待动作最终执行的 result 的反馈
    + `add_done_callback(self.get_result_callback)`：设置一个收到最终结果的回调函数

## 8 参数

+ 参数是机器人的全局字典（键值对）

### 8.1 第一个参数 `robot_name`

`param_declare.py`

```python
 import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类

class ParameterNode(Node):
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.timer = self.create_timer(2, self.timer_callback)    # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.declare_parameter('robot_name', 'mbot')              # 创建一个参数，并设置参数的默认值

    def timer_callback(self):                                      # 创建定时器周期执行的回调函数
        robot_name_param = self.get_parameter('robot_name').get_parameter_value().string_value   # 从ROS2系统中读取参数的值

        self.get_logger().info('Hello %s!' % robot_name_param)     # 输出日志信息，打印读取到的参数值

        new_name_param = rclpy.parameter.Parameter('robot_name',   # 重新将参数值设置为指定值
                            rclpy.Parameter.Type.STRING, 'mbot')
        all_new_parameters = [new_name_param]
        self.set_parameters(all_new_parameters)                    # 将重新创建的参数列表发送给ROS2系统

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ParameterNode("param_declare")            # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

```

+ `self.declare_parameter('robot_name', 'mbot')`：创建参数。参数名为 `robot_name`，参数值为 `mbot`
+ `self.get_parameter('robot_name').get_parameter_value().string_value`：`.get_papameter('robot_name')`，获得参数对象；`.get_paprmeter_value()`，获得参数对象的具体值；`.string_value`，获得值的字符串形式
+ `new_name_param = rclpy.parameter.Parameter('robot_name',rclpy.Parameter.Type.STRING, 'mbot')`：创建 parameter 对象，三个参数分别为参数名，类型，值。
+ 当调用`self.set_parameters()`时，它会遍历`all_new_parameters`列表中的每个`Parameter`对象，并尝试将每个参数的值更新为列表中指定的新值。

## 9 分布式通信

处于同一个网络下的设备就能互相通讯，非常神奇。

**分组机制**：在 Ubuntu 下只要在 `~/.bashrc` 下添加环境变量 `export ROS_DOMAIN_ID=<ID>` 就可以设置分组号了。同一个分组下的设备才能互相通信。

## 10 DDS

DDS，Data Distribution Service，即数据分发服务。专门为实时系统设计的数据分发/订阅标准。DDS 是一种数据通信的标准，按照 DDS 标准实现的数据通信系统可以有很多（来自不同企业或组织），为了实现对多个 DDS 的兼容， ros2 设计了一个 ros middleware，兼容性问题由 DDS 厂商解决

<img src="C:\Users\lm\Desktop\机器人\ROS\images\image-20240303202646189.png" alt="image-20240303202646189" style="zoom: 67%;" />

DDS 支持**质量服务策略（QoS）**，应用程序指定所需要的网络质量的行为，QoS 去实现这种要求。

---

可以使用 ros2 命令行配置 DDS 参数：

`ros2 topic pub /chatter std_msgs/msg/Int32 "<yaml format data>" --qos-reliability best_effort `

这里发布方和订阅方采用不同的通信模式是无法通信的。

---

通过代码配置 QoS：

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # ROS2 QoS类

qos_profile = QoSProfile(                                 # 创建一个QoS原则
    # reliability=QoSReliabilityPolicy.BEST_EFFORT,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
self.pub = self.create_publisher(String, "chatter", qos_profile)
```

订阅者类似，但发布者和订阅者的配置需要匹配。

## 11 Launch：多节点启动与配置脚本

所有节点一个一个启动太麻烦了，Launch 要解决的就是这一痛点。Launch 文件基于 python 描述。

一个示例：

```python
from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类

def generate_launch_description():             # 自动生成launch文件的函数
    return LaunchDescription([                 # 返回launch文件的描述信息
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_helloworld_pub', # 节点的可执行文件
        ),
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_helloworld_sub', # 节点的可执行文件名
        ),
    ])
```

---

另一个启动 `rviz2 ` 的示例

```python
import os
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法
from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类

def generate_launch_description():      # 自动生成launch文件的函数
   rviz_config = os.path.join(          # 找到配置文件的完整路径
      get_package_share_directory('learning_launch'),
      'turtle_rviz.rviz'
      )

   return LaunchDescription([           # 返回launch文件的描述信息
      Node(                             # 配置一个节点的启动
         package='rviz2',               # 节点所在的功能包
         executable='rviz2',            # 节点的可执行文件名
         name='rviz2',                  # 对节点重新命名
         arguments=['-d', rviz_config]  # 加载命令行参数
      )
   ])

```

+ `name` 参数可以将 node 重命名
+ `argumrnts` 参数可以加载命令的参数

---

另一种区分同名节点的方法是增加命名空间参数 `namespace` 如果给 node1 增加了命名空间 namespace1，它的名字就会变成 namespace1/node1

---

`remapping`参数在`launch`文件中用于重新映射（重命名）话题、服务或参数的名称。这是ROS中一个重要的功能，因为它允许开发者或用户在不修改节点内部代码的情况下，改变节点间通信的接口名称。

- **话题重映射**：允许你将一个节点发布或订阅的话题名称改为另一个名称。这在多个节点需要共享数据但使用了不同话题名称的情况下非常有用。
- **服务重映射**：类似于话题，允许将节点提供或使用的服务名称改为另一个名称。
- **参数重映射**：允许改变节点获取参数值时使用的参数名称。

---

以下是一个通过 launch 设置 parameter 的例子：

```python
from launch import LaunchDescription                   # launch文件的描述类
from launch.actions import DeclareLaunchArgument       # 声明launch文件内使用的Argument类
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node                    # 节点启动的描述类

def generate_launch_description():                     # 自动生成launch文件的函数
   background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')     # 创建一个Launch文件内参数（arg）background_r
   )
   background_g_launch_arg = DeclareLaunchArgument(
      'background_g', default_value=TextSubstitution(text='84')    # 创建一个Launch文件内参数（arg）background_g
   )
   background_b_launch_arg = DeclareLaunchArgument(
      'background_b', default_value=TextSubstitution(text='122')   # 创建一个Launch文件内参数（arg）background_b
   )

   return LaunchDescription([                                      # 返回launch文件的描述信息
      background_r_launch_arg,                                     # 调用以上创建的参数（arg）
      background_g_launch_arg,
      background_b_launch_arg,
      Node(                                                        # 配置一个节点的启动
         package='turtlesim',
         executable='turtlesim_node',                              # 节点所在的功能包
         name='sim',                                               # 对节点重新命名
         parameters=[{                                             # ROS参数列表
            'background_r': LaunchConfiguration('background_r'),   # 创建参数background_r
            'background_g': LaunchConfiguration('background_g'),   # 创建参数background_g
            'background_b': LaunchConfiguration('background_b'),   # 创建参数background_b
         }]
      ),
   ])

```

感觉这个还是挺麻烦的。

---

以下是通过 `yaml` 文件加载参数的例子：

```python
import os
from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch import LaunchDescription   # launch文件的描述类
from launch_ros.actions import Node    # 节点启动的描述类


def generate_launch_description():     # 自动生成launch文件的函数
   config = os.path.join(              # 找到参数文件的完整路径
      get_package_share_directory('learning_launch'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([          # 返回launch文件的描述信息
      Node(                            # 配置一个节点的启动
         package='turtlesim',          # 节点所在的功能包
         executable='turtlesim_node',  # 节点的可执行文件名
         namespace='turtlesim2',       # 节点所在的命名空间
         name='sim',                   # 对节点重新命名
         parameters=[config]           # 加载参数文件
      )
   ])

```

yaml 文件：

```yaml
/turtlesim2/sim:
   ros__parameters:
      background_b: 0
      background_g: 0
      background_r: 0
```

---

可以通过一个 launch 文件包含另外的 launch 文件。

```python
import os
from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法
from launch import LaunchDescription                 # launch文件的描述类
from launch.actions import IncludeLaunchDescription  # 节点启动的描述类
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction               # launch文件中的执行动作
from launch_ros.actions import PushRosNamespace      # ROS命名空间配置

def generate_launch_description():                   # 自动生成launch文件的函数
   parameter_yaml = IncludeLaunchDescription(        # 包含指定路径下的另外一个launch文件
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('learning_launch'), 'launch'),
         '/parameters_nonamespace.launch.py'])
      )
  
   parameter_yaml_with_namespace = GroupAction(      # 对指定launch文件中启动的功能加上命名空间
      actions=[
         PushRosNamespace('turtlesim2'),
         parameter_yaml]
      )

   return LaunchDescription([                        # 返回launch文件的描述信息
      parameter_yaml_with_namespace
   ])
```

一般需要给导入的 launch 文件加一个新的命名空间防止命名冲突。

---

`setup.py` 的配置：

```python
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
    ],
```



## 工具

`colcon build`：编译。在工作空间执行命令

`sudo apt install ros-humble-usb-cam`：标准相机驱动。通过 `ros2 run usb_cam usb_cam_node_exe ` 创建一个相机 node，名为 `usb_cam`，这个 `node` 通过 `image_raw` 话题发布图像
