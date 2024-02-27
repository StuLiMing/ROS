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

`ros2 node list `：打印当前系统的所有 node

`ros2 node info <node_name>`：打印 `node_name` 的信息

`ros2 topic list`：打印当前系统的所有 topic

`ros topic echo <topic_name>`：订阅 `topic_name`

`ros2 topic pub <topic_name>`：发布 `topic_name`

`ros2 bag record <topic_name>`：录制一个 topic。录制完后会在当前工作目录下产生一个文件夹![image-20240227020433435](C:\Users\lm\Desktop\ROS\images\image-20240227020433435.png)

这个文件夹的名称为 `rosbag2_年_月_日_时_分_秒`

`ros2 bag play <bag_path>`：回放一个 topic 的录制结果。这里的 `bag_path` 是文件夹的路径。

`ros2 pkg create --build-type <build-type> <package_name>`：创建功能包，这里 `build-type` 可以取 `ament_cmake`（C++）、`ament_python` （python）等。

## 工作空间与功能包

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

1. 没太懂，可能和代码的定位有关

`setup.cfg`

1. 也没太懂

