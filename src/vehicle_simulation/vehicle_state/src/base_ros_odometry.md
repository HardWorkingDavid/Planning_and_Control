参考文档： [navigationTutorialsRobotSetupOdom](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom) 

参考博客：

（1）[ROS机器人里程计模型](https://blog.csdn.net/qq_38156743/article/details/124183286)

（2）[ROS里程计消息nav_msgs/Odometry的可视化方法](https://blog.csdn.net/showing1998/article/details/121764968?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169866865816800225589249%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169866865816800225589249&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-4-121764968-null-null.142%5Ev96%5Epc_search_result_base4&utm_term=path_3d&spm=1018.2226.3001.4187)

###  1 常用坐标系系统模型
世界坐标系是描述机器人全局信息的坐标系；机器人坐标系是描述机器人自身信息的坐标系；传感器坐标系是描述传感器信息的坐标系。如下图所示坐标系之间的关系。

![在这里插入图片描述](https://img-blog.csdnimg.cn/c9a5c56ae1bd4bc6b46b4235caa01497.png)
>**世界坐标系是固定不变的，机器人坐标系和传感器坐标系是在世界坐标系下描述的**。机器人坐标系和传感器坐标系原点重合但是**存在一定的角度**，不同的机器人坐标系关系是不同的。当我们使用传感器数据时，这些坐标系间的关系就是我们变换矩阵的参数，因为**传感器的数据必定是要变换到机器人坐标系或者世界坐标系中使用的**


### 2 移动机器人位姿模型

移动机器人的位姿模型就是**机器人在世界坐标系下的状态**。常用随机变量$X_t =(x_t ,y_t ,θ_t )$来描述 $t$ 时刻的机器人在世界坐标系下的状态，简称**位姿**。

![在这里插入图片描述](https://img-blog.csdnimg.cn/0e340f52fe45491f826522bc5a69e98b.png)

### 3 移动机器人里程计的计算
移动机器人的里程计就是**机器人每时每刻在世界坐标系下位姿状态**。

里程计的计算是指以机器人上电时刻为世界坐标系的起点（机器人的航向角是世界坐标系X正方向）开始累积计算任意时刻机器人在世界坐标系下的位姿。通常计算里程计方法是速度积分推算：通过左右电机的编码器测得机器人的左右轮的速度VL和VR，**在一个短的时刻△t内，认为机器人是匀速运动**，并且**根据上一时刻机器人的`航向角`计算得出机器人在该时刻内世界坐标系上X和Y轴的增量**，然后**将增量进行累加处理**，关于**航向角θ采用的IMU的yaw值**。然后根据以上描述即可得到机器人的里程计。

![在这里插入图片描述](https://img-blog.csdnimg.cn/29489e3abd5c487690ebcc6e9ca97fc3.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/116fe67a209f4d36878d8c058ca301a6.png)

$tan(\delta)=\frac{L}{R}$（1）

$v=wR$（2）

由(1)(2)得

$w=\frac{v}{R}=\frac{vtan(\delta)}{L}$

$\theta=w*dt$


```cpp
  //在给定机器人速度的情况下，以一种典型的方式计算里程计
  31     double dt = (current_time - last_time).toSec();
  32     double delta_x = v * cos(th) * dt;
  33     double delta_y = v * sin(th) * dt;
  34     double delta_th = (v * tan(steer) / wheel_base) * dt;
  35
  36     x += delta_x;
  37     y += delta_y;
  38     th += delta_th;
```


### 4 示例代码
示例代码，用于通过ROS发布。`nav_msgs/Odometry`消息，并使用tf对只在圆圈中行驶的机器人进行转换。我们将首先展示整个代码，并在下面进行逐段解释。
```cpp
   1 #include <ros/ros.h>
   2 #include <tf/transform_broadcaster.h>
   3 #include <nav_msgs/Odometry.h>
   4 
   5 int main(int argc, char** argv){
   6   ros::init(argc, argv, "odometry_publisher");
   7 
   8   ros::NodeHandle n;
   9   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  10   tf::TransformBroadcaster odom_broadcaster;
  11 
  12   double x = 0.0;
  13   double y = 0.0;
  14   double th = 0.0;
  15 
  16   double vx = 0.1;
  17   double vy = -0.1;
  18   double vth = 0.1;
  19 
  20   ros::Time current_time, last_time;
  21   current_time = ros::Time::now();
  22   last_time = ros::Time::now();
  23 
  24   ros::Rate r(1.0);
  25   while(n.ok()){
  26 
  27     ros::spinOnce();               // 检查传入消息
  28     current_time = ros::Time::now();
  29 
  30     //在给定机器人速度的情况下，以一种典型的方式计算里程计
  31     double dt = (current_time - last_time).toSec();
  32     double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  33     double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  34     double delta_th = vth * dt;
  35 
  36     x += delta_x;
  37     y += delta_y;
  38     th += delta_th;
  39 
  40     //由于所有里程计都是6DOF，我们需要一个由yaw创建的四元数
  41     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  42 
  43     // 首先，我们将通过tf发布转换
  44     geometry_msgs::TransformStamped odom_trans;
  45     odom_trans.header.stamp = current_time;
  46     odom_trans.header.frame_id = "odom";
  47     odom_trans.child_frame_id = "base_link";
  48 
  49     odom_trans.transform.translation.x = x;
  50     odom_trans.transform.translation.y = y;
  51     odom_trans.transform.translation.z = 0.0;
  52     odom_trans.transform.rotation = odom_quat;
  53 
  54     // 发送转换
  55     odom_broadcaster.sendTransform(odom_trans);
  56 
  57     //接下来，我们将通过ROS发布里程计消息
  58     nav_msgs::Odometry odom;
  59     odom.header.stamp = current_time;
  60     odom.header.frame_id = "odom";
  61 
  62     //设置位置
  63     odom.pose.pose.position.x = x;
  64     odom.pose.pose.position.y = y;
  65     odom.pose.pose.position.z = 0.0;
  66     odom.pose.pose.orientation = odom_quat;
  67 
  68     //设置速度
  69     odom.child_frame_id = "base_link";
  70     odom.twist.twist.linear.x = vx;
  71     odom.twist.twist.linear.y = vy;
  72     odom.twist.twist.angular.z = vth;
  73 
  74     //发布消息
  75     odom_pub.publish(odom);
  76 
  77     last_time = current_time;
  78     r.sleep();
  79   }
  80 }
```
（1）我们需要实现“odom”参考系到“base_link”参考系的变换，以及nav_msgs/Odometry消息的发布，所以首先需要包含相关的头文件。

```cpp
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry>
```
（2）定义一个消息发布者来发布“odom”消息，在定义一个tf广播，来发布tf变换信息

```cpp
ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
tf::TransformBroadcaster odom_broadcaster;
```
（3）我们假设机器人最初从“odom”坐标系的原点开始。

```cpp
double x = 0.0;
double y = 0.0;
double th = 0.0;
```
（4）我们设置机器人的默认前进速度，让机器人的base_link参考系在odom参考系下以x轴方向0.1m/s，Y轴速度-0.1m/s，角速度0.1rad/s的状态移动，这种状态下，可以让机器人保持圆周运动。

```cpp
double vx = 0.1;
double vy = -0.1;
double vth = 0.1;
```
（5）在这个例子中，我们将以1Hz的速率发布里程计信息，以便于内省，大多数系统都希望以更高的速率发布里程计。

```cpp
ros::Rate r(1.0);
```
（6）使用我们设置的速度信息，来计算并更新里程计的信息，包括单位时间内机器人在x轴、y轴的坐标变化和角度的变化。在实际系统中，需要更具里程计的实际信息进行更新。

```cpp
  31     double dt = (current_time - last_time).toSec();
  32     double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  33     double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  34     double delta_th = vth * dt;
  35 
  36     x += delta_x;
  37     y += delta_y;
  38     th += delta_th;

```
（7）为了兼容二维和三维的功能包，让消息结构更加通用，里程计的偏航角需要转换成四元数才能发布，辛运的是，ROS为我们提供了偏航角与四元数相互转换的功能。

```cpp
  41    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th)
```
（8） 创建一个tf发布需要使用的TransformStamped类型消息，然后根据消息结构填充当前的时间戳、参考系id、子参考系id，注意两个参考系的id必须要是“odom”和“base_link”。

```cpp
  44     geometry_msgs::TransformStamped odom_trans;
  45     odom_trans.header.stamp = current_time;
  46     odom_trans.header.frame_id = "odom";
  47     odom_trans.child_frame_id = "base_link";
```
（9）在这里，我们从里程计数据中填写转换消息，然后使用TransformBroadcaster发送转换。

```cpp
  49     odom_trans.transform.translation.x = x;
  50     odom_trans.transform.translation.y = y;
  51     odom_trans.transform.translation.z = 0.0;
  52     odom_trans.transform.rotation = odom_quat;
  53 
  54     //send the transform
  55     odom_broadcaster.sendTransform(odom_trans);
```
（10）别忘了，我们还要发布`nav_msgs/Odometry`消息，让导航包获取机器人的速度。创建消息变量，然后填充时间戳。

```cpp
  57     //next, we'll publish the odometry message over ROS
  58     nav_msgs::Odometry odom;
  59     odom.header.stamp = current_time;
  60     odom.header.frame_id = "odom";
```
（11） 填充机器人的位置、速度，然后发布消息。我们将消息的child_frame_id设置为“base_link”帧，因为这是我们发送速度信息的坐标帧。

```cpp
  62     //set the position
  63     odom.pose.pose.position.x = x;
  64     odom.pose.pose.position.y = y;
  65     odom.pose.pose.position.z = 0.0;
  66     odom.pose.pose.orientation = odom_quat;
  67 
  68     //set the velocity
  69     odom.child_frame_id = "base_link";
  70     odom.twist.twist.linear.x = vx;
  71     odom.twist.twist.linear.y = vy;
  72     odom.twist.twist.angular.z = vth;
```
### 5 ROS里程计消息nav_msgs/Odometry的可视化方法
参考大佬的博客：[ROS里程计消息nav_msgs/Odometry的可视化方法](https://blog.csdn.net/showing1998/article/details/121764968?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169866865816800225589249%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169866865816800225589249&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-4-121764968-null-null.142%5Ev96%5Epc_search_result_base4&utm_term=path_3d&spm=1018.2226.3001.4187)
![在这里插入图片描述](https://img-blog.csdnimg.cn/5e9fe65ebb8c48118b0b43eacd464528.png)里程计消息中的pose包含了位置pose.position和姿态pose.orientation
`可视化方法`：

（1）在一个节点中订阅发布的里程计话题消息`nav_msgs/Odometry`。

（2）创建`geometry_msgs::PoseStamped`对象接收里程计的位姿。

（3）创建`nav_msgs/Path`对象作为容器，将赋值后的对象push_back进`nav_msgs/Path`中并发布。

然后即可在rviz中订阅包含`nav_msgs/Path`的话题并可视化轨迹。

（1）编写消息收发节点文件path_3d.cpp

```cpp
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
 
nav_msgs::Path  path; // 创建nav_msgs/Path对象
ros::Publisher  path_pub;
 
void pathCallback(const nav_msgs::Odometry::ConstPtr& odom_3d)
{
    geometry_msgs::PoseStamped position_3d;
    position_3d.pose.position.x = odom_3d->pose.pose.position.x; 
    position_3d.pose.position.y = odom_3d->pose.pose.position.y; 
    position_3d.pose.position.z = odom_3d->pose.pose.position.z;
    position_3d.pose.orientation = odom_3d->pose.pose.orientation;
 
 
    position_3d.header.stamp = odom_3d->header.stamp;
    position_3d.header.frame_id = "map";
 
    path.poses.push_back(position_3d);
    path.header.stamp = position_3d.header.stamp;
    path.header.frame_id = "map";
    path_pub.publish(path);
  
    std::cout << odom_3d -> header.stamp << ' ' << odom_3d->pose.pose.position.x << ' ' << odom_3d->pose.pose.position.y << ' ' << odom_3d->pose.pose.position.z << std::endl;
}
 
int main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");
    ros::NodeHandle ph;
 
    path_pub = ph.advertise<nav_msgs::Path>("odom3d_path", 10, true);
    ros::Subscriber odomSub = ph.subscribe<nav_msgs::Odometry>("/odometry_3d", 10, pathCallback);  //订阅里程计话题信息,其中"/odometry_3d"是自己发布的里程计话题名，别忘了修改
    
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```
（2）CMakeLists.txt如下

```bash
cmake_minimum_required(VERSION 2.8.3)
project(path_3d)
 
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
 
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)
 
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs
 )
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_3d
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(path_3d src/path_3d.cpp) #${PROJECT_NAME}_node
target_link_libraries(path_3d ${catkin_LIBRARIES}) # ${PROJECT_NAME}_node
add_dependencies(path_3d beginner_tutorials_generate_messages_cpp) #path_3d_node
```