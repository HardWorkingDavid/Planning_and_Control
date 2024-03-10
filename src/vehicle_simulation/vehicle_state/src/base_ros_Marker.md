参考博客：
（1）[ROS之rviz显示历史运动轨迹、路径的各种方法（visualization_msgs/Marker、nav_msgs/Path）](https://blog.csdn.net/u013834525/article/details/80447931?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromBaidu~Rate-1-80447931-blog-80483802.235%5Ev38%5Epc_relevant_sort_base3&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromBaidu~Rate-1-80447931-blog-80483802.235%5Ev38%5Epc_relevant_sort_base3&utm_relevant_index=2)

（2）[ROS笔记之visualization_msgs-Marker学习](https://blog.csdn.net/weixin_43297891/article/details/134046681?ops_request_misc=&request_id=&biz_id=102&utm_term=visualization_msgs::Marker&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-134046681.142%5Ev96%5Epc_search_result_base4&spm=1018.2226.3001.4187)

（3）[基于ROS发布里程计信息](https://blog.csdn.net/BigDavid123/article/details/134127952?spm=1001.2014.3001.5502)

参考文档：

（1） [rvizDisplayTypesMarker](http://wiki.ros.org/rviz/DisplayTypes/Marker) 

（2） [rvizTutorialsMarkers: Basic Shapes](http://wiki.ros.org/rviz/Tutorials/Markers:%20Basic%20Shapes) 

### 0 前言
（1）在ROS中，`odom`、`base_link`和`map`是常见的坐标系（frame）用于机器人定位和导航。

>- `odom`坐标系是机器人的里程计坐标系，相对于起始位置的运动。
>- `base_link`坐标系是机器人的本体坐标系，与机器人的移动无关。
>- `map`坐标系是机器人所在的全局地图坐标系，提供了固定的参考框架用于定位和导航机器人在全局环境中的位置。

（2）`visualization_msgs::MarkerArray`是ROS中的消息类型，用于在RViz中发布多个Marker的数组
`visualization_msgs::MarkerArray`消息由以下字段组成：

>markers（`std::vector<visualization_msgs::Marker>`）：包含多个`visualization_msgs::Marker`对象的数组。每个Marker对象都描述了一个可视化元素，如`点`、`线`、`箭头`、`文本`等。
>
通过使用`visualization_msgs::MarkerArray`消息，您可以同时**发布多个Marker**，并在RViz中以数组的形式显示它们。
（3）`visualization_msgs/Marker`消息的一些重要字段
>- **header**：消息的头部信息，包括frame_id和timestamp等。
>- ns：命名空间，用于将Marker分组。
>- **id**：Marker的唯一标识符，用于标识不同的Marker。
>- **type**：Marker的类型，指定要显示的形状。可以使用visualization_msgs/Marker消息定义的常量来设置，如visualization_msgs::Marker::SPHERE表示球体。
>- **action**：Marker的操作类型，指定在RViz中如何处理该Marker。常见的- 操作类型包括ADD、DELETE和DELETEALL。
>- **pose**：Marker的位姿，指定Marker在三维空间中的位置和方向。
> - **scale**：Marker的尺寸，用于控制Marker的大小或线宽等属性。
> - **color**：Marker的颜色，可以设置RGBA值来指定颜色和不透明度。
> - points：用于线条和多边形等形状的点列表。每个点由geometry_msgs/- Point消息表示。
> - text：用于文本形状的字符串内容。
> - **lifetime**：Marker的生命周期，用于控制Marker在RViz中的显示时间。

（4）各种标志物的类型
可以在[rvizDisplayTypesMarker](http://wiki.ros.org/rviz/DisplayTypes/Marker)查看
下面列举一些常用的：

```bash
ARROW=0//箭头
CUBE=1//立方体
SPHERE=2//球
CYLINDER=3//圆柱体
LINE_STRIP=4//线条（点的连线）
LINE_LIST=5//线条序列
CUBE_LIST=6//立方体序列
SPHERE_LIST=7//球序列
POINTS=8//点集
TEXT_VIEW_FACING=9//显示3D的文字
MESH_RESOURCE=10//网格
TRIANGLE_LIST=11//三角形序列
```
### 1 实现在rviz中画出圆形轨迹
参考大佬的博客：[ROS在rviz中实时显示轨迹（nav_msgs/Path消息的使用）](https://blog.csdn.net/ktigerhero3/article/details/70256437)
（1）新建工程

```bash
mkdir -p showpath/src
cd src
catkin_create_pkg showpath roscpp rospy sensor_msgs std_msgs nav_msgs tf
cd ..
catkin_make 
```
（2）编辑主函数showpath.cpp

```cpp
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="odom";


    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double vx = 0.1; // x方向速度
    double vy = -0.1;// y方向速度
    double vth = 0.1;// 角速度 

    ros::Rate loop_rate(1);
    while (ros::ok())
    {

        current_time = ros::Time::now();
        //compute odometry in a typical way given the velocities of the robot
        double dt = 0.1;
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;


        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = x;
        this_pose_stamped.pose.position.y = y;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom";
        path.poses.push_back(this_pose_stamped);

        path_pub.publish(path);
        ros::spinOnce();               // check for incoming messages

        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;
}
```
（3）vim CMakeLists.txt

```bash
add_executable(showpath src/showpath.cpp)
target_link_libraries(showpath ${catkin_LIBRARIES})
```
（4）编译和运行
移动到创建的工程工作区间

```bash
catkin_make
```
（5）在不同的终端下输入

```bash
roscore

source ./devel/setup.bash
rosrun showpath showpath

rostopic echo /trajectory

rosrun rviz rviz
```
在globel option的Fixed Fram输入odom
左边点击add
选中path
在path的topic选项中选
/trajectory
![在这里插入图片描述](https://img-blog.csdnimg.cn/df31fb31f14047e69e1ce78815d24aa8.png)
### 2 visualization_msgs/Marker
在`visualization_msgs::Marker`消息类型中,`pose.position`字段用于描述可视化元素的位置。它是一个`geometry_msgs::Point`类型的字段，包含了三个分量`x`、`y`和`z`，分别表示可视化元素在坐标系中的位置。这个位置是可视化元素的中心点或基准点。
1. 首先需要发布visualization_marker话题
```cpp
ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
```
2. 然后只需填写一条`visualization_msgs/Marker`消息并发布即可

```cpp
visualization_msgs::Marker marker;
marker.header.frame_id = "base_link";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::SPHERE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 1;
marker.pose.position.y = 1;
marker.pose.position.z = 1;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
vis_pub.publish(marker);
```
别忘了设置marker.color.a = 1，否则marker不可见

还有一个visualization_msgs/MarkerArray消息，它允许您同时发布多个marker。在这种情况下，您希望在visualization_marker_array主题中发布。

参考代码：

```cpp
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle nh;

    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_array", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);

    // 创建MarkerArray消息
    visualization_msgs::MarkerArray marker_array;

    // 创建第一个Marker
    visualization_msgs::Marker marker1;
    marker1.header.frame_id = "map";
    marker1.header.stamp = ros::Time::now();
    marker1.lifetime = ros::Duration();  // 设置持久化属性为false
    marker1.ns = "marker1";
    marker1.id = 1;
    marker1.type = visualization_msgs::Marker::SPHERE;
    marker1.pose.position.x = 1.0;
    marker1.pose.position.y = 2.0;
    marker1.pose.position.z = 0.0;
    marker1.pose.orientation.w = 1.0;
    marker1.scale.x = 0.5;
    marker1.scale.y = 0.5;
    marker1.scale.z = 0.5;
    marker1.color.r = 1.0;
    marker1.color.g = 0.0;
    marker1.color.b = 0.0;
    marker1.color.a = 1.0;

    // 创建第二个Marker
    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "map";
    marker2.header.stamp = ros::Time::now();
    marker2.lifetime = ros::Duration();  // 设置持久化属性为false
    marker2.ns = "marker2";
    marker2.id = 2;
    marker2.type = visualization_msgs::Marker::CUBE;
    marker2.pose.position.x = -1.0;
    marker2.pose.position.y = 2.0;
    marker2.pose.position.z = 0.0;
    marker2.pose.orientation.w = 1.0;
    marker2.scale.x = 0.5;
    marker2.scale.y = 0.5;
    marker2.scale.z = 0.5;
    marker2.color.r = 0.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;
    marker2.color.a = 1.0;

    marker_array.markers.clear();
    marker_array.markers.push_back(marker1);
    marker_array.markers.push_back(marker2);

    ros::Rate rate(1);  // 发布频率为1Hz

    while (ros::ok()) {
        // 发布MarkerArray消息
        marker_array_pub.publish(marker_array);

        // 发布单个Marker消息
        marker_pub.publish(marker1);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```

>请注意，附加到上面标记消息的时间戳是ros::Time()，它是时间Zero (0)。RViz对此的处理与其他任何时间都不同。如果使用ros::Time::now()或任何其他非零值，则rviz只会在该时间与当前时间足够近的情况下显示标记，其中“足够近”取决于TF。然而，在时间为0的情况下，无论当前时间如何，标记都会显示。

### 3 实战
（1）新建一个工程

```bash
mkdir -p catkin_ws/src
cd catkin_ws
catikin_create_pkg test roscpp rospy std_msgs nav_msgs tf
catkin_make
```
（2）在`catkin_ws/src/test/src`目录下新建test.cpp，修改CMakeLists.txt

```bash
vim test.cpp
```

```cpp
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

ros::Publisher car_model;
visualization_msgs::Marker car_marker;

void visual_car(const double x, const double y, const double th)
{
    car_marker.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    car_marker.header.frame_id = "map";
    car_marker.header.stamp = ros::Time::now();
    car_marker.ns = "basic_shapes";
    
    car_marker.id = 0;
    car_marker.type = visualization_msgs::Marker::CUBE;// 立方体
    car_marker.action = visualization_msgs::Marker::ADD;

    car_marker.pose.position.x = 3.0;
    car_marker.pose.position.y = 4.0;
    car_marker.pose.position.z = 0;
    car_marker.scale.x = 4.0; // 车长
    car_marker.scale.y = 2.0; // 车宽
    car_marker.scale.z = 1.5; // 车高
    car_marker.color.a = 1.0;
    car_marker.color.r = 1.0f;
    car_marker.color.g = 1.0f;
    car_marker.color.b = 0.0f;
    car_marker.lifetime = ros::Duration();
    car_model.publish(car_marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Car");
    ros::NodeHandle n;
    car_model = n.advertise<visualization_msgs::Marker>("/ydw/car", 100);
    
    ros::Rate rate(10);
    while (ros::ok())
    {
        visual_car(0.0, 0.0, 0.0);
        rate.sleep();
    }
    return 0;
}
```

```bash
vim CMakeLists.txt
```

```bash
cmake_minimum_required(VERSION 3.0.2)
project(test)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test1 src/test.cpp)

target_link_libraries(test1
  ${catkin_LIBRARIES}
)
```
（3）编译，开启如下终端，空行代表新建一个终端

```bash
catkin_make

roscore

source devel/setup.bash
rosrun test test1

rosrun rviz rviz
```
（4）打开rviz界面之后添加`Marker`
![在这里插入图片描述](https://img-blog.csdnimg.cn/e628a75dc5a34ef3865df2beac78749a.png)

