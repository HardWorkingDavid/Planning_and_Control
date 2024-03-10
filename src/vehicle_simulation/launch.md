博客：[Launch](https://blog.csdn.net/BigDavid123/article/details/136425216?spm=1001.2014.3001.5502)

### 1 ROS工作空间简介
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/a6e09d71907e4be391c1589b78e29869.png)
###  2 元功能包
src目录下可以包含多个功能包，假设需要使用机器人导航模块，但是这个模块中包含着地图、定位、路径规划等不同的功能包，它们的逻辑关系如下：

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/f926a5af030245ddbf56fb7f1239db65.png#pic_center)
在Linux系统中为了更方便的组织工程项目（这里针对的是项目文件，即功能包），出现了“元功能包”的概念。这个是一个“虚包”，就是这个功能包的src目录下没有源文件，因此自身不会实现专属功能，其功能的实现完全依赖于其他的功能包，起到一个组织功能包的作用。

以导航模块中的元功能包为例：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/55a19572b4ce41b39d467c650708d711.png)navigation功能包为元功能包（metapackage），元功能包中由于没有src目录因此无需添加任何依赖项，因为这个功能包没有自己的专属功能，它的功能是借助其他的功能包的功能来实现的。元功能包有两个文件即可：一个是package.xml文件：用于声明元功能包所依赖的其他功能包；另一个是CMakelist.txt文件：用于指定功能包之间的依赖关系。

`CMakelist.txt`

```bash
cmake_minimum_required(VERSION 3.0.2)  
project(navigation)  
find_package(catkin REQUIRED)  
catkin_metapackage() // 只需添加此条内容即可
```
`package.xml`

```bash
<exec_depend>amcl</exec_depend>  
<exec_depend>base_local_planner</exec_depend>  
<exec_depend>carrot_planner</exec_depend>  
<exec_depend>clear_costmap_recovery</exec_depend>  
<exec_depend>costmap_2d</exec_depend>  
<exec_depend>dwa_local_planner</exec_depend>  
<exec_depend>fake_localization</exec_depend>  
<exec_depend>global_planner</exec_depend>  
<exec_depend>map_server</exec_depend>  
<exec_depend>move_base</exec_depend>  
<exec_depend>move_base_msgs</exec_depend>  
<exec_depend>move_slow_and_clear</exec_depend>  
<exec_depend>navfn</exec_depend>  
<exec_depend>nav_core</exec_depend>  
<exec_depend>rotate_recovery</exec_depend>  
<exec_depend>voxel_grid</exec_depend>  
  
<export>  
    <metapackage/> // 表征：这个功能包为元功能包
</export>  
```
### 3 Launch文件
Launch文件：源文件的组织者
① 节点启动标签

```xml
<launch>  
    <node pkg = "turtlesim" type = "turtlesim_node" name = "my_node"/>  
    <node pkg = "turtlesim" type = "turtle_teleop_key" name = "my_key"/>  
</launch>  
```
Tip：因为ROS中采用多线程，因此节点的运行不会按照节点在launch中排列顺序进行。
>pkg：功能包的名称
>type：节点本来的名称，这个名称和节点所在.cpp源文件的文件名一致
>name：节点重映射的名称，相当于在系统中给节点所在源文件改了个名字

launch标签有一个子级标签deprecated，用于文本说明：

```xml
<launch deprecated="this vision is out-of-date!">  
</launch> 
```
如果认为给很多节点取名太麻烦，可以使用name=”$(anon node_name)”标签在节点node_name名称之后加一些随机数，使得该节点名称在整个catkin编译项目中唯一：

```xml
<launch deprecated="this vision is out-of-date!">  
    <!-- the topic of turtlesim_node is /turtle1/cmd_vel -->  
    <node pkg="turtlesim" type="turtlesim_node" name="$(anon my_node)"/>  
    <!-- the topic of turtle_teleop_key is /turtle1/cmd_vel -->  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/>  
</launch>  
```

意外关闭后自动启动的子级标签
respawn = true|false 表示：如果节点意外关闭是否重新启动
```xml
<launch>  
    <node pkg="turtlesim" type="turtlesim_node" name="my_node" respawn="true"/>  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" respawn="true"/>  
</launch>  
```
节点延迟启动的子级标签，一般结合节点重启动是使能标签respawn（如果节点异常退出运行，那么该节点会被重新启动）一起使用
```xml
<launch>  
    <node pkg="turtlesim" type="turtlesim_node" name="my_node" respawn="true" respawn_delay="10"/>  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" respawn="true" respawn_delay="10"/>  
</launch>  
```
如果XXX节点结束运行（XXX节点被杀死），则所有节点都停止运行

```xml
<launch>  
    <node pkg="turtlesim" type="turtlesim_node" name="my_node" required="true"/>  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" />  
</launch>  
```
给节点名称添加前缀（给节点添加命名空间）的子级标签

```xml
<launch>  
    <node pkg="turtlesim" type="turtlesim_node" name="my_node" ns="hello"/>  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key"/>  
</launch> 
```

② 参数设置标签
设置global全局参数
```xml
<launch>
	<param name="var" type="int" value="10"/>
</launch>
```
结合<node>标签设置带有命名空间的私有参数

```xml
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="my_node"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen">
		<param name="var1" type="int" value="20"/>
	</node>
</launch>
```
③ 参数打包输入输出删除的标签
从.yaml文件中读取参数:

```xml
<launch>
	<rosparam command="load" file="$(find test01)/launch/params.yaml"/>
	<node pkg="turtlesim" type="turtlesim_node" name="my_node"/>  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/> 
	<param name="var" type="int" value="10"/>  
</launch> 
```
将.yaml参数文件中的参数导入参数服务器时，我们还可以给这些参数添加namespace命名空间：

```xml
<launch>  
    <node pkg="turtlesim" type="turtlesim_node" name="my_node"/>  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/>  
    <param name="var" type="int" value="10"/>  
    <rosparam command="load" file="$(find test01)/launch/params.yaml" ns="hello"/>  
</launch> 
```
将参数打包输入进.yaml文件中，这样做啥变量都没导进去：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/160326fc1dc94c0aa5f1854061de3de3.png)
```xml
<launch>  
    <rosparam command="dump" file="$(find test01)/launch/input.yaml"/>  
    <node pkg="turtlesim" type="turtlesim_node" name="my_node"/>  
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/>  
    <param name="var" type="int" value="10"/>  
</launch>  
```
我们要想导入参数必须另建一个.launch文件，在使用上述launch文件启动完所有节点之后，在另一个launch文件中执行该功能包参数的导出操作：

```xml
<launch>
	<rosparam command="dump" file="$(find test01)/launch/input.yaml" />
</launch>
```

```xml
<launch>  
    <rosparam command="dump" file="$(find test01)/launch/input.yaml"/>  
    <rosparam command="delete" param="/hello/n1"/>  
</launch>  
```
④ 参数统一管理的标签

```xml
<launch>  
    <arg name="car_width" default="[1,2,3,4]" doc="the width of car"/>  
    <rosparam param="a_list">$(arg car_width)</rosparam>  
    <rosparam>  
        Name:  
            a: 9  
            b: "hello"  
            c: $(arg car_width)  
    </rosparam>  
</launch>
```
⑤ 改topic名称的标签

```xml
<launch>  
    <!-- the topic of turtlesim_node is /turtle1/cmd_vel -->  
    <node pkg="turtlesim" type="turtlesim_node" name="my_node"/>  
    <remap from="/turtle1/cmd_vel" to="new_topic"/>  
</launch>  
```
⑥ 节点组织标签
就是给被` <group>…</group> `包含的所有参数、节点的属性加上了namespace

```xml
<launch deprecated="this vision is out-of-date!">  
    <group ns="family">  
        <!-- the topic of turtlesim_node is /turtle1/cmd_vel -->  
        <node pkg="turtlesim" type="turtlesim_node" name="my_node"/>  
        <!-- the topic of turtle_teleop_key is /turtle1/cmd_vel -->  
        <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen"/>  
        <rosparam command="load" file="$(find test01)/launch/params.yaml" ns="hello"/>  
        <arg name="car_width" default="[1,2,3,4]" doc="the width of car"/>  
        <rosparam param="a_list" value="$(arg car_width)"/>  
        <rosparam>  
            Name:  
                a: 9  
                b: "hello"  
                c: [1,2,3,4]  
        </rosparam>  
        <param name="var" type="int" value="$(arg car_width)"/>  
    </group>  
</launch> 
```
⑦ 启动其他launch文件的标签

```xml
<launch>  
    <include file="$(find test01)/launch/test01_launch.launch">  
        <arg name="car_width" default="10"/>  
    </include>  
</launch>  
```
### 4 功能包/源文件/launch文件组织工具
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/63c9a2a9cd84403289d6301d1b7a8a4d.png)文件组织形式如下所示：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/e5b5a766c72d4911af7ba56d114762b2.png)
### 5 功能包绝对路径替换标签
标签格式：`$(find package_name)`
使用示例：

```xml
<launch>  
     <include file="$(find tf2_turtle)/launch/setupGUI.launch"/>  
</launch>  
```
### 6 工作空间下绝对路径替换标签
下面是test1.launch调用setupGUI.launch文件的代码，并且两个launch文件在一个文件夹之中：
```xml
<launch>  
     <include file="$(dirname)/setupGUI.launch"/>  
</launch> 
```
$(dirname)代表“test1.launch文件所在工作空间的绝对地址

### 7 Launch文件
列出几个Launch文件自测一下学习成果

```xml
<?xml version="1.0"?>
<launch>
    <!--  加载车模型 -->
    <include file="$(find vehicle_description)/launch/estima_black.launch" />
    <!--  -->
    <node pkg="car_simulation" type="car_model_node" name="car_simulation" output="screen" />
</launch>
```

```xml
<launch>
    <include file="$(find global_routing)/launch/global_routing.launch"/>
    <!-- 车辆仿真 -->
    <include file="$(find car_simulation)/launch/car_simulation.launch" />
    <!-- rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find global_routing)/config/planning_demo.rviz"/>  
</launch>
```

```xml
<?xml version="1.0"?>
<launch>
    <!-- 其他launch文件传入的参数 -->
    <arg name="is_planner"/>
    <arg name="is_lateral_optimization"/>
    <arg name="is_change_lane"/>
    <arg name="is_carla_simulation"/>
    <arg name="ego_vehicle_name"/>
    <arg name="is_parking"/>
    <arg name="is_goal"/>

    <!-- 模拟动态障碍物的加载文件，这些都是录好的轨迹点，播放这个文件就可以实现障碍物移动 -->
    <param name="obstacle_test_path" value="$(find dynamic_routing)/obstacle_files"/>
    <!-- 加载存储的其他参考线数据 -->
    <param name="referenceline_path" value="$(find dynamic_routing)/other_referenceline_files"/>
    <!-- yaml文件 -->
    <param name="yaml_path" value="$(find dynamic_routing)/config"/>

    <!-- 规划算法选择 -->
    <param name="use_what_planner"  value="$(arg is_planner)"/>
    <!-- 变道决策是否开启 -->
    <param name="change_lane"  value="$(arg is_change_lane)"/>
    <!-- 是否使用二次规划，选择了lattice规划，选择这个才有效果 -->
    <param name="use_lateral_optimization"  value="$(arg is_lateral_optimization)"/>
    <!-- 是否选择carla联合仿真 -->
    <param name="carla_simulation"  value="$(arg is_carla_simulation)"/>
    <!-- role_name -->
    <param name="role_name"  value="$(arg ego_vehicle_name)"/>
    <!-- carla 停车场景 -->
    <param name="parking_mode" value="$(arg is_parking)"/>

    <!-- 在frenet规划下的参数设置，lattice规划不用这些 -->
    <!-- COLLISION_CHECK_THRESHOLD 距离障碍物的最短距离 -->
    <param name="COLLISION_CHECK_THRESHOLD" type="double" value="2" />
    <!-- 调整轨迹的长度 -->
    <param name="MaxT" type="double" value="11" />
    <param name="MinT" type="double" value="9" />

    <!--  判断与终点的停车距离阈值 -->
     <param name="goal_distanse"  type="double" value="$(arg is_goal)"/>
    
    <!-- 打开 Hybrid_a_star 的测试图 -->
    <!-- mapserver提供了一个ROS节点，该节点通过一个ROS Service来提供地图数据 -->
	<node name="map_server" pkg="map_server" type="map_server" args="$(find dynamic_routing)/maps/map.yaml" >
		<param name="frame_id" value="map" />
	</node>
 
    <!--Open palnner的launch参数，顺便加载dynamic节点 -->
    <include file="$(find dynamic_routing)/launch/op_common_params.launch" />

    <!-- DWA -->
    <arg name="dwa_params" default="$(find dynamic_routing)/config/dwa_params.yaml"/>
    <rosparam command="load" file="$(arg dwa_params)"/>
</launch> 
```

```xml
<?xml version="1.0"?>
<launch>
    <!-- 是否使用carla联合仿真 -->
    <arg name="carla"  default="false"/>   
    <arg name="ego_vehicle_name"  default="ego_vehicle"/>  
    <param name="carla_simulation" value="$(arg carla)"/>
    <param name="role_name" value="$(arg ego_vehicle_name)"/>

    <!-- 不能改这里的参数 -->
    <arg name="parking" default="false"/>
    <param name="parking_mode" value="$(arg parking)"/>
    
    <!-- ros单独仿真下的controller，carla不适用: 
        1 stanley  
        2 lqr 
        3 pure_pursuit
        4 pid 
        5 mpc  
    -->
    <arg name="control"  value="2"/>  
    <param name="use_what_controller" value="$(arg control)"/>

    <!-- planner: 
        1是纯frenet规划
        2是lattice规划
        3是em_palnner规划
        4是混合A*规划
        5是op_planner规划
        6是DWA规划
        7是Teb规划
        8是simple_em(EM的简化版本，待更新)
    -->
    <arg name="planner"  value="7"/>    
    <param name="use_what_planner" value="$(arg planner)"/>

    <!-- 是否使用二次规划，选择了lattice规划，选择这个才有效果 -->
    <!-- false：lattice 采样规划，true：lattice 二次规划 -->
    <arg name="use_lateral_optimization" default="false"/>    

    <!-- 是否开启变道决策，变道选择的是Lattce采样规划，其他方法不使用 -->
    <arg name="change_lane"  default="false"/>    

    <!-- 参考线平滑的方式选择: true:CosThetaSmoother  false:FemPosSmooth-->
    <arg name="which_smoother" default="false"/>    
    <param name="which_smoothers" value="$(arg which_smoother)"/>

    <!--  判断与终点的停车距离阈值 -->
    <arg name="goal_dis"  value="0.5"/>  
    <param name="goal_distanse"  type="double" value="$(arg goal_dis)"/>

    <!-- 局部规划 -->
    <include file="$(find dynamic_routing)/launch/dynamic_routing.launch" >
        <arg name="is_planner" value="$(arg planner)" />
        <arg name="is_lateral_optimization" value="$(arg use_lateral_optimization)" />
        <arg name="is_change_lane" value="$(arg change_lane)" />
        <arg name="is_carla_simulation" value="$(arg carla)" />
        <arg name="ego_vehicle_name" value="$(arg ego_vehicle_name)" />
        <arg name="is_parking" value="$(arg parking)"/>
        <arg name="is_goal" value="$(arg goal_dis)"/>
    </include>

    <!-- carla联合仿真下的控制方法和参数  -->
    <!-- LQR_dynamics  LQR_kinematics Stanley PurePursuit -->
    <param name="control_method" value='LQR_kinematics'/>  
    <!-- "PurePursuit"增益系数 -->
    <param name="k_pure" type="double" value="0.3" /> 
    <!-- "Stanley"增益系数  -->
    <param name="k_cte" type="double" value="100" /> 
    <param name="kp"  value="0.5" />
    <param name="ki" type="double" value="0.02" />
    <param name="kd" type="double" value="0.05" />
    <!-- LQR Q R矩阵参数 -->
    <param name="Q_ed" type="double" value="20.0" />
    <param name="Q_ed_dot" type="double" value="1.0" />
    <param name="Q_ephi" type="double" value="10.0" />
    <param name="Q_ephi_dot" type="double" value="1.0" />
    <param name="R_value" type="double" value="40.0" />
    <!--  -->
    <param name="Q_ex_k" type="double" value="3.0" />
    <param name="Q_ed_k" type="double" value="3.0" />
    <param name="Q_ephi_k" type="double" value="1.5" />
    <param name="R_value_k" type="double" value="4.0" />

    <!-- 全局规划 -->
    <node pkg="global_routing" type="global_routing_node" name="global_routing" output="screen" />

</launch>
```

在Rviz中，每次手动配置比较麻烦，为了简化操作，可以执行如下操作：

1. 在Rviz手动配置后，不要关闭，在左上角点击“File”——>“Save Config As”——>选择地址，输入文件名称后点击保存

2. 如你没有要启动的launch文件，直接看第三步即可；如你有要启动的launch文件，则在你要启动的launch文件里（<launch>下面，</launch>上面）添加中间那行命令后，执行即可

```xml
<launch>
    <node pkg="rviz" type="rviz" name="rviz" args="-d rviz配置文件地址" required="true" />
</launch>
```