#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// 需要实现“map”参考系到“odom”参考系的变换，包含头文件tf
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

visualization_msgs::Marker vehicle_marker;
ros::Publisher vehicle_pub;
double speed, steer;
double init_x, init_y, init_theta;
double wheel_base = 2.947; // 轴距（前后轮之车轮轴距离）
// 回调控制量
void control_value(const geometry_msgs::Vector3& u) {
    speed = u.x;
    steer = u.y;
}
// 回调车初始位置
void init_vehicle(const geometry_msgs::Vector3& msg) {
    init_x = msg.x;
    init_y = msg.y;
    init_theta = msg.z;
    speed = 0;
    steer = init_theta;
}
// 在Rviz中可视化车
void visualization_vehicle(const double x, const double y, const double th) {
    vehicle_marker.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    vehicle_marker.header.frame_id = "map";
    vehicle_marker.header.stamp = ros::Time::now();
    vehicle_marker.ns = "basic_shapes";
    
    vehicle_marker.id = 0;
    vehicle_marker.type = visualization_msgs::Marker::ARROW;// 箭头
    vehicle_marker.action = visualization_msgs::Marker::ADD;

    vehicle_marker.pose.position.x = 3.0;
    vehicle_marker.pose.position.y = 4.0;
    vehicle_marker.pose.position.z = 0;
    vehicle_marker.scale.x = 3.29; // 车长
    vehicle_marker.scale.y = 1.6; // 车宽
    vehicle_marker.scale.z = 1.5; // 车高
    vehicle_marker.color.a = 1.0; // Don't forget to set the alpha!
    vehicle_marker.color.r = 1.0f;
    vehicle_marker.color.g = 1.0f;
    vehicle_marker.color.b = 0.0f;
    vehicle_marker.lifetime = ros::Duration();
    vehicle_pub.publish(vehicle_marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vehicle_simulation");
    ros::NodeHandle n;
    
    tf::TransformBroadcaster odom_broadcaster;
    // 发布里程计
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/bigdavid/odom", 100);
    // 发布箭头
    vehicle_pub = n.advertise<visualization_msgs::Marker>("/bigdavid/vehicle_state", 100);
    // 订阅控制量
    ros::Subscriber vehicle_u = n.subscribe("/bigdavid/control_vehicle", 100, control_value);
    // 订阅初始位置
    ros::Subscriber vehicle_init_pose = n.subscribe("/bigdavid/vehicle_init_pose", 20, init_vehicle);
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(10);
    while (n.ok()) {
        // 检查传入消息
        ros::spinOnce();               
        current_time = ros::Time::now();
        // 可视化小车
        visualization_vehicle(init_x, init_y, init_theta);

        // 在给定机器人速度的情况下，以一种典型的方式计算里程计
        double dt = (current_time - last_time).toSec();
        //std::cout << "dt: " << dt << std::endl;
        double delta_x = speed * cos(init_theta) * dt;
        double delta_y = speed * sin(init_theta) * dt;
        double delta_th = (speed * tan(steer) / wheel_base) * dt;

        init_x += delta_x;
        init_y += delta_y;
        init_theta += delta_th;

        // 由于所有里程计都是6DOF，我们需要一个由yaw创建的四元数
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(init_theta);
        // 首先，我们将通过tf发布转换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "odom";
        // 填充里程计的数据
        odom_trans.transform.translation.x = init_x;
        odom_trans.transform.translation.y = init_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // 发送转换
        odom_broadcaster.sendTransform(odom_trans);
        // 接下来，我们将通过ROS发布里程计消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "map";
        // 设置位置
        odom.pose.pose.position.x = init_x;
        odom.pose.pose.position.y = init_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom_pub.publish(odom);
        last_time = current_time;
        r.sleep();
    }
    return 0;
}

