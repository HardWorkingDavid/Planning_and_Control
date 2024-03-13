#include "../include/global_routing.h"

// 定义起点位置的回调函数
void GlobalRouting::init_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    if (start_dynamic == "0") {
        is_init_pose_set = true;
        init_pose_.position.x = msg->pose.pose.position.x;
        init_pose_.position.y = msg->pose.pose.position.y;
        init_pose_.orientation = msg->pose.pose.orientation;

        std::cout << "init_x:" << msg->pose.pose.position.x << std::endl;
        std::cout << "init_y:" << msg->pose.pose.position.y << std::endl;
        std::cout << "theta_init:" << tf::getYaw(msg->pose.pose.orientation) << "\n";
        publish_car_init_pose(init_pose_);

        // 起点当作参考线起点
        // hdmap_way_points(0, 0) = init_pose_.position.x;
        // hdmap_way_points(0, 1) = init_pose_.position.y;
        // hdmap_way_points(0, 2) = 0;

        // 显示，方便
        visualization_msgs::Marker marker;
        marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        marker.header.frame_id = Frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = obstacle_id++; // 注意了
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = init_pose_.position.x;
        marker.pose.position.y = init_pose_.position.y;
        marker.pose.position.z = 0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        obstacle_points_.markers.push_back(marker);
        obstacle_points_pub_.publish(obstacle_points_);
    } else {
        ROS_WARN("ego vehicle is running!");
    }
}

// 定义终点位置的回调函数
void GlobalRouting::goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // start_dynamic == "0"代表车已经到达终点,即准备开始新的规划路线
    if (start_dynamic == "0" && is_init_pose_set == true)
    {
        is_goal_pose_set = true;
        goal_pose_.position.x = msg->pose.position.x;
        goal_pose_.position.y = msg->pose.position.y;
        goal_pose_.orientation = msg->pose.orientation;
    } else if (start_dynamic == "0" && is_init_pose_set == false) {
        ROS_WARN("Please set the starting point!");
    } else if (start_dynamic != "0") {
        ROS_WARN("ego vehicle is running!");
    }
}

// 读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动
void GlobalRouting::odom_call_back(const nav_msgs::Odometry &msg) {
    car_odom_ = msg; // 车的里程信息，就是位置信息
    // 坐标转换
    geometry_msgs::Quaternion odom_quat = msg.pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_quat, quat);
    // 根据转换后的四元数，获取roll pitch yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_pose_.x = msg.pose.pose.position.x;
    current_pose_.y = msg.pose.pose.position.y;
    current_pose_.heading = yaw;

    current_pose_.velocity_x = msg.twist.twist.linear.x;
    current_pose_.velocity_y = msg.twist.twist.linear.y;
    current_pose_.velocity = std::sqrt(current_pose_.velocity_x * current_pose_.velocity_x + current_pose_.velocity_y * current_pose_.velocity_y);

    current_pose_.z = msg.pose.pose.position.z;
}
// 获取局部规划来的航向角
void GlobalRouting::Vehicle_Theta(const geometry_msgs::PoseArray &theta) {
    thetas_.clear();
    if (theta.poses.size() > 0) {
        for (int i = 0; i < theta.poses.size(); i++) {
            double x = theta.poses[i].position.x;
            thetas_.push_back(x);
        }
    }
}

// 获取局部规划来的曲率
void GlobalRouting::Vehicle_Kappa(const geometry_msgs::PoseArray &kappa) {
    kappas_.clear();
    if (kappa.poses.size() > 0) {
        for (int i = 0; i < kappa.poses.size(); i++) {
            double x = kappa.poses[i].position.x;
            kappas_.push_back(x);
        }
    }
}
// 获取局部轨迹的信号
void GlobalRouting::Vehicle_Go(const std_msgs::String::ConstPtr &go) {
    start_dynamic = go->data.c_str();
}
// 获取局部轨迹
void GlobalRouting::Vehicle_Traj(const waypoint_msgs::WaypointArray::ConstPtr &msg) {
    dynamic_points_.poses.clear();
    dynamic_points_.header.frame_id = Frame_id;
    dynamic_points_.header.stamp = ros::Time::now();
    speeds_.clear();
    local_waypoints_.resize(msg->waypoints.size());
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
        VehicleState temp_point;
        temp_point.x = msg->waypoints[i].pose.pose.position.x;
        temp_point.y = msg->waypoints[i].pose.pose.position.y;
        temp_point.velocity = msg->waypoints[i].twist.twist.linear.x; //速度
        local_waypoints_[i] = temp_point;

        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = Frame_id;
        pose_stamp.header.stamp = ros::Time::now();
        pose_stamp.pose.position.x = msg->waypoints[i].pose.pose.position.x;
        pose_stamp.pose.position.y = msg->waypoints[i].pose.pose.position.y;
        pose_stamp.pose.position.z = 0;
        dynamic_points_.poses.push_back(pose_stamp);
        speeds_.push_back(msg->waypoints[i].twist.twist.linear.x); //速度
    }
}
// 判断车是否到了路径的尽头，传入 终点，车里程
bool GlobalRouting::Vehicle_Stop(const geometry_msgs::Pose &goal_point, nav_msgs::Odometry &odom) {
    // 获取车的里程位置
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    // 获取路径点的最后一个位置
    double xt = goal_point.position.x;
    double yt = goal_point.position.y;
    // 判断如果两点坐标接近
    double dis = sqrt((x - xt) * (x - xt) + (y - yt) * (y - yt));
    if (dis < goal_distance) return true;
    else return false;
}

// 布车辆在路径的起始位置,传入起点
void GlobalRouting::publish_car_init_pose(const geometry_msgs::Pose &init_pose) {
    geometry_msgs::Vector3 state;
    // get 起始位置的yaw
    double set_vehicle_yaw = tf::getYaw(init_pose_.orientation);
    // get vehicle start pose
    state.x = init_pose_.position.x;
    state.y = init_pose_.position.y;
    state.z = set_vehicle_yaw;
    // 发布车的起点位置
    vehicle_init_pose_pub_.publish(state);
}

void GlobalRouting::create_map() {
    std::vector<double> map_1_x = {48.6505, 21.935, 0.32034, -8.76959};
    std::vector<double> map_1_y = {-52.271, -27.4259, -7.07192, 1.55404};

    hdmap_way_points = Eigen::MatrixXd::Zero(map_1_x.size(), 3); // 初始化零矩阵
    for (int i = 0; i < map_1_x.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        marker.header.frame_id = Frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = i; // 注意了
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = map_1_x[i];
        marker.pose.position.y = map_1_y[i];
        marker.pose.position.z = 0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        map_points_.markers.push_back(marker);
    }
    // 加入起点的话i = 1
    for (int i = 0; i < map_1_x.size(); i++)
    {
        hdmap_way_points(i, 0) = map_1_x[i];
        hdmap_way_points(i, 1) = map_1_y[i];
        hdmap_way_points(i, 2) = 0;
    }
    sleep(2);
    map_points_pub_.publish(map_points_);
}

// 初始化参数
GlobalRouting::GlobalRouting() {
    ros::NodeHandle n;
    n.param("goal_distance", goal_distance, 0.5);
    start_pose_subscriber_ = n.subscribe("/init_pose", 10, &GlobalRouting::init_pose_call_back, this);
    goal_pose_subscriber_ = n.subscribe("/goal", 10, &GlobalRouting::goal_pose_call_back, this);

    odom_sub_ = n.subscribe("/bigdavid/odom", 10, &GlobalRouting::odom_call_back, this);
    map_points_pub_ = n.advertise<visualization_msgs::MarkerArray>("/bigdavid/maps/map_points", 10);               // ros仿真下的地图路线点显示
    obstacle_points_pub_ = n.advertise<visualization_msgs::MarkerArray>("/bigdavid/obstacle/obstacle_points", 10); // 障碍物的顶点设置
    control_data_pub_ = n.advertise<geometry_msgs::Vector3>("/bigdavid/car/control_car", 10);                      // ros仿真时发布车的控制数据
    vehicle_init_pose_pub_ = n.advertise<geometry_msgs::Vector3>("/bigdavid/car/car_init_pose", 10);             // 发布车的起点位置

    // 局部规划订阅
    vehicle_theta = n.subscribe("/bigdavid/planning/dynamic_paths_t", 10, &GlobalRouting::Vehicle_Theta, this);
    vehicle_kappa = n.subscribe("/bigdavid/planning/dynamic_paths_k", 10, &GlobalRouting::Vehicle_Kappa, this);
    vehicle_traj = n.subscribe("/bigdavid/planning/dynamic_waypoints", 10, &GlobalRouting::Vehicle_Traj, this);
    vehicle_go = n.subscribe("/bigdavid/planning/start_Dynamic", 10, &GlobalRouting::Vehicle_Go, this);
    // 初始化 标志位
    is_init_pose_set = false;
    is_goal_pose_set = false;
    is_begin_reference = false;
    is_vehicle_stop_set = true;
    start_dynamic = "0";
    obstacle_id = 0;
    // 车的初始值设置
    vehicle_speed = 0;
    // 创建map
    create_map();
    // 创建线程
    //routing_thread_ = new boost::thread(boost::bind(&GlobalRouting::thread_routing, this));
}

/*析构函数：释放线程*/
GlobalRouting::~GlobalRouting() {
  delete routing_thread_;
}

// 启动车，控制车的行驶
void GlobalRouting::thread_routing() {
    double frequency = 10.0;
    ros::NodeHandle n;
    ros::Rate loop_rate(frequency);
    double turn_angle = 0;
    int out_index_ = 0;
    std::vector<double> prev_p(2);
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    // Q矩阵
    Q.setZero(3, 3);
    Q(0, 0) = Q_ex_k_;
    Q(1, 1) = Q_ed_k_;
    Q(2, 2) = Q_ephi_k_;
    // R矩阵
    R.setZero(2, 2);
    R(0, 0) = R_value_k_;
    R(1, 1) = R_value_k_;

    rviz_pnc::Controller controller();

    while (n.ok()) {
        if (hdmap_way_points.rows() > 0 && is_init_pose_set == true &&
            is_goal_pose_set == true && is_begin_reference == false) {
            // 局部规划参考线:hdmap_way_points
            R_.referenceLine_split(hdmap_way_points);
            is_begin_reference = true;
        }
        // is_init_pose_set== true已经设置起点
        // is_goal_pose_set== true已经设置终点
        // start_dynamic 局部规划轨迹开始生成
        if (is_init_pose_set == true && is_goal_pose_set == true && start_dynamic == "1") {
            is_vehicle_stop_set = Vehicle_Stop(goal_pose_, car_odom_); // 判断是否到达终点
            msg_ros_.y = turn_angle;           // 转角
            // 未到达终点
            if (is_vehicle_stop_set == false) {
                vehicle_speed = speeds_[out_index_];
                msg_ros_.x = vehicle_speed;
                control_data_pub_.publish(msg_ros_);
                ROS_INFO("speed: %2f, turn_angle: %2f", vehicle_speed, turn_angle);
            } else {// 已经到达终点
                ROS_WARN("Arrive at goal");
                msg_ros_.x = 0;
                control_data_pub_.publish(msg_ros_);
                is_init_pose_set = false;
                is_goal_pose_set = false;
                is_begin_reference = false;
                start_dynamic = "0";
                vehicle_speed = 0;
            }
            
        } else if (is_init_pose_set == true && is_goal_pose_set == true && start_dynamic == "2") {// 没有轨迹强制停车
            msg_ros_.x = 0;
            control_data_pub_.publish(msg_ros_);
            vehicle_speed = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}