/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/rgbd_orb.hpp"

//* Constructor
RGBDMode::RGBDMode() : Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    home_dir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    std::string default_pkg_path = "workspaces/wheeltec_robot_ros2/src/ros2_orb_slam3/";
    this->declare_parameter("pkg_path", default_pkg_path);
    this->get_parameter_or("pkg_path", packagePath, default_pkg_path);

    std::string default_experimentConfig = "Astra_S";
    this->declare_parameter("experimentConfig", default_experimentConfig);
    this->get_parameter_or("experimentConfig", experimentConfig, default_experimentConfig);

    std::string default_odom_link = "odom";
    this->declare_parameter("odom_link", odom_link);
    this->get_parameter_or("odom_link", odom_link, default_odom_link);

    std::string default_odom_parent_link = "map";
    this->declare_parameter("odom_parent_link", odom_parent_link);
    this->get_parameter_or("odom_parent_link", odom_parent_link, default_odom_parent_link);

    // this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    // nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = home_dir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = home_dir + "/" + packagePath + "orb_slam3/config/Stereo/";
    }
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    
    subImgMsgName = "/camera/color/image_raw"; // topic to receive RGB image messages
    subDepthImgMsgName = "/camera/depth/image_raw"; // topic to receive RGB image messages

    //* subscrbite to the image messages coming from the camera
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&RGBDMode::Img_callback, this, _1));
    subDepthImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subDepthImgMsgName, 1, std::bind(&RGBDMode::DepthImg_callback, this, _1));
    odom_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pubOdometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/orb_slam3/odom", 10);

    vslam_timer = this->create_wall_timer(std::chrono::milliseconds(25), std::bind(&RGBDMode::vslam_timer_cb, this));

    initializeVSLAM(experimentConfig);
}

//* Destructor
RGBDMode::~RGBDMode()
{   
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;
}


//* Method to bind an initialized VSLAM framework to this node
void RGBDMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::RGBD; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "RGBDMode node initialized" << std::endl; // TODO needs a better message
}


//* Callback to process image message and run SLAM node
void RGBDMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        rgb_image = cv_bridge::toCvCopy(msg); // Local scope
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    old_timestamp = image_timestamp;
    image_rcl_timestamp = msg.header.stamp;
    image_timestamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) / 1e9;
    rgb_img_is_fresh = true;
}


//* Callback to process image message and run SLAM node
void RGBDMode::DepthImg_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        depth_image = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    depth_img_is_fresh = true;
}


void RGBDMode::vslam_timer_cb()
{
    if (rgb_img_is_fresh == true && depth_img_is_fresh == true && old_timestamp != 0)
    {
        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.child_frame_id = odom_link;
        odom_transform.header.frame_id = odom_parent_link;
        odom_transform.header.stamp = image_rcl_timestamp;

        Sophus::SE3f Tcw = pAgent->TrackRGBD(rgb_image->image, depth_image->image, image_timestamp); 
        Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

        // Extract the translation (x, y, z)
        Eigen::Vector3f translation = Twc.translation();
        float x = translation.x();
        float y = translation.y();
        float z = translation.z();

        // Extract the quaternion directly from the transformation
        Eigen::Quaternionf q(Twc.rotationMatrix());

        double dpitch = -M_PI_2;
        double dyaw = M_PI_2;

        // tf2::Quaternion q_camera;
        // q_camera.setW(q.w());
        // q_camera.setX(q.x());
        // q_camera.setY(q.y());
        // q_camera.setZ(q.z());

         // Change axes of rotation
        double roll_, pitch_, yaw_;
        double roll, pitch, yaw;
        // tf2::Matrix3x3 m(q_camera);
        // m.getRPY(roll_, pitch_, yaw_);

        // roll = yaw_;
        // pitch = -roll_;
        // yaw = -pitch_;

        // // tf2::Quaternion q_correction;
        // q_camera.setRPY(roll, pitch, yaw);
        // q_camera.normalize();

        odom_transform.transform.translation.x = z;
        odom_transform.transform.translation.y = -x;
        odom_transform.transform.translation.z = -y;

        // view roll = -yaw_real
        // view pitch = roll_real
        // view yaw = -pitch_real
        // odom_transform.transform.rotation.w = q.w();
        // odom_transform.transform.rotation.x = q.y();
        // odom_transform.transform.rotation.y = q.z();
        // odom_transform.transform.rotation.z = q.x();

        // swap z <-> y // yaw_real + roll_real

        // view roll = roll_real
        // view pitch = -yaw_real
        // view yaw = -pitch_real
        // odom_transform.transform.rotation.w = q.w();
        // odom_transform.transform.rotation.x = q.z();
        // odom_transform.transform.rotation.y = q.y();
        // odom_transform.transform.rotation.z = q.x();
        
        // swap y <-> x // yaw_real + pitch_real

        // view roll = roll_real
        // view pitch = -pitch_real
        // view yaw = -yaw_real
        odom_transform.transform.rotation.w = q.w();
        odom_transform.transform.rotation.x = q.z();
        odom_transform.transform.rotation.y = -q.x();
        odom_transform.transform.rotation.z = -q.y();

        nav_msgs::msg::Odometry odom;
        odom.child_frame_id = odom_transform.child_frame_id;
        
        odom.header = odom_transform.header;
        
        odom.pose.pose.position.x = odom_transform.transform.translation.x;
        odom.pose.pose.position.y = odom_transform.transform.translation.y;
        odom.pose.pose.position.z = odom_transform.transform.translation.z;

        odom.pose.pose.orientation.w = odom_transform.transform.rotation.w;
        odom.pose.pose.orientation.x = odom_transform.transform.rotation.x;
        odom.pose.pose.orientation.y = odom_transform.transform.rotation.y;
        odom.pose.pose.orientation.z = odom_transform.transform.rotation.z;

        if (got_odometry_at_least_once == true)
        {
            float dt = (float(odom.header.stamp.sec) + float(odom.header.stamp.nanosec) / 1e9) - (float(previous_odometry.header.stamp.sec) + float(previous_odometry.header.stamp.nanosec) / 1e9);
            RCLCPP_WARN_STREAM(this->get_logger(), "Odometry delta time = " << dt);
            if (dt == 0)
            {
                return;
            }

            odom.twist.twist.linear.x = (odom.pose.pose.position.x - previous_odometry.pose.pose.position.x) / dt;
            odom.twist.twist.linear.y = (odom.pose.pose.position.y - previous_odometry.pose.pose.position.y) / dt;
            odom.twist.twist.linear.z = (odom.pose.pose.position.z - previous_odometry.pose.pose.position.z) / dt;

            tf2::Quaternion prev_orientation;
            tf2::Quaternion curr_orientation;

            prev_orientation.setX(previous_odometry.pose.pose.orientation.x); 
            prev_orientation.setW(previous_odometry.pose.pose.orientation.w); 
            prev_orientation.setY(previous_odometry.pose.pose.orientation.y); 
            prev_orientation.setZ(previous_odometry.pose.pose.orientation.z); 

            curr_orientation.setX(odom.pose.pose.orientation.x); 
            curr_orientation.setW(odom.pose.pose.orientation.w); 
            curr_orientation.setY(odom.pose.pose.orientation.y); 
            curr_orientation.setZ(odom.pose.pose.orientation.z); 

            tf2::Matrix3x3 m_prev(prev_orientation);
            tf2::Matrix3x3 m_curr(curr_orientation);

            // Previous RPY are underscored, current are not
            m_prev.getRPY(roll_, pitch_, yaw_);
            m_curr.getRPY(roll, pitch, yaw);

            double droll = roll - roll_;
            double dpitch = pitch - pitch_;
            double dyaw = yaw - yaw_;

            odom.twist.twist.angular.x = droll / dt;
            odom.twist.twist.angular.y = dpitch / dt;
            odom.twist.twist.angular.z = dyaw / dt;
        }

        odom_tf_broadcaster->sendTransform(odom_transform);
        pubOdometry_->publish(odom);

        // After everything
        previous_odometry = odom;
        got_odometry_at_least_once = true;
    }
}