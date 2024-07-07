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
        // timeStep = image_timestamp - old_timestamp;
        // RCLCPP_WARN_STREAM(this->get_logger(), "WARNING! Timestep calculation may be broken. Check ros time to double conversion. Timestep = " << timeStep);
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

        tf2::Quaternion q_camera;
        q_camera.setW(q.w());
        q_camera.setX(q.x());
        q_camera.setY(q.y());
        q_camera.setZ(q.z());

        double roll_, pitch_, yaw_;
        tf2::Matrix3x3 m(q_camera);
        m.getRPY(roll_, pitch_, yaw_);

        double roll, pitch, yaw;
        roll = yaw_;
        pitch = -roll_;
        yaw = -pitch_;
        // tf2::Quaternion q_correction;
        q_camera.setRPY(roll, pitch, yaw);
        q_camera.normalize();

        // q_orig = q_orig * q_correction;

// Y - 90
// Z 90

        // Access quaternion components:
        // float w = q.w(); 
        // float x = q.x();
        // float y = q.y();
        // float z = q.z();

        odom_transform.transform.translation.x = z;
        odom_transform.transform.translation.y = -x;
        odom_transform.transform.translation.z = -y;

        odom_transform.transform.rotation.w = q_camera.getW();
        odom_transform.transform.rotation.x = q_camera.getX();
        odom_transform.transform.rotation.y = q_camera.getY();
        odom_transform.transform.rotation.z = q_camera.getZ();
        

        odom_tf_broadcaster->sendTransform(odom_transform);
    }
}