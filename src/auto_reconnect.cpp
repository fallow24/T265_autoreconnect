// System
#include <cmath>
#include <thread>
#include <mutex>
#include <memory>

// Librealsense2
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

// Custom msg definition, see "../msg/CameraPoseAngularVelocity.msg"
#include "../../../devel/include/realsense_pipeline_fix/CameraPoseAngularVelocity.h"

const char* pose_topic = "/camera/pose";
const char* cam_imu_topic = "/camera/imu";
const char* cam_pose_imu_topic = "/camera/poseAndImu";
const char* cam_images_topic = "/camera/images";
const char* camera_frame = "camera_frame";

// Setup ros publishers
std::shared_ptr<ros::Publisher> cam_pose;
std::shared_ptr<ros::Publisher> cam_imu;
std::shared_ptr<ros::Publisher> cam_pose_imu;
std::shared_ptr<ros::Publisher> cam_imgs;

// ROS msg objects
sensor_msgs::Imu imu_msg;
geometry_msgs::PoseStamped output_msg;
realsense_pipeline_fix::CameraPoseAngularVelocity published_msg;
sensor_msgs::Image image_msg;

// Also setup Transform Broadcaster for ROS
std::shared_ptr<tf2_ros::TransformBroadcaster> br;
geometry_msgs::TransformStamped tf_msg;

/**
* Publishes a realsense2 frame to a ROS topic
*/
int rosPublishRS2Frame(const rs2::frame& frame)
{
	// 6 DoF pose data
	if (auto pose_frame = frame.as<rs2::pose_frame>())
	{
		auto pose_data = frame.as<rs2::pose_frame>().get_pose_data();

		// Sanity check for NaN 
		if (std::isnan(pose_data.translation.x) || 
			std::isnan(pose_data.translation.y) ||  
			std::isnan(pose_data.translation.z) || 
			std::isnan(pose_data.rotation.x) || 
			std::isnan(pose_data.rotation.y) || 
			std::isnan(pose_data.rotation.z) || 
			std::isnan(pose_data.rotation.w)) 
			{
				ROS_WARN("Pose data contains NaN values.");
				return 0;
			}

		// Convert everything into ROS geometry_msgs::PoseStamped
		output_msg.header.frame_id = camera_frame;
		output_msg.header.stamp = ros::Time::now();
		output_msg.pose.position.x = -pose_data.translation.z;
		output_msg.pose.position.y = -pose_data.translation.x;
		output_msg.pose.position.z = pose_data.translation.y;
		output_msg.pose.orientation.x = -pose_data.rotation.z;
		output_msg.pose.orientation.y = -pose_data.rotation.x;
		output_msg.pose.orientation.z = pose_data.rotation.y;	
		output_msg.pose.orientation.w = pose_data.rotation.w;
		published_msg.tracker_confidence = pose_data.tracker_confidence;	// Save camera pose confidence (0 = Failed, 1 = Low, 2 = Medium, 3 = High confidence)

		// Also construct a transformation on the /tf topic between camera_frame and map
		tf_msg.header.stamp = output_msg.header.stamp;
		tf_msg.header.frame_id = "map";
		tf_msg.child_frame_id = camera_frame;
		tf_msg.transform.translation.x = output_msg.pose.position.x;
		tf_msg.transform.translation.y = output_msg.pose.position.y;
		tf_msg.transform.translation.z = output_msg.pose.position.z;
		tf_msg.transform.rotation.x = output_msg.pose.orientation.x;
		tf_msg.transform.rotation.y = output_msg.pose.orientation.y;
		tf_msg.transform.rotation.z = output_msg.pose.orientation.z;
		tf_msg.transform.rotation.w = output_msg.pose.orientation.w;
		cam_pose->publish( output_msg );
		br->sendTransform( tf_msg );
		return 1;
	}

	// 3 DoF gyroscope data
	else if (auto motion_frame = frame.as<rs2::motion_frame>())
	{
		auto imu_data = frame.as<rs2::motion_frame>().get_motion_data();

		// Sanity check for NaN 
		if (std::isnan(imu_data.x) || 
			std::isnan(imu_data.y) ||  
			std::isnan(imu_data.z)) 
			{
				ROS_WARN("Motion data contains NaN values.");
				return 0;
			}

		// Convert everything into ROS sensor_msgs::Imu
		imu_msg.header.frame_id = camera_frame;
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.angular_velocity.x = -imu_data.z;
		imu_msg.angular_velocity.y = -imu_data.x;
		imu_msg.angular_velocity.z = imu_data.y;

		// Wrap camera pose and cameras imu data into custom message type
		published_msg.pose = output_msg;
		published_msg.imu = imu_msg;
		published_msg.header = output_msg.header;

		// Publish on ROS topic
		cam_imu->publish( imu_msg );
		cam_pose_imu->publish( published_msg );
		return 1;
	}

	// Fisheye image data
	else if (auto image_frame = frame.as<rs2::video_frame>())
	{
		auto camera_data = frame.as<rs2::video_frame>();
		if (camera_data.get_profile().stream_index() != 1) {
			return 0; // ret val of 0 indicates that no msg has been published
		}
		// Convert everything into ROS sensor_msgs::Image
		image_msg.header.frame_id = camera_frame;
		image_msg.header.stamp = ros::Time::now();
		image_msg.height = camera_data.get_height();
		image_msg.width = camera_data.get_width();
		image_msg.encoding = "mono8";

		// copy data
		int stepSize = camera_data.get_stride_in_bytes();
		int dataSize = image_msg.height * stepSize;
		image_msg.data.resize(dataSize);
		memcpy(&image_msg.data[0], camera_data.get_data(), dataSize);

		// Publish on ROS topic
		cam_imgs->publish(image_msg);
		return 1;
	}
	return 0; // ret val of 1 indicates that 1 msg has been published
}

int main(int argc, char** argv) {
	
	// Only for logging
	std::map<int, int> counters;
  std::map<int, std::string> stream_names;
	
	// Mutex lock for frame pipeline
	std::mutex mutex;

	/**
	* Callback function for librealsense frames.
	* This function will recognize if the frame contains motion, pose, or image data.
	* Then it will publish the frame as a ros msgs accordingly.
	*/ 
	auto rs2FramesCallback = [&](const rs2::frame& frame) 
	{
		// Lock the mutex as we access common memory from multiple threads
		std::lock_guard<std::mutex> lock(mutex);	
		
    // All synchronized stream will arrive in a single frameset
		if (rs2::frameset fs = frame.as<rs2::frameset>())
    {
      for (const rs2::frame& f : fs)
			{	
				rosPublishRS2Frame(f);
				// Populate logging
        counters[f.get_profile().unique_id()]++;
			}
    }
    // Stream that bypass synchronization (such as IMU) will produce single frames
    else
    {
			rosPublishRS2Frame(frame);
			// Populate logging
      counters[frame.get_profile().unique_id()]++;
    }

	};

	// Setting up ROS Nodehandle
	ros::init(argc, argv, "custom_t265_node");
	ROS_INFO("Custom T265 Intel node started. Publishing %s", pose_topic);
	ros::NodeHandle nh;
	ros::Rate node_frequency(10000); //Hz
	// Init Realsense camera
	rs2::log_to_console(RS2_LOG_SEVERITY_INFO); //verbose, can make this _INFO for less or _DEBUG for more
	
	cam_pose = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1000));
	cam_imu = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::Imu>(cam_imu_topic, 1000));
	cam_pose_imu = std::make_shared<ros::Publisher>(nh.advertise<realsense_pipeline_fix::CameraPoseAngularVelocity>(cam_pose_imu_topic, 1000));
  cam_imgs = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::Image>(cam_images_topic, 1000));
	br.reset(new tf2_ros::TransformBroadcaster());

	// Setup Librealsense2 config
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 1, 848, 800, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 2, 848, 800, RS2_FORMAT_Y8, 30); // we only need left image

	// Create librealsense2 pipeline object which gets the data
	ROS_INFO("Setting up Pipeline");
	auto pipe = std::make_shared<rs2::pipeline>();
	rs2::pipeline_profile profiles = pipe->start(cfg, rs2FramesCallback);

	// Collect the enabled streams names
  for (auto p : profiles.get_streams())
  	stream_names[p.unique_id()] = p.stream_name();

  std::cout << "RealSense callback sample" << std::endl << std::endl;
  while (ros::ok())
  {
		// TODO: Detect broken pipe (maybe within the callback) and restart the pipe here
  	std::this_thread::sleep_for(std::chrono::seconds(1));
    std::lock_guard<std::mutex> lock(mutex);

    std::cout << "\r";
    for (auto p : counters)
    {
      std::cout << stream_names[p.first] << "[" << p.first << "]: " << p.second << " [frames] || ";
    }
  }

	return 0;
} // end main
