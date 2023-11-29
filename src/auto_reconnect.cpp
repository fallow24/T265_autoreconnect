#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

const char* pose_topic = "/camera/pose";
const char* camera_frame = "camera_frame";

int main(int argc, char** argv) {

// Setting up ROS Nodehandle
ros::init(argc, argv, "custom_t265_node");
ROS_INFO("Custom T265 Intel node started. Publishing %s", pose_topic);
ros::NodeHandle nh;
ros::Publisher cam_pose = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1000);
geometry_msgs::PoseStamped output_msg;
bool first_msg_sent = false;

// Also setup Transform Broadcaster for ROS
tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped tf_msg;
// Init Realsense camera
rs2::log_to_console(RS2_LOG_SEVERITY_INFO); //verbose, can make this _INFO for less
rs2::config cfg;
cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_W10);

auto pipe = std::make_shared<rs2::pipeline>();
pipe->start(cfg);
unsigned int to = 20000;

// Get Data 
while (ros::ok())
{
    try {
            
        auto frames = pipe->wait_for_frames(to);
        to = 40;
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
	
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
				
				// Publish on ROS topic
				cam_pose.publish( output_msg );
				br.sendTransform( tf_msg );	

				if (!first_msg_sent)
				{
					first_msg_sent = true;
					ROS_INFO("First msg received!");
				}

				ros::spinOnce();	
	} catch (const rs2::error& e) {
        to = 20000;
        ROS_INFO("T265 stopped working. Try reconnecting...");
				pipe->stop();
        pipe = std::make_shared<rs2::pipeline>();
        pipe->start(cfg);
        ROS_INFO("... restarted.");
    }
}

} // end main
