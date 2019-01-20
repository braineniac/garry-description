#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#define WHEEL_RADIUS 2.7

int main(int argc, char** argv) {
	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(30);

	const double degree = M_PI/180;

	 // robot state
	 double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

	 // message declarations
	 sensor_msgs::JointState joint_state;
	 geometry_msgs::TransformStamped odom_trans_right_wheel;
	 geometry_msgs::TransformStamped odom_trans_left_wheel;
	 geometry_msgs::TransformStamped odom_trans_base;
	 
	 odom_trans_base.header.frame_id="odom";
	 odom_trans_base.child_frame_id="base_link";
	 odom_trans_right_wheel.header.frame_id = "right_gearbox";
	 odom_trans_right_wheel.child_frame_id = "right_wheel";
 	 odom_trans_left_wheel.header.frame_id = "left_gearbox";
	 odom_trans_left_wheel.child_frame_id = "left_wheel";


	 while (ros::ok()) {
		 //update joint_state
		 
		 joint_state.header.stamp = ros::Time::now();
		 joint_state.name.resize(1);
		 joint_state.position.resize(1);
		 joint_state.name[0] ="swivel";
		 joint_state.position[0] = swivel;
		 
		 odom_trans_base.header.stamp = ros::Time::now();
		 odom_trans_base.transform.translation.x = 0;
		 odom_trans_base.transform.translation.y = sqrt(2*2.7*2.7*(1-(cos(angle))));
		 odom_trans_base.transform.translation.z = 0;
		 odom_trans_base.transform.rotation = tf::createQuaternionMsgFromYaw(0);



		 odom_trans_right_wheel.header.stamp = ros::Time::now();
		 odom_trans_right_wheel.transform.translation.x = 4.575;
		 odom_trans_right_wheel.transform.translation.y = -4.5;
		 odom_trans_right_wheel.transform.translation.z = -3.3;
		 odom_trans_right_wheel.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(-angle+M_PI/2,0,0);

		 odom_trans_left_wheel.header.stamp = ros::Time::now();
		 odom_trans_left_wheel.transform.translation.x = -4.575;
		 odom_trans_left_wheel.transform.translation.y = -4.5;
		 odom_trans_left_wheel.transform.translation.z = -3.3;
		 odom_trans_left_wheel.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(-angle+M_PI/2,0,0);

		 //send the joint state and transform
		 joint_pub.publish(joint_state);
		 broadcaster.sendTransform(odom_trans_base);
		 broadcaster.sendTransform(odom_trans_right_wheel);
		 broadcaster.sendTransform(odom_trans_left_wheel);


		 // Create new robot state
//		 tilt += tinc;
//		 if (tilt<-.5 || tilt>0) tinc *= -1;
//		 height += hinc;
//		 if (height>.2 || height<0) hinc *= -1;
		 swivel += degree;
		 angle += degree/4;

		 // This will adjust as needed per iteration
		 loop_rate.sleep();
	 }

	 return 0;

}
