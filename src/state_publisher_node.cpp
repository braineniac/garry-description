#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>

class StatePublisher {
    public:
        StatePublisher();

        void publish_odom_transform();

    private:
        ros::NodeHandle nh;
        ros::Publisher joint_pub;
        ros::Subscriber imu_sub;
        tf::TransformBroadcaster broadcaster;

        void imu_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

        geometry_msgs::TwistStamped imu_pos;

        std::vector<double> odom_pos_linear  = std::vector<double>(3);
        std::vector<double> odom_pos_angular = std::vector<double>(3);
};

StatePublisher::StatePublisher() {
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    imu_sub   = nh.subscribe<geometry_msgs::TwistStamped>("/imu_pos", 10, &StatePublisher::imu_cb, this);
}

void StatePublisher::imu_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    StatePublisher::imu_pos.header = msg->header;
    StatePublisher::imu_pos.twist  = msg->twist;

    StatePublisher::odom_pos_linear[0] += msg->twist.linear.x;
    StatePublisher::odom_pos_linear[1] += msg->twist.linear.y;
    StatePublisher::odom_pos_linear[2] += 0; // skipping this, since garry doesn't fly

    StatePublisher::odom_pos_angular[0] += 0; // skipping since there is no tilt forward
    StatePublisher::odom_pos_angular[1] += 0; // skipping since there is no tilt to the side
    StatePublisher::odom_pos_angular[2] += msg->twist.angular.z;
}

void StatePublisher::publish_odom_transform() {

    geometry_msgs::TransformStamped odom_to_base_trans;

    odom_to_base_trans.header.stamp = ros::Time::now();
    odom_to_base_trans.header.frame_id="odom";
    odom_to_base_trans.child_frame_id="base_link";

    odom_to_base_trans.transform.translation.x = StatePublisher::odom_pos_linear[0];
    odom_to_base_trans.transform.translation.y = StatePublisher::odom_pos_linear[1];
    odom_to_base_trans.transform.rotation = tf::createQuaternionMsgFromYaw(StatePublisher::odom_pos_angular[2]);

    StatePublisher::broadcaster.sendTransform(odom_to_base_trans);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher_node");
    StatePublisher state_publisher_node;
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        state_publisher_node.publish_odom_transform();

        loop_rate.sleep();
    }

    return 0;

}
