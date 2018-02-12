#include "main.h"

ros::NodeHandle  nh;

void motor_cb(const std_msgs::UInt16& cmd_msg) {
}

// ROS subscribers and transform publishers
ros::Subscriber<std_msgs::UInt16> sub("motor", motor_cb);
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// PID setup
PID pid = PID(0.1, 0.01, 0);

// Odometry vars
double x = 0.0;
double y = 0.0;
double theta = 0.0;

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    broadcaster.init(nh);

    // PID set params
    pid.setOutputLimits(-100,100);
	pid.setOutputRampRate(10);
}

void loop() {
    nh.spinOnce();
    delay(1);
}

void transform_broadcaster() {
    // tf odom->base_link
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
    
    broadcaster.sendTransform(t);
}