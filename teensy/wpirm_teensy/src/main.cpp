#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "PID.h"

// ROS setup

ros::NodeHandle  nh;

// ROS subscribers and transform publishers
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// PID setup (kp, ki, kd)

PID rPid = PID(0.1, 0.01, 0);
PID lPid = PID(0.1, 0.01, 0);

// Odometry setup (meters)

#define WHEEL_BASE 0.18
#define WHEEL_RADIUS 0.06
#define WHEEL_RADIUS 4128

double x = 0.0;
double y = 0.0;
double theta = 0.0;

char base_link[] = "/base_link";
char odom[] = "/odom";

// Encoder setup

#define ENCODER_R_A 0
#define ENCODER_R_B 1
#define ENCODER_L_A 2
#define ENCODER_L_B 3
#define TICKS_PER_REV 6000.0

Encoder leftEncoder(ENCODER_L_A, ENCODER_L_B);
Encoder rightEncoder(ENCODER_R_A, ENCODER_R_B);

long lastRightEncoder = 0;
long lastLeftEncoder = 0;
unsigned long lastTime;

// Current encoder vels
double evr = 0.0;
double evl = 0.0;

// Motor set vels
double mvr = 0.0;
double mvl = 0.0;

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    broadcaster.init(nh);

    // PID set params
    rPid.setOutputLimits(-100,100);
	rPid.setOutputRampRate(10);
    lPid.setOutputLimits(-100,100);
	lPid.setOutputRampRate(10);
}

void loop() {
    transform_broadcaster()
    nh.spinOnce();
    delay(1);
}

// Odometry broadcaster
void transform_broadcaster() {
    calculate_odom();

    // tf odom->base_link
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
    
    broadcaster.sendTransform(t);
}
// TODO: Refactor to use meters and new ticks_to_meters() method.
void calculate_odom() {
    long L_ticks, R_ticks;
    double left_inches, right_inches, inches;

    L_ticks = leftEncoder.read();
    R_ticks = rightEncoder.read();

    left_inches = (double)L_ticks/LEFT_CLICKS_PER_INCH;
    right_inches = (double)R_ticks/RIGHT_CLICKS_PER_INCH;

    inches = (left_inches + right_inches) / 2.0;

    theta = (left_inches - right_inches) / WHEEL_BASE;
    theta -= (double)((int)(theta/TWOPI))*TWOPI;

    y = inches * cos(theta); 
    x = inches * sin(theta); 
}

// TODO: Finish this returns: evl and evr
void calculate_vels() {
    unsigned long curTime = millis();
    long difTime = curTime - lastTime;
    long curRightEncoder = rightEncoder.read();
    long curLeftEncoder = leftEncoder.read();

    

    lastRightEncoder = curRightEncoder;
    lastLeftncoder = curLeftEncoder;
    lastTime = curTime;
}

double ticks_to_meters(long ticks) {
    // Circumference * number of rotations
    return 2.0*PI*WHEEL_RADIUS*((double)ticks/TICKS_PER_REV);
}

void cmd_vel_cb(const geometry_msgs::Twist& msg) {
    vr = (msg.angular.z * WHEEL_BASE) / 2.0 + msg.linear.x;
	vl = msg.linear.x * 2.0 - vr;
    set_motors(vr, vl);
}

void set_motors() {
    double rOut = rPid.getOutput();
    double rOut = lPid.getOutput();
}

void set_motor(double motor, double speed) {
    if(motor == 0) {
        if (speed < 0) {
            Serial1.write(0xC5);
            Serial1.write(abs(speed));
        } 
        else {
            Serial1.write(0xC6);
            Serial1.write(speed);
        }
    }
    else if(motor == 1) {
        if (speed < 0) {
            Serial1.write(0xCD);
            Serial1.write(abs(speed));
        } 
        else {
            Serial1.write(0xCE);
            Serial1.write(speed);
        }
    }
}