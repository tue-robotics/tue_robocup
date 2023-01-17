#include "ros/ros.h"
#include "rosapi/SetParam.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"

geometry_msgs::Twist twist_message(float a, float b, float c, float d, float e, float f){
    geometry_msgs::Twist pub;
    pub.angular.x = a;
    pub.angular.y = b;
    pub.angular.z = c;
    pub.linear.x = d;
    pub.linear.y = e;
    pub.linear.z = f;
    return pub;
}

geometry_msgs::PoseWithCovarianceStamped pose_message(float a, float b, float c, float d, float e, float f, float g){
    geometry_msgs::PoseWithCovarianceStamped pub ;
    pub.pose.pose.position.x = a;
    pub.pose.pose.position.y = b;
    pub.pose.pose.position.z = c;

    pub.pose.pose.orientation.x = d;
    pub.pose.pose.orientation.y = e;
    pub.pose.pose.orientation.z = f;
    pub.pose.pose.orientation.w = g;
    return pub;
}

class doorOpener {

    public:
        //ros variables
        ros::NodeHandle* nh;
        ros::ServiceServer service;
        ros::Publisher chatter_pose;
        ros::Publisher chatter_twist;
        //ros::Publisher chatter_joint_body;
        //ros::Publisher chatter_joint_gripper;
        ros::Subscriber sub;
        

        boost::shared_ptr<sensor_msgs::LaserScan const> sharedLaserMessage;
        sensor_msgs::LaserScan laserMessage;

        //other variables
        bool find_end;


        //constructor 
        doorOpener(ros::NodeHandle* nh_ptr): nh(nh_ptr) {
            //initialise service and publisher
            this -> service  = nh -> advertiseService("SetParam", &doorOpener::setParam_callback, this);
            chatter_pose = nh -> advertise<geometry_msgs::PoseWithCovarianceStamped>("/hero/initialpose",1);
            chatter_twist = nh -> advertise<geometry_msgs::Twist>("/hero/base/references",1);
            //chatter_joint_body = nh -> advertise<control_msgs::FollowJointTrajectoryActionGoal>("/hero/body/joint_trajectory_action/goal",1);
            //chatter_joint_gripper = nh -> advertise<control_msgs::FollowJointTrajectoryActionGoal>("/hero/gripper_controller/follow_joint_trajectory/goal",1);
            
            }

        void go_treshold(float limite){
            while (ros::ok() && !(this -> find_end)) {

                    //wait for scan message
                    sharedLaserMessage = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hero/base_laser/scan",*nh);
                    if (sharedLaserMessage != NULL) laserMessage = *sharedLaserMessage;
                    else ROS_INFO("problem in go_treshold");

                    int position_angle_zero = int(-(sharedLaserMessage->angle_min)/sharedLaserMessage->angle_increment);
                    float distance = sharedLaserMessage->ranges[position_angle_zero];

                    if (distance <= limite) {
                        this -> find_end = true;
                    }

                    else {
                        geometry_msgs::Twist pub = twist_message(0, 0, 0, 0.05, 0, 0);
                        chatter_twist.publish(pub);
                    }

                } 
            ROS_INFO("we can grab the handle");
        }

        bool isDoorOpen(){
            //call once the sensor data in order to know if we are in front of the door
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedLaserMessage;
            sharedLaserMessage = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hero/base_laser/scan",*nh);

            if(sharedLaserMessage == NULL) {
                ROS_INFO("error in isDooropen");
            return 15;
            }

            int position_angle_zero = int(-(sharedLaserMessage->angle_min)/sharedLaserMessage->angle_increment);
            float distance = sharedLaserMessage->ranges[position_angle_zero];
            ROS_INFO("distance = %f", distance);
            return false;
            // if (distance < 0.630) {
            //     return false;
            // }
            // else return true;
        }

        bool setParam_callback(rosapi::SetParam::Request &msg_rqst, rosapi::SetParam::Response &msg_rsps) {
            ros::Rate sleeping_time(0.5);
            if(msg_rqst.name == "goIFOdoor") {
                ROS_INFO("go to the door");
                geometry_msgs::PoseWithCovarianceStamped pub1 = pose_message(6.55, 0.381, 0, 0, 0, 0.003, 0.99);
                chatter_pose.publish(pub1);
                return true;
            }

            else if(msg_rqst.name == "goIFOdoor2") {
                ROS_INFO("go to the door2");
                geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(6.5, 0.381, 0, 0, 0, 0.003, 0.99);
                chatter_pose.publish(pub3);
                return true;
            }

            else if(msg_rqst.name == "goBehindDoor") {
                ROS_INFO("go behinf the door");
                geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(8.1, 0.37, 0, 0, 0, -0.99, 0.022);
                chatter_pose.publish(pub3);
                return true;
            }

            else if (msg_rqst.name == "go_treshold") {
                ROS_INFO("going forward until threshold");
                //start the subscription to laser 
                this -> find_end = false;
                this -> go_treshold(0.60);
                return true;
            }

            else if (msg_rqst.name == "go_treshold_behind") {
                ROS_INFO("going forward until threshold");
                //start the subscription to laser 
                this -> find_end = false;
                this -> go_treshold(0.42);
                return true;
            }

            else if (msg_rqst.name == "is_door_open") {
                ROS_INFO("check of door is open or not");
                bool result = this -> isDoorOpen();
                return true;
            }

            else if (msg_rqst.name == "push_door") {
                ROS_INFO("go forward to push the door");
                geometry_msgs::Twist pub2 = twist_message(0, 0, 0, 1, 0, 0);
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                return true;
            }

            else if (msg_rqst.name == "go_forward") {
                ROS_INFO("go forward");
                geometry_msgs::Twist pub2 = twist_message(0, 0, 0, 0.18, 0, 0);
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                return true;
            }

            else {
                ROS_INFO("you sent a commande that does not exist");
                return false;
            }

            
        }
    
};


int main(int argc, char**argv) {
    ros::init(argc, argv, "open_the_door");

    ros::NodeHandle nh;

    doorOpener opener = doorOpener(&nh);

    ros::spin();

    return 15;
}


