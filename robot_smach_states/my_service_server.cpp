#include "ros/ros.h"
//SIMULATION
//#include "opening_door/door_info.h"
//REALITY
#include "robot_smach_states/door_info.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "cb_base_navigation_msgs/LocalPlannerActionResult.h"
#include <cmath>

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

geometry_msgs::PoseStamped poseStamped_message(float a, float b, float c, float d, float e, float f, float g) {
    geometry_msgs::PoseStamped pub;
    pub.pose.position.x = a;
    pub.pose.position.y = b;
    pub.pose.position.z = c;

    pub.pose.orientation.x = d;
    pub.pose.orientation.y = e;
    pub.pose.orientation.z = f;
    pub.pose.orientation.w = g;

    return pub;
}


class doorOpener {

    public:
        //ros variables
        ros::NodeHandle* nh;
        ros::ServiceServer service;
        ros::Publisher chatter_pose;
        ros::Publisher chatter_twist;
        ros::Publisher chatter_planner;
        ros::Subscriber sub;

        boost::shared_ptr<sensor_msgs::LaserScan const> sharedLaserMessage;
        sensor_msgs::LaserScan laserMessage;

        boost::shared_ptr<cb_base_navigation_msgs::LocalPlannerActionResult const> sharedPlannerMessage;
        cb_base_navigation_msgs::LocalPlannerActionResult PlannerMessage;

        //other variables
        bool find_end;
        bool arrive_at_destination;

        //constructor
        doorOpener(ros::NodeHandle* nh_ptr): nh(nh_ptr) {
            //initialise service and publisher
            this -> service  = nh -> advertiseService("door_info", &doorOpener::doorInfo_callback, this);
            chatter_pose = nh -> advertise<geometry_msgs::PoseWithCovarianceStamped>("/hero/initialpose",1);
            chatter_twist = nh -> advertise<geometry_msgs::Twist>("/hero/base/references",1);
            chatter_planner = nh -> advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
            }

        void position_achieve() {
           
            //wait for result of action_server message
            sharedPlannerMessage = ros::topic::waitForMessage<cb_base_navigation_msgs::LocalPlannerActionResult>("/hero/local_planner/action_server/result",*nh);
            
            if (sharedPlannerMessage != NULL) {
                PlannerMessage = *sharedPlannerMessage;
                arrive_at_destination = true;
            }
            else {
                ROS_INFO("problem in position_achieve");
                arrive_at_destination = true;
            }
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
                        geometry_msgs::Twist pub = twist_message(0, 0, 0, 0.05, -0.01, 0);
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

        int getDoorState(float rot_y) {
            float size_to_check = 0.3; //the door is around 0.92m.
            //call once the sensor data in order to know if we are in front of the door
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedLaserMessage;
            sharedLaserMessage = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hero/base_laser/scan",*nh);

            if(sharedLaserMessage == NULL) {
                ROS_INFO("error in getDoorState");
                return 15;
            }

            float position_angle_zero_according_robot = -(sharedLaserMessage->angle_min)/sharedLaserMessage->angle_increment;
            //ROS_INFO("before update : %f, angle increment = %f, rot_y = %f", position_angle_zero_according_robot, sharedLaserMessage->angle_increment, rot_y);

            int position_angle_zero = int(position_angle_zero_according_robot - (rot_y/sharedLaserMessage->angle_increment)); //to take into account the rotation of the robot
            //ROS_INFO("after update : %d",position_angle_zero);
            float distance = sharedLaserMessage->ranges[position_angle_zero];
            //ROS_INFO("distance = %f", distance); //get the distance to the door
            float angle_max = atan(size_to_check/distance); //get the angle we have to ckeck the value

            int number_of_value_from_position_angle_zero = int(angle_max/sharedLaserMessage->angle_increment); //get the difference from angle 0
            //ROS_INFO("number_of_value = %d", number_of_value_from_position_angle_zero);
            //get the the position of the 2 value to check
            int value_to_check_min = position_angle_zero - number_of_value_from_position_angle_zero;
            int value_to_check_max = position_angle_zero + number_of_value_from_position_angle_zero;

            float d_angle_min = sharedLaserMessage->ranges[value_to_check_min];
            float d_angle_max = sharedLaserMessage->ranges[value_to_check_max];

            //ROS_INFO("d_angle_min = %f, d_angle_max = %f", d_angle_min, d_angle_max); //get the distance to the door

            float d_value = (pow(size_to_check,2) + pow(distance,2));
            d_value = pow(d_value,0.5);

            //ROS_INFO("d_value = %f", d_value);

            //calcul on the value we have
            if (distance > 1.4)  {
                ROS_INFO("door state : door is totally open");
                return 1;
            }

            if (abs(d_angle_max-d_angle_min)>0.08){
                ROS_INFO("door state : door is open, but not totally");
                return 2;
            }

            else {
                ROS_INFO("door state : door is close");
                return 3;
            }
        }

        bool doorInfo_callback(opening_door::door_info::Request &msg_rqst, opening_door::door_info::Response &msg_rsps) {
            ros::Rate sleeping_time(0.5);

            if(msg_rqst.input_string == "goIFOhandle") {
                ROS_INFO("go to the handle");
                geometry_msgs::PoseStamped pub1 = poseStamped_message(6.55, 0.381, 0, 0, 0, 0.001, 0.99);
                chatter_planner.publish(pub1);
                
                //Set arrive at destination on false. When it becomes tue, it means we arrive at destination
                this -> arrive_at_destination = false;

                // geometry_msgs::PoseWithCovarianceStamped pub1 = pose_message(6.55, 0.381, 0, 0, 0, 0.003, 0.99);
                // chatter_pose.publish(pub1);
                return true;
            }

            else if(msg_rqst.input_string == "goIFOhandle2") {
                ROS_INFO("go to the handle");
                geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(6.5, 0.381, 0, 0, 0, 0.003, 0.99);
                chatter_pose.publish(pub3);
                return true;
            }

            else if(msg_rqst.input_string == "goBehindDoor") {
                ROS_INFO("go behind the handle");
                geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(8.1, 0.37, 0, 0, 0, -0.99, 0.022);
                chatter_pose.publish(pub3);
                return true;
            }

            else if(msg_rqst.input_string == "goIFOdoor") {
                ROS_INFO("go IFO door");
                geometry_msgs::PoseStamped pub3 = poseStamped_message(6, 0.450, 0, 0, 0, 0.01, 0.99);
                chatter_planner.publish(pub3);
                
                //Set arrive at destination on false. When it becomes tue, it means we arrive at destination
                this -> arrive_at_destination = false;

                // geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(6, 0.450, 0, 0, 0, 0.01, 0.99);
                // chatter_pose.publish(pub3);
                return true;
            }

            else if (msg_rqst.input_string == "go_treshold") {
                ROS_INFO("going forward until threshold");
                //start the subscription to laser
                this -> find_end = false;
                //REALITY
                this -> go_treshold(0.40);
                //SIMULATION
                //this -> go_treshold(0.59);
                return true;
            }

            else if (msg_rqst.input_string == "go_treshold_behind") {
                ROS_INFO("going forward until threshold");
                //start the subscription to laser
                this -> find_end = false;
                this -> go_treshold(0.42);
                return true;
            }

            else if (msg_rqst.input_string == "is_door_open") {
                ROS_INFO("check of door is open or not");
                bool result = this -> isDoorOpen();
                return true;
            }

            else if (msg_rqst.input_string == "push_door") {
                ROS_INFO("go forward to push the door");
                geometry_msgs::Twist pub2 = twist_message(0, 0, 0, 1, 0, 0);
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                return true;
            }

            else if (msg_rqst.input_string == "go_forward") {
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

            else if (msg_rqst.input_string == "door_state") {
                //ROS_INFO("get door state");
                //start the subscription to laser
                int result = this -> getDoorState(msg_rqst.input_int);
                msg_rsps.output_int = result;
                return true;
            }

            else if (msg_rqst.input_string == "go_other_side") {
                ROS_INFO("go other side of the door");
                geometry_msgs::PoseStamped pub3 = poseStamped_message(8.1, 0.37, 0, 0, 0, -0.01, 0.99);
                chatter_planner.publish(pub3);
                return true;
            }

            else if (msg_rqst.input_string == "arrive_at_destination"){
                //wait to acheve the goal
                while(ros::ok() && !this -> arrive_at_destination){
                    this -> position_achieve();
                }
                msg_rsps.output_int = 1;  
                return true;
            }

            else {
                ROS_INFO("you sent a commande that does not exist");
                return false;
            }


        }

};


int main(int argc, char**argv) {
    ros::init(argc, argv, "door_informations");

    ros::NodeHandle nh;

    doorOpener opener = doorOpener(&nh);

    ros::spin();

    return 15;
}


