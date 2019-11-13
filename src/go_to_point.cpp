#include <cmath>


#include <ros/ros.h>
#include <ros/timer.h>
#include <tf/tf.h>
//ros_msg
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
//
#include <Eigen/Dense>
#include <math.h>
#include <turtle_walk/transform_utility.hpp>


namespace turtle_walk {

    using namespace std;
    using namespace Eigen;

    class GoalReacher {
    public:
        GoalReacher() {
        }
        virtual  ~GoalReacher(){
        }

        void onInit(){
            p_nh = ros::NodeHandle("~");
            // init param
            liner_velosity = p_nh.param("liner_velosity",0.3f);
            anglur_velosity = p_nh.param("anglur_velosity",0.8f);
            turn_line_theta = p_nh.param("turn_line_theta",6.00f);
            turn_line_theta = turn_line_theta/180*M_PI;
            target_x = p_nh.param("target_x",1.50f);
            target_y = p_nh.param("target_y",1.00f);

            //init
            curr_position = nav_msgs::Odometry();
            curr_position.pose.pose.position.x=0;
            curr_position.pose.pose.position.y=0;
            turn_threshold = 1.75*turn_line_theta;
            tar << 0,0;
            cmd_puber = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",10);//cmd_vel_mux/input/navi
            odom_suber = nh.subscribe("/odom", 2, &GoalReacher::odom_callback, this);
            cmd_timer = nh.createTimer(ros::Duration(0.05),&GoalReacher::velo_cmd_sender,this);
            goal_srv = nh.advertiseService("setting_new_course_srv",&GoalReacher::setting_new_goal,this);
            back_srv = nh.advertiseService("setting_start_goal_srv",&GoalReacher::setting_start_goal,this);
            stop_srv = nh.advertiseService("stop_srv",&GoalReacher::stop_motion,this);
            hand_auto_switch = nh.advertiseService("hand_auto_srv",&GoalReacher::hand_auto_change,this);
        }
        bool hand_auto_change(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res){
            auto_polit=!auto_polit;
            res.message="switch hand auto";
            res.success= true;
            return true;
        }
        bool setting_new_goal(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res){
            tar[0]=target_x;
            tar[1]=target_y;
            res.message="walking to goal";
            res.success= true;
            return true;
        }
        bool stop_motion(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res){
            tar[0]=curr_position.pose.pose.position.x;
            tar[1]=curr_position.pose.pose.position.y;
            res.message="stop at current pos";
            res.success= true;
            return true;
        }
        bool setting_start_goal(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res){
            tar[0]=0;
            tar[1]=0;
            res.message="walking to start_point";
            res.success= true;
            return true;
        }

        void velo_cmd_sender(const ros::TimerEvent& event){
            if (!auto_polit)
                return;
            geometry_msgs::Twist twist;
            if(check_reached()){
                twist.linear.x=0;
                twist.angular.z=0;
                cout<<"goal reached "<<endl;

            }else{
                int forward_ori = cal_ori();
                if (forward_ori!=0)
                    turn_threshold=turn_line_theta;
                else
                    turn_threshold = 3*turn_line_theta;
                twist.linear.x=liner_velosity;
                twist.angular.z = forward_ori*anglur_velosity;
                cout<<"forward with anglur "<<forward_ori<<endl;
            }
            cmd_puber.publish(twist);
        }

        void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){
            curr_position = *odom_msg;
        }

        bool check_reached() const {
            Eigen::Vector2f cur ( curr_position.pose.pose.position.x,curr_position.pose.pose.position.y);
            Eigen::Vector2f diff = tar-cur;
            return diff.norm()<0.1;
        }
        int cal_ori() const {
            Eigen::Vector2f cur ( curr_position.pose.pose.position.x,curr_position.pose.pose.position.y);
            Eigen::Vector2f diff = tar-cur;
            Eigen::Quaternionf quaternionf;
            quaternionf.x() = curr_position.pose.pose.orientation.x;
            Vector3f curr_ori = odom2euler(curr_position);
            float theta = atan2(diff[1],diff[0]) - curr_ori[2];
            if(std::fabs(theta)<turn_threshold)// about 20 degree
                return 0;
            else
                return sgn(theta);
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;
        ros::Publisher cmd_puber;
        ros::Subscriber odom_suber;
        //timer
        ros::Timer cmd_timer;
        //para goal
        float target_x;
        float target_y;
        Eigen::Vector2f tar;
        float liner_velosity;
        float anglur_velosity;
        float turn_line_theta;
        float turn_threshold;
        bool auto_polit= false;
        nav_msgs::Odometry curr_position;
        //service
        ros::ServiceServer goal_srv;
        ros::ServiceServer back_srv;
        ros::ServiceServer stop_srv;
        ros::ServiceServer hand_auto_switch;


    };//End of class SubscribeAndPublish


}

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "goal_reacher_node");
    turtle_walk::GoalReacher goalReacher;
    goalReacher.onInit();
    ros::spin();

    return 0;
}
//#include <ros/ros.h>
//#include <tf/tf.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Pose2D.h>
//#include <nav_msgs/Odometry.h>
//
//#include <math.h>
//
//
//namespace Goal_reacher {
//
//    class Goal_reacher {
//    public:
//        Goal_reacher() {
//        }
//        virtual  ~Goal_reacher(){
//        }
//
//        void onInit(){
//            p_nh = ros::NodeHandle("~");
//            // init
//
//            //Topic you want to publish
////            pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
//
//            //Topic you want to subscribe
//            raw_pc_suber = nh.subscribe("/subscribed_topic", 1, &Lidar_analyse::callback, this);
//        }
//
//        void callback(const sensor &input) {
//            PUBLISHED_MESSAGE_TYPE output;
//            .... do something with the input and generate the output...
//            pub_.publish(output);
//        }
//
//    private:
//        ros::NodeHandle nh;
//        ros::NodeHandle p_nh;
//        ros::Publisher pub_;
//        ros::Subscriber raw_pc_suber;
//
//    };//End of class SubscribeAndPublish
//
//    int main(int argc, char **argv) {
//        //Initiate ROS
//        ros::init(argc, argv, "lidar_analyse_node");
//        Goal_reacher goalReacher;
//        goalReacher.onInit();
//        ros::spin();
//
//        return 0;
//    }
//}
