#include <cmath>


#include <ros/ros.h>
#include <ros/timer.h>
#include <tf/tf.h>
//ros_msg
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
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
            // init
            curr_position = nav_msgs::Odometry();
            velosity = 0.1;
            yaw_velo = 0.5;
            turn_reached =0.10;
            turn_threshold = 4*turn_reached;
            target_position.pose.pose.position.x=1;
            target_position.pose.pose.position.y=1.5;
            tar << target_position.pose.pose.position.x,target_position.pose.pose.position.y;
            cmd_puber = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",10);
            odom_suber = nh.subscribe("/odom", 2, &GoalReacher::odom_callback, this);
            cmd_timer = nh.createTimer(ros::Duration(0.05),&GoalReacher::velo_cmd_sender,this);
        }

        void velo_cmd_sender(const ros::TimerEvent& event){
            geometry_msgs::Twist twist;

            if(check_reached()){
                twist.linear.x=0;
                twist.angular.z=0;
            }else{
                int forward_ori = cal_ori();
                if (forward_ori!=0)
                    turn_threshold=turn_reached;
                else
                    turn_threshold = 3*turn_reached;
                twist.linear.x=velosity;
                twist.angular.z = forward_ori*yaw_velo;
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
            if(std::fabs(theta)<turn_reached)// about 20 degree
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
        Eigen::Vector2f tar;
        float velosity;
        float yaw_velo;
        float turn_reached;
        float turn_threshold;
        nav_msgs::Odometry target_position;
        nav_msgs::Odometry curr_position;

    };//End of class SubscribeAndPublish


}

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "lidar_analyse_node");
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