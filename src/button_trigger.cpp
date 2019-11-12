//
// Created by vickylzy on 19-11-12.
//

#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Empty.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>
#include <sensor_msgs/Joy.h>
#include <math.h>


namespace turtle_walk {

    class ButtonCaller {
    public:
        ButtonCaller() {
        }
        virtual  ~ButtonCaller(){
//            kobuki_msgs::Led led;
//            led.value=led.BLACK;
//            led_pub.publish(led);
        }

        void onInit(){
            p_nh = ros::NodeHandle("~");
            // init

            //srv client
            back_cli = nh.serviceClient<std_srvs::Trigger>("setting_start_goal_srv");
            goal_cli = nh.serviceClient<std_srvs::Trigger>("setting_new_course_srv");
            stop_cli = nh.serviceClient<std_srvs::Trigger>("stop_srv");
            switch_cli = nh.serviceClient<std_srvs::Trigger>("hand_auto_srv");
            //Topic you want to subscribe
            button_suber = nh.subscribe("/mobile_base/events/button", 2, &ButtonCaller::callback, this);
            joy_cmd_suber = nh.subscribe("/joy",2,&ButtonCaller::long_cmd_callback,this);
            reset_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1);
            led_pub = nh.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1",10, true);
            led_switch_pub = nh.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2",10, true);
            sound_pub = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1,true);


        }
        void long_cmd_callback(const sensor_msgs::Joy::ConstPtr &joy_msg){
            if(joy_msg->buttons[5]==1){
                std_srvs::Trigger trigger;
                kobuki_msgs::Led led;
                if(joy_msg->buttons[2]==1){
                    led.value=led.RED;
                    led_pub.publish(led);
                    reset_pub.publish(std_msgs::Empty());
                    back_cli.call(trigger);
                }else if(joy_msg->buttons[1]==1){
                    led.value=led.GREEN;
                    led_pub.publish(led);
                    back_cli.call(trigger);
                }else if(joy_msg->buttons[3]==1){
                    led.value=led.ORANGE;
                    led_pub.publish(led);
                    goal_cli.call(trigger);
                } else if(joy_msg->buttons[0]==1){
                    led.value=led.BLACK;
                    led_pub.publish(led);
                    stop_cli.call(trigger);
                } else if(joy_msg->buttons[11]==1){
                    switch_cli.call(trigger);
                    auto_polit=!auto_polit;
                    if(auto_polit){
                        led.value=led.RED;
                    }else{
                        led.value=led.GREEN;
                    }
                    led_switch_pub.publish(led);
                }
            }
        }

        void callback(const kobuki_msgs::ButtonEvent &input_button) {
            if (input_button.state == 0){
                std_srvs::Trigger trigger;
                kobuki_msgs::Led led;
                if (input_button.button == 0){
                    led.value=led.GREEN;
                    led_pub.publish(led);
                    back_cli.call(trigger);
                }
                else if(input_button.button == 1){
                    led.value=led.ORANGE;
                    led_pub.publish(led);
                    goal_cli.call(trigger);
                }
                else if(input_button.button == 2){
//                    kobuki_msgs::Sound sound;
//                    sound.value = sound.CLEANINGSTART;
//                    sound_pub.publish(sound);
                    led.value=led.RED;
                    led_pub.publish(led);
                    reset_pub.publish(std_msgs::Empty());
                    back_cli.call(trigger);
                }
            }
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;
        ros::Publisher reset_pub;
        ros::Publisher led_pub;
        ros::Publisher led_switch_pub;
        ros::Publisher sound_pub;

        ros::Subscriber button_suber;
        ros::Subscriber joy_cmd_suber;
        ros::ServiceClient back_cli;
        ros::ServiceClient goal_cli;
        ros::ServiceClient stop_cli;
        ros::ServiceClient switch_cli;
        bool auto_polit= false;

    };//End of class SubscribeAndPublish


}
int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "button_caller_node");
    turtle_walk::ButtonCaller buttonCaller;
    buttonCaller.onInit();
    ros::spin();

    return 0;
}