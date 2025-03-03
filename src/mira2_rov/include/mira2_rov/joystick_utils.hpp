#include <ros/ros.h>
#include <custom_msgs/commands.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>

class joystick_controller{
    public:
        ros::Time                init_time = ros::Time::now();
        ros::Subscriber          joy_sub;
        ros::Publisher           base_pwm_pub;
        float                    sensitivity_all_axes, sensitivity_yaw;
        int                      prev_msg;
        int                      arm_disarm;
        custom_msgs::commands    msg_to_pub;
        joystick_controller(ros::NodeHandle nh){ 
            joy_sub              = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &joystick_controller::joyCallback, this);
            base_pwm_pub         = nh.advertise<custom_msgs::commands>("/maaster/commands",1);
            msg_to_pub.arm       = 0;
            msg_to_pub.mode      = "STABILIZE";    
            msg_to_pub.forward   = 1500;
            msg_to_pub.lateral   = 1500;
            msg_to_pub.thrust    = 1500;
            msg_to_pub.pitch     = 1500;
            msg_to_pub.yaw       = 1500;
            msg_to_pub.roll      = 1500;
            msg_to_pub.servo1    = 1500;
            msg_to_pub.servo2    = 1500;
            sensitivity_all_axes          = 0.6;
            sensitivity_yaw      = 0.5;

        }

        
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) { 

            int pitch_button       = msg->buttons[5];        
            int mode_stabilise     = msg->buttons[1];
            int mode_acro          = msg->buttons[7];
            int mode_manual        = msg->buttons[3];
            int mira_switch        = msg->buttons[8];
            int yaw_hold_button    = msg->buttons[6];
            int pitch_up_button    = msg->buttons[4];
            prev_msg               = arm_disarm;
            arm_disarm             = msg->buttons[0];
            ros::Time time_now     = ros::Time::now();
            msg_to_pub.pitch       = 1500;
            msg_to_pub.roll        = 1500+(((msg->buttons[4])*-400)+((msg->buttons[5])*400))*sensitivity_all_axes;
            msg_to_pub.thrust      = 1500+((((msg->axes[5])+1)*-200)+(((msg->axes[2])+1)*200))*sensitivity_all_axes;
            msg_to_pub.forward     = 1500+(((msg->axes[4])*400))*1;//sensitivity_all_axes;
            msg_to_pub.lateral     = 1500+(((msg->axes[3])*-400))*sensitivity_all_axes;
            msg_to_pub.yaw     = 1500+((((msg->axes[0])*-400))*sensitivity_yaw);
            
            if(arm_disarm==1 && prev_msg==0){

                if(msg_to_pub.arm==0){
                    msg_to_pub.arm=1;
                    ROS_WARN("VEHICLE ARMED");
                }
                else{
                    msg_to_pub.arm=0;
                    ROS_WARN("VEHICLE DISARMED");
                }
            }

            if (pitch_button==1) {
                msg_to_pub.pitch = 1400;
            }
            else if (pitch_up_button==1) {
                msg_to_pub.pitch = 1600;
            }
            else {
                msg_to_pub.pitch = 1500;
            }
            if(mode_stabilise==1 && msg_to_pub.mode!="STABILIZE"){
                msg_to_pub.mode="STABILIZE";
                ROS_INFO("Mode changed to STABILIZE");
            }
            if(mode_acro==1  && msg_to_pub.mode!="ACRO"){
                msg_to_pub.mode="ACRO";
                ROS_INFO("Mode changed to ACRO ");
            } 
            if(mode_manual==1  && msg_to_pub.mode!="MANUAL"){
                msg_to_pub.mode="MANUAL";
                ROS_INFO("Mode changed to MANUAL ");
            }
            base_pwm_pub.publish(msg_to_pub);     
        }
};