#include <ros/ros.h>
#include <Eigen/Dense>
#include <ros/callback_queue.h>
#include <string>
#include <future>

#include <sensor_msgs/Joy.h>

#include <geometry_msgs/Twist.h>

#include <crazyflie_driver/Stop.h>
#include <crazyflie_driver/UpdateParams.h>
#include <crazyflie_driver/Land.h>
#include <crazyflie_driver/Takeoff.h>
#include <crazyflie_driver/Position.h>
#include <crazyflie_driver/Hover.h>
#include <crazyflie_driver/GenericLogData.h>


#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#define ATT_CONTROL_TOPIC                               "cmd_vel"
#define POS_CONTROL_TOPIC                               "cmd_position"
#define VEL_CONTROL_TOPIC                               "cmd_hover"
#define STOP_TOPIC                                      "stop"
#define TAKEOFF_TOPIC                                   "takeoff"
#define LAND_TOPIC                                      "land"
#define UPDATE_PARAMS_TOPIC                             "update_params"

#define DEG_2_RAD 3.14159265359/180.0

#define POS_VEL_CONTROL                 3
#define ACC_ATT_CONTROL                 5     

#define MAIN_LOOP_RATE             		100            		//main loop rate for getting more faster all the subscriber datas

#define PI_                   			180.0		//3.1415926535897
#define PITCH_MAX               		(PI_/20.0)    		//Max pitch angle allowed
#define ROLL_MAX                		(PI_/20.0)    		//Max roll angle allowed
#define YAW_MAX_RATE                    (PI_/3.0)           //in deg per seg

#define THROTTLE_MAX             		55000           	//Max throttle allowed

#define STEP_Z_RATE                     0.05
#define STEP_Y_X_RATE                   0.05
#define STEP_ROLL_PITCH                 (PI_/90.0)          //increase the roll and yaw max value by this at every key pressed
#define STEP_YAW_RATE                   (PI_/10.0)          //increase max yaw_rate by this
//Joystick configuration

#define THROTTLE_AXE            		1            		//Up/Down left axis
#define YAW_LEFT_AXE            		2            		//LT
#define YAW_RIGHT_AXE            		5            		//RT
#define PITCH_AXE                		4            		//Up/down right axis          
#define ROLL_AXE                		3            		//left/right right axis

#define POS_ATT		                   	5					//RB
#define VEL_ACC                         4                   //LB

#define ARM_MOTOR                		7            		//Start button
#define POS_VEL 						2					//X button
#define ATT_ACC 						0					//A button
#define TAKEOFF							1					//B button
#define LAND							3					//Y button

#define STEP_Z_INCR                		13            		//cross key up
#define STEP_Z_DECR                		14            		//cross key down
#define STEP_LEFT_RIGHT_INCR    		12            		//cross key right
#define STEP_LEFT_RIGHT_DECR    		11            		//cross key left 	   


//Publisher for differents kind of quad control
ros::Publisher pos_control_pub;
ros::Publisher att_control_pub;
ros::Publisher vel_control_pub;

ros::ServiceClient stop_client;                            //Stop service client
ros::ServiceClient takeoff_client;                         //Takeoff service client
ros::ServiceClient land_client;                            //Land service client
ros::ServiceClient update_params_client;                   //Update_params service client

//position topic name
std::string positionTopic;

//Axes mapping for moving around the quad
int left_axe_up_down;                          //Up|down left axis
int left_axe_left_right;                       //Left/Right left axis
int yaw_left_axe;                              //LT joystick
int yaw_right_axe;                             //Rt joystick
int right_axe_up_down;                         //Up|down right axes
int right_axe_left_right;                      //Left|right right axes 

//Button action to trigger control state changes
int LB;                                        //Lb button
int RB;                                        //Rb button

int B;                                         //B button
int Y;                                         //Y button

int X;                                         //X button
int A;                                         //A button

int vertical_rate_incr;                        //cross key up
int vertical_rate_decr;                        //cros key down
int horizontal_rate_incr;                      //cross key right
int horizontal_rate_decr;                      //cross key left

int start;                                     //Start button

//attitude control max values
double pitch_max = PITCH_MAX;
double roll_max = ROLL_MAX;
double yaw_max_rate = YAW_MAX_RATE;
double throttle_max = THROTTLE_MAX;

//velocity control step for  x,y and z axis
double max_speed_z = 1.6;
double max_speed_y = 1.3;
double max_speed_x = 1.3;

// Takeoff and landing parameters
double takeoff_height = 0.5; // meters
double takeoff_duration = 2.0; // seconds
double land_duration = 4.0; //seconds
int groupMask = 0;

//Main values to send like position, velocity and acceleration
crazyflie_driver::Position pos_control_msg;
geometry_msgs::Twist att_control_msg;
crazyflie_driver::Hover vel_control_msg;

//Getting feedback from the fcu
geometry_msgs::Point current_pose;
double mc_given_yaw;
bool contain_yaw = false;

double current_yaw;
geometry_msgs::Point current_pos;

//store current control state of the quad
unsigned char current_state = -1;

//save last ARM button press status
bool last_is_arm = false;
bool is_arm = false;

//save last state of takeoff and land
bool last_is_takeoff = false;
bool is_taking_off = false;

bool last_is_landing = false;
bool is_landing = false; 

// bool current quad state
bool is_posctl = false;
bool is_attctl = false;
bool is_velctl = false;

//Store axes moves of the joystick
double up_down(0) ,  left_right(0) , back_forward(0) , yaw(0);


//Main joy callback function for dealing with Joystick event
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){

    bool reset_control_type = false;

    // Arm or disarm motors
    if (msg->buttons[start] == 1 && !last_is_arm){
        is_arm = !is_arm;
        if (!is_arm){
            /*crazyflie_driver::Stop stop_srv;
            stop_srv.request.groupMask = groupMask;
            stop_client.call(stop_srv);*/
            is_posctl = false;
            is_velctl = false;
            is_attctl = false;
            is_taking_off = false;
            is_landing = false;
        }
    }

    //Take off started
    if (msg->buttons[B] == 1 && !last_is_takeoff ){
        is_posctl = false;
        is_velctl = false;
        is_attctl = false;
        is_taking_off = true;
        is_landing = false;
        is_arm = true;
        crazyflie_driver::Takeoff takeoff_srv;
        takeoff_srv.request.height = takeoff_height;
        takeoff_srv.request.duration = ros::Duration(takeoff_duration);
        takeoff_srv.request.groupMask = groupMask;
        takeoff_client.call(takeoff_srv);
    }

    //landing started
    if (msg->buttons[Y] == 1 && !last_is_landing){
        is_posctl = false;
        is_velctl = false;
        is_attctl = false;
        is_taking_off = false;
        is_landing = true;
        is_arm = false;
        crazyflie_driver::Land land_srv;
        land_srv.request.height = 0;
        land_srv.request.duration = ros::Duration(land_duration);
        land_srv.request.groupMask = groupMask;
        land_client.call(land_srv);
    }

    //Enable attitude control
    if (msg->buttons[A] == 1){
        current_state = ACC_ATT_CONTROL;
        ROS_WARN("ATTITUDE CONTROL SELECTED ! ");
    }

    //Enable position or velocity control
    if (msg->buttons[X] == 1){
        is_attctl = false;
        current_state = POS_VEL_CONTROL;
        ROS_WARN("POS/VEL CONTROL SELECTED ! ");
    }

    //Enable position control
    if(current_state == POS_VEL_CONTROL && msg->buttons[RB] ==1){
        is_attctl = false;
        is_posctl = true;
        is_velctl = false;
        is_taking_off = false;
        is_landing = false;
        is_arm = true;
        reset_control_type = true;
        if (contain_yaw)
            current_yaw = mc_given_yaw;
        ROS_WARN("POSITION CONTROL ACTIVATED ! ");
    }

    //Enable Velocity control
    if(current_state == POS_VEL_CONTROL && msg->buttons[LB] ==1){
        is_attctl = false;
        is_posctl = false;
        is_velctl = true;
        is_taking_off = false;
        is_landing = false;
        is_arm = true;
        reset_control_type = true;
        ROS_WARN("VELOCITY CONTROL ACTIVATED !");
    }

    //Enable attitude control
    if(current_state == ACC_ATT_CONTROL && msg->buttons[RB] == 1){
        is_attctl = true;
        is_posctl = false;
        is_velctl = false;
        is_taking_off = false;
        is_landing = false;
        is_arm = true;
        reset_control_type = true;
        ROS_WARN("ATTITUDE CONTROL ACTIVATED !");
    }

    //Make a call to the service if a type of control is requested
    if(reset_control_type){
        // current_yaw = mc_given_yaw;
        current_pos = current_pose;
    }

    //Saving moves command
    up_down = msg->axes[left_axe_up_down];
    left_right = msg->axes[right_axe_left_right];
    back_forward = msg->axes[right_axe_up_down];
    yaw = -((msg->axes[yaw_left_axe] - 1.0)/2.0) + ((msg->axes[yaw_right_axe] - 1.0)/2.0);

    if ( msg->axes[vertical_rate_incr]==1 ||  msg->axes[horizontal_rate_incr]==1){
        if(is_attctl){
            yaw_max_rate += msg->axes[vertical_rate_incr]*STEP_YAW_RATE; //- msg->buttons[vertical_rate_decr]*STEP_YAW_RATE;
            pitch_max += msg->axes[horizontal_rate_incr]*STEP_ROLL_PITCH; // - msg->buttons[horizontal_rate_decr]*STEP_ROLL_PITCH;
            roll_max  += msg->axes[horizontal_rate_incr]*STEP_ROLL_PITCH; //- msg->buttons[horizontal_rate_decr]*STEP_ROLL_PITCH;
            ROS_WARN("[ATTITUDE CONTROL] CURRENT pitch_max , roll_max , yaw_rate_max : %f   , %f   , %f ", pitch_max , roll_max , yaw_max_rate);
        }else if(is_velctl || is_posctl ){
            max_speed_z += msg->axes[vertical_rate_incr]*STEP_Z_RATE; // - msg->buttons[vertical_rate_decr]*STEP_Z_RATE;
            max_speed_y += msg->axes[horizontal_rate_incr]*STEP_Y_X_RATE; //- msg->buttons[horizontal_rate_decr]*STEP_Y_X_RATE;
            max_speed_x += msg->axes[horizontal_rate_incr]*STEP_Y_X_RATE; //- msg->buttons[horizontal_rate_decr]*STEP_Y_X_RATE;
            ROS_WARN("[SPEED RATES] CURRENT vz_max , vy_max , vx_max : %f  , %f  , %f ", max_speed_z , max_speed_y , max_speed_x);
        }
    }

    //Resetting is_arm variable for being able to toggle between armed and disarmed 
    last_is_arm = (msg->buttons[start] == 1);
    last_is_takeoff = (msg->buttons[B] == 1);
    last_is_landing = (msg->buttons[Y] == 1);

}

//Pose data callback
void curr_pos_callback(const crazyflie_driver::GenericLogData::ConstPtr& msg){
    current_pose.x = msg->values[0];
    current_pose.y = msg->values[1];
    current_pose.z = msg->values[2];
    if (msg->values.size() == 6){
        contain_yaw = true;
        mc_given_yaw =  msg->values[5] * DEG_2_RAD;
    }
    // current_pose = msg->pose;
    // mc_given_yaw = tf::getYaw(msg->pose.orientation);
}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"joy_command_node");
    ros::NodeHandle nh_params("~");

    if (!nh_params.getParam("positionTopic", positionTopic)){
        positionTopic = "position";
        ROS_WARN("No parameter positionTopic provided. Using default value %s !", positionTopic.c_str());
    }

    if(!nh_params.getParam("LB",LB)){
        LB = VEL_ACC;
        ROS_WARN("No parameter LB provided. Using default value %d !",LB);
    }

    if(!nh_params.getParam("RB",RB)){
        RB = POS_ATT;
        ROS_WARN("No parameter RB provided. Using default value %d !",RB);
    }

    if(!nh_params.getParam("LT",yaw_left_axe)){
        yaw_left_axe = YAW_LEFT_AXE;
        ROS_WARN("No parameter LT provided. Using default value %d !",yaw_left_axe);
    }

    if(!nh_params.getParam("RT",yaw_right_axe)){
        yaw_right_axe = YAW_RIGHT_AXE;
        ROS_WARN("No parameter RT provided. Using default value %d !", yaw_right_axe);
    }

    if(!nh_params.getParam("Start",start)){
        start = ARM_MOTOR;
        ROS_WARN("No parameter Start provided. Using default value %d !",start);
    }

    if(!nh_params.getParam("X", X)){
        X = POS_VEL;
        ROS_WARN("No parameter X provided. Using default value %d !",X);
    }

    if(!nh_params.getParam("Y" , Y)){
        Y = LAND;
        ROS_WARN("No parameter Y provided. Using default value %d !",Y);
    }

    if(!nh_params.getParam("A" , A)){
        A = ATT_ACC;
        ROS_WARN("No parameter A provided. Using default value %d !",A);
    }

    if(!nh_params.getParam("B" , B)){
        B = TAKEOFF;
        ROS_WARN("No parameter B provided. Using default value %d !", B);
    }

    if(!nh_params.getParam("up_down_axis_left" , left_axe_up_down)){
        left_axe_up_down = THROTTLE_AXE;
        ROS_WARN("No parameter up_down_axis_left provided. Using default value %d !",left_axe_up_down);
    }

    if(!nh_params.getParam("left_right_axis_right" , right_axe_left_right)){
        right_axe_left_right = ROLL_AXE;
        ROS_WARN("No parameter left_right_axis_right provided. Using default value %d !",right_axe_left_right);
    }

    if(!nh_params.getParam("up_down_axis_right" , right_axe_up_down)){
        right_axe_up_down = PITCH_AXE;
        ROS_WARN("No parameter up_down_axis_right provided. Using default value %d !",right_axe_up_down);
    }

    if(!nh_params.getParam("left_cross_key", horizontal_rate_decr)){
        horizontal_rate_decr = STEP_LEFT_RIGHT_DECR;
        ROS_WARN("No parameter left_cross_key provided. Using default value %d !",horizontal_rate_decr);
    }

    if(!nh_params.getParam("right_cross_key" , horizontal_rate_incr)){
        horizontal_rate_incr = STEP_LEFT_RIGHT_INCR;
        ROS_WARN("No parameter right_cross_key provided. Using default value %d !",horizontal_rate_incr);
    }

    if(!nh_params.getParam("down_cross_key" , vertical_rate_decr)){
        vertical_rate_decr = STEP_Z_DECR;
        ROS_WARN("No parameter down_cross_key provided. Using default value %d !",vertical_rate_decr);    
    }

    if(!nh_params.getParam("up_cross_key" , vertical_rate_incr)){
        vertical_rate_incr = STEP_Z_INCR;
        ROS_WARN("No parameter up_cross_key provided. Using default value %d !",vertical_rate_incr); 
    }

    if(!nh_params.getParam("throttle_max" , throttle_max)){
        throttle_max = THROTTLE_MAX;
        ROS_WARN("No parameter throttle_max provided. Using default value %f deg!",throttle_max); 
    }

    if(!nh_params.getParam("yaw_rate_max" , yaw_max_rate)){
        yaw_max_rate = YAW_MAX_RATE;
        ROS_WARN("No parameter yaw_rate_max provided. Using default value %f deg!",180.0*yaw_max_rate/PI_); 
    }

    if(!nh_params.getParam("pitch_max" , pitch_max)){
        pitch_max = PITCH_MAX;
        ROS_WARN("No parameter pitch_max provided. Using default value %f deg!",180.0 * pitch_max/PI_); 
    }

    if(!nh_params.getParam("roll_max" , roll_max)){
        roll_max = ROLL_MAX;
        ROS_WARN("No parameter roll_max provided. Using default value %f deg!",180.0 * roll_max /PI_); 
    }

    if (!nh_params.getParam("takeoff_height" , takeoff_height)){
        ROS_WARN("No parameter takeoff_height provided. Using default %f m", takeoff_height);
    }
    if (!nh_params.getParam("takeoff_duration" , takeoff_duration)){
        ROS_WARN("No parameter takeoff_duration provided. Using default %f s", takeoff_duration);
    }
    if (!nh_params.getParam("land_duration" , land_duration)){
        ROS_WARN("No parameter land_duration provided. Using default %f s", land_duration);
    }
    if (!nh_params.getParam("groupMask" , groupMask)){
        ROS_WARN("No parameter groupMask provided. Using default %d ", groupMask);
    }

    bool only_command;
    if(!nh_params.getParam("only_command" , only_command)){
        only_command = false;
        ROS_WARN("No parameter only_command provided. Using default value %d !",only_command); 
    }

    //Appropriate node_handle
    ros::CallbackQueue m_callback_queue;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&m_callback_queue);

    //Publishers
    pos_control_pub = nh.advertise<crazyflie_driver::Position>(POS_CONTROL_TOPIC , 10);
    att_control_pub = nh.advertise<geometry_msgs::Twist>(ATT_CONTROL_TOPIC , 10);
    vel_control_pub = nh.advertise<crazyflie_driver::Hover>(VEL_CONTROL_TOPIC , 10);

    //Subscribers
    ros::Subscriber joy_sub =nh.subscribe<sensor_msgs::Joy>("joy",10,joy_callback);
    ros::Subscriber pose_subscriber = nh.subscribe<crazyflie_driver::GenericLogData>(positionTopic,10,curr_pos_callback);

    //Service
    takeoff_client = nh.serviceClient<crazyflie_driver::Takeoff>(TAKEOFF_TOPIC);
    land_client = nh.serviceClient<crazyflie_driver::Land>(LAND_TOPIC);
    stop_client = nh.serviceClient<crazyflie_driver::Stop>(STOP_TOPIC);
    update_params_client =  nh.serviceClient<crazyflie_driver::UpdateParams>(UPDATE_PARAMS_TOPIC);
    
    //Main loop rate
    ros::Rate main_rate(MAIN_LOOP_RATE);

    // Need to arm a first time before anything else
    while (!last_is_arm){
        m_callback_queue.callAvailable(ros::WallDuration(0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    crazyflie_driver::UpdateParams m_params;
    m_params.request.params.push_back("commander/enHighLevel");
    ros::param::set("commander/enHighLevel" , 1);
    update_params_client.call(m_params);

    //setting initial yaw and initial position
    if (contain_yaw)
	   current_yaw = mc_given_yaw;
    else
        current_yaw = 0;
	current_pos = current_pose;

	ROS_WARN("[JOY NODE ]:  initial pos : %f , %f , %f ",current_pos.x , current_pos.y , current_pos.z);

    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration dt;

    while(ros::ok()){

        current_time = ros::Time::now();
        dt = current_time - last_time;
        last_time = current_time;

        if (!is_posctl && !is_attctl && !is_velctl && !is_taking_off && !is_landing){
            /*geometry_msgs::Twist arm_twist_msg;
            if (is_arm)
                arm_twist_msg.linear.z = 10000;
            else
                arm_twist_msg.linear.z = 0;
            att_control_pub.publish(arm_twist_msg);*/
        }

        if(!only_command && is_posctl){       //Position control
            current_pos.x += back_forward * max_speed_x * dt.toSec();
            current_pos.y += left_right * max_speed_y * dt.toSec();
            current_pos.z += up_down * max_speed_z * dt.toSec();
            current_yaw += yaw * (yaw_max_rate) * dt.toSec();
            pos_control_msg.header.seq += 1;
            pos_control_msg.header.stamp = ros::Time::now();
            pos_control_msg.yaw = current_yaw;
            pos_control_msg.x = current_pos.x;
            pos_control_msg.y = current_pos.y;
            pos_control_msg.z = current_pos.z;
            pos_control_pub.publish(pos_control_msg);
        }
        if(!only_command && is_velctl){   //Velocity control state
            current_pos.z += up_down * max_speed_z * dt.toSec();
            vel_control_msg.header.seq += 1;
            vel_control_msg.header.stamp = ros::Time::now();
            vel_control_msg.vx = back_forward * max_speed_x;
            vel_control_msg.vy = left_right * max_speed_y;
            vel_control_msg.zDistance = current_pos.z; // up_down * max_speed_z;
            vel_control_msg.yawrate = - yaw * (yaw_max_rate);
            current_yaw += yaw * (yaw_max_rate) * dt.toSec();
            vel_control_pub.publish(vel_control_msg);
        }
        if(!only_command && is_attctl){       //Attitude control
            att_control_msg.linear.y = - left_right * roll_max;
            att_control_msg.linear.x = back_forward * pitch_max;
            current_yaw += yaw * (yaw_max_rate) * dt.toSec();
            att_control_msg.linear.z = up_down * throttle_max ;
            att_control_msg.angular.z = - yaw * (yaw_max_rate);
            att_control_pub.publish(att_control_msg);
        }

        m_callback_queue.callAvailable(ros::WallDuration(0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ros::param::set("commander/enHighLevel" , 0);
    update_params_client.call(m_params);

    return 0;
}