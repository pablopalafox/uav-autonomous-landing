/*
 * summit_xl_pad
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the roboy controller, sending the messages received from the joystick device
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <robotnik_msgs/ptz.h>
// Not yet catkinized 9/2013
// #include <sound_play/sound_play.h>
#include <unistd.h>
#include <robotnik_msgs/set_mode.h>
#include <summit_xl_pad/enable_disable_pad.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/ptz.h>
#include <robotnik_msgs/home.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_LINEAR_Y       0
#define DEFAULT_AXIS_ANGULAR		2
#define DEFAULT_AXIS_LINEAR_Z       3	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		2.0
#define DEFAULT_SCALE_LINEAR_Z      1.0 

    
////////////////////////////////////////////////////////////////////////
//                               NOTE:                                //
// This configuration is made for a THRUSTMASTER T-Wireless 3in1 Joy  //
//   please feel free to modify to adapt for your own joystick.       //   
// 								      //


class SummitXLPad
{
	public:
	SummitXLPad();
	void Update();

	private:
	void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
	bool EnableDisablePad(summit_xl_pad::enable_disable_pad::Request &req, summit_xl_pad::enable_disable_pad::Response &res );

	ros::NodeHandle nh_;

	int linear_x_, linear_y_, linear_z_, angular_;
	double l_scale_, a_scale_, l_scale_z_; 
	//! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
	ros::Publisher vel_pub_, ptz_pub_;
	//! It will be suscribed to the joystick
	ros::Subscriber pad_sub_;
	//! Name of the topic where it will be publishing the velocity
	std::string cmd_topic_vel_;
	//! Name of the service where it will be modifying the digital outputs
	std::string cmd_service_io_;
	//! Name of the topic where it will be publishing the pant-tilt values	
	std::string cmd_topic_ptz_;
	double current_vel;
	//! Number of the DEADMAN button
	int dead_man_button_;
	//! Number of the button for increase or decrease the speed max of the joystick	
	int speed_up_button_, speed_down_button_;
	int button_output_1_, button_output_2_;
	int output_1_, output_2_;
	bool bOutput1, bOutput2;
	//! button to change kinematic mode
	int button_kinematic_mode_;
	//! kinematic mode
	int kinematic_mode_;
	//! Service to modify the kinematic mode
	ros::ServiceClient setKinematicMode;  
	//! Name of the service to change the mode
	std::string cmd_set_mode_;
        //! button to start the homing service
	int button_home_;
	//! Service to start homing
	ros::ServiceClient doHome;
	//! Name of the service to do the homing
	std::string cmd_home_;
	//! buttons to the pan-tilt-zoom camera
	int ptz_tilt_up_, ptz_tilt_down_, ptz_pan_right_, ptz_pan_left_;
	int ptz_zoom_wide_, ptz_zoom_tele_;	
	//! Enables/disables the pad
	ros::ServiceServer enable_disable_srv_;
	//! Service to modify the digital outputs
	ros::ServiceClient set_digital_outputs_client_;  
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; // 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; // 	
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;
	//! Client of the sound play service
	//  sound_play::SoundClient sc;
	//! Pan & tilt increment (degrees)
	int pan_increment_, tilt_increment_;
	//! Zoom increment (steps)
	int zoom_increment_;
	//! Add a dead zone to the joystick that controls scissor and robot rotation (only useful for xWam) 
	std::string joystick_dead_zone_;
};


SummitXLPad::SummitXLPad():
  linear_x_(1),
  linear_y_(0),
  angular_(2),
  linear_z_(3)
{
	current_vel = 0.1;
	// 
	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	// MOTION CONF
	nh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
  nh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
	nh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
	nh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
	nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
	nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
	nh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
	nh_.param("cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_);
	nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
	nh_.param("button_speed_up", speed_up_button_, speed_up_button_);  //4 Thrustmaster
	nh_.param("button_speed_down", speed_down_button_, speed_down_button_); //5 Thrustmaster
	nh_.param<std::string>("joystick_dead_zone", joystick_dead_zone_, "true");
	
	// DIGITAL OUTPUTS CONF
	nh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
	nh_.param("button_output_1", button_output_1_, button_output_1_);
	nh_.param("button_output_2", button_output_2_, button_output_2_);
	nh_.param("output_1", output_1_, output_1_);
	nh_.param("output_2", output_2_, output_2_);
	// PANTILT-ZOOM CONF
	nh_.param("cmd_topic_ptz", cmd_topic_ptz_, cmd_topic_ptz_);
	nh_.param("button_ptz_tilt_up", ptz_tilt_up_, ptz_tilt_up_);
	nh_.param("button_ptz_tilt_down", ptz_tilt_down_, ptz_tilt_down_);
	nh_.param("button_ptz_pan_right", ptz_pan_right_, ptz_pan_right_);
	nh_.param("button_ptz_pan_left", ptz_pan_left_, ptz_pan_left_);
	nh_.param("button_ptz_zoom_wide", ptz_zoom_wide_, ptz_zoom_wide_);
	nh_.param("button_ptz_zoom_tele", ptz_zoom_tele_, ptz_zoom_tele_);
  nh_.param("button_home", button_home_, button_home_);
	nh_.param("pan_increment", pan_increment_, 1);
	nh_.param("tilt_increment",tilt_increment_, 1);
	nh_.param("zoom_increment", zoom_increment_, 1);

	// KINEMATIC MODE 
	nh_.param("button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_);
	nh_.param("cmd_service_set_mode", cmd_set_mode_, cmd_set_mode_);
  nh_.param("cmd_service_home", cmd_home_, cmd_home_);
	kinematic_mode_ = 1;
	
	ROS_INFO("SummitXLPad num_of_buttons_ = %d", num_of_buttons_);	
	for(int i = 0; i < num_of_buttons_; i++){
		bRegisteredButtonEvent[i] = false;
		ROS_INFO("bREG %d", i);
		}

	/*ROS_INFO("Service I/O = [%s]", cmd_service_io_.c_str());
	ROS_INFO("Topic PTZ = [%s]", cmd_topic_ptz_.c_str());
	ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());
	ROS_INFO("Axis linear = %d", linear_);
	ROS_INFO("Axis angular = %d", angular_);
	ROS_INFO("Scale angular = %d", a_scale_);
	ROS_INFO("Deadman button = %d", dead_man_button_);
	ROS_INFO("OUTPUT1 button %d", button_output_1_);
	ROS_INFO("OUTPUT2 button %d", button_output_2_);
	ROS_INFO("OUTPUT1 button %d", button_output_1_);
	ROS_INFO("OUTPUT2 button %d", button_output_2_);*/	

  	// Publish through the node handle Twist type messages to the guardian_controller/command topic
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
	//  Publishes msgs for the pant-tilt cam
	ptz_pub_ = nh_.advertise<robotnik_msgs::ptz>(cmd_topic_ptz_, 1);

 	// Listen through the node handle sensor_msgs::Joy messages from joystick 
	// (these are the references that we will sent to summit_xl_controller/command)
	pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SummitXLPad::padCallback, this);
	
 	// Request service to activate / deactivate digital I/O
	set_digital_outputs_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(cmd_service_io_);
	bOutput1 = bOutput2 = false;

        // Request service to set kinematic mode 
	setKinematicMode = nh_.serviceClient<robotnik_msgs::set_mode>(cmd_set_mode_);

        // Request service to start homing
	doHome = nh_.serviceClient<robotnik_msgs::home>(cmd_home_);

	// Diagnostics
	updater_pad.setHardwareID("None");
	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel_.c_str(), updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));

	// Advertises new service to enable/disable the pad
	enable_disable_srv_ = nh_.advertiseService("/summit_xl_pad/enable_disable_pad",  &SummitXLPad::EnableDisablePad, this);

	bEnable = true;	// Communication flag enabled by default
}


/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void SummitXLPad::Update(){
	updater_pad.update();
}

/*
 *	\brief Enables/Disables the pad
 *
 */
bool SummitXLPad::EnableDisablePad(summit_xl_pad::enable_disable_pad::Request &req, summit_xl_pad::enable_disable_pad::Response &res )
{
	bEnable = req.value;

	ROS_INFO("SummitXLPad::EnablaDisablePad: Setting to %d", req.value);
	res.ret = true;
	return true;
}

void SummitXLPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
	robotnik_msgs::ptz ptz;
    bool ptzEvent = false;

	vel.angular.x = 0.0;  vel.angular.y = 0.0; vel.angular.z = 0.0;
	vel.linear.x = 0.0;   vel.linear.y = 0.0; vel.linear.z = 0.0;

  	// Actions dependant on dead-man button
 	if (joy->buttons[dead_man_button_] == 1) {
		//ROS_ERROR("SummitXLPad::padCallback: DEADMAN button %d", dead_man_button_);
		// Set the current velocity level
		if ( joy->buttons[speed_down_button_] == 1 ){

			if(!bRegisteredButtonEvent[speed_down_button_]) 
				if(current_vel > 0.1){
		  			current_vel = current_vel - 0.1;
					bRegisteredButtonEvent[speed_down_button_] = true;
					ROS_INFO("Velocity: %f%%", current_vel*100.0);	
					char buf[50]="\0";
 					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}	 	
		}else{
			bRegisteredButtonEvent[speed_down_button_] = false;
		 }
		 
		if (joy->buttons[speed_up_button_] == 1){
			if(!bRegisteredButtonEvent[speed_up_button_])
				if(current_vel < 0.9){
					current_vel = current_vel + 0.1;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Velocity: %f%%", current_vel*100.0);
  					char buf[50]="\0";
					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}
		  
		}else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		
		
		
		
		vel.linear.x = current_vel*l_scale_*joy->axes[linear_x_];
		vel.linear.y = current_vel*l_scale_*joy->axes[linear_y_];
		
		if(joystick_dead_zone_=="true")
		{
			// limit scissor movement or robot turning (they are in the same joystick)
			if(joy->axes[angular_] == 1.0 || joy->axes[angular_] == -1.0) // if robot turning
			{
				// Same angular velocity for the three axis
				vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
				vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
				vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
				
				vel.linear.z = 0.0;
			}
			else if (joy->axes[linear_z_] == 1.0 || joy->axes[linear_z_] == -1.0) // if scissor moving 
			{
				vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_]; // scissor movement
			
				// limit robot turn
				vel.angular.x = 0.0;
				vel.angular.y = 0.0;
				vel.angular.z = 0.0;
			}
			else
			{
				// Same angular velocity for the three axis
				vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
				vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
				vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
				vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_]; // scissor movement
			} 
		}
		else // no dead zone
		{
			vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
			vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
			vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
			vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_];
		}
		
		
		
		

		// LIGHTS
		if (joy->buttons[button_output_1_] == 1) {

			if(!bRegisteredButtonEvent[button_output_1_]){
				//ROS_INFO("SummitXLPad::padCallback: OUTPUT1 button %d", button_output_1_);
				robotnik_msgs::set_digital_output write_do_srv;
				write_do_srv.request.output = output_1_;
				bOutput1=!bOutput1;
				write_do_srv.request.value = bOutput1;
				if(bEnable){
					set_digital_outputs_client_.call( write_do_srv );
					bRegisteredButtonEvent[button_output_1_] = true;
				}
			}
		}else{
			bRegisteredButtonEvent[button_output_1_] = false;
		}

		if (joy->buttons[button_output_2_] == 1) {
                        
			if(!bRegisteredButtonEvent[button_output_2_]){                               
				//ROS_INFO("SummitXLPad::padCallback: OUTPUT2 button %d", button_output_2_);
				robotnik_msgs::set_digital_output write_do_srv;
				write_do_srv.request.output = output_2_;
				bOutput2=!bOutput2;
				write_do_srv.request.value = bOutput2;

				if(bEnable){
					set_digital_outputs_client_.call( write_do_srv );
					bRegisteredButtonEvent[button_output_2_] = true;
				}
			}                     		  	
		}else{
			bRegisteredButtonEvent[button_output_2_] = false;
		}
		 

                // HOMING SERVICE 
		if (joy->buttons[button_home_] == 1) {
			if (!bRegisteredButtonEvent[button_home_]) {
				robotnik_msgs::home home_srv;
				home_srv.request.request = true;
				if (bEnable) {
                                   ROS_INFO("SummitXLPad::padCallback - Home");
   				   doHome.call( home_srv );			
                                   bRegisteredButtonEvent[button_home_] = true;
                                   }                                  
			}
			// Use this button also to block robot motion while moving the scissor
			vel.angular.x = 0.0;
	                vel.angular.y = 0.0;
        	        vel.angular.z = 0.0;
                	vel.linear.x = 0.0;
	                vel.linear.y = 0.0;
		}
		else {
			bRegisteredButtonEvent[button_home_]=false;
		}

		// SPHERECAM
		// TILT-MOVEMENTS (RELATIVE POS)
		ptz.pan = ptz.tilt = ptz.zoom = 0.0;
		ptz.relative = true;
		if (joy->buttons[ptz_tilt_up_] == 1) {		
			if(!bRegisteredButtonEvent[ptz_tilt_up_]){
				ptz.tilt = tilt_increment_;
				//ROS_INFO("SummitXLPad::padCallback: TILT UP");
				bRegisteredButtonEvent[ptz_tilt_up_] = true;
				ptzEvent = true;
			}
		}else {
			bRegisteredButtonEvent[ptz_tilt_up_] = false;
		}

		if (joy->buttons[ptz_tilt_down_] == 1) {
			if(!bRegisteredButtonEvent[ptz_tilt_down_]){
			  	ptz.tilt = -tilt_increment_;
				//ROS_INFO("SummitXLPad::padCallback: TILT DOWN");
				bRegisteredButtonEvent[ptz_tilt_down_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_tilt_down_] = false;
		}
		 
		// PAN-MOVEMENTS (RELATIVE POS)
		if (joy->buttons[ptz_pan_left_] == 1) {			
			if(!bRegisteredButtonEvent[ptz_pan_left_]){
				ptz.pan = -pan_increment_;
				//ROS_INFO("SummitXLPad::padCallback: PAN LEFT");
				bRegisteredButtonEvent[ptz_pan_left_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_pan_left_] = false;
		}

		if (joy->buttons[ptz_pan_right_] == 1) {
			if(!bRegisteredButtonEvent[ptz_pan_right_]){
			  	ptz.pan = pan_increment_;
				//ROS_INFO("SummitXLPad::padCallback: PAN RIGHT");
				bRegisteredButtonEvent[ptz_pan_right_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_pan_right_] = false;
		}

		// ZOOM Settings (RELATIVE)
		if (joy->buttons[ptz_zoom_wide_] == 1) {			
			if(!bRegisteredButtonEvent[ptz_zoom_wide_]){
				ptz.zoom = -zoom_increment_;
				//ROS_INFO("SummitXLPad::padCallback: ZOOM WIDe");
				bRegisteredButtonEvent[ptz_zoom_wide_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_zoom_wide_] = false;
		}

		if (joy->buttons[ptz_zoom_tele_] == 1) {
			if(!bRegisteredButtonEvent[ptz_zoom_tele_]){
			  	ptz.zoom = zoom_increment_;
				//ROS_INFO("SummitXLPad::padCallback: ZOOM TELE");
				bRegisteredButtonEvent[ptz_zoom_tele_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_zoom_tele_] = false;
		}

		if (joy->buttons[button_kinematic_mode_] == 1) {

			if(!bRegisteredButtonEvent[button_kinematic_mode_]){
				// Define mode (inc) - still coupled
				kinematic_mode_ += 1;
				if (kinematic_mode_ > 2) kinematic_mode_ = 1;
 				ROS_INFO("SummitXLJoy::joyCallback: Kinematic Mode %d ", kinematic_mode_);
				// Call service 
				robotnik_msgs::set_mode set_mode_srv;
				set_mode_srv.request.mode = kinematic_mode_;
				setKinematicMode.call( set_mode_srv );
				bRegisteredButtonEvent[button_kinematic_mode_] = true;
			}
		}else{
			bRegisteredButtonEvent[button_kinematic_mode_] = false;			
		}



	}
   	else {
		vel.angular.x = 0.0;	vel.angular.y = 0.0; vel.angular.z = 0.0;
		vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
	}

	sus_joy_freq->tick();	// Ticks the reception of joy events

        // Publish 
	// Only publishes if it's enabled
	if(bEnable){
        	if (ptzEvent) ptz_pub_.publish(ptz);
		vel_pub_.publish(vel);
		pub_command_freq->tick();
		}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "summit_xl_pad");
	SummitXLPad summit_xl_pad;

	ros::Rate r(50.0);

	while( ros::ok() ){
		// UPDATING DIAGNOSTICS
		summit_xl_pad.Update();
		ros::spinOnce();
		r.sleep();
		}
}

