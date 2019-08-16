#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <sstream>

namespace my_controller_ns
{

	class MyPositionController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
	{
		
		bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
		{
			std::string my_joint;
			if(!n.getParam("joint", my_joint))
			{
				ROS_ERROR("Could not find joint name");
				return false;
			}
			joint_ = hw->getHandle(my_joint);
			command_ = joint_.getPosition();
			
			if(!n.getParam("gain", gain_))
			{
				ROS_ERROR("Could not find the gain parameter");
				return false;
			}
			
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &MyPositionController::setCommandCB, this);
			
			velocity_pub_ = n.advertise<std_msgs::Float64>("vel", 10);
			
			return true;
		}
		
		void update(const ros::Time& time, const ros::Duration& period)
		{
			
			double now = time.toSec();
			double time_diff = now - call_time_;
			
			if( time_diff < 5 && now > 5 ){ ///Se estiver a fazer ordens
				if( time_diff < 2 ){
					command_velocity_ = time_diff * aceleration_;
				} else if( time_diff < 3){
					//command_velocity_ = command_velocity_;
				} else if( time_diff < 5 ){
					command_velocity_ = max_vel_ - (time_diff - 3) * (aceleration_);
				}
			} else { ///Se já acabaram os 5 segundos, manter a posição com Controlador porporcional
				double error = command_ - joint_.getPosition();
				command_velocity_ = error*gain_;
			}
			
			///Aply the results and publish
			joint_.setCommand(command_velocity_);
			msg_.data = joint_.getVelocity();
			velocity_pub_.publish(msg_);

		}
		
		void setCommandCB( const std_msgs::Float64ConstPtr& msg )
		{
			command_ = msg->data;
			max_vel_ = (command_ - joint_.getPosition() )*2/(5+1);
			aceleration_ = max_vel_/2;
			call_time_ = ros::Time::now().toSec();
		}
		
		void starting(const ros::Time& time){}
		void stopping(const ros::Time& time){}
		
		private:
			hardware_interface::JointHandle joint_;
			double gain_;
			double command_;
			ros::Subscriber sub_command_;
			
			ros::Publisher velocity_pub_;
			std_msgs::Float64 msg_;
			
			double max_vel_;
			double aceleration_;
			double call_time_;
			double command_velocity_= 0.0;
						
	};
	
	PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyPositionController, controller_interface::ControllerBase);
	
}
