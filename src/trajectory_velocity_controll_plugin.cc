/**
 * Universidade de Aveiro
 * @author Ricardo Pinto
 * @date 08/08/2019
 * @brief This class is responsable for calculating the joint positions by inverse kinematics
 * 		  and then moving the robot using velocity PID controllers
 * */

#ifndef _SIMPLE_PLUGIN_HH_
#define _SIMPLE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <vector>
#include <math.h>

namespace gazebo
{
	/// \brief A simple plugin to print a message.
	class TrajectoryVelocityControllPlugin : public ModelPlugin
	{
		
		private:
			/// \brief Pointer to the model.
			physics::ModelPtr model;
			/// \brief Pointer to the joint.
			physics::Joint_V joints; // All the joints
			std::vector< int > rev_joints; //Revolute Joints
			/// \brief A PID controller for the joint.
			std::vector< common::PID > pids;
			/// \brief For comunications
			transport::NodePtr node;
			transport::SubscriberPtr sub;
			
			void OnMsg(ConstVector3dPtr &_msg){
				double x, y, z;
				x = _msg->x();
				y = _msg->y();
				z = _msg->z();
				double q2 = calculateQ2( y, z );
				double q1 = calculateQ1( y, z ,q2 );
				q2 = -q2;
				if( q1 > 3.14159 ) q1 = q1 - 2*3.14159; //ser positivo ou negativo influencia devido aos limites das joints
				if( q1 < -3.14159 ) q1 = q1 + 2*3.14159;
				std::cerr << "----> O valor de Q1 é " << q1 << " <----\n";
				std::cerr << "----> O valor de Q2 é " << q2 << " <----\n\n";
				
				setVelocity( 0, q1 );
				setVelocity( 2, q2 );
			}
			
			double calculateQ2( const double &_x, const double &_y ){
				double x_2 = _x*_x;
				double y_2 = _y*_y;
				double a = 0.35;
				double b = 0.275;
				double a_2 = a*a;
				double b_2 = b*b;
				double q2 = 0;
				if( _y >= 0) { q2 = -1*std::acos( (-a_2-b_2+x_2+y_2) / (2*a*b) ); }
				else { q2 = std::acos( (-a_2-b_2+x_2+y_2) / (2*a*b) ); }
				return q2;
			}
			
			double calculateQ1( double &_x, double &_y, double &_q2){
				double a = 0.35;
				double b = 0.275;
				double q1 = 0;
				if( _x > -0.000001 && _x < 0.000001  ) _x = 0.000001; //divisão por 0... -Inf +Inf
				q1 = std::atan( _y/_x ) + std::atan( b*std::sin( _q2 )/( a + b*std::cos( _q2 ) ) );
				if( _x < 0 ) q1 = q1 + 3.14159;
				return q1;
			}
			
			void filterRevoluteJoints(){
				const physics::Base::EntityType revolute = physics::Base::HINGE_JOINT;
				
				const int len = this->model->GetJointCount();
				for(int i = 0; i < len; i++){
					if( this->joints[i]->HasType(revolute) ){
						this->rev_joints.push_back( i );
					}
				}
			}
		
		public:
			/// \brief Constructor
			TrajectoryVelocityControllPlugin() {}

			/// \brief The load function is called by Gazebo when the plugin is
			/// inserted into simulation
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
			{
				std::cerr << "\nLoading the Joint Controll Plugin\n";
				// Safety check
				if (_model->GetJointCount() == 0)
				{
					std::cerr << "Invalid joint count, Joint Controll Plugin not loaded\n";
					return;
				}
				this->model = _model;
				this->joints = _model->GetJoints();
				pids.push_back( common::PID(200, 10, 0) );
				pids.push_back( common::PID(60, 10, 0) );
				pids.push_back( common::PID(100, 10, 0) );
				pids.push_back( common::PID(60, 10, 0) );
				pids.push_back( common::PID(60, 10, 0) );
				// Create the node
				this->node = transport::NodePtr(new transport::Node());
				#if GAZEBO_MAJOR_VERSION < 8
					this->node->Init(this->model->GetWorld()->GetName());
				#else
					this->node->Init(this->model->GetWorld()->Name());
				#endif
				
				this->filterRevoluteJoints();
				
				//Create PID controllers for every joint and set it to 0
				for(int i=0; i < rev_joints.size(); i++){
					physics::JointPtr j = this->joints[this->rev_joints[i]];
					this->model->GetJointController()->SetVelocityPID( j->GetScopedName(), this->pids[i] );
					this->model->GetJointController()->SetVelocityTarget( j->GetScopedName(), 0 );
				}
				// Subscribe to the topic, and register a callback
				std::string topicName = "~/" + this->model->GetName() + "/pos_cmd";
				this->sub = this->node->Subscribe( topicName, &TrajectoryVelocityControllPlugin::OnMsg, this );
				
				// Just output a message
				std::cerr << "\nThe Joint Controll Plugin is attach to model[" << _model->GetName() << "]\n";
			}
			
			void setVelocity(int i, double &_pos){
				this->model->GetJointController()->SetVelocityTarget( this->joints[ this->rev_joints[i] ]->GetScopedName(), _pos );
			}
		
	};
	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(TrajectoryVelocityControllPlugin)
}
#endif
