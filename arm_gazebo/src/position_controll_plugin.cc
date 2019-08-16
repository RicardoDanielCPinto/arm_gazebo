#ifndef _SIMPLE_PLUGIN_HH_
#define _SIMPLE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <vector>

namespace gazebo
{
	/// \brief A simple plugin to print a message.
	class PositionControllPlugin : public ModelPlugin
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
			transport::SubscriberPtr sub0;
			transport::SubscriberPtr sub1;
			transport::SubscriberPtr sub2;
			transport::SubscriberPtr sub3;
			transport::SubscriberPtr sub4;
			
			void OnMsg_joint0(ConstVector3dPtr &_msg){
				this->setPosition( 0, _msg->x() );
			}
			void OnMsg_joint1(ConstVector3dPtr &_msg){
				this->setPosition( 1, _msg->x() );
			}
			void OnMsg_joint2(ConstVector3dPtr &_msg){
				this->setPosition( 2, _msg->x() );
			}
			void OnMsg_joint3(ConstVector3dPtr &_msg){
				this->setPosition( 3, _msg->x() );
			}
			void OnMsg_joint4(ConstVector3dPtr &_msg){
				this->setPosition( 4, _msg->x() );
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
			PositionControllPlugin() {}

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
				
				std::vector< std::string > names;
				// Subscribe to the topic, and register a callback
				for(int i=0; i < rev_joints.size(); i++){
					physics::JointPtr j = this->joints[this->rev_joints[i]];
					this->model->GetJointController()->SetPositionPID( j->GetScopedName(), this->pids[i] );
					this->model->GetJointController()->SetPositionTarget( j->GetScopedName(), 0 );
					std::string topicName = "~/" + this->model->GetName() + "/" + j->GetName() + "/pos_cmd";
					names.push_back( topicName );
				}
				this->sub0 = this->node->Subscribe( names[0], &PositionControllPlugin::OnMsg_joint0, this );
				this->sub1 = this->node->Subscribe( names[1], &PositionControllPlugin::OnMsg_joint1, this );
				this->sub2 = this->node->Subscribe( names[2], &PositionControllPlugin::OnMsg_joint2, this );
				this->sub3 = this->node->Subscribe( names[3], &PositionControllPlugin::OnMsg_joint3, this );
				this->sub4 = this->node->Subscribe( names[4], &PositionControllPlugin::OnMsg_joint4, this );
				
				// Just output a message
				std::cerr << "\nThe Joint Controll Plugin is attach to model[" << _model->GetName() << "]\n";
			}
			
			void setPosition(int i, const double &_pos){
				this->model->GetJointController()->SetPositionTarget( this->joints[ this->rev_joints[i] ]->GetScopedName(), _pos );
			}
		
	};
	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(PositionControllPlugin)
}
#endif
