#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
	#include <gazebo/gazebo.hh>
#else
	#include <gazebo/gazebo_client.hh>
#endif

int main(int _argc, char **_argv)
{
	// Load gazebo as a client
	gazebo::client::setup(_argc, _argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// Publish to the robotic_arm topic
	gazebo::transport::PublisherPtr pub0 = node->Advertise<gazebo::msgs::Vector3d>("~/robotic_arm/joint1/pos_cmd");
	gazebo::transport::PublisherPtr pub1 = node->Advertise<gazebo::msgs::Vector3d>("~/robotic_arm/joint2/pos_cmd");
	gazebo::transport::PublisherPtr pub2 = node->Advertise<gazebo::msgs::Vector3d>("~/robotic_arm/joint4/pos_cmd");
	gazebo::transport::PublisherPtr pub3 = node->Advertise<gazebo::msgs::Vector3d>("~/robotic_arm/joint5/pos_cmd");
	gazebo::transport::PublisherPtr pub4 = node->Advertise<gazebo::msgs::Vector3d>("~/robotic_arm/joint7/pos_cmd");

	// Create a a vector3 message
	gazebo::msgs::Vector3d msg;

	// Set the position in the x-component
	gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atoi(_argv[2]), 0, 0));

	switch(std::atoi(_argv[1])){
		case 0:
			pub0->WaitForConnection();
			pub0->Publish(msg);
			break;
		case 1:
			pub1->WaitForConnection();
			pub1->Publish(msg);
			break;
		case 2:
			pub2->WaitForConnection();
			pub2->Publish(msg);
			break;
		case 3:
			pub3->WaitForConnection();
			pub3->Publish(msg);
			break;
		case 4:
			pub4->WaitForConnection();
			pub4->Publish(msg);
			break;
		default:
			break;
	}

	// Make sure to shut everything down.
	gazebo::client::shutdown();
}
