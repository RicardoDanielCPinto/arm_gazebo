/**
 * Universidade de Aveiro
 * @author Ricardo Pinto
 * @date 08/08/2019
 * @brief A program that accepts 3D coordenates and send them for ~/robotic_arm/pos_cmd topic
 * */

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
	gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/robotic_arm/pos_cmd");

	// Create a a vector3 message
	gazebo::msgs::Vector3d msg;

	// Set the position in the x-component
	gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), std::atof(_argv[2]), std::atof(_argv[3])));
	
	std::cerr << "Sending message: x=" << std::atof(_argv[1]) << ", y=" << std::atof(_argv[2]) << ", z=" << std::atof(_argv[3]) << "\n";
	pub->WaitForConnection();
	pub->Publish(msg);
	
	// Make sure to shut everything down.
	gazebo::client::shutdown();
}
