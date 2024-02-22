#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bridge_lib/queue.h"
#include "bridge_lib/ros2_bridge.h"
#include "bridge_lib/serial_settings.h"
#include "bridge_lib/msgs.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Bridge : public rclcpp::Node
{
public:
	Bridge()
  	: Node("bridge")
	{
		/**
		 * One publisher for demo purposes but when being used in a real app the will 
		 * probably be 
		 * -	a publisher for each different message type comming from
		 * 		the communication thread
		 * 
		 * -	a subscriber for each message type the communication thread needs to send to
		 * 		the micro controller 
		*/
		m_count = 0;
		m_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
		m_timer = this->create_wall_timer(15000ms, std::bind(&Bridge::timer_callback, this));
		m_comms_obj_ptr = {};
	}
	void set_communication_object(ros2_bridge::Bridge* p)
	{
		if(!p) {
			throw std::runtime_error("bridge pointer was nullptr in set_bridge");
		}
		m_comms_obj_ptr = p;
	}
	/**
	 * Called whenever a message is available from the mocro controller
	*/
	void guard_condition_callback(std::size_t n)
	{
		Message msg;
		bool gotone = m_comms_obj_ptr.value()->m_client_queue.get_nowait(msg);
		if(gotone) {
			printf("guard condition callback n: %ld  msg: %s\n", n, msg.c_str());
		} else {
			printf("DID NOT GET ONE \n");
		}
	}
	/**
	 * Periodically send a message to the micro controller
	*/
  	void timer_callback()
	{
		m_count++;
		// The next line is a hack way of creating a message from a std::string.
		Message echo_cmd{std::format("echo {} AAAAA  BBBBBBBB CCC", m_count)};
		RCLCPP_INFO(this->get_logger(), "send_threadsafe : '%s'", echo_cmd.c_str());
		m_comms_obj_ptr.value()->send_threadsafe(echo_cmd);
	}
	
	rclcpp::GuardCondition::SharedPtr m_guard_condition_sptr;
	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
	std::optional<ros2_bridge::Bridge*> m_comms_obj_ptr;
	size_t m_count;
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node_sptr = std::make_shared<Bridge>();
	/**
	 * Setup a guard condition to be triggered by the communication
	 * thread when it has a message for the node to process.
	 * Thhis all happens inside threadsafe::TriggerQueue() 
	*/
	auto context = node_sptr->get_node_base_interface()->get_context();
	auto guard_condition_sptr = std::make_shared<rclcpp::GuardCondition>(context);
	auto f = std::bind(&Bridge::guard_condition_callback, node_sptr.get(), std::placeholders::_1);
	guard_condition_sptr->set_on_trigger_callback(f);
	
	/**
	 * Create a TriggerQueue - a threadsafe queue that will trigger the guard_condition
	 * everytime a 'message' is put into the queue by the communication thread.
	 * 
	 * And an FdQueue which is how the node sends messasge to comms to be transmitted
	 * to the micro processor.
	 * 
	 * The communications object exercises control flow on these queues to ensure
	 * the node/serial line are keeping up with demand.
	*/
	auto trigger_function = [&](){
		guard_condition_sptr->trigger();
	};
	threadsafe::TriggerQueue<Message> receive_from_serial_queue{trigger_function};
	threadsafe::FdQueue<Message>      send_to_serial_queue{};
	/**
	 * Create the bridge object the does the communication on the communication thread.
	*/
	auto port_path_list = list_serial_devices();
	if(port_path_list.size() != 1) {
		throw std::runtime_error("could not find exactly one suitable serial port path");
	}
	auto port = port_path_list[0];
	int fd = open_serial(port);
	apply_default_settings(fd);
	ros2_bridge::Bridge comms{fd, receive_from_serial_queue, send_to_serial_queue};
	node_sptr->set_communication_object(&comms);


	std::jthread communication_thread{[&]() {
		printf("Communication thread started\n");
		comms.run();
	}};
	rclcpp::spin(node_sptr);
	rclcpp::shutdown();
	communication_thread.join();
	return 0;
}
