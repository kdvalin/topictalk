#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "topictalk/topic.hpp"

using namespace std::chrono_literals;

#define MAX_ASCII 0x7f
#define MIN_ASCII 0x20

namespace TopicTalk {
class Publisher : public rclcpp::Node {
public:
	Publisher() : Node("topictalk_publisher") {
		for(int i = 0; i < MESSAGE_LENGTH; ++i) {
			_payload.push_back((char) ((rand() % (MAX_ASCII - MIN_ASCII)) + MIN_ASCII));
		}
		this->_publisher = this->create_publisher<std_msgs::msg::Header>(TOPIC_NAME, 10);

		RCLCPP_DEBUG(this->get_logger(), "Opened %s topic", TOPIC_NAME);
		this->_timer = this->create_wall_timer(500ms, std::bind(&Publisher::callback, this));
		RCLCPP_DEBUG(this->get_logger(), "Created timer");
	}
private:
	rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr _publisher;
	rclcpp::TimerBase::SharedPtr _timer;
	std::string _payload;

	void callback() {
		auto message = std_msgs::msg::Header();
		message.frame_id = _payload;
		message.stamp = this->get_clock()->now();
		this->_publisher->publish(message);
	}
};

}

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	srand(time(NULL));
	rclcpp::spin(std::make_shared<TopicTalk::Publisher>());
	rclcpp::shutdown();
	return 0;
}
