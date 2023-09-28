#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "topictalk/topic.hpp"

namespace TopicTalk {
class Subscriber : public rclcpp::Node {
public:
	Subscriber() : Node("topictalk_sub") {
		this->_sub = this->create_subscription<std_msgs::msg::Header>(TOPIC_NAME, 10, std::bind(Subscriber::callback, this));
	}
private:
	rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _sub;
    size_t byte_counter;
    time_t start_time;

    void callback(const std_msgs::msg::Header &data) const {
        size_t bytes_received = data.frame_id.length();
        auto recv_time = data.stamp;

        auto transmission_time = rclcpp::Clock().now() - recv_time;
        RCLCPP_DEBUG(this->get_logger(), "Received %d bytes in %d nanoseconds", bytes_received, transmission_time.nanoseconds());
    }
};

}

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	srand(time(NULL));
	rclcpp::spin(std::make_shared<TopicTalk::Subscriber>());
	rclcpp::shutdown();
	return 0;
}
