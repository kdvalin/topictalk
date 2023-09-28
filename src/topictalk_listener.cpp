#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "topictalk/topic.hpp"

using std::placeholders::_1;

namespace TopicTalk {
class Subscriber : public rclcpp::Node {
public:
	Subscriber() : Node("topictalk_sub") {
        this->_start_time = this->get_clock()->now();
		this->_sub = this->create_subscription<std_msgs::msg::Header>(TOPIC_NAME, 10, std::bind(&Subscriber::callback, this, _1));
	}
    ~Subscriber() {
        auto end = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Average: %f B/s", byte_counter/((end - this->_start_time).seconds()));
    }
private:
	rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _sub;
    size_t byte_counter;
    rclcpp::Time _start_time;

    void callback(const std_msgs::msg::Header &data) {
        size_t bytes_received = data.frame_id.length() + sizeof(data.stamp.sec)*2;
        auto recv_time = data.stamp;

        auto transmission_time = this->get_clock()->now() - recv_time;
        RCLCPP_INFO(this->get_logger(), "Received %ld bytes in %f seconds (%ld nanoseconds).  %f B/s", bytes_received, transmission_time.seconds(), transmission_time.nanoseconds(), bytes_received);
        this->byte_counter += bytes_received;
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
