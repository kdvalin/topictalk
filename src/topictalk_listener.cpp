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
        RCLCPP_INFO(this->get_logger(), "Average: %f B/s", format_bytes(byte_counter/(transmission_time/1e9)));
    }
private:
	rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _sub;
    size_t byte_counter;
    rclcpp::Time _start_time;
    time_t transmission_time;

    void callback(const std_msgs::msg::Header &data) {
        size_t bytes_received = data.frame_id.length() + sizeof(data.stamp.sec)*2;
        auto recv_time = data.stamp;

        auto transmission_time = this->get_clock()->now() - recv_time;
        RCLCPP_INFO(this->get_logger(), "Received %s in %f seconds (%ld nanoseconds).  %s/s", format_bytes(bytes_received), transmission_time.seconds(), transmission_time.nanoseconds(), format_bytes(bytes_received / transmission_time.seconds()));
        this->byte_counter += bytes_received;
        this->transmission_time += transmission_time.nanoseconds();
    }
};

}

std::string format_bytes(double bytes) {
    constexpr const char FILE_SIZE_UNITS[8][4] {
        "B", "KiB", "MiB", "GiB", "TiB", "PiB", "EiB", "ZiB"
    };

    int index = log2(bytes)/10;
    bytes = bytes/(pow(2,index*10));
    std::stringstream ss{};
    ss << bytes << " " << FILE_SIZE_UNITS[index];
    return ss.str();
}

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	srand(time(NULL));
	rclcpp::spin(std::make_shared<TopicTalk::Subscriber>());
	rclcpp::shutdown();
	return 0;
}
