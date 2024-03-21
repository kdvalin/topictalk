#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "topictalk/topic.hpp"

using std::placeholders::_1;

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

//Assumes sorted
double percentile(double percentile, const std::vector<double> &arr) {
	int index = arr.size() * percentile;
	return (arr[(int) floor(index)] + arr[(int) ceil(index)])/2;
}

namespace TopicTalk {
class Subscriber : public rclcpp::Node {
public:
	Subscriber() : Node("topictalk_sub") {
		declare_parameter(HUMAN_UNITS, true);
		this->human_units = get_parameter(HUMAN_UNITS).as_bool();
		this->_start_time = this->get_clock()->now();
		this->_sub = this->create_subscription<std_msgs::msg::Header>(TOPIC_NAME, 10, std::bind(&Subscriber::callback, this, _1));
	}
	~Subscriber() {
		std::sort(this->transmission_times.begin(), this->transmission_times.end());
		
		double min = transmission_times[0];
		double q1 = percentile(0.25, transmission_times);
		double median = percentile(0.5, transmission_times);
		double q3 = percentile(0.75, transmission_times);
		double p99 = percentile(0.99, transmission_times);
		double max = transmission_times[transmission_times.size() - 1];

		RCLCPP_INFO_STREAM(
			this->get_logger(),
			"End of run: " << status_message(byte_counter, transmission_time/1e9) << std::endl <<
			"min lat " << min << "s" << std::endl << "25% lat " << q1 << "s" << std::endl <<
			"median lat " << median << "s" << std::endl << "75% lat " << q3 << "s" << std::endl <<
			"99% lat " << p99 << "s" << std::endl << "max lat " << max << "s" << std::endl; 
		);
	}
private:
	rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _sub;
	size_t byte_counter;
	rclcpp::Time _start_time;
	time_t transmission_time;
	bool human_units;
	std::vector<double> transmission_times;

	std::string status_message(size_t bytes_received, double duration_s) {
		std::stringstream ss;
		ss << "Received " << format_bytes(bytes_received) << " in " <<
			duration_s << "s ";
		
		double rate = bytes_received / duration_s;
		if(human_units) {
			ss << format_bytes(rate);
		} else {
			ss << rate << " B";
		}
		ss << "/s";
		return ss.str();
	}

	void callback(const std_msgs::msg::Header &data) {
		size_t bytes_received = data.frame_id.length() + sizeof(data.stamp.sec)*2;
		auto recv_time = data.stamp;

		auto transmission_time = this->get_clock()->now() - recv_time;
		this->transmisssion_times.push_back(transmission_time);


		RCLCPP_INFO_STREAM(
			this->get_logger(),
			status_message(bytes_received, transmission_time.seconds())
		);
		this->byte_counter += bytes_received;
		this->transmission_time += transmission_time.nanoseconds();
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
