#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <thread>
#include <nlohmann/json.hpp>
#include <cstdio>
#include <map>

using json = nlohmann::json;

class BandwidthNode : public rclcpp::Node {
public:
    BandwidthNode() : Node("bandwidth_node") {
        // Declare top-level parameters
        declare_parameter<std::string>("server_ip", "127.0.0.1");
        declare_parameter<std::string>("iperf.protocol", "-u");
        declare_parameter<std::string>("iperf.bandwidth", "20K");
        declare_parameter<std::string>("iperf.length", "100");
        declare_parameter<std::string>("iperf.time", "2");
        declare_parameter<std::string>("iperf.format", "-J");

        publisher_ = create_publisher<std_msgs::msg::Float32>("iperf3_bandwidth", rclcpp::QoS(10).best_effort());

        timer_ = create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&BandwidthNode::measure_bandwidth_async, this)
        );
    }

private:
    void measure_bandwidth_async() {
        std::thread(&BandwidthNode::measure_bandwidth, this).detach();
    }

    void measure_bandwidth() {
        std::string ip = get_parameter("server_ip").as_string();
        std::string protocol = get_parameter("iperf.protocol").as_string();
        std::string bandwidth = get_parameter("iperf.bandwidth").as_string();
        std::string length = get_parameter("iperf.length").as_string();
        std::string time = get_parameter("iperf.time").as_string();
        std::string format = get_parameter("iperf.format").as_string();


        std::string cmd = "nice -n 10 iperf3 -c " + ip +
                          " " + protocol +
                          " -b " + bandwidth +
                          " -l " + length +
                          " -t " + time +
                          " " + format;


        RCLCPP_INFO(get_logger(), "Executing command: %s", cmd.c_str());

        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            RCLCPP_ERROR(get_logger(), "iperf3 command failed to execute.");
            return;
        }

        char buffer[1024];
        std::string result;
        while (fgets(buffer, sizeof(buffer), pipe)) {
            result += buffer;
        }
        pclose(pipe);

        parse_and_publish(result);
    }

    void parse_and_publish(const std::string& result) {
        try {
            auto j = json::parse(result);
            float bandwidth_mbps = j["end"]["sum"]["bits_per_second"].get<float>() / 1e6;

            auto msg = std_msgs::msg::Float32();
            msg.data = bandwidth_mbps;
            publisher_->publish(msg);

            RCLCPP_INFO(get_logger(), "Bandwidth: %.3f Mbps", bandwidth_mbps);
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "JSON parsing failed: %s", e.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BandwidthNode>());
    rclcpp::shutdown();
    return 0;
}

