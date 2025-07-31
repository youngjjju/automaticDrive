#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

class SpeedSteerPublisher : public rclcpp::Node
{
public:
    SpeedSteerPublisher()
        : Node("speed_steer_publisher"), count_(0)
    {
        speed_pub_ = this->create_publisher<std_msgs::msg::Int8>("speed_subscriber", 10);
        steer_pub_ = this->create_publisher<std_msgs::msg::Int8>("steer_subscriber", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() {
            auto speed_msg = std_msgs::msg::Int8();
            auto steer_msg = std_msgs::msg::Int8();

            speed_msg.data = static_cast<int8_t>(40 * std::sin(count_ * 0.2));   // -40 ~ 40 범위의 sine 파형
            steer_msg.data = static_cast<int8_t>(80 * std::cos(count_ * 0.2));  // -80 ~ 80 범위의 cosine 파형

            speed_pub_->publish(speed_msg);
            steer_pub_->publish(steer_msg);

            RCLCPP_INFO(this->get_logger(), "전송: speed=%d, steer=%d", speed_msg.data, steer_msg.data);

            count_++;
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr steer_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedSteerPublisher>());
    rclcpp::shutdown();
    return 0;
}
