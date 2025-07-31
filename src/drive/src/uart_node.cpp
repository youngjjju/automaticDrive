// 속도와 조향 값은 퍼블리셔로부터 -120 ~ 120으로 제한되어 들어옴
// 이에 128 씩 더해서, 아두이노에 전송되는 값은 8 ~ 248로 제한됨
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp" 
#include "config.hpp"
#include <fcntl.h>     
#include <unistd.h>     
#include <termios.h>    
#include <cstring>      

class UartNode : public rclcpp::Node
{
public: 
    UartNode() : Node("uart_node"), speed_(0), steer_(0)  
    {
        // init_uart(config::uart_port, B9600);

        speed_subscriber_ = this->create_subscription<std_msgs::msg::Int8>(
            "speed_subscriber", 10,
            [this](std_msgs::msg::Int8::SharedPtr msg){
                speed_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "속도 변경: %d", speed_);
            });

        steer_subscriber_ = this->create_subscription<std_msgs::msg::Int8>(
            "steer_subscriber", 10,
            [this](std_msgs::msg::Int8::SharedPtr msg){
                steer_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "조향 변경: %d", steer_);
            });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
            
            uint8_t speed_byte = static_cast<uint8_t>(speed_ + 128); 
            uint8_t steer_byte = static_cast<uint8_t>(steer_ + 128);
            uint8_t end_byte = 0xFF;
            
            uint8_t buffer[3] = {speed_byte, steer_byte, end_byte};
            // write(uart_fd, buffer, 3);
            
            RCLCPP_INFO(this->get_logger(), "UART 전송: speed %d → %u, steer %d → %u", speed_, speed_byte, steer_, steer_byte);
        });
        
    }

private:
    // UART 초기화
    // void init_uart(const std::string &device, int baudrate)
    // {
    //     uart_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    //     if (uart_fd_ < 0)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "UART 열기 실패: %s", strerror(errno));
    //         return;
    //     }

    //     struct termios tty;
    //     memset(&tty, 0, sizeof tty);
    //     if (tcgetattr(uart_fd_, &tty) != 0)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "속성 읽기 실패");
    //         return;
    //     }

    //     cfsetospeed(&tty, baudrate);
    //     cfsetispeed(&tty, baudrate);
    //     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    //     tty.c_iflag &= ~IGNBRK;
    //     tty.c_lflag = 0;
    //     tty.c_oflag = 0;
    //     tty.c_cc[VMIN]  = 1;
    //     tty.c_cc[VTIME] = 5;
    //     tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    //     tty.c_cflag |= (CLOCAL | CREAD);
    //     tty.c_cflag &= ~(PARENB | PARODD);
    //     tty.c_cflag &= ~CSTOPB;
    //     tty.c_cflag &= ~CRTSCTS;

    //     if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0)
    //         RCLCPP_ERROR(this->get_logger(), "속성 설정 실패");
    // }

    // 멤버 변수
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr speed_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr steer_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    int8_t speed_;
    int8_t steer_;
    int uart_fd_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UartNode>());
    rclcpp::shutdown();
    return 0;
}