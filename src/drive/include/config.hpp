#pragma once
#include <string>

namespace config {
    inline const std::string uart_port = "/dev/ttyUSB0"; // UART_NODE에서 아두이노로 보내는 포트
    inline constexpr int uart_baudrate = 9600;           // UART_NODE에서 아두이노로 보내는 비트레이트
}