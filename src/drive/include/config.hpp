#pragma once
#include <string>

namespace config {
    inline const std::string uart_port = "/dev/ttyUSB0";
    inline constexpr int uart_baudrate = 9600;
}