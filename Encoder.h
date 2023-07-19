#ifndef ENCODER_H
#define ENCODER_H

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cw_can_uart/msg/can_uart.hpp"
#include "cw_encoder/msg/encoder.hpp"
#include "cw_encoder/srv/set_crd.hpp"

#include "RosDevice.h"

class Encoder : public RosDevice
{
private:
    std::string m_LaunchFileName{"./hardware/launch/cw_encoder.launch"};
    std::string m_TopicName{"/cw/hardware/encoder"};
    float m_Coordinates{0.0f};
    float m_Speed{0.0f};
    float m_VirtualSpeed{0.0f};
    rclcpp::Service<cw_encoder::srv::SetCrd>::SharedPtr m_Service{nullptr};
    std::chrono::time_point<std::chrono::system_clock> m_LastMessageTime;
    std::vector<std::string>m_Args
    {
        "can_id:= "
        "step:= ",
        "crd:= 0.0"
    };

public:
    Encoder();

    float ServiceCallback(const std::shared_ptr<cw_encoder::srv::SetCrd::Request> request, std::shared_ptr<cw_encoder::srv::SetCrd::Response> response);

    std::string SetCoordinates(float coordinates);

    void SetVirtualSpeed(float speed);

    void SetLaunchArgs();

    void DataCallback(const std::shared_ptr<cw_encoder::msg::Encoder> data);

};

#endif //ENCODER_H
