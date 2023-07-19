#include "Encoder.h"

Encoder::Encoder() : RosDevice("/cw_encoder_node"){}

float Encoder::ServiceCallback(const std::shared_ptr<cw_encoder::srv::SetCrd::Request> request, std::shared_ptr<cw_encoder::srv::SetCrd::Response> response)
{
    return response->result;
}

void Encoder::DataCallback(const std::shared_ptr<cw_encoder::msg::Encoder> data)
{
    if(m_VirtualSpeed == 0.0f)
    {
        m_LastMessageTime = std::chrono::system_clock::now();
        m_Coordinates = data->crd;
        m_Speed = data->speed;
    }
}

std::string Encoder::SetCoordinates(float coordinates)
{
    if(!IsActive())
    {
        m_Coordinates = coordinates;
        return "enc_offline";
    }

    try
    {
        rclcpp::Service<cw_encoder::srv::SetCrd>::SharedPtr service = this->create_service<cw_encoder::srv::SetCrd>("/cw/hardware/encoder/set_crd", std::bind(&Encoder::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    }
    catch (std::exception&exception)
    {
        return "set_failed";
    }
    return{};
}

void Encoder::SetVirtualSpeed(float speed)
{
    m_VirtualSpeed = speed;
}

void Encoder::SetLaunchArgs()
{
    m_Args[2] = "crd:= " + std::to_string(m_Coordinates);
}
