#ifndef PX_FOUR_H
#define PX_FOUR_H
#include "RosDevice.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include "mavros_msgs/msg/global_position_target.h"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/mavlink_convert.hpp"
#include "mavros_msgs/msg/command_code.h"
#include <future>

class PxFour : public RosDevice
{
private:

    constexpr static float MIN_VOLTAGE{22.0};
    constexpr static float MAX_VOLTAGE{25.1};
    constexpr static float ZERO_PWM{1500};
    constexpr static float MAX_PWM{2000};
    constexpr static float MIN_PWM{1000};
    constexpr static float MOTOR_PORTS{6};

    std::string m_LaunchFileName{"./hardware/launch/px4.launch"};
    std::string m_SatellitesCnt{" "};

    geometry_msgs::msg::Pose m_LocalPosition;
    geometry_msgs::msg::Quaternion m_Orientation;
    geometry_msgs::msg::PoseStamped m_LocalTarget;

    float m_Charge{0};

    double m_Heading{0};
    double m_Altitude{0};
    double m_MoveSpeed{0};

    uint32_t m_BaudRate{0};

    uint16_t m_Port{0};
    int m_GpsFix{-1};

    bool m_Armed{false};
    bool m_IsMoving{false};
    bool m_WheelReverse{false};
    bool m_MovingForward{false};

    std::chrono::time_point<std::chrono::system_clock> m_LastGpsTime;
    std::chrono::time_point<std::chrono::system_clock> m_MoveTime;

    std::map<std::string, int>m_NavStatus
    {
            {"NO_FIX", -1},
            {"FIX", 0},
            {"SBAS_FIX", 1},
            {"GBAS_FIX", 2}
    };

    std::vector<double>m_Gps;

    std::future<void> m_FutureMover;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> m_ImuSub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> m_GpsSub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> m_HdgSub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::BatteryState>> m_BatterySub;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> m_LocalPositionSub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>> m_RelAltSub;
    std::shared_ptr<rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>> m_DiagnosticSub;
    //std::shared_ptr<rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>> m_GlobalPublisher;
    //std::shared_ptr<rclcpp::Publisher<mavros_msgs::msg::PoseStamped>> m_LocalPublisher;

public:

    void RunTick() override;

    void BeforeRun() override;

    void AfterRun() override;

    void PublishPoint();

    void CrateFlightMission();

    void SetServo(float port, float pwm);

    void SetPointPositionLocal(float x = 0, float y = 0, float z = 0, float yaw = 0);

    void UploadFlightMission();

    void Move(double speed);

    void Mover();

    void GpsCallback(const sensor_msgs::msg::NavSatFix& data);

    void ImuCallback(const sensor_msgs::msg::Imu& data);

    void HdgCallback(const std_msgs::msg::Float64 &data);

    void BatteryCallback(const sensor_msgs::msg::BatteryState &data);

    void DiagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray&data);

    void AltCallback(const std_msgs::msg::Float64& data);

    void LocalPositionCallback(const geometry_msgs::msg::PoseStamped&data);

    void SetMoveSettings(double speed);
};

#endif //PX_FOUR_H
