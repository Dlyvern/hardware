#include "PxFour.h"

#include <cmath>

void PxFour::RunTick()
{
   // if((std::chrono::system_clock::now() - m_LastGpsTime).count() > 2)
}

void PxFour::UploadFlightMission()
{
    CrateFlightMission();

    if(m_Armed) return;

}

void PxFour::Move(double speed)
{
    int int_speed = (int)speed;

    if(int_speed < - 10 || int_speed > 10)
        return;


    float dir_value = m_WheelReverse ? -1 : 1;
    float step_up = std::floor((MAX_PWM - ZERO_PWM - 100) / 10);
    float step_down = std::floor((ZERO_PWM - MIN_PWM - 100) / 10);

    Log( "UP: " + std::to_string(step_up) + ", DOWN: " + std::to_string(step_down), INFO_LEVEL_LOG);

    float pwm;

    if(int_speed > 0)
    {
        Log("FORWARD MOVE", INFO_LEVEL_LOG);
        m_MovingForward = true;
        pwm = ZERO_PWM + ((100.0f + int_speed + step_up) + dir_value);
    }

    else if(int_speed < 0)
    {
        Log("BACKWARD MOVE", INFO_LEVEL_LOG);
        m_MovingForward = false;
        pwm = ZERO_PWM + ((int_speed * step_down - 100.0f) * dir_value);
    }

    else if(int_speed == 0)
    {
        if(m_Brake)
        {
            auto brake_value = m_BrakeValue;
            if(m_MovingForward)
                brake_value += m_BrakeForward;
            else
            {
                brake_value += m_BrakeBack;
                brake_value *= -1;
            }
            pwm += (brake_value * dir_value);
        }
        auto res = SetServo();
    }

    auto res = RepeatServo();
}

void PxFour::SetServo(float port, float pwm)
{
    if(port <= MOTOR_PORTS)
        return;

    try
    {
        auto cm = create_service<mavros_msgs::msg::>("/mavros/cmd/command");
        cm
    }

    catch(std::exception&exception)
    {
        Log("Unable to call mavros cmd service" + exception.what())
    }
}

void PxFour::BeforeRun()
{
    m_ImuSub = this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", 1, std::bind(&PxFour::ImuCallback, this, std::placeholders::_1));
    m_GpsSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("mavros/global_position/global", 1, std::bind(&PxFour::GpsCallback, this, std::placeholders::_1));
    m_HdgSub = this->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg", 1, std::bind(&PxFour::HdgCallback, this, std::placeholders::_1));
    m_BatterySub = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", 1, std::bind(&PxFour::BatteryCallback, this, std::placeholders::_1));
    m_LocalPositionSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", 1, std::bind(&PxFour::LocalPositionCallback, this, std::placeholders::_1));
    m_RelAltSub = this->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/rel_alt", 1, std::bind(&PxFour::AltCallback, this, std::placeholders::_1));
    m_DiagnosticSub = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1, std::bind(&PxFour::DiagnosticCallback, this, std::placeholders::_1));
    //m_GlobalPublisher = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>("mavros/setpoint_position/global", 1);
    //m_LocalPublisher = this->create_publisher<mavros_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 1);
}

void PxFour::DiagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray &data)
{
    DataCallback();
    m_SatellitesCnt = data.status[1].values[0].value;
}


void PxFour::ImuCallback(const sensor_msgs::msg::Imu &data)
{
    Debug("IMU CALLBACK", 4);
    DataCallback();
    m_Orientation = data.orientation;
}

void PxFour::GpsCallback(const sensor_msgs::msg::NavSatFix &data)
{
    Debug("GPS CALLBACK", 4);
    DataCallback();
    m_LastGpsTime = std::chrono::system_clock::now();
    m_Gps = {data.latitude, data.longitude};

    std::stringstream ss;
    ss << data.status.status;

    m_GpsFix = m_NavStatus.find(ss.str())->second;
}

void PxFour::HdgCallback(const std_msgs::msg::Float64 &data)
{
    Debug("HDG CALLBACK", 4);
    DataCallback();
    m_Heading = data.data;
}

void PxFour::BatteryCallback(const sensor_msgs::msg::BatteryState &data)
{
    float voltage = data.voltage;
    //float charge_pc = (voltage - MIN_VOLTAGE) / ((MAX_VOLTAGE - MIN_VOLTAGE) / 100);
    m_Charge = std::round(voltage) / 10;
}

void PxFour::AltCallback(const std_msgs::msg::Float64 &data)
{
    DataCallback();
    m_Altitude = data.data;
}

void PxFour::LocalPositionCallback(const geometry_msgs::msg::PoseStamped&data)
{
    Debug("LOCALPOS CALLBACK", 4);
    DataCallback();
    m_LocalPosition = data.pose;
}

void PxFour::SetPointPositionLocal(float x, float y, float z, float yaw)
{
    //The pitch is the angle between the front vector and the horizontal plane
    //The yaw is the angle between the projection of the front vector onto the global horizontal plan
    //The roll is the angle between the local right vector and the global up vector, minus pi/2

    //    auto from_quaternion_to_normalize_quaternion = [=]()->double
//    {
//        double magnitude = std::sqrt(x * x + y * y + z * z + yaw * yaw);
//        return x / magnitude, y / magnitude, z / magnitude, yaw / magnitude;
//    };
//
//    double normalizedX, normalizedY, normalizedZ, normalizedYaw = from_quaternion_to_normalize_quaternion();

    auto from_quaternion_to_euler = [](double x, double y, double z, double yaw)
    {
        double return_yaw = atan2(2.0 * (y * z + yaw * x), yaw * yaw - x * x - y * y + z * z);
        double pitch = asin(-2.0 * (x * z - yaw * y));
        double roll = atan2(2.0 * (x * y + yaw * z), yaw * yaw + x * x - y * y - z * z);

        return return_yaw, pitch, roll;
    };

    auto to_quaternion = [](double yaw, double pitch, double roll)
    {
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;

        return w, x, y, z;
    };

    if(!IsActive())
        return;

    geometry_msgs::msg::Point point = m_LocalPosition.position;
    geometry_msgs::msg::Quaternion orientation = m_LocalPosition.orientation;

    double roll, pitch, new_yaw = from_quaternion_to_euler(x, y, z, yaw);

    new_yaw += yaw;

    double qw, qx, qy, qz = to_quaternion(new_yaw, pitch, roll);

    orientation.w = qw;
    orientation.x = qx;
    orientation.y = qy;
    orientation.z = qz;

    geometry_msgs::msg::PoseStamped message = geometry_msgs::msg::PoseStamped();

    message.header.stamp =  this->get_clock()->now();

    point.x += x * cos(new_yaw) - y * sin(new_yaw);

    point.y += x * sin(new_yaw) + y * cos(new_yaw);

    point.z += z;

    message.pose.position = point;
    message.pose.orientation =orientation;

    m_LocalTarget = message;

    PublishPoint();
}

void PxFour::AfterRun()
{
    m_FutureMover = std::async(std::launch::async, [this]{Mover();});
    Log("Mover started", INFO_LEVEL_LOG);
}

void PxFour::Mover()
{
    using namespace std::chrono_literals;

    while(IsRunning())
    {
        if((std::chrono::system_clock::now() - m_MoveTime).count() < 1)
        {
            Move(m_MoveSpeed);
            m_IsMoving = true;
        }

        else if(m_IsMoving)
        {
            Log("No move commands for 1s, auto-braking", WARN_LEVEL_LOG);
            Move(0);
            m_IsMoving = false;
        }

        m_FutureMover.wait_for(0.9s);
    }
}

void PxFour::PublishPoint()
{
//    if(m_GlobalTarget)
//        m_GlobalPublisher->publish(m_GlobalTarget);
//    else if(m_LocalTarget)
//        m_LocalPublisher->publish(m_LocalTarget);
}

void PxFour::SetMoveSettings(double speed)
{
    if(speed == 0)
    {
        Log("BRAKE", INFO_LEVEL_LOG);
        m_MoveSpeed = 0;
        //m_MoveTime = 0;
        return;
    }

    m_MoveSpeed = speed;
    m_MoveTime = std::chrono::system_clock::now();
    Log("Set new move speed: " + std::to_string(speed), INFO_LEVEL_LOG);
}
