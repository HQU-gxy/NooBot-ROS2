#include <vector>
#include <array>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "serial/serial.h"

static const std::array<double, 36> odom_pose_covariance_imu{1e-3, 0, 0, 0, 0, 0,
                                                             0, 1e-3, 0, 0, 0, 0,
                                                             0, 0, 1e6, 0, 0, 0,
                                                             0, 0, 0, 1e6, 0, 0,
                                                             0, 0, 0, 0, 1e6, 0,
                                                             0, 0, 0, 0, 0, 1e3};

static const std::array<double, 36> odom_pose_covariance_enc{1e-9, 0, 0, 0, 0, 0,
                                                             0, 1e-3, 1e-9, 0, 0, 0,
                                                             0, 0, 1e6, 0, 0, 0,
                                                             0, 0, 0, 1e6, 0, 0,
                                                             0, 0, 0, 0, 1e6, 0,
                                                             0, 0, 0, 0, 0, 1e-9};

static const std::array<double, 36> odom_twist_covariance_imu{1e-3, 0, 0, 0, 0, 0,
                                                              0, 1e-3, 0, 0, 0, 0,
                                                              0, 0, 1e6, 0, 0, 0,
                                                              0, 0, 0, 1e6, 0, 0,
                                                              0, 0, 0, 0, 1e6, 0,
                                                              0, 0, 0, 0, 0, 1e3};

static const std::array<double, 36> odom_twist_covariance_enc{1e-9, 0, 0, 0, 0, 0,
                                                              0, 1e-3, 1e-9, 0, 0, 0,
                                                              0, 0, 1e6, 0, 0, 0,
                                                              0, 0, 0, 1e6, 0, 0,
                                                              0, 0, 0, 0, 1e6, 0,
                                                              0, 0, 0, 0, 0, 1e-9};

class NooBot : public rclcpp::Node
{
private:
    // Parameters
    std::string serialPortPath;
    int baudrate;
    std::string baseFrameId;
    std::string gyroFrameId;
    std::string odomTopic;
    std::string odomFrameId;

    serial::Serial botSerial;
    rclcpp::TimerBase::SharedPtr checkStatusTimer;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubsriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;

    rclcpp::Time lastUpdateOdomTime;

    static constexpr auto READ_STATUS_PERIOD = std::chrono::milliseconds(50);

    float positionX = 0;
    float positionY = 0;
    float orientation = 0;

    struct __attribute__((packed)) IMUData
    {
        // Accel in m/s^2
        float accelX;
        float accelY;
        float accelZ;

        // Ang-vel in rad/s
        float gyroX;
        float gyroY;
        float gyroZ;
    };

    struct __attribute__((packed)) BotStatusData
    {
        uint8_t header = 0x69;
        float currentLinear;     // Linear speed in m/s
        float currentAngular;    // Angular speed in rad/s
        uint8_t powerPercentage; // Battery percentage
        IMUData imu;
        uint8_t checksum;
    };

    void updateOdom(float currentLinear, float currentAngular)
    {
        auto timeNow = rclcpp::Node::now();
        auto duration = (timeNow - lastUpdateOdomTime).seconds();

        // Calculate the bot's position and orientation
        positionX += currentLinear * cos(currentAngular) * duration;
        positionY += currentLinear * sin(currentAngular) * duration;
        orientation += currentAngular * duration;

        // Prepare the odom msg
        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = timeNow;
        odomMsg.header.frame_id = odomFrameId;

        // Set the position and velocity
        odomMsg.pose.pose.position.x = positionX;
        odomMsg.pose.pose.position.y = positionY;

        odomMsg.child_frame_id = baseFrameId;
        odomMsg.twist.twist.linear.x = currentLinear;
        odomMsg.twist.twist.angular.z = currentAngular;

        // Use different covariance matrices depending on whether the bot is moving or not
        if (currentAngular == 0 && currentAngular == 0)
        {
            odomMsg.pose.covariance = odom_pose_covariance_enc;
            odomMsg.twist.covariance = odom_twist_covariance_enc;
        }
        else
        {
            odomMsg.pose.covariance = odom_pose_covariance_imu;
            odomMsg.twist.covariance = odom_twist_covariance_imu;
        }

        // Set the orientation in quaternion form
        tf2::Quaternion q;
        q.setRPY(0, 0, orientation);
        auto quatMsg = tf2::toMsg(q);
        odomMsg.pose.pose.orientation = quatMsg;

        odomPublisher->publish(odomMsg);

        lastUpdateOdomTime = timeNow;
    }

    void updateIMUData(const IMUData &imuData)
    {
        sensor_msgs::msg::Imu imuMsg;
        imuMsg.header.stamp = rclcpp::Node::now();
        imuMsg.header.frame_id = gyroFrameId;

        // Angular velocity
        imuMsg.angular_velocity.x = -imuData.gyroX; // It's inverted
        imuMsg.angular_velocity.y = imuData.gyroY;
        imuMsg.angular_velocity.z = -imuData.gyroZ; // It's inverted

        // Magic numbers
        imuMsg.angular_velocity_covariance[0] = 1e6;
        imuMsg.angular_velocity_covariance[4] = 1e6;
        imuMsg.angular_velocity_covariance[8] = 1e-6;

        // Acceleration
        imuMsg.linear_acceleration.x = -imuData.accelX; // It's inverted
        imuMsg.linear_acceleration.y = imuData.accelY;
        imuMsg.linear_acceleration.z = -imuData.accelZ; // It's inverted
        imuPublisher->publish(imuMsg);
    }

    static uint8_t calcSum(const uint8_t *buf, size_t len)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < len; i++)
        {
            sum ^= buf[i];
        }
        return sum;
    }

    void checkForStatus0()
    {
        if (!botSerial.available())
            return;

        uint8_t buf[96];
        auto readSize = botSerial.read(buf, sizeof(buf));

        for (size_t i = 0; i < readSize; i++)
        {
            // Not enough bytes
            if (readSize - i < static_cast<int>(sizeof(BotStatusData)))
            {
                RCLCPP_WARN(get_logger(), "Unexpected end of data");
                break;
            }
            // Find the header byte
            if (buf[i] == 0x69)
            {
                auto parsed = reinterpret_cast<BotStatusData *>(buf + i);
                auto sum = calcSum(buf + i, sizeof(BotStatusData) - 1);
                if (sum == parsed->checksum)
                {
                    updateOdom(parsed->currentLinear, parsed->currentAngular);
                    updateIMUData(parsed->imu);
                    return;
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Link checksum error: %02x, %02x", sum, parsed->checksum);
                }
            }
        }
    }

    void checkForStatus()
    {
        if (botSerial.available() && botSerial.read()[0] == 0x69)
        {
            BotStatusData readData;
            botSerial.read(reinterpret_cast<uint8_t *>(&readData) + 1, sizeof(BotStatusData) - 1);
            uint8_t sum = 0;
            for (uint8_t i = 0; i < sizeof(BotStatusData) - 1; i++)
            {
                sum ^= reinterpret_cast<uint8_t *>(&readData)[i];
            }

            if (sum != readData.checksum)
            {
                RCLCPP_WARN(get_logger(), "Link checksum error: %02x, %02x", sum, readData.checksum);
                return;
            }

            updateOdom(readData.currentLinear, readData.currentAngular);
            updateIMUData(readData.imu);
            botSerial.flushInput();
        }
    }

    void sendCmd(const float linear, const float angular)
    {
        struct __attribute__((packed)) UpLinkCommand
        {
            uint8_t header = 0x7b;
            float targetLinear;  // Linear speed in m/s
            float targetAngular; // Angular speed in rad/s
            uint8_t checksum = 0;
        } cmd{
            .targetLinear = linear,
            .targetAngular = angular};

        RCLCPP_INFO(this->get_logger(), "Sending cmd: %f, %f", linear, angular);
        cmd.checksum = calcSum(reinterpret_cast<const uint8_t *>(&cmd), sizeof(cmd) - 1);
        try
        {
            botSerial.write(reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
        }
    }

    void sendCmd(const geometry_msgs::msg::Twist &msg)
    {
        sendCmd(static_cast<float>(msg.linear.x), static_cast<float>(msg.angular.z));
    }

public:
    NooBot() : Node("noobot_link")
    {
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyNooBot");
        this->declare_parameter<std::string>("odom_topic", "odom");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

        this->get_parameter("serial_port", serialPortPath);
        this->get_parameter("serial_baud_rate", baudrate);
        this->get_parameter("odom_topic", odomTopic);
        this->get_parameter("odom_frame_id", odomFrameId);
        this->get_parameter("base_frame_id", baseFrameId);
        this->get_parameter("gyro_frame_id", gyroFrameId); // IMU topics correspond to TF coordinates //IMU话题对应TF坐标

        odomPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);      // Create the odometer topic publisher //创建里程计话题发布者
        imuPublisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2); // Create an IMU topic publisher //创建IMU话题发布者
        cmdVelSubsriber = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 2, [this](geometry_msgs::msg::Twist msg)
            { sendCmd(msg); });

        try
        {
            botSerial.setPort(serialPortPath);
            botSerial.setBaudrate(baudrate);
            auto timeout = serial::Timeout::simpleTimeout(200);
            botSerial.setTimeout(timeout);
            botSerial.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
        }

        if (botSerial.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            exit(1);
        }

        lastUpdateOdomTime = rclcpp::Node::now();

        checkStatusTimer = this->create_wall_timer(READ_STATUS_PERIOD, [this]
                                                   { checkForStatus(); });
    }

    ~NooBot()
    {
        sendCmd(0, 0);
        botSerial.close();
        RCLCPP_INFO(this->get_logger(), "Fucked off!");
    }
};