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

    /**
     * @brief Update the position and publish odom msg
     *
     * @param currentLinear Current linear velocity of the bot in m/s
     * @param currentLinear Current angular velocity of the bot in rad/s
     */
    void updateOdom(float currentLinear, float currentAngular)
    {
        auto timeNow = rclcpp::Node::now();
        auto duration = (timeNow - lastUpdateOdomTime).seconds();

        // Prepare the odom msg
        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = timeNow;
        odomMsg.header.frame_id = odomFrameId;

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
        orientation += currentAngular * duration;
        tf2::Quaternion q;
        q.setRPY(0, 0, orientation);
        auto quatMsg = tf2::toMsg(q);
        odomMsg.pose.pose.orientation = quatMsg;

        odomPublisher->publish(odomMsg);

        lastUpdateOdomTime = timeNow;
    }

    /**
     * @brief Publish IMU msg
     *
     * @param imuData IMU data with linear acceleration and angular velocity of the bot
     */
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

    /**
     * @brief Calculate the 'sum' by XOR
     *
     * @param data The pointer to the data
     * @param len The length of the data
     *
     * @return The 'sum' of the data in 1 byte
     */
    static uint8_t calcSum(const uint8_t *data, size_t len)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < len; i++)
        {
            sum ^= data[i];
        }
        return sum;
    }

    /**
     * @brief Check if the bot has sent valid status data here
     */
    void checkForStatus()
    {
        if (botSerial.available() && botSerial.read()[0] == 0x69)
        {
            BotStatusData readData;
            auto readsize = botSerial.read(reinterpret_cast<uint8_t *>(&readData) + 1, sizeof(BotStatusData) - 1);

            if (readsize != sizeof(BotStatusData) - 1)
            {
                RCLCPP_WARN(get_logger(), "Unexpected end of data");
                return;
            }

            auto sum = calcSum(reinterpret_cast<uint8_t *>(&readData), sizeof(BotStatusData));
            if (sum != 0) // It's XOR, so the checksum should be 0
            {
                RCLCPP_WARN(get_logger(), "Link checksum error: %02x", sum);
                return;
            }

            updateOdom(readData.currentLinear, readData.currentAngular);
            updateIMUData(readData.imu);
            botSerial.flushInput();
        }
    }

    /**
     * @brief Send velocity command to the bot
     *
     * @param linear Linear velocity in m/s
     * @param linear Angular velocity in rad/s
     */
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

    /**
     * @brief Send velocity command to the bot
     *
     * @param msg Twist msg with linear.x and angular.z
     */
    void sendCmd(const geometry_msgs::msg::Twist &msg)
    {
        sendCmd(static_cast<float>(msg.linear.x), static_cast<float>(msg.angular.z));
    }

public:
    NooBot() : Node("noobot_link")
    {
        this->declare_parameter<int>("serial_baud_rate", 115200);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
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

        try
        {
            botSerial.setPort(serialPortPath);
            botSerial.setBaudrate(baudrate);
            auto timeout = serial::Timeout::simpleTimeout(200);
            botSerial.setTimeout(timeout);
            botSerial.open();
            botSerial.flushInput();
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

        odomPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);      // Create the odometer topic publisher //创建里程计话题发布者
        imuPublisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2); // Create an IMU topic publisher //创建IMU话题发布者
        cmdVelSubsriber = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 2, [this](geometry_msgs::msg::Twist msg)
            { sendCmd(msg); });

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