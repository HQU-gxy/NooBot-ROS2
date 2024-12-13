#include "turn_on_wheeltec_robot/wheeltec_robot.h"
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"
// #include "wheeltec_robot_msg/msg/data.hpp"

sensor_msgs::msg::Imu Mpu6050; // Instantiate an IMU object //实例化IMU对象

using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;

/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);    // ROS initializes and sets the node name //ROS初始化 并设置节点名称
  turn_on_robot Robot_Control; // Instantiate an object //实例化一个对象
  Robot_Control.Control();     // Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
  return 0;
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  auto cmd = buildCommand(twist_aux->linear.x, twist_aux->angular.z);
  try
  {
    tankSerial.write(reinterpret_cast<uint8_t *>(cmd.get()), sizeof(UpLinkCommand)); // Sends data to the downloader via serial port //通过串口向下位机发送数据
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port")); // If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub; // Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = rclcpp::Node::now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id;       // IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack
                                                      // IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; // A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; // Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; // Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
  imu_publisher->publish(Imu_Data_Pub); // Pub IMU topic //发布IMU话题
}
/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
  // Convert the Z-axis rotation Angle into a quaternion for expression
  // 把Z轴转角转换为四元数进行表达
  tf2::Quaternion q;
  q.setRPY(0, 0, Robot_Pos.Z);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  nav_msgs::msg::Odometry odom; // Instance the odometer topic data //实例化里程计话题数据
  odom.header.stamp = rclcpp::Node::now();
  ;
  odom.header.frame_id = odom_frame_id;    // Odometer TF parent coordinates //里程计TF父坐标
  odom.pose.pose.position.x = Robot_Pos.X; // Position //位置
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = Robot_Pos.Z;
  odom.pose.pose.orientation = odom_quat; // Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

  odom.child_frame_id = robot_frame_id;    // Odometer TF subcoordinates //里程计TF子坐标
  odom.twist.twist.linear.x = Robot_Vel.X; // Speed in the X direction //X方向速度
  // odom.twist.twist.linear.y = Robot_Vel.Y;  // Speed in the Y direction //Y方向速度
  odom.twist.twist.angular.z = Robot_Vel.Z; // Angular velocity around the Z axis //绕Z轴角速度

  // There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
  // 这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
  if (Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0)
    // If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
    // 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
        memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  else
    // If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
    // 如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
        memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
  odom_publisher->publish(odom); // Pub odometer topic //发布里程计话题
}

/**************************************
Date: November 18, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
Update Note: This checking method can lead to read error data or correct data not to be processed. Instead of this checking method, frame-by-frame checking is now used.
             Refer to Get_ Sensor_ Data_ New() function
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
更新说明：该校验方法会导致出现读取错误数据或者正确数据不处理的情况，现在已不用该校验方法，换成逐帧校验方式，参考Get_Sensor_Data_New()函数
***************************************/
bool turn_on_robot::Get_Sensor_Data()
{
  struct __attribute__((packed)) StatusData
  {
    float currentLinear;  // Linear speed in m/s
    float currentAngular; // Angular speed in rad/s
    // Accel in m/s^2
    IMUData imu;
    uint8_t checksum;
  };

  if (tankSerial.available() && tankSerial.read()[0] == 0x69)
  {
    StatusData readData;
    tankSerial.read(reinterpret_cast<uint8_t *>(&readData), sizeof(StatusData));
    uint8_t sum = 0x69; // The header
    for (uint8_t i = 0; i < sizeof(StatusData) - 1; i++)
    {
      sum ^= reinterpret_cast<uint8_t *>(&readData)[i];
    }

    if (sum != readData.checksum)
    {
      RCLCPP_WARN(get_logger(), "Link checksum error: %d, %d", sum, readData.checksum);
      return false;
    }

    // Get the movement speed
    Robot_Vel.X = readData.currentLinear;
    Robot_Vel.Z = readData.currentAngular;

    // Get the IMU data
    Mpu6050.linear_acceleration.set__x(readData.imu.accelX).set__y(readData.imu.accelY).set__z(readData.imu.accelZ);
    Mpu6050.angular_velocity.set__x(readData.imu.gyroX).set__y(readData.imu.gyroY).set__z(readData.imu.gyroZ);
    return true;
  }

  return false;
}

/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_robot::Control()
{
  //_Last_Time = ros::Time::now();
  _Last_Time = rclcpp::Node::now();
  while (rclcpp::ok())
  {
    try
    {
      //_Now = ros::Time::now();
      _Now = rclcpp::Node::now();
      Sampling_Time = (_Now - _Last_Time).seconds(); // Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage)
                                                     // 获取时间间隔，用于积分速度获得位移(里程)
      if (Get_Sensor_Data())                         // The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
                                                     // 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
      {
        Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; // Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
        Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; // Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
        Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;                                                       // The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad

        // Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
        // 通过IMU绕三轴角速度与三轴加速度计算三轴姿态
        Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                            Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

        Publish_Odom();      // Pub the speedometer topic //发布里程计话题
        Publish_ImuSensor(); // Pub the IMU topic //发布IMU话题
        // Publish_Voltage();   // Pub the topic of power supply voltage //发布电源电压话题

        _Last_Time = _Now; // Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔
      }

      rclcpp::spin_some(this->get_node_base_interface()); // The loop waits for the callback function //循环等待回调函数
    }

    catch (const rclcpp::exceptions::RCLError &e)
    {
      RCLCPP_ERROR(this->get_logger(), "unexpectedly failed whith %s", e.what());
    }
  }
}
/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot() : rclcpp::Node("wheeltec_robot")
{
  Sampling_Time = 0;
  // Clear the data
  // 清空数据
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));

  // ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  // The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  // private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值

  this->declare_parameter<int>("serial_baud_rate");
  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyTank");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

  this->get_parameter("serial_baud_rate", serial_baud_rate); // Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  this->get_parameter("usart_port_name", usart_port_name);   // Fixed serial port number //固定串口号
  this->get_parameter("odom_frame_id", odom_frame_id);       // The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  this->get_parameter("robot_frame_id", robot_frame_id);     // The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  this->get_parameter("gyro_frame_id", gyro_frame_id);       // IMU topics correspond to TF coordinates //IMU话题对应TF坐标

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);      // Create the odometer topic publisher //创建里程计话题发布者
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2); // Create an IMU topic publisher //创建IMU话题发布者

  // Set the velocity control command callback function
  // 速度控制命令订阅回调函数设置
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "wheeltec_robot Data ready"); // Prompt message //提示信息

  try
  {
    // Attempts to initialize and open the serial port //尝试初始化与开启串口
    tankSerial.setPort(usart_port_name);                          // Select the serial port number to enable //选择要开启的串口号
    tankSerial.setBaudrate(serial_baud_rate);                     // Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); // Timeout //超时等待
    tankSerial.setTimeout(_time);
    tankSerial.open(); // Open the serial port //开启串口
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), "wheeltec_robot can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if (tankSerial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "wheeltec_robot serial port opened"); // Serial port opened successfully //串口开启成功提示
  }
}

std::unique_ptr<UpLinkCommand> turn_on_robot::buildCommand(float linear, float angular)
{
  auto cmd = std::make_unique<UpLinkCommand>();

  cmd->targetAngular = angular;
  cmd->targetLinear = linear;

  uint8_t sum = 0;
  for (uint8_t i = 0; i < sizeof(UpLinkCommand) - 1; i++)
  {
    sum ^= reinterpret_cast<uint8_t *>(cmd.get())[i];
  }
  cmd->checksum = sum;
  return cmd;
}

/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  // Sends the stop motion command to the lower machine before the turn_on_robot object ends
  // 对象turn_on_robot结束前向下位机发送停止运动命令

  auto cmd = buildCommand(0, 0);

  try
  {
    tankSerial.write(reinterpret_cast<uint8_t *>(cmd.get()), sizeof(UpLinkCommand)); // Send data to the serial port //向串口发数据
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  tankSerial.close();                               // Close the serial port //关闭串口
  RCLCPP_INFO(this->get_logger(), "Shutting down"); // Prompt message //提示信息
}
