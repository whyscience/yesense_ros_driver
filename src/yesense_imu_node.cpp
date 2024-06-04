#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include "serial/serial.h"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>

#include "analysis_data.h"

enum PARSE_STATES {
    FIND_HEAD_0,
    FIND_HEAD_1,
    FIND_LENGTH,
    FIND_END
};

sensor_msgs::msg::Imu g_imu;
rclcpp::Clock::SharedPtr clock_;

void publish_imu(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub,
                 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub,
                 const protocol_info_t &imu_data) {
    // publish imu message
    g_imu.header.stamp = clock_->now();

    tf2::Quaternion q;
    q.setRPY(imu_data.roll / 180.0 * M_PI, imu_data.pitch / 180.0 * M_PI, imu_data.yaw / 180.0 * M_PI);
    g_imu.orientation.w = q.w();
    g_imu.orientation.x = q.x();
    g_imu.orientation.y = q.y();
    g_imu.orientation.z = q.z();

    g_imu.angular_velocity.x = imu_data.angle_x / 180.0 * M_PI;
    g_imu.angular_velocity.y = imu_data.angle_y / 180.0 * M_PI;
    g_imu.angular_velocity.z = imu_data.angle_z / 180.0 * M_PI;

    g_imu.linear_acceleration.x = imu_data.accel_x;
    g_imu.linear_acceleration.y = imu_data.accel_y;
    g_imu.linear_acceleration.z = imu_data.accel_z;

    imu_pub->publish(g_imu);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "imu_link";
    pose.header.stamp = g_imu.header.stamp;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = g_imu.orientation;
    imu_pose_pub->publish(pose);
}

int main(int argc, char **argv) {
    serial::Serial ser;
    std::string port;
    int baud_rate = 460800;
    int buf_size = 1000;
    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string frame_id;
    double time_offset_in_seconds;
    bool broadcast_tf;
    double linear_acceleration_stddev;
    double angular_velocity_stddev;
    double orientation_stddev;
    // uint8_t last_received_message_number;
    // bool received_message = false;
    // int data_packet_start;

    tf2::Quaternion orientation;
    tf2::Quaternion zero_orientation;

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("yesense_imu_node");
    clock_ = node->get_clock();

    node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    node->declare_parameter<int>("baud_rate", 460800);
    node->declare_parameter<std::string>("tf_parent_frame_id", "imu_base");
    node->declare_parameter<std::string>("tf_frame_id", "imu_link");
    node->declare_parameter<std::string>("frame_id", "imu_link");
    node->declare_parameter<double>("time_offset_in_seconds", 0.0);
    node->declare_parameter<bool>("broadcast_tf", true);
    node->declare_parameter<double>("linear_acceleration_stddev", 0.0);
    node->declare_parameter<double>("angular_velocity_stddev", 0.0);
    node->declare_parameter<double>("orientation_stddev", 0.0);

    node->get_parameter("port", port);
    node->get_parameter("baud_rate", baud_rate);
    node->get_parameter("tf_parent_frame_id", tf_parent_frame_id);
    node->get_parameter("tf_frame_id", tf_frame_id);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("time_offset_in_seconds", time_offset_in_seconds);
    node->get_parameter("broadcast_tf", broadcast_tf);
    node->get_parameter("linear_acceleration_stddev", linear_acceleration_stddev);
    node->get_parameter("angular_velocity_stddev", angular_velocity_stddev);
    node->get_parameter("orientation_stddev", orientation_stddev);

    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
    auto imu_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("imu/pose", 100);

    rclcpp::Rate r(200); // 200 hz

    g_imu.header.frame_id = frame_id;

    g_imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
    g_imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
    g_imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

    g_imu.angular_velocity_covariance[0] = angular_velocity_stddev;
    g_imu.angular_velocity_covariance[4] = angular_velocity_stddev;
    g_imu.angular_velocity_covariance[8] = angular_velocity_stddev;

    g_imu.orientation_covariance[0] = orientation_stddev;
    g_imu.orientation_covariance[4] = orientation_stddev;
    g_imu.orientation_covariance[8] = orientation_stddev;

    std::string input;
    std::string read;
    PARSE_STATES state = FIND_HEAD_0;
    int data_len = 0;
    // int head_ind = 0;

    while (rclcpp::ok()) {
        try {
            if (ser.isOpen()) {
                // read string from serial device
                if (ser.available()) {
                    read = ser.read(ser.available());

                    if ((int) input.size() > buf_size) {
                        RCLCPP_WARN(node->get_logger(), "input.size() > buf_size: %ld/%d", input.size(), buf_size);
                        input.clear();
                        state = FIND_HEAD_0;
                    }

                    for (unsigned int i = 0; i < read.size(); i++) {
                        if (state == FIND_HEAD_0) {
                            if (read.at(i) == (char) (0x59)) {
                                input.clear();
                                input += read.at(i);
                                state = FIND_HEAD_1;
                            }
                        } else if (state == FIND_HEAD_1) {
                            if (read.at(i) == (char) (0x53)) {
                                input += read.at(i);
                                state = FIND_LENGTH;
                            } else {
                                state = FIND_HEAD_0;
                            }
                        } else if (state == FIND_LENGTH) {
                            input += read.at(i);
                            if (input.size() > 7) {
                                unsigned char *len = (unsigned char *) input.data() + 4;
                                data_len = *len;
                                data_len += 7;
                                state = FIND_END;
                            }
                        } else if (state == FIND_END) {
                            input += read.at(i);
                            if ((int) input.size() >= data_len) {
                                std::string sentence = input;
                                int e = analysis_data((unsigned char *) sentence.data(), sentence.size());
                                if (e != analysis_ok) {
                                    RCLCPP_WARN(node->get_logger(), "analysis_data() FAILED!!!! %d", e);
                                    for (unsigned int i = 0; i < sentence.size(); i++) {
                                        char aaa[1024] = {0};
                                        sprintf(aaa, "%02X ", (unsigned char) sentence.at(i));
                                        std::cout << aaa;
                                    }
                                    std::cout << "data len: " << data_len << std::endl;
                                } else {
                                    publish_imu(imu_pub, imu_pose_pub, g_output_info);
                                }

                                state = FIND_HEAD_0;
                            }
                        }
                    }
                }
            } else {
                // try and open the serial port
                try {
                    RCLCPP_INFO(node->get_logger(), "port:%s, rate:%d", port.c_str(), baud_rate);
                    ser.setPort(port);
                    ser.setBaudrate(baud_rate);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser.setTimeout(to);
                    ser.open();
                } catch (serial::IOException &e) {
                    RCLCPP_ERROR(node->get_logger(), "Unable to open serial port %s. Trying again in 5 seconds.",
                                 ser.getPort().c_str());
                    rclcpp::sleep_for(std::chrono::seconds(5));
                }

                if (ser.isOpen()) {
                    RCLCPP_DEBUG(node->get_logger(), "Serial port %s initialized and opened.", ser.getPort().c_str());
                }
            }
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(node->get_logger(), "Error reading from the serial port %s. Closing connection.",
                         ser.getPort().c_str());
            ser.close();
        }
        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
