#include <chrono>
#include <memory>
#include <queue>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class CoordinateTransformer : public rclcpp::Node
{
public:
    CoordinateTransformer() : Node("coordinate_transformer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // 订阅接收到的左手系点数据
        point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/left_hand_point",
            10,
            std::bind(&CoordinateTransformer::convertAndStorePoint, this, std::placeholders::_1));

        // 创建 Marker 发布器
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    }

private:
    // TF buffer 和 listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 订阅器
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;

    // Marker 发布器
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // 存储转换后的点队列
    std::queue<geometry_msgs::msg::Point> point_queue_;

    // 将左手坐标系转换到右手坐标系
    geometry_msgs::msg::Point leftToRightHand(const geometry_msgs::msg::Point &point)
    {
        geometry_msgs::msg::Point right_hand_point;
        right_hand_point.x = point.x;
        right_hand_point.y = point.y;
        right_hand_point.z = -point.z; // 左手系到右手系的转换
        return right_hand_point;
    }

    // 将点转换并存储到队列中
    void convertAndStorePoint(const geometry_msgs::msg::Point::SharedPtr point_msg)
    {
        // 第一步：将左手坐标系转换为右手坐标系
        geometry_msgs::msg::Point right_hand_point = leftToRightHand(*point_msg);

        // 构造 PointStamped 消息
        geometry_msgs::msg::PointStamped point_stamped;
        point_stamped.header.frame_id = "camera_link"; // 相机光心坐标系
        point_stamped.header.stamp = this->get_clock()->now();
        point_stamped.point = right_hand_point;

        // 第二步：使用 tf2 将相机光心坐标系下的点转换到 base_link 坐标系
        try
        {
            geometry_msgs::msg::PointStamped transformed_point;
            transformed_point = tf_buffer_.transform(
                point_stamped,
                "base_link",
                tf2::durationFromSec(1.0));

            // 将转换后的点存储到队列中
            point_queue_.push(transformed_point.point);

            // 保持队列中最多有两个点
            if (point_queue_.size() > 2)
            {
                point_queue_.pop();
            }

            // 发布 Marker
            if (point_queue_.size() == 2)
            {
                publishMarker();
                RCLCPP_INFO(this->get_logger(), "Point queue size: %ld", point_queue_.size());

            }

            RCLCPP_INFO(this->get_logger(), "Transformed point: [%.2f, %.2f, %.2f]",
                        transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    // 发布 Marker
    void publishMarker()
    {
        // 获取队列中的两个点
        geometry_msgs::msg::Point start_point = point_queue_.front();
        geometry_msgs::msg::Point end_point = point_queue_.back();

        // 创建 Marker 消息
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "coordinate_transformer";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置箭头的起点和终点
        marker.points.resize(2);
        marker.points[0] = start_point;
        marker.points[1] = end_point;

        // 设置箭头的属性
        marker.scale.x = 0.05;  // 箭头的轴的直径
        marker.scale.y = 0.1;   // 箭头的头的直径
        marker.scale.z = 0.1;   // 箭头的头的长度

        marker.color.r = 1.0;   // 红色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;   // 不透明

        // 发布 Marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Publishing marker");

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinateTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
