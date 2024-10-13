#include <chrono>
#include <memory>
#include <queue>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp" // 用于接收敌人坐标
#include <queue> // 添加缺少的库文件

using namespace std::chrono_literals;

namespace rmoss_cam
{

// 创建一个继承自 rclcpp::Node 的 ImageSubscriberPublisher 类
class ImageSubscriberPublisher : public rclcpp::Node
{
public:
    // 构造函数，初始化节点名称，帧数和帧率
    ImageSubscriberPublisher() : Node("image_sub_pub"), frame_count_(0), fps_(0.0)
    {
        // 创建订阅者，订阅主题 "input_image"，队列长度10
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "input_image", 10,
            std::bind(&ImageSubscriberPublisher::topic_callback, this, std::placeholders::_1));
        
        // 创建发布者，发布到主题 "output_image"，队列长度10
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
        // 初始化时间戳
        last_time_ = this->now();

        // 创建订阅者，订阅敌人坐标信息 "enemy_coordinates"
        enemy_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "enemy_coordinates", 10,
            std::bind(&ImageSubscriberPublisher::enemy_callback, this, std::placeholders::_1));
        
        // 创建一个定时器，每1秒刷新一次队列
        timer_ = this->create_wall_timer(1s, std::bind(&ImageSubscriberPublisher::refresh_queue, this));
    }

private:
    // 订阅者的回调函数，当接收到图像时调用
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 将 ROS 图像消息转换为 OpenCV 格式的图像矩阵
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // 计算帧率
        frame_count_++;
        auto current_time = this->now();
        auto elapsed = current_time - last_time_;
        if (elapsed.seconds() > 1.0) {
            fps_ = frame_count_ / elapsed.seconds();
            frame_count_ = 0;
            last_time_ = current_time;
        }

        // 在图像上显示帧率信息
        cv::putText(frame, "FPS: " + std::to_string(fps_), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 255, 0), 2);

        // 将带有帧率的图像转换回 ROS 图像消息格式并发布
        auto output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*output_msg);

        // 显示图像（用于调试，可选）
        // cv::imshow("Image with FPS", frame);
        // cv::waitKey(1);
    }

    // 敌人坐标的回调函数
    void enemy_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // 将接收到的坐标数据存储到队列中
        if (enemy_coordinates_queue_.size() >= 10) {
            enemy_coordinates_queue_.pop(); // 队列已满时，删除最老的数据
        }
        enemy_coordinates_queue_.push(*msg);
    }

    // 每秒刷新一次队列
    void refresh_queue()
    {
        RCLCPP_INFO(this->get_logger(), "Refreshing enemy coordinates queue...");
        // 打印当前队列中的坐标信息（用于调试）
        std::queue<geometry_msgs::msg::Point> temp_queue = enemy_coordinates_queue_;
        while (!temp_queue.empty()) {
            geometry_msgs::msg::Point point = temp_queue.front();
            temp_queue.pop();
            RCLCPP_INFO(this->get_logger(), "Enemy at: [%.2f, %.2f, %.2f]", point.x, point.y, point.z);
        }
    }

    // 订阅者，用于订阅输入图像
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // 发布者，用于发布带有帧率信息的输出图像
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    // 上一次计算帧率的时间戳
    rclcpp::Time last_time_;
    // 帧计数器
    int frame_count_;
    // 当前帧率
    double fps_;

    // 用于接收敌人坐标的订阅者
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr enemy_subscription_;
    // 用于存储敌人坐标的队列，长度10
    std::queue<geometry_msgs::msg::Point> enemy_coordinates_queue_;
    // 定时器，用于每秒刷新一次队列
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rmoss_cam

// 主函数，初始化节点，开始执行回调函数并保持运行直到节点关闭
int main(int argc, char * argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建 ImageSubscriberPublisher 对象并启动节点
    rclcpp::spin(std::make_shared<rmoss_cam::ImageSubscriberPublisher>());
    // 节点关闭后清理资源
    rclcpp::shutdown();
    return 0;
}

/*
自行通过 ROS2 内置的调试工具查看坐标数据的数据类型：
1. 使用 `ros2 topic list` 查看当前运行中的所有主题。
2. 使用 `ros2 topic info <topic_name>` 查看特定主题的信息，返回信息中有主题的数据类型。
3. 使用 `ros2 topic echo <topic_name>` 查看特定主题的数据内容，确认数据格式。
4. 使用 `ros2 interface show <message_type>` 查看指定消息类型的结构，以确定应该使用的消息接口。
*/