#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono> // 引入时间相关头文件

using namespace std::chrono_literals;
//加上这一行后，你就可以直接在代码里写 1000ms（1000毫秒）或 1s（1秒），而不需要写复杂的 std::chrono::milliseconds(1000)。

class TurtleCircle : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_; // 定时器智能指针
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 创建发布者指针，此处指针为智能指针中的共享指针

public:
  explicit TurtleCircle(const std::string& node_name) : Node(node_name) // explicit杜绝隐式转换
  {
  	// 调用继承而来的父类函数创建发布者
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    // 尖括号中为消息类型。第一个参数为话题名字，第二个参数为消息长度

    // 调用继承而来的父类函数创建定时器
    timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
    // 第一个参数为周期，bind中第一个参数为回调函数的位置，第二个参数为指向当前对象的指针（类的内部直接用this代替，不需要&+名字）
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 1.0;
    msg.angular.z = 0.5;
    publisher_->publish(msg); //通过创建好的共享指针调用Publish方法
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleCircle>("turtle_square");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}