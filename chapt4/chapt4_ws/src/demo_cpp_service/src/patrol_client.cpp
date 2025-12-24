#include <chrono>
#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono> // 引入时间相关头文件

//加上这一行后，你就可以直接在代码里写 1000ms（1000毫秒）或 1s（1秒），而不需要写复杂的 std::chrono::milliseconds(1000)。
using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;
using SetP = rcl_interfaces::srv::SetParameters;

class PatrolClient : public rclcpp::Node
{
public:
  PatrolClient() : Node("patrol_client")
  {
    // 初始化
    srand(time(NULL)); // 初始化随机数种子
    patrol_client_ = this->create_client<Patrol>("patrol"); 
    // timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
    timer_ = this->create_wall_timer(10s, [&]()->void{
        // 1.检测服务端是否上线
        while (!this->patrol_client_->wait_for_service(1s)) //如果wait_for_service返回false说明服务没有上线，前面加！表示true，没上线就一直循环
        {
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(),"等待服务上线过程中，rclcpp挂了，我退下了");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"等待服务上线中...");
            
        }

        // 2.构造请求的对象
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 15; // 取余数，将值控制在0~14之间
        request->target_y = rand() % 15;
        RCLCPP_INFO(this->get_logger(), "请求巡逻：(%f,%f)", request->target_x, request->target_y);
        // 3.发送异步请求，然后等待返回，返回时调用回调函数

        //运行过程：此处异步调用之后，不可能立即返回response对象，他会返回SharedFuture的对象，在sharedfuture对象中，他放了response，但是刚调用完异步发送出去之后，response是空的，服务处理完成后才会有真正的response
        patrol_client_->async_send_request( // 调用该函数发送请求给服务端，服务端处理完成后会自动调用我们的回调函数，服务端处理完之后会自动调用我们的回调函数，在回调函数中我们就知道服务端的处理结果了
            request,
            [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void
            {
            auto response = result_future.get();
            if (response->result == Patrol::Response::SUCCESS)
            { 
                RCLCPP_INFO(this->get_logger(), "请求巡逻目标点成功");
            }
            else if (response->result == Patrol::Response::FAIL)
            {
                RCLCPP_INFO(this->get_logger(), "请求巡逻目标点失败");
            }
        });
        
    }); // 使用lambda表达式

  }

private:
  rclcpp::TimerBase::SharedPtr timer_; // 定时器
  rclcpp::Client<Patrol>::SharedPtr patrol_client_; // 客户端
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}