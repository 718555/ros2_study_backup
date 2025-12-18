#include "rclcpp/rclcpp.hpp"
#include "iostream"

//isotream是系统库无需查找，rclcpp不是系统库所以需要在cmakelist中寻找并添加

int main(int argc, char** argv) //argc和argv为入口参数
{

    rclcpp::init(argc,argv); //初始化
    auto node = std::make_shared<rclcpp::Node>("cpp_node");//创建节点
    RCLCPP_INFO(node->get_logger(), "你好， C++ 节点");//打印日志
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}