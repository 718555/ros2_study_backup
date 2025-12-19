import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus # 导入消息接口

# 获取系统信息 
import psutil
import platform

class SysStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # 第一个参数消息接口，第二个参数话题名字，第三个参数qos
        self.status_publisher_ = self.create_publisher(
            SystemStatus, 'sys_status', 10)
        
        # 定时发布，一秒钟发布一次
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        cpu_percent = psutil.cpu_percent() # 获取cpu使用率
        memory_info = psutil.virtual_memory() # 获取系统内存信息
        net_io_counters = psutil.net_io_counters() # 网络相关输入输出信息

        '''
        builtin_interfaces/Time stamp # 记录时间戳
        string host_name 		# 主机名字
        float32 cpu_percent 	# CPU使用率
        float32 memory_percent 	# 内存使用率
        float32 memory_total 	# 内存总量
        float32 memory_available 	# 剩余有效内存
        float64 net_sent 		# 网络发送数据总量MB 1MB = 8Mb
        float64 net_recv 		# 网络接收数据总量
        '''

        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg() # 把系统当前时间转换为时钟消息
        msg.host_name = platform.node()
        msg.cpu_percent = cpu_percent
        msg.memory_percent = memory_info.percent
        msg.memory_total = memory_info.total / 1024 / 1024
        msg.memory_available = memory_info.available / 1024 / 1024
        msg.net_sent = net_io_counters.bytes_sent / 1024 / 1024
        msg.net_recv = net_io_counters.bytes_recv / 1024 / 1024  # 除1024是为了把字节变成兆

        self.get_logger().info(f'发布:{str(msg)}')
        self.status_publisher_.publish(msg)


def main():
    rclpy.init()
    node = SysStatusPub('sys_status_pub')
    rclpy.spin(node)
    rclpy.shutdown()