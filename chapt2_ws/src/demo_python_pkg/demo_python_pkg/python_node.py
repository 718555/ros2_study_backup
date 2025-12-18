import rclpy
from rclpy.node import Node

def main():
    rclpy.init() #初始化工作，分配资源
    node = Node("python_node") # 创造节点的实例对象
    node.get_logger().warn('你好 Python 节点1')
    rclpy.spin(node)  # 程序会在这里“卡住”循环等待,不断检测node是否收到新的话题数据，直到被关闭
    rclpy.shutdown() # 释放资源

#此处没有if __name__ == '__main__':
#ros2会自动在功能包里生成可执行文件去帮我们调用代码，在setup.py中entry_points添加'python_node=demo_python_pkg.python_node:main'也就是可执行文件的名称