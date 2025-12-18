import rclpy
from rclpy.node import Node

def main():
    rclpy.init() #初始化工作，分配资源
    node = Node("python_node") # 创造节点的实例对象
    node.get_logger().warn('你好 Python 节点1')
    rclpy.spin(node)  # 程序会在这里“卡住”循环等待,不断检测node是否收到新的话题数据，直到被关闭
    rclpy.shutdown() # 释放资源

if __name__ == '__main__':
    main()

#它的意思是：“如果我现在是作为主程序（Main Program）在运行，那就执行 main() 函数；
# 如果我只是被别人导入（Import）来调用的，那就什么也不做（不要自动执行 main）。”