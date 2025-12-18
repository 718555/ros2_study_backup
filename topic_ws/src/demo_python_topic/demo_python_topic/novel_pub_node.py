import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue # 队列相关库

class NovelPubNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name},启动！')
        self.novels_queue_ = Queue() # 创建队列
        # 队列要在self.create_timer前创建，不然在def timer_callback(self):中可能找不到
        
        # 创建话题发布者
        self.novel_publisher_ = self.create_publisher(String,'novel',10)
        # 第一个参数为消息类型，第二个参数为话题名字，第三个表示要存储的消息的队列长度 ，剩下几个参数暂时不用管 
        # 不想让消息有缓存，就从10改1，发的消息和收到消息不一致，就改大

        # 定时发布
        self.create_timer(5,self.timer_callback)
        # 第一个参数为浮点型时间周期单位为秒，第二个callback为回调函数
        # 每间隔五秒钟调用一次回调函数

    def timer_callback(self):
        if self.novels_queue_.qsize() > 0:
            # self.novel_publisher_.publish()  # 发布话题
            # 命名中下划线为命名规范表示是个成员不是函数
            line = self.novels_queue_.get() # 从队列中取出来数据
            msg = String() # 创建消息对象
            msg.data = line
            self.novel_publisher_.publish(msg) # 发布消息
            self.get_logger().info(f'发布了：{msg}')

    def download(self,url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        text = response.text
        # text.splitlines() # 按行分割为数组

        # 把小说按行分割为数组放入队列
        for line in text.splitlines():
            self.novels_queue_.put(line)

        self.get_logger().info(f'下载{url},{len(text)}')
        # self.novel_publisher_.publish()  # 发布话题
        # response.text 

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub')
    node.download('http://localhost:8001/novel1.txt')
    rclpy.spin(node)
    rclpy.shutdown()
        