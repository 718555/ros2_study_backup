import espeakng 
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue # 队列相关库
import threading
import time # 休眠相关库

class NovelSubNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name},启动！')

        # 创建队列去使用数据
        self.novels_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(String,'novel',self.novel_callback,10) # 创建订阅者
        # 第一个参数消息类型，第二个参数话题，第三个参数为callback为回调函数，第四个参数为qos

        # 创建一个单线程去读小说
        self.speech_thread_ = threading.Thread(target=self.speake_thread)
        self.speech_thread_.start() # python线程不会自动启动，需要手动启动，不然的话队列中有数据，但是speake_thread没有运行


    def novel_callback(self,msg):
        # 把小说放入队列
        self.novels_queue_.put(msg.data) # msg本身是string的一个对象，我们只需要里面的数据也就是data

    def speake_thread(self):
        speaker = espeakng.Speaker() # 生成一个类的对象叫speaker
        speaker.voice = 'zh' # 表示读的是中文

        while rclpy.ok(): # 检测当前ros上下文是否ok，一般返回是
            if self.novels_queue_.qsize()>0:
                text = self.novels_queue_.get()
                self.get_logger().info(f'朗读{text}') # 打印一下检测是否正确
                speaker.say(text) # 说
                speaker.wait() # 程序会卡在这里，直到语音播报完毕
            else:
                # 队列中没有数据时，让当前的线程休眠1s，不用的时候休眠可以降低CPU功耗
                time.sleep(1)


def main():
    rclpy.init()
    node = NovelSubNode('novel_sub')
    rclpy.spin(node)
    rclpy.shutdown()
        