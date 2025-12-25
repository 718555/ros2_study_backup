import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import os
import time


class FaceDetectorClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector, '/face_detect') # 创建客户端
        # 第一个参数为服务srv的类型，第二个参数为服务的名字，
        self.bridge = CvBridge()
        self.test1_image_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/test1.jpg') 
        self.image = cv2.imread(self.test1_image_path)

    # 发送请求给客户端
    def send_request(self): # self指代FaceDetectorClient类的实例也就是face_detect_client = FaceDetectorClient()中的face_detect_client
        # 1.判断服务是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False: 
            # 等待的时候阻塞该函数，等待了一秒钟如果服务端还没上线的话他就返回，返回值是false
            self.get_logger().info(f'等待服务端上线....')

        # 2.构造 Request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image) # 把cv2格式转换为msg格式

        # 3.发送请求并 spin 等待服务处理完成
        future = self.client.call_async(request) # 创建服务请求并异步的获取结果
        # 现在的future并没有包含响应结果，需要等待服务端处理完成才会把结果放到future中 
        
        # while not future.done():
        #     time.sleep(1.0) # 休眠当前线程，等待服务处理完成===由于ros2默认单线程，这会造成当前线程无法再接受来自服务端的返回，导致永远没有办法完成
        # rclpy.spin_until_future_complete(self, future) # 此处回等待服务端返回响应 。为了不阻塞，使用回调函数
        # 此命令会在后台边去查看future是否完成边去spin接收结果，有结果了就把结果放到future中，这行代码就结束


        def result_callback(result_future): 
                response = result_future.result() # 获取响应
                self.get_logger().info(f'接收到响应: 图像中共有：{response.number}张脸，耗时{response.use_time}')
                self.show_face_locations(response)

        future.add_done_callback(result_callback) # 此处传入的回调函数result_callback调用的参数就是future

    # 对响应结果的处理
    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top),
                          (right, bottom), (255, 0, 0), 2)

        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0) # 也是阻塞的，会导致spin无法运行，本案例中，只发送一次请求没事，若发送多次请求，需要先把waitkey关掉


def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetectorClient()
    face_detect_client.send_request()
    rclpy.spin(face_detect_client)
    rclpy.shutdown()