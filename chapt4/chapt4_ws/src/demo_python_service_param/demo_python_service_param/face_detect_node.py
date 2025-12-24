import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector 
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # 用于转换格式
import cv2
import face_recognition
import time
from rcl_interfaces.msg import SetParametersResult
import os

class FaceDetectorionNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.bridge = CvBridge() # 把ros的消息格式转换为opencv格式
        self.service = self.create_service(FaceDetector, '/face_detect', self.detect_face_callback) 
        # 第一个参数为服务类型；第二个参数为服务的名字；第三个为回调函数；最后两个有默认值不用管
        
        #self.defaut_image_path = get_package_share_directory('demo_python_service')+'/resource/default.jpg'
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/default.jpg') # 第一个参数为功能包的名字

        self.upsample_times = 1
        self.model = "hog"
        self.get_logger().info("人脸检测服务已启动")

        # 声明和获取参数
        self.declare_parameter('face_locations_upsample_times', 1) # 第一个参数为名字第二个参数为值
        self.declare_parameter('face_locations_model', "hog")
        self.model = self.get_parameter("face_locations_model").value # 获取参数的值
        self.upsample_times = self.get_parameter("face_locations_upsample_times").value
        self.set_parameters([rclpy.Parameter('face_locations_model', rclpy.Parameter.Type.STRING, 'cnn')])
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(
                f'参数 {parameter.name} 设置为：{parameter.value}')
            if parameter.name == 'face_locations_upsample_times':
                self.upsample_times = parameter.value
            if parameter.name == 'face_locations_model':
                self.mode = parameter.value
        return SetParametersResult(successful=True)


    def detect_face_callback(self, request, response):
        # 如果客户端传的图像数据为空，则使用resource下的默认数据
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(
                request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info('传入图像为空，使用默认图像')
            # cv_image已经是一个opencv格式的图像了

        # 检测人脸
        start_time = time.time()
        self.get_logger().info('加载完图像，开始检测')
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.upsample_times, model=self.model)
        # face_locations为人脸个数。第一个参数为图像，第二个参数为上采样的次数，越高的次数就能找到越小的点，默认值为1，第三个参数为识别人脸的模型，默认为'hog'
        # 'hog'模型计算快但精度一般，除此之外还可以设置cnn模型
        end_time = time.time()
        self.get_logger().info(f'检测完成，耗时{end_time-start_time}')
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response # 必须返回response。人脸参数放到response中返回，ros帮我们传递给客户端


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorionNode()
    rclpy.spin(node)
    rclpy.shutdown()