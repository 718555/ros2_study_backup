import math # 角度转弧度函数
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster # 静态坐标发布器
from geometry_msgs.msg import TransformStamped # 消息接口
from tf_transformations import quaternion_from_euler # 欧拉角转四元数


class StaticTFBroadcaster(Node):

    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self) # 创建静态坐标发布器对象，第一个参数为node，也就是self
        self.publish_static_tf() # 发布静态TF

    def publish_static_tf(self):
        '''
        发布静态TF 从base_link到camera_link之间的坐标关系
        '''
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg() # 把系统当前时间转换为时钟消息
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "camera_link"
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6
        # 欧拉角转四元数
        rotation_quat = quaternion_from_euler(math.radians(180), 0, 0) # math.radians(180)，把角度值转弧度制
        # quaternion_from_euler返回值是一个元组，顺序为q = x,y,z,w

        # 转四元数之后对transform的旋转部分进行赋值
        transform.transform.rotation.x = rotation_quat[0]
        transform.transform.rotation.y = rotation_quat[1]
        transform.transform.rotation.z = rotation_quat[2]
        transform.transform.rotation.w = rotation_quat[3]
      	# 发布静态坐标变换
        self.static_broadcaster_.sendTransform(transform)
        self.get_logger().info(f"发布 TF:{transform}")


def main():
    rclpy.init()
    static_tf_broadcaster = StaticTFBroadcaster()
    rclpy.spin(static_tf_broadcaster)
    rclpy.shutdown()