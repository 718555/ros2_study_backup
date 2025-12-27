import math # 角度转弧度函数
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster # 静态坐标发布器
from geometry_msgs.msg import TransformStamped # 消息接口
from tf_transformations import quaternion_from_euler # 欧拉角转四元数


class TFBroadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')
        self.broadcaster_ = TransformBroadcaster(self) # 创建静态坐标发布器对象，第一个参数为node，也就是self
        # 动态TF需要持续发布，这里发布频率设置为 100HZ，每隔0.01s调用publish_tf函数
        self.timer_ = self.create_timer(0.01, self.publish_tf)

    def publish_tf(self):
        '''
        发布动态TF 从camera_link到bottle_link之间的坐标关系
        '''
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg() # 把系统当前时间转换为时钟消息
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = "bottle_link"
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.5
        # 欧拉角转四元数
        rotation_quat = quaternion_from_euler(0, 0, 0) 
        # quaternion_from_euler返回值是一个元组，顺序为q = x,y,z,w

        # 转四元数之后对transform的旋转部分进行赋值
        transform.transform.rotation.x = rotation_quat[0]
        transform.transform.rotation.y = rotation_quat[1]
        transform.transform.rotation.z = rotation_quat[2]
        transform.transform.rotation.w = rotation_quat[3]
      	# 发布静态坐标变换
        self.broadcaster_.sendTransform(transform)
        self.get_logger().info(f"发布 TF:{transform}")


def main():
    rclpy.init()
    tf_node = TFBroadcaster()
    rclpy.spin(tf_node)
    rclpy.shutdown()