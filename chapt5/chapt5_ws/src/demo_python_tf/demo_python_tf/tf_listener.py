import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer # 坐标监听器
from tf_transformations import euler_from_quaternion # 四元数转欧拉角，欧拉角：翻滚、俯仰、偏航


class TFListener(Node):

    def __init__(self):
        super().__init__("tf2_listener")
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.timer_ = self.create_timer(1, self.get_transform)

    def get_transform(self):
        '''
        实时查询坐标关系
        '''
        try:
            result = self.buffer_.lookup_transform("base_link", "bottle_link", rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            # 查询base_link到bottle_link的坐标变换关系，rclpy.time.Time(seconds=0)表示最新的坐标关系，
            # rclpy.time.Duration为超时时间，这里的1s是阻塞等待，定时器每1s尝试一次，每次等待1s获取数据，没获取到数据抛出异常

            transform = result.transform
            # transform中存储的为四元数，需要转换为欧拉角，此处转换后的顺序为欧拉角：翻滚、俯仰、偏航（roll,pitch,yaw）
            rotation_euler = euler_from_quaternion([ # 需要传入数组
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.get_logger().info(f"平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}")
        except Exception as e:
            self.get_logger().warn(f"不能够获取坐标变换，原因: {str(e)}")


def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()