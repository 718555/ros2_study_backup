import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, node_name: str, name: str, age: int) -> None:
        print('PersonNode__init__方法被调用了,添加了两个属性')
        super().__init__(node_name)
        self.age = age
        self.name = name

    def eat(self, food_name: str):
        self.get_logger().info(f'我叫{self.name},今年{self.age}岁，我现在正在吃{food_name}')

def main():
    rclpy.init()
    node = PersonNode('person_node_zhang', '法外狂徒张三', '18')
    node1 = PersonNode('person_node_li', '法外狂徒李四', '30')
    node.eat('鱼香肉丝')
    node1.eat('番茄')
    rclpy.spin(node)
    rclpy.shutdown()