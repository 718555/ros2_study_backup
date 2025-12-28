import launch
import launch_ros.parameter_descriptions
from ament_index_python.packages import get_package_share_directory # 找目录
import os


def generate_launch_description():
    # 获取默认的urdf路径
    urdf_package_path = get_package_share_directory('fishbot_description') # 获取到了share目录
    default_urdf_path = os.path.join(urdf_package_path,'urdf','first_robot.urdf')
    # default_urdf_path = urdf_package_path + '/urdf/fish_robot.urdf'
    # default_rviz_config_path = urdf_package_path + '/config/rviz/display_model.rviz'
    default_rviz_config_path = os.path.join(urdf_package_path,'config','rviz/display_model.rviz')

    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_urdf_path),
        description='URDF 的绝对路径')
    
    # 通过文件路径获取文件内容，并转换成参数值对象，以供传入robot_state_publisher_node，生成新的参数
    substitutions_command_result = launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]) # 相当于运行cat＋路径，显示文件的内容。需要传入数组，加中括号，cat后面有空格
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,value_type=str) 
    # launch_ros.parameter_descriptions.ParameterValue用于转换参数值对象，value_type=str说明返回值是字符串类型

    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path] # parameters为节点的参数，arguments的传递方式是在命令行后面加东西
        # 和 ros2 run rviz2 rviz2 -d /root/study/chapt6/chapt6_ws/src/fishbot_description/config/rviz/display_model.rviz效果一样
        # parameters为 ros2 run rviz2 rviz2 --ros-args -p xx: =xxxvalue
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])