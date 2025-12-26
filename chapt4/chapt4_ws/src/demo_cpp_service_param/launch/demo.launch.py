import launch
import launch_ros

def generate_launch_description():
    """产生launch描述"""
    # 1. 声明launch参数 
    action_declare_arg_max_spped = launch.actions.DeclareLaunchArgument('launch_max_speed', default_value='2.0') #同样也要加到下面的launch_description 
    # 第一个参数为名字，第二个参数为默认值
    # 2. 把launch的参数手动传递给某个节点
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service_param',  
        executable="turtle_control", # 可执行文件
        output='screen', # 输出到屏幕
        parameters=[{'max_speed': launch.substitutions.LaunchConfiguration(
    'launch_max_speed', default='2.0')}], # 此处要获取'launch_max_speed'的值，此处2.0必须是字符串
        # 意思是从launch参数中取到launch_max_speed的值，转换成可以给到节点的参数，即把launch的参数转换成能被节点用的参数

    )
    action_node_patrol_client = launch_ros.actions.Node(
        package='demo_cpp_service_param',
        executable="patrol_client",
        output='log', # 输出到日志文件
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='both', # 两者都输出
    )
   # 合成启动描述并返回
    launch_description = launch.LaunchDescription([
        # actions动作
        action_declare_arg_max_spped,
        action_node_turtle_control,
        action_node_patrol_client,
        action_node_turtlesim_node
    ])
    return launch_description