import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory # 通过功能包名字获取它所在share目录

def generate_launch_description():
    action_declare_startup_rqt = launch.actions.DeclareLaunchArgument(
        'startup_rqt', default_value = 'False' # 默认不启动
    )
    startup_rqt = launch.substitutions.LaunchConfiguration(
        'startup_rqt',default="False" # 替换
    )

    # 动作1-启动其他 launch
    multisim_launch_path = [get_package_share_directory('turtlesim'), '/launch', 'multisim.launch.py'] # 需要传数组所以加中括号
    # 传入turtlesim功能包的名字找到功能包的路径，找到之后再找'launch'文件夹，再找'launch'文件的名字
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource( # 此处调用ros2 launch turtlesim multisim.launch.py
            multisim_launch_path
        )
    )

    # 动作2-打印数据
    action_log_info = launch.actions.LogInfo(msg = str(multisim_launch_path))

    # 动作3-执行进程，其实就是执行一个命令行 ros2 topic list
    # action_topic_list = launch.actions.ExecuteProceses(
    #     cmd = ['ros2','topic','list'], # 看不到日志，只能启动，比如下面的rqt
    # )

    # 下面命令意思为
    # if startup_rqt:
    #     run:rqt
    action_topic_list = launch.actions.ExecuteProcess( # ExecuteProcess功能为运行一个普通的终端指令
        condition = launch.conditions.IfCondition(startup_rqt), # 只有startup_rqt变量被解析为true时，才启动rqt，如果命令行不加参数，rqt就不会弹出来
        cmd = ['rqt'],
    )

    # 动作4-组织动作成组,把多个动作放到一组
    action_group = launch.actions.GroupAction([
        # 动作5-定时器，定时launch启动的时间
        launch.actions.TimerAction(period=2.0, actions=[action_include_launch]), # 2s时，启动action_include_launch
        launch.actions.TimerAction(period=4.0, actions=[action_topic_list]), # 4s时，启动 ros2 topic list
    ])

    return launch.LaunchDescription([
        # action动作
        action_declare_startup_rqt,
        action_log_info,
        action_group
    ])