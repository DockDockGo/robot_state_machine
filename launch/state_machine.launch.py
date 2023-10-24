import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()

    #### Define the params ####

    use_namespace = LaunchConfiguration('use_namespace', default=True)
    namespace = LaunchConfiguration('namespace', default="/robot1")

    if not use_namespace:
        namespace = ""

    #### Define Nodes to Launch ####

    #! FIX LATER, not launching service, need to do manual ros2 run
    # Launch the GetRobotPose Client node with arguments
    # launch_ros.actions.Node(
    #     package='robot_action_interfaces',
    #     executable='robot_pose_server',
    #     name='robot_pose_server',
    #     output='screen',
    # ),

    #! TODO: Add namespace
    # Launch the MotionActionServer node with arguments
    StateMachine_Node = launch_ros.actions.Node(
        package='robot_state_machine',
        executable='robot_state_machine_node', # NOTE: Executable name defined in setup.py
        name='robot_state_machine_node',
        output='screen',
    )

    ld.add_action(StateMachine_Node)

    return ld