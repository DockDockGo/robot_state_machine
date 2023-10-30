import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()

    #### Define the params ####

    use_namespace = LaunchConfiguration('use_namespace', default=False)
    namespace = LaunchConfiguration('namespace', default="robot1")

    if not use_namespace:
        namespace = ""

    # Launch the State Machine node with arguments
    StateMachine_Node = launch_ros.actions.Node(
        package='robot_state_machine',
        executable='robot_state_machine_node', # NOTE: Executable name defined in setup.py
        name='robot_state_machine_node',
        output='screen',
        parameters=[{'namespace_param': namespace}]
    )

    ld.add_action(StateMachine_Node)

    return ld