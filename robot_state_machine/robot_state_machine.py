import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from robot_action_interfaces.srv import GetRobotPose
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from copy import deepcopy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
import math
import time

class StateMachineActionServer(Node):
    
    def __init__(self):
        #! TODO: Add namespace
        super().__init__('state_machine_action_server')
        self.get_logger().info("Starting State Machine Action Server")
        

        # Construct the action server
        self._action_server = ActionServer(
            self,
            StateMachine,
            'StateMachine',
            self.execute_callback)

        # Construct the action client (node and name should be same as defined in action server)
        self._dock_undock_action_client = ActionClient(self, DockUndock, 'DockUndock')
        self._navigate_action_client = ActionClient(self, Navigate, 'Navigate')


    def execute_callback(self, goal_handle):
        """
        Each Robot Task will be split into Undocking -> Navigation -> Docking
        """

        # Define and fill the feedback message to upstream
        feedback_msg = StateMachine.Feedback()

        result = StateMachine.Result()
        result.success = True
        return result

    ########## Dock/Undock #########################################################

    def dock_undock_send_goal(self, order):
        
        self.get_logger().info('Executing Undocking...')
        goal_msg = DockUndock.Goal()
        goal_msg.secs = 0.2

        self._dock_undock_action_client.wait_for_server()

        self._send_goal_future = self._dock_undock_action_client.send_goal_async(goal_msg, feedback_callback=self.client_feedback_callback)

        self._send_goal_future.add_done_callback(self.client_goal_response_callback)
    
    ########## Navigation #########################################################

    def dock_undock_send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._navigate_action_client.wait_for_server()

        self._send_goal_future = self._navigate_action_client.send_goal_async(goal_msg, feedback_callback=self.client_feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    ########### Common #############################################################

    def client_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.client_get_result_callback)

    def client_get_result_callback(self, future):
        result = future.result().result
        result_string = str(result.success)
        self.get_logger().info(f'Result: {result_string}')
        rclpy.shutdown()

    def client_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        feedback_str = feedback.pose_feedback
        self.get_logger().info(f'Received feedback: {feedback_str}')


def StateMachineServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)

    # start the MotionActionServer
    state_machine_action_server = StateMachineActionServer()
    rclpy.spin(state_machine_action_server)


if __name__ == '__main__':
    StateMachineServer()