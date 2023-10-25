import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from robot_action_interfaces.srv import GetRobotPose
from robot_action_interfaces.action import StateMachine, DockUndock, Navigate
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

        self.re_init_goal_states()

    def re_init_goal_states(self):
        self._goal_accepted = None
        self._goal_reached = None

    def get_final_result(self, success_status):
        result = StateMachine.Result()
        result.success = success_status
        return result


    ########## Dock/Undock #########################################################

    def dock_undock_send_goal(self, order):

        self.get_logger().info('Executing Undocking...')
        goal_msg = DockUndock.Goal()
        goal_msg.secs = 0.2 # order is what is actually supposed to be used

        self._dock_undock_action_client.wait_for_server()

        self._send_goal_future = self._dock_undock_action_client.send_goal_async(goal_msg, feedback_callback=self.client_feedback_callback)

        self._send_goal_future.add_done_callback(self.client_goal_response_callback)

    ########## Navigation #########################################################

    def navigation_send_goal(self, order):
        self.get_logger().info(f"Here 2")
        self.get_logger().info('Executing Navigation...')
        goal_msg = Navigate.Goal()
        goal_msg.secs = 0.2

        self._navigate_action_client.wait_for_server()

        self.get_logger().info(f"Here 3")
        self._send_goal_future = self._navigate_action_client.send_goal_async(goal_msg, feedback_callback=self.client_feedback_callback)

        self.get_logger().info(f"Here 4")
        self._send_goal_future.add_done_callback(self.client_get_result_callback)

    ########### Common #############################################################

    def client_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_accepted = True

        self.get_logger().info(f"Here 6")
        self._get_result_future = goal_handle.get_result_async()
        self.get_logger().info(f"Here 7")
        self._get_result_future.add_done_callback(self.client_get_result_callback)

    def client_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Here 8")
        result_string = str(result.success)
        self.get_logger().info(f'Result: {result_string}')
        self.get_logger().info(f"Here 9")
        if(result_string == "True"):
            self._goal_reached = True
        else:
            self._goal_reached = False
        self.get_logger().info(f"Here 10")

        if(self._goal_accepted is False):
            return self.get_final_result(False)

        self.get_logger().info(f"Goal reached status is {str(self._goal_reached)}")
        if(self._goal_reached is True):
            pass
        else:
            return self.get_final_result(False)

        # self.re_init_goal_states()
        # self.navigation_send_goal(0.2)

        return

    def client_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        feedback_str = str(feedback.pose_feedback)
        self.get_logger().info(f'Received feedback: {feedback_str}')


    def execute_callback(self, goal_handle):
        """
        Each Robot Task will be split into Undocking -> Navigation -> Docking
        """

        # Define and fill the feedback message to upstream
        feedback_msg = StateMachine.Feedback()

        ######### Undocking Phase ###########
        self.re_init_goal_states()
        self.get_logger().info(f"Here 1")
        self.dock_undock_send_goal(0.2)
        self.get_logger().info(f"Here 1-2")

        ######### Navigation Phase ###########
        # called immediately after Undocking Phase

        ############ Return Final Result of State Machine Action Server ###########
        return self.get_final_result(True)


def StateMachineServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)

    # start the MotionActionServer
    state_machine_action_server = StateMachineActionServer()
    rclpy.spin(state_machine_action_server)


if __name__ == '__main__':
    StateMachineServer()