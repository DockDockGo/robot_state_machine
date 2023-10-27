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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time
from threading import Event

class StateMachineActionServer(Node):

    def __init__(self):
        #! TODO: Add namespace
        super().__init__('state_machine_action_server')
        self.get_logger().info("Starting State Machine Action Server")


        self.callback_group = ReentrantCallbackGroup()
        # Construct the action server
        self._action_server = ActionServer(
            self,
            StateMachine,
            'StateMachine',
            self.execute_callback,
            callback_group=self.callback_group)

        # Construct the action client (node and name should be same as defined in action server)
        self._dock_undock_action_client = ActionClient(self, DockUndock, 'DockUndock')
        self._navigate_action_client = ActionClient(self, Navigate, 'Navigate')

        self.re_init_goal_states()
        self._undock_package = None
        self._dock_package = None
        self._state_machine_success = False
        self.action_done_event = Event()

    def re_init_goal_states(self):
        self._goal_accepted = None
        self._goal_reached = None

    def get_final_result(self, success_status):
        result = StateMachine.Result()
        result.success = success_status
        return result


    ########## Dock/Undock #########################################################

    def dock_undock_send_goal(self, order):
        """
        Args:
            order : 'dock' or 'undock'
        """
        self.get_logger().info('Executing Undocking...')
        goal_msg = DockUndock.Goal()
        goal_msg.secs = 2.2 # order is what is actually supposed to be used

        try:
            self._dock_undock_action_client.wait_for_server(timeout_sec=5)
        except:
            self.get_logger().error('Timeout: Action server not available, waited for 5 seconds')
            return

        if(order == 'undock'):
            self._send_goal_future = self._dock_undock_action_client.send_goal_async(goal_msg,
                                feedback_callback=self.dock_undock_client_feedback_callback)
            self._send_goal_future.add_done_callback(self.dock_undock_client_goal_response_callback)

        if(order == 'dock'):
            goal_msg.secs = 0.2
            self._send_goal_future = self._dock_undock_action_client.send_goal_async(goal_msg,
                                feedback_callback=self.dock_undock_client_feedback_callback)
            self._send_goal_future.add_done_callback(self.dock_undock_client_goal_response_callback)

    ########## Navigation #########################################################

    def navigation_send_goal(self, order):
        self.get_logger().info('Executing Navigation...')
        goal_msg = Navigate.Goal()
        goal_msg.secs = 0.2

        try:
            self._navigate_action_client.wait_for_server(timeout_sec=5)
        except:
            self.get_logger().error('Timeout: Action server not available, waited for 5 seconds')
            return

        self._send_goal_future = self._navigate_action_client.send_goal_async(goal_msg,
                                        feedback_callback=self.nav_client_feedback_callback)
        self._send_goal_future.add_done_callback(self.nav_client_goal_response_callback)

    ########### Dock/Undock Functions ##########################################################

    def dock_undock_client_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_accepted = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.dock_undock_client_get_result_callback)

    def dock_undock_client_get_result_callback(self, future):
        result = future.result().result
        result_string = str(result.success)
        self.get_logger().info(f'Result: {result_string}')
        if(result_string == "True"):
            self._goal_reached = True
        else:
            self._goal_reached = False

        self.get_logger().info(f"Goal reached status is {str(self._goal_reached)}")
        if(self._goal_reached is True):
            self.get_logger().info(f"Goal Reached")
            self._state_machine_success = True
            self.action_done_event.set()
        else:
            self._state_machine_success = False
            self.get_logger().error(f"Goal Not Reached!")

        return

    def dock_undock_client_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        feedback_str = str(feedback.pose_feedback)
        self.get_logger().info(f'Received feedback: {feedback_str}')


    ########### Navigation Functions ##########################################################

    def nav_client_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_accepted = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_client_get_result_callback)

    def nav_client_get_result_callback(self, future):
        result = future.result().result
        result_string = str(result.success)
        self.get_logger().info(f'Result: {result_string}')
        if(result_string == "True"):
            self._goal_reached = True
        else:
            self._goal_reached = False

        self.get_logger().info(f"Goal reached status is {str(self._goal_reached)}")
        if(self._goal_reached is True):
            self.get_logger().info(f"Goal Reached")
            self._state_machine_success = True
            self.action_done_event.set()
        else:
            self._state_machine_success = False
            self.get_logger().error(f"Goal Not Reached!")

        return

    def nav_client_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        feedback_str = str(feedback.pose_feedback)
        self.get_logger().info(f'Received feedback: {feedback_str}')


    ############### MAIN LOOP START ################################################
    def execute_callback(self, goal_handle):
        """
        Each Robot Task will be split into Undocking -> Navigation -> Docking
        """

        # Define and fill the messages used
        self._undock_package = (goal_handle.request.secs, 'undock', 0.2)
        self._dock_package = (goal_handle.request.secs, 'dock', 0.2)
        feedback_msg = StateMachine.Feedback()
        self._state_machine_goal_handle = goal_handle

        ######### Start with Undocking Phase ###########
        self.re_init_goal_states()
        self.action_done_event.clear()
        self.dock_undock_send_goal('undock')
        self.action_done_event.wait()
        self.get_logger().info('Got result')

        ######### Navigation Phase ###########
        self.re_init_goal_states()
        self.action_done_event.clear()
        self.navigation_send_goal('undock')
        self.action_done_event.wait()
        self.get_logger().info('Got result')

        ######### Docking Phase ###########
        self.re_init_goal_states()
        self.action_done_event.clear()
        self.dock_undock_send_goal('dock')
        self.action_done_event.wait()
        self.get_logger().info('Got result')

        if self._state_machine_success and self._goal_accepted and self._goal_reached:
            self.get_logger().info("Returning Success")
            goal_handle.succeed()
            return self.get_final_result(True)
        else:
            return self.get_final_result(False)


def StateMachineServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)
    executor = MultiThreadedExecutor()
    # start the MotionActionServer
    state_machine_action_server = StateMachineActionServer()
    rclpy.spin(state_machine_action_server, executor)


if __name__ == '__main__':
    StateMachineServer()