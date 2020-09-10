import rclpy
from rclpy.node import Node
from conditions.condition_publisher import ConditionPublisher, TopicAndValuesPair
from condition_msgs.msg import Condition
from acf_msg_types.msg import DoorState
from acf_msg_types.msg._door_state import Metaclass_DoorState
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from typing import Optional, List
from rclpy.executors import MultiThreadedExecutor, Executor
from rclpy.task import Future
from rclpy.timer import Timer
from rosidl_runtime_py import set_message_fields

latching_qos = QoSProfile(depth=1,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

class MockSystemNode(Node):
    """Publishes Data that can be used to drive conditions to test the ConditionPublisher"""

    def __init__(self):

        super().__init__('mock_system_node')

        self.__intrabay_door_state_publisher = self.create_publisher(
            DoorState, 'intrabay_door_state', latching_qos)

        self.__process_cell_floorvalve_state_publisher = self.create_publisher(
            DoorState, 'process_cell_floorvalve_state', latching_qos)

        self.__maintanance_cell_floorvalve_state_publisher = self.create_publisher(
            DoorState, 'maintanance_cell_floorvalve_state', latching_qos)

        self.__cth_hot_side_door_state_publisher = self.create_publisher(
            DoorState, 'cth_hot_side_door_state', latching_qos)

        self.__lhs_crane_position_publisher = self.create_publisher(
            Point, 'lhs_crane_position', latching_qos)

        self.__grapple_crane_position_publisher = self.create_publisher(
            Point, 'grapple_crane_position', latching_qos)

        self.publish_new(
            intrabay_door_state = DoorState(door_state=DoorState.CLOSED),
            process_cell_floorvalve_state = DoorState(door_state=DoorState.CLOSED),
            maintanance_cell_floorvalve_state = DoorState(door_state=DoorState.CLOSED),
            cth_hot_side_door_state = DoorState(door_state=DoorState.CLOSED),
            lhs_crane_position = Point(x=100.0, y=100.0, z=0.0),
            grapple_crane_position = Point(x=200.0, y=200.0, z=0.0)
        )

    def publish_new(self,
        *,
        intrabay_door_state: Optional[DoorState] = None,
        process_cell_floorvalve_state: Optional[DoorState] = None,
        maintanance_cell_floorvalve_state: Optional[DoorState] = None,
        cth_hot_side_door_state: Optional[DoorState] = None,
        lhs_crane_position: Optional[Point] = None,
        grapple_crane_position: Optional[Point] = None
    ):
        if intrabay_door_state is not None:
            self.__intrabay_door_state_publisher.publish(intrabay_door_state)
        if process_cell_floorvalve_state is not None:
            self.__process_cell_floorvalve_state_publisher.publish(process_cell_floorvalve_state)
        if maintanance_cell_floorvalve_state is not None:
            self.__maintanance_cell_floorvalve_state_publisher.publish(maintanance_cell_floorvalve_state)
        if cth_hot_side_door_state is not None:
            self.__cth_hot_side_door_state_publisher.publish(cth_hot_side_door_state)
        if lhs_crane_position is not None:
            self.__lhs_crane_position_publisher.publish(lhs_crane_position)
        if grapple_crane_position is not None:
            self.__grapple_crane_position_publisher.publish(grapple_crane_position)
        

class TestConditionPublisherNode(Node):
    """Subscribes to data, caches values, performs computation, and publishes a condition message""" 
    def __init__(self):
        super().__init__('test_cond_pub_node')

        ConditionPublisher(self,
            condition_name = 'acf_process_cell_is_sheilded',
            all_of_equality_check = [
                TopicAndValuesPair(
                    topic_name='intrabay_door_state',
                    topic_type='acf_msg_types/msg/DoorState',
                    values = { 'door_state' : DoorState.CLOSED }
                ),
                TopicAndValuesPair(
                    topic_name='process_cell_floor_valve_state',
                    topic_type='acf_msg_types/msg/DoorState',
                    values = { 'door_state' : DoorState.CLOSED }
                ),
                TopicAndValuesPair(
                    topic_name='maintanance_cell_floor_valve_state',
                    topic_type='acf_msg_types/msg/DoorState',
                    values = { 'door_state' : DoorState.CLOSED }
                ),
                TopicAndValuesPair(
                    topic_name='cth_hot_side_door_state',
                    topic_type='acf_msg_types/msg/DoorState',
                    values = { 'door_state' : DoorState.CLOSED }
                )
            ]
        )


class TestConditionSubscriberNode(Node):
    """Subscribes to condition messages for the purpose of testing"""

    __msg_received: Optional[Future] = None

    def __init__(self):
        super().__init__('test_cond_sub_node')

        self.create_subscription(Condition, 'acf_process_cell_is_sheilded',
        callback = lambda msg : (

            self.get_logger().info("recieved {}".format(msg)),

            #If a future has been created, set the result as the message, otherwise discard the message
            self.__msg_received.set_result(msg) if self.__msg_received is not None else (
                self.get_logger().info("not expecting message")
            )
        ),
        qos_profile = latching_qos)

    def expect_message_within_time(self, timeout, expected_value) -> bool:
        """blocking fucntion"""

        expected_msg = Condition()
        expected_msg.condition = int(expected_value)
        
        self.__msg_received = Future(executor=self.executor)

        timeout_timer = self.create_timer(
        timer_period_sec = timeout,
        callback = lambda : (
            self.get_logger().info("Timeout!"),
            self.__msg_received.set_result(False),
            timeout_timer.cancel()
        ))

        #Wait for either timeout or new message to arrive
        while not self.__msg_received.done():
            pass

        #Return true or false based on result of future
        if self.__msg_received.result == False:
            return False
        elif type(self.__msg_received) is not Condition:
            self.get_logger().info("Wrong message type stored in future")
            return False
        else:
            pass


def test_1(system_node: MockSystemNode, cond_sub_node: TestConditionSubscriberNode, future_results: Future):
    """test that the start state is that the acf is confined"""

    """open the intrabay door"""
    system_node.publish_new(intrabay_door_state=DoorState.OPENING)

    """test that the condition is now inactive"""

def test_2(system_node: MockSystemNode, cond_sub_node: TestConditionSubscriberNode, future_results: Future):
    pass

def test_3(system_node: MockSystemNode, cond_sub_node: TestConditionSubscriberNode, future_results: Future):
    pass

def main(args=None):

    rclpy.init()
    # system_node = MockSystemNode()
    cond_pub_node = TestConditionPublisherNode()
    #cond_sub_node = TestConditionSubscriberNode()

    rclpy.spin(cond_pub_node)

    # exe = MultiThreadedExecutor()
    # exe.add_node(system_node)
    # exe.add_node(cond_pub_node)
    # exe.add_node(cond_sub_node)

    # future_results = Future(executor=exe)

    # exe.create_task(test_1, system_node, cond_pub_node, future_results)
    # exe.create_task(test_2, system_node, cond_pub_node, future_results)
    # exe.create_task(test_3, system_node, cond_pub_node, future_results)

    # try:
    #     exe.spin_until_future_complete(future_results)
    # except:
    #     pass

if __name__ == "__main__":
    main()