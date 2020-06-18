import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from conditions.message_equality_tester_node import MessageEqualityTesterNode, MultiMessageEqualityTesterNode, EqualityType, TopicAndValuesPair
import pytest
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from typing import Dict
from conditions.minimal_latching_publisher import MinimalPublisher, latching_qos
import time

def start_minimal_publisher(function):
    """
    Decorator for a function that has arguments (future_message : Future, exe : MultiThreadedExecutor)
    Starts a minimal publisher before executing the function, and tears it down after the function
    has finished
    """

    def decorated_function():
        # SETUP 

        minimal_publisher = None
        future_message = Future()

        print("Setup fixture")
        rclpy.init()
        minimal_publisher = MinimalPublisher()
        exe = MultiThreadedExecutor()
        exe.add_node(minimal_publisher)

        #TEST

        result = function(future_message, exe)

        #TEARDOWN

        print("Teardown fixture")
        minimal_publisher.destroy_node()
        rclpy.shutdown()
        time.sleep(0.2)

        return result

    return decorated_function

def start_3_publishers(function):
    """
    Decorator...
    """
    def decorated_function():
        #SETUP

        rclpy.init()

        node = Node('minimal_publisher')

        pub1 = node.create_publisher(String, 'topic1', latching_qos)
        msg1 = String()
        msg1.data = "hello1"
        node.get_logger().info("Publishing \"{}\" on topic \"{}\"".format("hello1", "topic1"))
        pub1.publish(msg1)

        pub2 = node.create_publisher(String, 'topic2', latching_qos)
        msg2 = String()
        msg2.data = "hello2"
        node.get_logger().info("Publishing \"{}\" on topic \"{}\"".format("hello2", "topic2"))
        pub2.publish(msg2)

        pub3 = node.create_publisher(String, 'topic3', latching_qos)
        msg3 = String()
        msg3.data = "hello3"
        node.get_logger().info("Publishing \"{}\" on topic \"{}\"".format("hello3", "topic3"))
        pub3.publish(msg3)

        exe = MultiThreadedExecutor()
        exe.add_node(node)

        #TEST

        function(exe)

        #TEARDOWN

        print("Teardown fixture")
        node.destroy_node()
        rclpy.shutdown()
        time.sleep(0.2)

    return decorated_function

def start_a_message_equality_tester_node_and_get_result(future_result : Future, exe : MultiThreadedExecutor, expected_data : Dict):

    node = MessageEqualityTesterNode('topic', 'std_msgs/msg/String', expected_data, future_result)
    
    exe.add_node(node)

    exe.spin_until_future_complete(future_result)

    node.destroy_node()

    return future_result.result()

def start_multi_equality_tester(exe : MultiThreadedExecutor):

    topic_and_expected_values_pairs = [
        TopicAndValuesPair('topic1', 'std_msgs/msg/String', {'data' : 'hello1'}),
        TopicAndValuesPair('topic2', 'std_msgs/msg/String', {'data' : 'hello2'}),
        TopicAndValuesPair('topic3', 'std_msgs/msg/String', {'data' : 'hello2'}),
    ]

    future = Future()

    node = MultiMessageEqualityTesterNode(topic_and_expected_values_pairs, EqualityType.ALLOF, future)

    exe.add_node(node)

    exe.spin_until_future_complete(future)

    node.destroy_node()


#BEGIN TESTS

@pytest.mark.skip
def test_msg_is_equal():
    
    @start_minimal_publisher
    def test(future_message, exe):
        return start_a_message_equality_tester_node_and_get_result(future_message, exe, {'data' : 'Hello World: 0'})
    
    result = test()
    assert result is True

@pytest.mark.skip
def test_msg_is_not_equal():
    
    @start_minimal_publisher
    def test(future_message, exe):
        return start_a_message_equality_tester_node_and_get_result(future_message, exe, {'data' : 'Hello Worldd: 0'})
    
    result = test()
    assert result is False

def test_all_msgs_on_3_topics_equal_expected():

    @start_3_publishers
    def test(exe):
        start_multi_equality_tester(exe)
    test()