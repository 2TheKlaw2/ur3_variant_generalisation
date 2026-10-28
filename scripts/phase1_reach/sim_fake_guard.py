"""Common fail-closed checks for the fake-hardware-only V1 simulator actions."""
import os
import re
import time

from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters


def await_result(node, future, timeout=7.0):
    import rclpy
    deadline = time.monotonic()+timeout
    while rclpy.ok() and time.monotonic()<deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise RuntimeError('ROS service/action timed out')
    result = future.result()
    if result is None:
        raise RuntimeError('ROS service/action returned no result')
    return result


def verify_fake_only(node):
    if os.environ.get('ROS_DOMAIN_ID') != '73':
        raise RuntimeError('FAKE-only guard requires ROS_DOMAIN_ID=73')

    rp = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    if not rp.wait_for_service(timeout_sec=7):
        raise RuntimeError('robot_state_publisher parameter service unavailable')
    request = GetParameters.Request()
    request.names = ['robot_description']
    response = await_result(node, rp.call_async(request))
    if len(response.values) != 1 or not response.values[0].string_value:
        raise RuntimeError('Missing expanded robot_description; refusing movement')
    desc = response.values[0].string_value
    # The expanded URDF must explicitly contain a mock hardware plugin.
    # Do not infer safety merely from a launch argument or a controller name.
    is_mock = bool(re.search(
        r'<hardware\s*>\s*<plugin\s*>\s*'
        r'(?:mock_components|fake_components)/GenericSystem\s*'
        r'</plugin\s*>', desc))
    if not is_mock:
        raise RuntimeError('No mock_components/GenericSystem in expanded robot URDF')
    if 'ur_robot_driver/URPositionHardwareInterface' in desc:
        raise RuntimeError('Real UR hardware driver appears in the robot description')

    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=7):
        raise RuntimeError('Controller manager unavailable')
    response = await_result(node, client.call_async(ListControllers.Request()))
    states = {c.name: c.state for c in response.controller}
    if states.get('joint_trajectory_controller') != 'active':
        raise RuntimeError('SIM joint_trajectory_controller must be active')
    if states.get('scaled_joint_trajectory_controller') == 'active':
        raise RuntimeError('Scaled real-robot controller unexpectedly active')
    print('FAKE HARDWARE VERIFIED: mock plugin, isolated domain, simulation controller')
