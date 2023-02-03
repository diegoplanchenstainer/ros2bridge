"""ROS Action Client.

This implementation ROS Action Client functionalities.

Class:
    WSActionClient:
        Create ros Action client.
"""


import json
from dataclasses import dataclass
from functools import partial
from typing import Any, Dict

from rclpy.action import ActionClient
from rclpy.action.client import ActionClient as ActionCli
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from ros2bridge.protocols.ws_server import WSServerProtocol
from ros2bridge.utils.data_parser import RosDataParser, RosDataType


@dataclass
class WSActionClient:
    """ROS Action Client.

    Create ros Action client.

    Attributes:
        client: Dict[str, Any]
        data_parser: RosDataParser

    Methods:
        handle_operation(self, data: Dict[str, Any]) -> None:
            Create and call ROS Action client on client request.
    """

    client: Dict[str, Any]
    data_parser = RosDataParser(data_type=RosDataType.ACTION)
    
    def follow_waypoints(self, data, module):
        goal_poses = []

        for elem in data['poses']:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = elem['header']['frame_id']
            t = Time(seconds = elem['header']['stamp']['sec'], nanoseconds = elem['header']['stamp']['nanosec'])
            goal_pose.header.stamp = t.to_msg()
            print(t)
            goal_pose.pose.position.x = elem['pose']['position']['x']
            goal_pose.pose.position.y = elem['pose']['position']['y']
            goal_pose.pose.position.z = elem['pose']['position']['z']
            goal_pose.pose.orientation.x = elem['pose']['orientation']['x']
            goal_pose.pose.orientation.y = elem['pose']['orientation']['y']
            goal_pose.pose.orientation.z = elem['pose']['orientation']['z']
            goal_pose.pose.orientation.w = elem['pose']['orientation']['w']
            goal_poses.append(goal_pose)

        setattr(module, 'poses', goal_poses)

        return module

    def handle_operation(self, data: Dict[str, Any]) -> None:
        """Create and call ROS Action client on client request.

        Args:
            data (Dict): Request from ws client.
        """
        _client_name: str = self.client['client_id']
        self._client: WSServerProtocol = self.client['client']
        self._node: Node = self.client['client_node']

        action_name = data['action_name']
        action_type = data['action_type']
        action = data['action']

        if action == 'create':
            print(
                f'Client: {_client_name} created a action client. | ' +
                f'Action Name: {action_name} | Type: {action_type}'
            )

            _action_type = self.data_parser.import_type(action_type)
            action_cli = ActionClient(
                node=self._node,
                action_type=_action_type,
                action_name=action_name
            )

            self.client['action_client'][action_name] = {
                'client': action_cli,
                'srv_type': action_type
            }

        elif action == 'call':
            message = data['message']

            if action_type.find('FollowWaypoints') != -1:
                goal = self.follow_waypoints(
                    data = message,
                    module=self.data_parser.get_module_instance(
                        module=action_type
                    ))
            else:
                goal = self.data_parser.pack_data_to_ros(
                    data=message,
                    module=self.data_parser.get_module_instance(
                        module=action_type
                    )
                )

            _action_client: ActionCli = self.client[
                'action_client'
            ][action_name]['client']

            _action_client.wait_for_server()

            feedback_callback = partial(self.feedback_callback, data)
            response_callback = partial(self.response_callback, data)

            _goal = _action_client.send_goal_async(
                goal=goal,
                feedback_callback=feedback_callback
            )

            _goal.add_done_callback(
                callback=response_callback
            )

    def feedback_callback(self, header: Dict[str, Any], feedback: Any) -> None:
        """Feedback.

        Callback when receiving feedback from action server.

        Args:
            header (Dict[str, Any]): ws client request.
            feedback (Any): feedback from acton server.
        """
        _feedback = feedback.feedback

        header['action_response'] = 'feedback'
        self.unpack_data(_feedback, header)

    def response_callback(
            self, header: Dict[str, Any], response: Future) -> None:
        """Response.

        Status of the goal when requested.

        Args:
            header (Dict[str, Any]): ws client request.
            response (Future): Response from action server.
        """
        _goal = response.result()
        result_callback = partial(self.result_callback, header)

        msg = 'Goal rejected.'
        header['action_response'] = 'response'

        if _goal.accepted:
            msg = 'Goal Accepted.'
            header['message'] = msg

            self._client.send_message(json.dumps(header))

            _result = _goal.get_result_async()
            _result.add_done_callback(result_callback)

    def result_callback(self, header: Dict[str, Any], result: Future) -> None:
        """Result. # noqa: D401.

        Action client result callback.

        Args:
            header (Dict[str, Any]): ws client request.
            result (Future): Result of the request.
        """
        _result = result.result().result

        header['action_response'] = 'result'
        self.unpack_data(_result, header)

    def unpack_data(self, module: Any, header: Dict[str, Any]) -> None:
        """Unpack ros data.

        used to avoid code duplication.

        Args:
            module (Any): Rose data.
            header (Dict[str, Any]): User request.
        """
        msg = self.data_parser.pack_data_to_json(module=module, output={})
        header['message'] = msg
        self._client.send_message(json.dumps(header))
