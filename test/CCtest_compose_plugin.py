import unittest
import composer.compose_plugin
from unittest.mock import Mock, patch, MagicMock
import rclpy
import json
from muto_msgs.srv import ComposePlugin
from muto_msgs.msg import PluginResponse, StackManifest, PlanManifest
import dummy_data as data

class TestComposePlugin(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
    

    def setUp(self):
        self.magic_mock = MagicMock()
        self.node = composer.compose_plugin.MutoDefaultComposePlugin()
        self.plan = PlanManifest(
            current=StackManifest(type="json", stack = '{"Current_name": "Current_test"}', 
                result = PluginResponse(result_code = 0, error_message = "", error_description = "")),
            next=StackManifest(type="json", stack = '{"Next_name": "Next_test"}', 
                result = PluginResponse(result_code = 0, error_message = "", error_description = "")),
            pipeline="test",
            planned=StackManifest(
                type="json",
                stack=""
            ),
            result=PluginResponse(
                result_code=0, error_message="", error_description=""
            )
        )
    
    def tearDown(self):
        self.node.destroy_node()
    
    
    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()
    
    def test_handle_compose(self):
        req = ComposePlugin.Request()
        res = ComposePlugin.Response()
        req.input=PlanManifest(pipeline='kill', current=StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=PluginResponse(result_code=0, error_message='', error_description='')), next=StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=PluginResponse(result_code=0, error_message='', error_description='')), planned=StackManifest(type='', stack='', result=PluginResponse(result_code=0, error_message='', error_description='')), result=PluginResponse(result_code=0, error_message='', error_description=''))
        code_return = self.node.handle_compose(req, res)
        self.assertIsNotNone(code_return)

    def test_get_next_stack(self):
        next_stack_manifest = json.loads(self.plan.next.stack)
        returned_value = self.node._get_next_stack(next_stack_manifest)
        self.assertEqual(returned_value.manifest, {'Next_name': 'Next_test'})
        
        
    def test_create_plan_manifest(self):
        merged_manifest = {'name': 'RVIZ Talker-Listener Stack', 'context': 'eteration_office', 'stackId': 'org.eclipse.muto.sandbox:rviz_talker_listener', 'param': [], 'arg': [], 'stack': [], 'composable': [], 'node': [{'env': [], 'param': [], 'remap': [], 'pkg': 'demo_nodes_cpp', 'lifecycle': '', 'exec': 'add_two_ints_server', 'plugin': '', 'name': 'add_two_ints_server', 'ros_args': '', 'args': '', 'namespace': '', 'launch-prefix': None, 'output': 'both', 'if': '', 'unless': '', 'action': 'stop'}, {'env': [], 'param': [], 'remap': [], 'pkg': 'demo_nodes_cpp', 'lifecycle': '', 'exec': 'talker', 'plugin': '', 'name': 'talker', 'ros_args': '', 'args': '', 'namespace': '', 'launch-prefix': None, 'output': 'both', 'if': '', 'unless': '', 'action': 'start'}, {'env': [], 'param': [], 'remap': [], 'pkg': 'demo_nodes_cpp', 'lifecycle': '', 'exec': 'add_two_ints_client', 'plugin': '', 'name': 'add_two_ints_client', 'ros_args': '', 'args': '', 'namespace': '', 'launch-prefix': None, 'output': 'both', 'if': '', 'unless': '', 'action': 'stop'}, {'env': [], 'param': [], 'remap': [], 'pkg': 'rviz2', 'lifecycle': '', 'exec': 'rviz2', 'plugin': '', 'name': 'rviz2', 'ros_args': '', 'args': '', 'namespace': '', 'launch-prefix': None, 'output': 'both', 'if': '', 'unless': '', 'action': 'none'}, {'env': [], 'param': [], 'remap': [], 'pkg': 'demo_nodes_cpp', 'lifecycle': '', 'exec': 'listener', 'plugin': '', 'name': 'listener', 'ros_args': '', 'args': '', 'namespace': '', 'launch-prefix': None, 'output': 'both', 'if': '', 'unless': '', 'action': 'start'}]}
        expected_output = PlanManifest(
            current=self.plan.current,
            next=self.plan.next,
            pipeline=self.plan.pipeline,
            planned=StackManifest(type="json", stack=json.dumps(merged_manifest)),
            result=PluginResponse(result_code=0, error_message="", error_description="")
        )
        returned_value = self.node._create_plan_manifest(self.plan, merged_manifest)
        self.assertEqual(expected_output, returned_value)


    @patch('composer.compose_plugin.rclpy')
    def test_main(self, mock_rclpy):
        mock_node = MagicMock()
        composer.compose_plugin.MutoDefaultComposePlugin = MagicMock(return_value=mock_node)
        composer.compose_plugin.main()
        mock_rclpy.init.assert_called_once()
        mock_rclpy.spin.assert_called_once_with(mock_node)
        mock_node.destroy_node.assert_called_once()
        mock_rclpy.shutdown.assert_called_once()
        
        
if __name__ == '__main__':
    unittest.main()


