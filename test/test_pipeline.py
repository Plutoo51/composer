import unittest
from unittest.mock import patch, MagicMock
from composer.pipeline import Pipeline
import rclpy
from muto_msgs.msg import StackManifest, PlanManifest
import muto_msgs.msg

class TestPipeline(unittest.TestCase):
    
    def setUp(self) -> None:
        self.device = MagicMock()
        self.name = 'start'
        self.steps = [{'sequence': [{'service': 'muto_compose', 'plugin': 'ComposePlugin'}, {'service': 'muto_start_stack', 'plugin': 'ComposePlugin'}]}]
        self.compensation = [{'service': 'muto_kill_stack', 'plugin': 'ComposePlugin'}]
        self.pipeline = Pipeline(self.device, self.name, self.steps, self.compensation)
    
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()
        
    @patch('composer.pipeline.Pipeline.execute_step')
    @patch('composer.pipeline.Pipeline.execute_compensation')
    def test_execute(self, mock_execute_compensation, mock_execute_step):
        mock_execute_step.return_value.output.result.result_code = 0
        command = 'start'
        current = {'name': 'RVIZ Client-Server Stack', 'context': 'eteration_office', 'stackId': 'org.eclipse.muto.sandbox:rviz_client_server', 'node': [{'name': 'add_two_ints_client', 'pkg': 'demo_nodes_cpp', 'exec': 'add_two_ints_client'}, {'name': 'add_two_ints_server', 'pkg': 'demo_nodes_cpp', 'exec': 'add_two_ints_server'}]}
        next = {'stackId': 'org.eclipse.muto.sandbox:rviz_client_server'}
        self.pipeline.execute(command, current, next)
        self.assertEqual(mock_execute_step.call_count, 2)
        mock_execute_compensation.assert_not_called()
    
    @patch('composer.pipeline.Pipeline.execute_step')
    @patch('composer.pipeline.Pipeline.execute_compensation')
    def test_execute_exception(self, mock_execute_compensation, mock_execute_step):
        mock_execute_step.return_value.output.result.result_code = 404
        command = 'start'
        current = {'name': 'RVIZ Client-Server Stack', 'context': 'eteration_office', 'stackId': 'org.eclipse.muto.sandbox:rviz_client_server', 'node': [{'name': 'add_two_ints_client', 'pkg': 'demo_nodes_cpp', 'exec': 'add_two_ints_client'}, {'name': 'add_two_ints_server', 'pkg': 'demo_nodes_cpp', 'exec': 'add_two_ints_server'}]}
        next = {'stackId': 'org.eclipse.muto.sandbox:rviz_client_server'}
        self.pipeline.execute(command, current, next)
        mock_execute_step.assert_called_once()
        mock_execute_compensation.assert_called()
        
        
    @patch('rclpy.spin_until_future_complete')
    def test_execute_step(self, mock_spin):       
        self.pipeline.create_client = MagicMock()
        self.pipeline.wait_for_service = MagicMock() 
        plan = muto_msgs.msg.PlanManifest(pipeline='kill', current=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), next=muto_msgs.msg.StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), planned=muto_msgs.msg.StackManifest(type='', stack='', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description=''))
        step = {'service': 'muto_compose', 'plugin': 'ComposePlugin'}

        self.pipeline.execute_step(plan, step)
        self.pipeline.create_client.assert_called()
        self.pipeline.wait_for_service.assert_called()
        mock_spin.assert_called()
        
        
    @patch('composer.pipeline.Pipeline.execute_step')
    def test_execute_compensation(self, mock_execute_step):
        plan = muto_msgs.msg.PlanManifest(pipeline='kill', current=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), next=muto_msgs.msg.StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), planned=muto_msgs.msg.StackManifest(type='', stack='', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description=''))
        self.pipeline.execute_compensation(plan)
        mock_execute_step.assert_called()
        
        
if __name__ == '__main__':
    unittest.main()