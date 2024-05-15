import unittest
from unittest.mock import patch, MagicMock
from composer.launch_plugin import MutoDefaultLaunchPlugin
import rclpy
from muto_msgs.srv import ComposePlugin
import muto_msgs.msg
import muto_msgs.srv
from muto_msgs.msg import PluginResponse, PlanManifest, StackManifest
import dummy_data as data

class TestLaunchPlugin(unittest.TestCase):

    def setUp(self):
        self.launch_plugin = MutoDefaultLaunchPlugin()
        self.req = ComposePlugin.Request()
        self.res = ComposePlugin.Response()

    def tearDown(self):
        self.launch_plugin.destroy_node()

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    @patch('composer.launch_plugin.twin.Twin')
    @patch('composer.launch_plugin.edge.EdgeDevice')
    def test_bootstrap(self, mock_edge_device, mock_twin):
        self.launch_plugin._bootstrap()
        mock_edge_device.assert_called_once()
        mock_twin.assert_called_once_with(node='MutoLaunchPlugin',config = data.test_bootstrap_twin_assert_with_config)


    @patch('composer.launch_plugin.edge.EdgeDevice')
    @patch('composer.launch_plugin.MutoDefaultLaunchPlugin._handle_stack_operation')
    def test_handle_apply(self, mock_handle_stack_operation, mock_edge_device):         #####
        mock_edge_device.return_value = MagicMock()
        self.launch_plugin.device = MagicMock()
        self.req.input = data.test_handle_apply_req_input
        self.device = mock_edge_device
        self.launch_plugin.handle_apply(self.req, self.res)
        mock_handle_stack_operation.assert_called_once_with(self.req, self.res, self.launch_plugin.device.apply)


    @patch('composer.launch_plugin.edge.EdgeDevice')
    @patch('composer.launch_plugin.MutoDefaultLaunchPlugin._handle_stack_operation')
    def test_handle_kill(self, mock_handle_stack_operation, mock_edge_device):      #####
        mock_edge_device.return_value = MagicMock()
        self.req.input = data.test_handle_kill_req_input
        self.device = mock_edge_device
        self.launch_plugin.handle_kill(self.req, self.res)
        mock_handle_stack_operation.assert_called_once_with(self.req, self.res, self.launch_plugin.device.kill)


    @patch('composer.launch_plugin.edge.EdgeDevice')
    @patch('composer.launch_plugin.MutoDefaultLaunchPlugin._handle_stack_operation')
    def test_handle_start(self, mock_handle_stack_operation, mock_edge_device):     #####
        mock_edge_device.return_value = MagicMock()
        self.req.input = data.test_handle_start_req_input
        self.device = mock_edge_device
        self.launch_plugin.handle_start(self.req, self.res)
        mock_handle_stack_operation.assert_called_once_with(self.req, self.res, self.launch_plugin.device.activate)

    @patch('composer.launch_plugin.MutoDefaultLaunchPlugin._success_response')
    @patch('composer.launch_plugin.edge.EdgeDevice')
    def test_handle_stack_operation(self, mock_edge_device, mock_sucess_response):
        self.req.input = data.test_handle_stack_operation_req_input
        self.launch_plugin._handle_stack_operation(self.req, self.res, mock_edge_device.apply)
        mock_edge_device.apply.assert_called_once_with(data.test_handle_stack_operation_with)
        mock_sucess_response.assert_called_with(self.req.input.planned)
   
   
    @patch('composer.launch_plugin.MutoDefaultLaunchPlugin._error_response')
    @patch('composer.launch_plugin.MutoDefaultLaunchPlugin._success_response')
    @patch('composer.launch_plugin.edge.EdgeDevice')
    def test_handle_stack_operation_error(self, mock_edge_device, mock_sucess_response, mock_error_response):
        mock_error_response.return_value = PlanManifest()
        self.req.input = data.test_handle_stack_operation_error_req_input
        self.launch_plugin._handle_stack_operation(self.req, self.res, mock_edge_device.apply)
        mock_edge_device.apply.assert_not_called()
        mock_sucess_response.assert_not_called()
        mock_error_response.assert_called_once_with("Exception during operation")


    def test_success_response(self):
        input = data.test_success_response_input
        expected_value = data.test_success_response_expected_value
        returned_value = self.launch_plugin._success_response(input.planned)
        self.assertEqual(returned_value, expected_value)
        
        
    def test_error_response(self):
        error_message = "Exception during operation"
        expected_return = data.test_error_response_expected_return
        returned_value = self.launch_plugin._error_response(error_message)
        self.assertEqual(returned_value, expected_return)


if __name__ == "__main__":
    unittest.main()