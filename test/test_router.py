import unittest
from unittest.mock import patch, MagicMock
from composer.router import Router

class TestRouter(unittest.TestCase):
    
    def setUp(self):
        self.payload = {'stackId': 'org.eclipse.muto.sandbox:rviz_talker_listener', 'action': 'kill'}
        self.pipelines = {'start': MagicMock(), 'kill': MagicMock(), 'apply': MagicMock()}
        self.pipeline = self.pipelines.get('kill')


    @patch('composer.router.EdgeDevice')
    def test_route(self, mock_edge_device):
        main_route = Router(mock_edge_device, self.pipelines)
        main_route.route(self.payload.get('action', ''), self.payload)
        self.pipeline.execute.assert_called_with('kill', mock_edge_device.current_stack.manifest, self.payload)
        mock_edge_device.bootstrap.assert_called_once_with()
        
        
    @patch('composer.router.EdgeDevice')
    def test_route_get_logger(self, mock_edge_device):
        main_route = Router(mock_edge_device, self.pipelines)
        main_route.route('action', self.payload)
        mock_edge_device.node.get_logger().info.assert_called_once_with("No pipeline found for action: action")