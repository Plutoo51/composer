import unittest
from unittest.mock import MagicMock, patch
from composer.model.stack import Stack
import dummy_data as data
from composer.model.edge_device import EdgeDevice
import rclpy

class TestEdgeDevice(unittest.TestCase):
    
    def setUp(self):
        self.mock_node = MagicMock()
        self.mock_twin = MagicMock()
        self.edge_device = EdgeDevice(self.mock_node, self.mock_twin)
        self.edge_device.current_stack = None
    
    
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()
    
    
    @patch('composer.model.edge_device.Stack')
    def test_update_current_stack(self, mock_stack):
        self.edge_device._update_current_stack(data.test_manifest, 'active')  
        mock_stack.assert_called()
        self.mock_twin.set_current_stack.assert_called()
        
    @patch('composer.model.edge_device.Stack')
    def test_update_current_stack_logger(self, mock_stack):
        self.edge_device.node.get_logger = MagicMock()
        self.edge_device._update_current_stack(definition=None) 
        mock_stack.assert_called()
        self.edge_device.node.get_logger().warn.assert_called_once_with('Empty definition or stack in edge device stack updating method.')

    
    def test_handle_exception(self):
        self.edge_device.node.get_logger = MagicMock()
        self.edge_device._handle_exception(action='activation', exception=None)
        self.edge_device.node.get_logger().error.assert_called_once_with('An exception occurred during activation: None')
        
        
    @patch('composer.model.edge_device.EdgeDevice._update_current_stack')    
    @patch('composer.model.edge_device.EdgeDevice.stack')
    def test_bootstrap(self, mock_stack, mock_update_current_stack):
        mock_stack.return_value = MagicMock()
        mock_update_current_stack.return_value = MagicMock()
        self.edge_device.node.get_logger = MagicMock()
        self.edge_device.bootstrap()
        
        mock_stack.assert_called_once()
        mock_update_current_stack.assert_called_once()
        self.assertEqual(self.edge_device.node.get_logger().info.call_count, 2)
      
      
      
    @patch('composer.model.stack.Stack.launch')      
    def test_activate(self, mock_launch):
        self.edge_device.activate()
        self.assertTrue(mock_launch.called)
        
    @patch('composer.model.edge_device.EdgeDevice._update_current_stack')
    def test_activate_current_stack(self, mock_update_current_stack):
        mock_update_current_stack.return_value = MagicMock()
        self.edge_device.activate()
        mock_update_current_stack.assert_called_once_with(None, 'active')
        
    @patch('composer.model.stack.Stack.apply')
    def test_apply(self, mock_apply):
        self.edge_device.apply(MagicMock())
        self.assertTrue(mock_apply.called)
        
    
    @patch('composer.model.edge_device.EdgeDevice._update_current_stack') 
    def test_apply_update(self, mock_update_current_stack):
        mock_obj = MagicMock()
        self.edge_device.apply(mock_obj)
        mock_update_current_stack.assert_called_once_with(mock_obj, 'active')
        
                       
    @patch('composer.model.edge_device.EdgeDevice._update_current_stack')
    def test_kill_current_stack(self, mock_update_current_stack):
        mock_obj = MagicMock()
        mock_update_current_stack.return_value = MagicMock()
        self.edge_device.kill(payload=mock_obj)
        mock_update_current_stack.assert_called_once_with(mock_obj, 'killed')
        
        
        
if __name__ == "__main__":
    unittest.main()
    