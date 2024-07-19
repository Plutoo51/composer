import unittest
from unittest.mock import MagicMock, patch
from composer.model.node import Node
import dummy_data as data
class TestNode(unittest.TestCase):
    
    def setUp(self) -> None:
        self.node = Node(stack=MagicMock(), manifest=data.node_test_toManifest, container=None)
        
    def tearDown(self):
        pass
    
    def test_toManifest(self):
        returned = self.node.toManifest()
        self.assertEqual(returned, data.node_test_toManifest)
    
    @patch('rclpy.spin_until_future_complete')
    @patch('rclpy.create_node')
    def test_change_state(self, mock_create_node, mock_spin):
        self.node.change_state(['configure'])
        mock_create_node.assert_called_once_with('change_state_node')
        mock_spin.assert_called()    
    
if __name__ == '__main__':
    unittest.main()