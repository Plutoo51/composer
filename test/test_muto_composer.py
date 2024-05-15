import unittest
from unittest.mock import patch, MagicMock
from composer.muto_composer import MutoComposer
import rclpy
import dummy_data as data


class TestMutoComposer(unittest.TestCase):
    
    def setUp(self):
        rclpy.init()
        self.node = MutoComposer()
    
    def tearDown(self):
        rclpy.shutdown()
        self.node.destroy_node()
    
    
    @patch('composer.muto_composer.Twin')
    @patch('composer.muto_composer.EdgeDevice')
    @patch('composer.muto_composer.Router')
    @patch('composer.muto_composer.MutoComposer._init_pipelines')
    def test_bootstrap(self, mock_pipelines ,mock_router, mock_edge_device, mock_twin):
        self.node.muto = 'muto_config'
        
        mock_pipelines.return_value = MagicMock()
        self.node.pipelines = MagicMock()
        self.node.edge_device = MagicMock()
        
        self.node._bootstrap()
        mock_router.assert_called()
        mock_edge_device.assert_called()
        mock_pipelines.assert_called_with()
        mock_twin.assert_called_with(node='muto_composer', config='muto_config')
        

    def test_bootstrap_exception(self):
        self.assertRaises(Exception, self.node._bootstrap())
    
    def test_init_pipelines(self):
        muto_composer = MutoComposer()
        expected_start = {'service': 'muto_start_stack', 'plugin': 'ComposePlugin'}
        expected_kill = {'service': 'muto_kill_stack', 'plugin': 'ComposePlugin'}
        expected_apply = {'service': 'muto_apply_stack', 'plugin': 'ComposePlugin'}
        muto_composer.pipelines = data.muto_composer_pipelines 
        
        muto_composer._init_pipelines()        
        
        self.assertEqual(len(muto_composer.pipelines), 3)
        self.assertEqual((muto_composer.pipelines['kill']).compensation, expected_kill)
        self.assertEqual((muto_composer.pipelines['start']).compensation, expected_start)
        self.assertEqual((muto_composer.pipelines['apply']).compensation, expected_apply)
                
        
    def test_on_stack_callback(self):
        msg = MagicMock()
        msg.payload = '{"value":"org.eclipse.muto.sandbox:rviz_talker_listener"}'
        msg.method = "kill"

        self.node.router = MagicMock()
        self.node.on_stack_callback(msg)
        self.node.router.route.assert_called_with("kill", "org.eclipse.muto.sandbox:rviz_talker_listener")

    @patch('composer.muto_composer.Node.get_logger')
    def test_on_stack_callback_invalid(self, mock_get_logger):
        mock_logger = MagicMock()
        mock_get_logger.return_value = mock_logger

        msg = MagicMock()
        msg.payload = 'json'
        msg.method = "test_method"

        self.node.on_stack_callback(msg)
        mock_logger.error.assert_called_with('Invalid payload coming to muto composer: Expecting value: line 1 column 1 (char 0)')
        
        
if __name__ == '__main__':
    unittest.main()