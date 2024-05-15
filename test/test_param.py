     
import unittest
import composer.model.param as param
from unittest.mock import MagicMock, patch
import dummy_data as data

class TestComposable(unittest.TestCase):
    
    def setUp(self):
        self.mock_stack = MagicMock()
        self.param = param.Param(stack=self.mock_stack, manifest={"name": "test-333", "from": "test", "command": "start"})
  
    def test_toManifest(self):
        self.param_resolve_value = param.Param(stack=self.mock_stack, manifest={"name": "test-333"})
        expected = {'name': 'test-333', 'value': None, 'from': 'test', 'command': 'start'}
        returned = self.param.toManifest()
        self.assertEqual(returned, expected)
            
    
    def test_parse_value(self):
        returned1 = self.param._parse_value('TRUE')
        returned2 = self.param._parse_value('FALSE')
        returned3 = self.param._parse_value('1')
        returned4 = self.param._parse_value('one')
        self.assertTrue(returned1)
        self.assertFalse(returned2)
        self.assertEqual(returned3, 1)
        self.assertEqual(returned4, 'one')
        
        

    @patch('builtins.open', unittest.mock.mock_open(read_data='node_namespace/node_name:\n  ros__parameters:\n    param1: value1'))
    @patch('yaml.safe_load')
    def test_resolve_from_file_success(self, mock_yaml_safe_load):
        mock_yaml_safe_load.return_value = {'node_namespace/node_name': {'ros__parameters': {'param1': 'value1'}}}
        self.param.node = MagicMock()
        self.param.node.namespace = 'node_namespace'
        self.param.node.name = 'node_name'
        self.param.node.ros_params = []
        self.param._resolve_from_file('dummy_path')
        self.assertEqual(self.param.node.ros_params, [{'param1': 'value1'}])
        
        
    def test_resolve_value(self):
        manifest = {'value': 'test_value'}
        returned_value = self.param._resolve_value(manifest)
        self.assertEqual(returned_value, 'test_value')
    
    
    @patch('composer.model.param.Param._resolve_from_file')
    def test_resolve_value_from(self, mock_resolve_from_file):
        manifest = {'from': '/opt/ros/humble/share/demo_nodes_py'}
        self.param._resolve_value(manifest)
        mock_resolve_from_file.assert_called()

    
    @patch('composer.model.param.Param._execute_command')
    def test_resolve_value_command(self, mock_execute_command):
        manifest = {'command': 'ls'}
        self.param._resolve_value(manifest)
        mock_execute_command.assert_called()
        
        
    @patch('composer.model.param.Param._resolve_param_expression')
    def test_resolve_value_expression(self, mock_param_expression):
        manifest = {'value': '$(find composer)'}
        self.param._resolve_value(manifest)
        mock_param_expression.assert_called()
        
    def test_resolve_param_expression(self):
        manifest = {'value': '$(find composer)'}
        expected_return = '/home/sam/imuto/install/composer/share/composer'
        returned_value = self.param._resolve_param_expression(manifest.get('value', ''))
        self.assertEqual(returned_value, expected_return)
        
        
    def test_execute_command(self):
        value = self.param._execute_command(command="echo muto")
        self.assertEqual(value, 'muto')
        

        
if __name__ == '__main__':
    unittest.main()