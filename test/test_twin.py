import unittest
from composer.twin import Twin
import json
from unittest.mock import patch, MagicMock , Mock
import rclpy
import dummy_data as data
import io
import sys
class TestComposerTwin(unittest.TestCase):
        
    def setUp(self):
        self.muto = {'name': 'example-01', 'namespace': 'org.eclipse.muto.sandbox', 'stack_topic': 'stack', 'twin_topic': 'twin', 'anonymous': False, 'twin_url': 'http://ditto:ditto@sandbox.composiv.ai'}
        self.twin = Twin(node='muto_compose_plugin', config=self.muto)
        self.stack = Mock()


    @patch('composer.twin.Twin.stack')
    def test_get_current_stack(self, mock_stack):
        self.twin.get_current_stack()
        mock_stack.assert_called_once()

    
    
    @patch('composer.twin.requests.get')
    def test_stack(self, get):
        get.return_value.status_code = 204
        get.return_value.text = json.dumps(data.properties_test)
        thing_id = "org.eclipse.muto.sandbox:test-333"
        returned_value = self.twin.stack(thing_id)
        self.assertEqual(returned_value, data.expected_properties)
        
    @patch('composer.twin.requests.get')
    def test_stack_status_code(self, get):
        get.return_value.status_code = 404
        get.return_value.text = json.dumps(data.properties_test)
        thing_id = "org.eclipse.muto.sandbox:test-333"
        returned_value = self.twin.stack(thing_id)
        self.assertEqual(returned_value, {})
        
        
    @patch('composer.twin.requests.get')
    def test_stack_exception(self, get):
        get.return_value.status_code = 204
        thing_id = "org.eclipse.muto.sandbox:test-333"
        suppress_text = io.StringIO()
        sys.stdout = suppress_text 
        self.assertRaises(Exception, self.twin.stack(thing_id))
        sys.stdout = sys.__stdout__
        
        
    @patch('builtins.print')
    def test_set_current_stack_none(self, mock_print):
        self.twin.set_current_stack(None)
        mock_print.assert_called_once_with("No stack to set")
        
    
    @patch("requests.put")
    def test_set_current_stack_with_stackId(self, mock_put):
        self.stack.manifest = {"stackId": "org.eclipse.muto.sandbox:test-333"}
        self.twin.set_current_stack(self.stack)
        mock_put.assert_called_once_with('http://ditto:ditto@sandbox.composiv.ai/api/2/things/org.eclipse.muto.sandbox:example-01/features/stack/properties/current', headers={'Content-type': 'application/json'}, json={'stackId': 'org.eclipse.muto.sandbox:test-333', 'state': 'unknown'})
        
        
    @patch("requests.post")
    def test_set_current_stack_multiple_stacks(self, mock_post):
        self.stack.manifest = {"stack": [{"thingId": "org.eclipse.muto.sandbox:test-333"}, {"thingId": "org.eclipse.muto.sandbox:samet-333"}]}
        self.twin.set_current_stack(self.stack)
        self.assertEqual(mock_post.call_count, 2)
        
        
    @patch("builtins.print")
    @patch("requests.put")
    def test_set_current_stack_exception(self, mock_put, mock_print):
        mock_put.side_effect = Exception("Test Exception")
        self.stack.manifest = {"stackId": "org.eclipse.muto.sandbox:test-333"}
        self.twin.set_current_stack(self.stack)
        mock_print.assert_called_with('Setting stack ended with exception: Test Exception')
        
if __name__ == '__main__':
    unittest.main()