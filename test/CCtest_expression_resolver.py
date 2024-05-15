import unittest
from unittest.mock import MagicMock, Mock, patch
import composer.model.stack
import json
import rclpy
import inspect
import os
from composer.model.node import Node
from composer.model.arg import Arg
import dummy_data as data
from composer.expression_resolver import ExpressionResolver

class TestStack(unittest.TestCase):

    def setUp(self) -> None:
        self.maxDiff = None
        mock_node = MagicMock()
        self.resolver = ExpressionResolver(stack=composer.model.stack.Stack(node=mock_node))
        
    def test_has_expression_true(self):
        returned = self.resolver.has_expression('$(arg map)')
        self.assertTrue(returned)
        
    def test_has_expression_false(self):
        returned = self.resolver.has_expression('$')
        self.assertFalse(returned)        
        
    def test_resolve_expression_find_arg(self):
        dummy_arg =  Arg()
        dummy_arg.name="map"
        dummy_arg.value="path_to_map"
        
        expected_result_arg = "path_to_map"
        self.resolver.stack.arg = [
            dummy_arg
        ]
        
        returned_arg = self.resolver.resolve_expression('$(arg map)')
        self.assertEqual(returned_arg, expected_result_arg)

        
    def test_resolve_expression_find(self):
        returned_find = self.resolver.resolve_expression('$(find demo_nodes_py)')
        expected_result_find = "/opt/ros/humble/share/demo_nodes_py"
        self.assertEqual(returned_find, expected_result_find)
        

 
        
        
if __name__ == '__main__':
    unittest.main()    