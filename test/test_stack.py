import unittest
from unittest.mock import MagicMock, Mock, patch
import composer.model.arg
import composer.model.composable
import composer.model.param
import composer.model.stack
import json
import rclpy
import inspect
import os
from composer.model.node import Node
from composer.model.arg import Arg
from launch import LaunchDescription
import warnings
from composer.introspection.introspector import Introspector
import dummy_data as data
from composer.expression_resolver import ExpressionResolver

class TestStack(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
    

    def setUp(self) -> None:
        warnings.simplefilter("ignore")
        self.maxDiff = None
        self.intor = Introspector()
        self.mock_stack = MagicMock()
        self.node = composer.model.stack.Stack(node=self.mock_stack)

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()
    
    def test_merge_attributes(self):
        test_stack = composer.model.stack.Stack(node=self.mock_stack, manifest={'name': 'RVIZ Talker-Listener Stack', 'context': 'eteration_office', 'stackId': 'org.eclipse.muto.sandbox:rviz_talker_listener', 'node': [{'name': 'talker', 'pkg': 'demo_nodes_cpp', 'exec': 'talker'}, {'name': 'listener', 'pkg': 'demo_nodes_cpp', 'exec': 'listener'}]})
        
        other_stack = composer.model.stack.Stack(node=self.mock_stack, manifest={'name': 'test_stack_name', 'context': 'test_context', 'stackId': 'org.eclipse.muto.sandbox:test_stack_id', 'node': [{'name': 'talker', 'pkg': 'demo_nodes_cpp', 'exec': 'talker'}, {'name': 'listener', 'pkg': 'demo_nodes_cpp', 'exec': 'listener'}]})
        
        self.node._merge_attributes(test_stack, other_stack)
        
        returned_value = other_stack  #_merge_attributes changes the first stack attributes but returns nothing
        
        self.assertEqual(test_stack.context, returned_value.context)
        self.assertEqual(test_stack.name, returned_value.name)
        self.assertEqual(test_stack.stackId, returned_value.stackId)

        
    def test_flatten_nodes(self):     #OK
        main_stack = composer.model.stack.Stack(node=self.mock_stack)
        child_stack1 = composer.model.stack.Stack(node=self.mock_stack)
        child_stack2 = composer.model.stack.Stack(node=self.mock_stack)
        main_stack.node = data.main_stack_node
        child_stack1.node = data.child_stack_node1
        child_stack2.node = data.child_stack_node2
        main_stack.stack = [child_stack1, child_stack2]
        result = main_stack.flatten_nodes([])
        expected = data.expected_test_flatten_nodes
        self.assertEqual(result, expected)
        

    def test_toShallowManifest(self):     #OK
        expected_response = data.expected_test_toShallowManifest
        response = self.node.toShallowManifest()
        self.assertEqual(response, expected_response)

    
    def test_merge(self):      #OK
        stack_obj = composer.model.stack.Stack(node=self.mock_stack, manifest=data.stack_obj_manifest_test_merge)
        child_stack_obj = composer.model.stack.Stack(node=self.mock_stack, manifest=data.child_stack_obj_manifest_test_merge)
        expected_manifest_1 = data.expected_manifest_test_merge_1
        expected_manifest_2 = data.expected_manifest_test_merge_2
        expected_name = 'RVIZ Talker-Listener Stack'
        expected_context = 'eteration_office'
        expected_stackId = 'org.eclipse.muto.sandbox:rviz_talker_listener'
        
        returned_value = stack_obj.merge(child_stack_obj)
        
        self.assertIn(returned_value.manifest, [expected_manifest_1, expected_manifest_2])
        self.assertEqual(returned_value.name, expected_name)
        self.assertEqual(returned_value.context, expected_context)
        self.assertEqual(returned_value.stackId, expected_stackId)
        self.assertEqual(type(stack_obj),type(returned_value))


    def test_compare_nodes(self):   #OK
        test_self_manifest = {'name': 'RVIZ Talker-Listener Stack', 'manifest': '', 'stackId': 'org.eclipse.muto.sandbox:rviz_talker_listener', 'node': [{'name': 'talker', 'pkg': 'demo_nodes_cpp', 'exec': 'talker'}, {'name': 'listener', 'pkg': 'demo_nodes_cpp', 'exec': 'listener'}]}
        test_other_manifest = {'name': 'RVIZ Client-Server Stack', 'manifest': '', 'stackId': 'org.eclipse.muto.sandbox:rviz_client_server', 'node': [{'name': 'add_two_ints_client', 'pkg': 'demo_nodes_cpp', 'exec': 'add_two_ints_client'}, {'name': 'add_two_ints_server', 'pkg': 'demo_nodes_cpp', 'exec': 'add_two_ints_server'}]}
    
        test_node_mock = MagicMock(manifest=test_self_manifest)
        other_node_mock = MagicMock(manifest = test_other_manifest)    
        
        test_stack = composer.model.stack.Stack(node=test_node_mock, manifest=test_self_manifest)
        other_stack = composer.model.stack.Stack(node=other_node_mock, manifest=test_other_manifest)

        common, difference, added = test_stack.compare_nodes(other_stack)
        
        self.assertIsNotNone(common)
        self.assertIsNotNone(difference)
        self.assertIsNotNone(added)
        self.assertEqual(len(common), 0)            
        self.assertEqual(len(difference), 2)       
        self.assertEqual(len(added), 2)             
        

    def test_merge_composables(self):
        main_stack = composer.model.stack.Stack(node=self.mock_stack, manifest=data.main_composables_manifest)
        child_stack_for_merged= composer.model.stack.Stack(node=self.mock_stack)
        child_stack_for_other = composer.model.stack.Stack(node=self.mock_stack, manifest=data.child_composables_manifest)
        returned_value = main_stack._merge_composables(child_stack_for_merged, child_stack_for_other)
        for i in range(3):
            for k in range(2):
                if (returned_value.composable[0].nodes[k] == returned_value.composable[1].nodes[i]):
                    self.assertEqual(returned_value.composable[0].nodes[k].action, 'start') ## 'none'
                else:
                    self.assertEqual(returned_value.composable[0].nodes[k].action, 'start')
                    self.assertEqual(returned_value.composable[1].nodes[i].action, 'stop')
                    
        self.assertEqual(type(returned_value), type(main_stack))
        self.assertEqual(len(returned_value.composable), 2)
        self.assertEqual(len(returned_value.composable[0].nodes), 2)
        self.assertEqual(len(returned_value.composable[1].nodes), 3)
        
        
    def test_compare_and_mark_nodes(self):
        main_stack = composer.model.stack.Stack(node=self.mock_stack, manifest=data.main_compare_and_mark_nodes)
        child_stack_for_merged= composer.model.stack.Stack(node=self.mock_stack)
        child_stack_for_other = composer.model.stack.Stack(node=self.mock_stack, manifest=data.child_compare_and_mark_nodes)

        main_containers = {(c.namespace, c.name): c for c in main_stack.composable}
        other_composable = {(c.namespace, c.name): c for c in child_stack_for_other.composable}
        
        for key, container in other_composable.items():
            current_container = main_containers[key]
            self.node.compare_and_mark_nodes(current_container, container, child_stack_for_merged)

        self.assertEqual(main_stack.composable[0].nodes[0].action, 'stop')
        self.assertEqual(main_stack.composable[0].nodes[1].action, 'stop')
        self.assertEqual(main_stack.composable[0].nodes[2].action, 'none')
        self.assertEqual(child_stack_for_other.composable[0].nodes[0].action, 'none')
        self.assertEqual(child_stack_for_other.composable[0].nodes[1].action, 'start')
        

    def test_flatten_composable(self):  #OK
        main_stack = composer.model.stack.Stack(node=self.mock_stack)
        child_stack1 = composer.model.stack.Stack(node=self.mock_stack)
        child_stack2 = composer.model.stack.Stack(node=self.mock_stack)
        
        main_stack.composable = data.flatten_composable_main
        child_stack1.composable = data.flatten_composable_child1
        child_stack2.composable = data.flatten_composable_child2
        
        main_stack.stack = [child_stack1, child_stack2]
        result = main_stack.flatten_composable([])
        self.assertEqual(result, data.flatten_composable_expected)


    @patch('composer.model.stack.Stack.get_active_nodes')
    def test_should_node_run(self, get_active_nodes):         #OK
        get_active_nodes.return_value = [('talker', '/')]

        node_mock = MagicMock()
        node_mock.namespace =  ''
        node_mock.name = 'talker'
        returned_value = self.node.should_node_run(node_mock)
        self.assertFalse(returned_value)
        
        second_mock = MagicMock()
        node_mock.namespace =  ''
        node_mock.name = 'listener'
        returned_value_second = self.node.should_node_run(second_mock)
        self.assertTrue(returned_value_second)
        

    @patch('composer.model.stack.Stack.kill_diff')
    @patch('composer.model.stack.Stack.launch')
    def test_apply(self, mock_launcher, mock_kill_diff):
        launcher = MagicMock()
        self.node.apply(launcher)
        mock_kill_diff.assert_called_once()
        mock_launcher.assert_called_once_with(launcher)

        
    @patch('composer.introspection.introspector.Introspector.kill')
    def test_kill_diff(self, kill):         #OK
        launcher_mock = MagicMock()
        stack_mock = composer.model.stack.Stack(node=self.mock_stack)
        introspector_mock =self.intor
        
        node_stop = MagicMock(exec='node_stop', action='stop')
        node_start = MagicMock(exec='node_start', action='start')
        stack_mock.node = [node_stop, node_start]
        
        composable_stop = MagicMock(exec='composable_stop', action='stop')
        container_mock = MagicMock(nodes=[composable_stop])
        stack_mock.composable = [container_mock]
        
        launcher_mock._active_nodes = [
            {'node_stop': 1},
            {'composable_stop': 2},
            {'node_start': 3} 
        ]
        
        self.node.kill_diff(launcher_mock, stack_mock)
        
        introspector_mock.kill.assert_any_call('node_stop', 1)
        introspector_mock.kill.assert_any_call('composable_stop', 2)
        self.assertEqual(introspector_mock.kill.call_count, 2)
        
    
    def test_compare_composable_common(self):   #OK
        node_mock = MagicMock()
        stack_main = composer.model.stack.Stack(node_mock, manifest=data.main_compare_composable)
        stack_common = composer.model.stack.Stack(node_mock, manifest=data.child_compare_composable) 
        common, added, removed = stack_main.compare_composable(stack_common)
        
        self.assertIsNotNone(common)
        self.assertIsNotNone(added)
        self.assertIsNotNone(removed)
        self.assertEqual(len(common), 1)            
        self.assertEqual(len(added), 0)        
        self.assertEqual(len(removed), 0) 
        
        
    def test_compare_composable_added_removed(self):        #OK
        node_mock = MagicMock()
        stack_main = composer.model.stack.Stack(node_mock, manifest=data.main_compare_composable)
        stack_added_removed = composer.model.stack.Stack(node_mock, manifest=data.child_compare_composable_added_removed)
        common, added, removed = stack_main.compare_composable(stack_added_removed)

        self.assertEqual(vars(added[0])['name'], 'muto_demo_container_added')
        self.assertEqual(vars(removed[0])['name'], 'muto_demo_container')   
            
        self.assertIsNotNone(common)
        self.assertIsNotNone(added)
        self.assertIsNotNone(removed)
        self.assertEqual(len(common), 0)            
        self.assertEqual(len(added), 1)        
        self.assertEqual(len(removed), 1) 
        
    @patch('composer.introspection.introspector.Introspector.kill')
    def test_kill_all(self, mock_Introspector):     #OK
        mock_launcher = MagicMock()
        mock_launcher._active_nodes = [{'node1': 1234}, {'node2': 5678}]
        mock_Introspector = MagicMock()
        mock_intrspc = Introspector()

        returned_value = self.node.kill_all(mock_launcher)
            
        calls = [unittest.mock.call('node1', 1234), unittest.mock.call('node2', 5678)]
        mock_intrspc.kill.assert_has_calls(calls, any_order=True)
                
                
    
    def test_handle_regular_nodes(self):    #OK
        self.nodes = MagicMock()
        self.launch_description = LaunchDescription()
        self.launcher = MagicMock()
        nodes = []
        e = Node(self)
        e.action = 'start'
        nodes.append(e)
        main_stack = composer.model.stack.Stack(self.nodes)
        main_stack.handle_regular_nodes(nodes, self.launch_description, self.launcher)

    
if __name__ == '__main__':
    unittest.main()