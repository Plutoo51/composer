import unittest
import composer.model.composable as composable
from unittest.mock import MagicMock
import dummy_data as data

class TestComposable(unittest.TestCase):
    
    def setUp(self):
        self.mock_stack = MagicMock()
        self.composable = composable.Container(stack=self.mock_stack)
    
        
    def test_toManifest(self):
        expected_value = {'package': '', 'executable': '', 'name': '', 'namespace': '', 'node': [], 'output': 'screen', 'remap': [], 'action': ''}
        returned_value = self.composable.toManifest()
        self.assertEqual(returned_value, expected_value)
    
        
    def test_eq(self):
        main_composable = composable.Container(stack=self.mock_stack, manifest=data.composable_equal_manifest)
        other_composable = composable.Container(stack=self.mock_stack, manifest=data.composable_equal_manifest)
        returned = main_composable.__eq__(other_composable)
        self.assertTrue(returned)
    
    
    def test_eq_false(self):
        main_composable = composable.Container(stack=self.mock_stack, manifest=data.composable_equal_manifest)
        other_composable = composable.Container(stack=self.mock_stack, manifest=data.composable_equal_manifest_other)
        returned = main_composable.__eq__(other_composable)
        self.assertFalse(returned)
        
    def test_hash(self):
        expected_return = -3226225171056353554
        returned = self.composable.__hash__()
        self.assertEqual(expected_return, returned)
    
if __name__ == '__main__':
    unittest.main()