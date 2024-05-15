import unittest
from unittest.mock import MagicMock, patch
from composer.model.arg import Arg

class TestArg(unittest.TestCase):
    
    def setUp(self):
        self.arg = Arg(stack=MagicMock())
    
    
    @patch('composer.model.arg.ExpressionResolver.resolve_expression')
    @patch('composer.model.arg.ExpressionResolver.has_expression')
    def test_resolve_arg(self, mock_has_expression, mock_resolve_expression):
        mock_has_expression.return_value = True
        self.arg.resolve_arg(1200)
        mock_resolve_expression.assert_called_once_with(1200)
    
    
    @patch('composer.model.arg.ExpressionResolver.resolve_expression')
    @patch('composer.model.arg.ExpressionResolver.has_expression')
    def test_resolve_arg_has_not_expression(self, mock_has_expression, mock_resolve_expression):
        mock_has_expression.return_value = False
        returned_value = self.arg.resolve_arg(1200)
        mock_resolve_expression.assert_not_called()
        self.assertEqual(returned_value, 1200)
    
if __name__ == '__main__':
    unittest.main()