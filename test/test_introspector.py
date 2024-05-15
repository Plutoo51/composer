import unittest
from unittest.mock import patch, MagicMock
import rclpy
from composer.introspection.introspector import Introspector

class TestIntrospector(unittest.TestCase):

    def setUp(self):
        self.introspector = Introspector()
        self.introspector.get_logger = MagicMock()

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    @patch('subprocess.run')
    def test_kill_success(self, mock_run):
        mock_run.return_value.returncode = 0
        self.introspector.kill('test_process', "test_variable")
        mock_run.assert_called_once_with(['kill', 'test_variable'], check=True, capture_output=True, text=True)
        self.introspector.get_logger().info.assert_called_with('Successfully killed test_process with PID: test_variable')

    @patch('subprocess.run')
    def test_kill_failure(self, mock_run):
        error = mock_run.CalledProcessError(returncode=1, cmd='kill', stderr='Error message')
        mock_run.side_effect = error

        self.introspector.kill('test_process', "test_variable")
        mock_run.assert_called_once_with(['kill', "test_variable"], check=True, capture_output=True, text=True)
        self.introspector.get_logger().error.assert_called_once()

    @patch('subprocess.run')
    def test_kill_exception(self, mock_run):
        mock_run.side_effect = Exception()
        self.introspector.kill('test_process', "test_variable")
        self.introspector.get_logger().error.assert_called_once_with('Unexpected error while trying to kill test_process. Exception message: ')

if __name__ == '__main__':
    unittest.main()