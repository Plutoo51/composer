import unittest
from unittest.mock import MagicMock, patch
from composer.introspection.launcher import Ros2LaunchParent
import multiprocessing
from launch import LaunchService

class TestLauncher(unittest.TestCase):
    def setUp(self):
        self.launcher = Ros2LaunchParent()
        self.launcher._stop_event = multiprocessing.Event()
        self.launcher._process = multiprocessing.Process(None)
    
    
    @patch('multiprocessing.Event')
    @patch('multiprocessing.Process')
    def test_start(self, mock_process, mock_event):
        self.launcher._stop_event = mock_event
        self.launcher._process = mock_process
        self.launcher.start(MagicMock())
        self.launcher._process.start.assert_called_once()
        mock_event.assert_called_once()
        mock_process.assert_called_once()
        
  
    @patch('multiprocessing.Process')
    @patch('multiprocessing.Event')
    def test_shutdown_alive_is_false(self, mock_event, mock_process):
        self.launcher._process = mock_process
        self.launcher._stop_event = mock_event
        self.launcher._process.is_alive.return_value = False
        self.launcher.shutdown()
        
        self.launcher._process.join.assert_called_with(timeout=20.0)
        self.launcher._process.terminate.assert_not_called()
        self.launcher._stop_event.set.assert_called()
        
    @patch('multiprocessing.Process')
    @patch('multiprocessing.Event')
    def test_shutdown_alive_is_true(self, mock_event, mock_process):
        self.launcher._process = mock_process
        self.launcher._stop_event = mock_event
        self.launcher._process.is_alive.return_value = True
        self.launcher.shutdown()
        
        self.launcher._process.join.assert_called_with(timeout=20.0)
        self.launcher._process.terminate.assert_called_once()
        self.launcher._stop_event.set.assert_called_once()
        
        
    def test_event_handler_start(self):
        event = MagicMock()
        event.process_name = 'rviz_talker_listener'
        event.pid = 1200
        
        self.launcher._event_handler('start', event, self.launcher._active_nodes, self.launcher._lock)
        
        self.assertEqual(len(self.launcher._active_nodes), 1)
        self.assertEqual(self.launcher._active_nodes[0], {'rviz_talker_listener': 1200})

    @patch('multiprocessing.Process')
    def test_event_handler_exit(self, mock_process):
        self.launcher._process = mock_process
        event = MagicMock()
        event.process_name = 'rviz_talker_listener'
        event.pid = 1200
        self.launcher._active_nodes.append({'rviz_talker_listener': 1200})
        self.launcher._event_handler('exit', event, self.launcher._active_nodes, self.launcher._lock)
        self.assertEqual(len(self.launcher._active_nodes), 0)
        
    @patch('multiprocessing.Process')
    def test_event_handler_exit_another_process(self, mock_process):
        self.launcher._process = mock_process
        event = MagicMock()
        event.process_name = 'rviz_talker_listener'
        event.pid = 1200
        self.launcher._active_nodes.append({'rviz_client_server': 1300})
        self.launcher._active_nodes.append({'rviz_talker_listener': 1200})
        self.launcher._event_handler('exit', event, self.launcher._active_nodes, self.launcher._lock)
        self.assertEqual(len(self.launcher._active_nodes), 1)
        self.assertEqual(self.launcher._active_nodes[0], {'rviz_client_server': 1300})
        
        
    @patch('asyncio.new_event_loop')    
    @patch('asyncio.set_event_loop')
    def test_run_process(self, mock_set_event_loop, mock_new_event_loop):        
        event = MagicMock()
        event.process_name = 'rviz_talker_listener'
        event.pid = 1200
        
        mock_new_event_loop.return_value = MagicMock()
        mock_launch_description = MagicMock()
        
        self.launcher._run_process(event, mock_launch_description)
        self.assertEqual(mock_launch_description.add_action.call_count, 2)
        
        mock_new_event_loop.assert_called_once()
        mock_set_event_loop.assert_called_once_with(mock_new_event_loop())
        
        with patch.object(LaunchService, 'include_launch_description') as mock_include:
            self.launcher._run_process(event, mock_launch_description)
            mock_include.assert_called_once()
        
        
if __name__ == '__main__':
    unittest.main()