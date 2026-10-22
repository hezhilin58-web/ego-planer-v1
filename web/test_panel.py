#!/usr/bin/env python3
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import signal
import subprocess

import yaml

from inspection_panel import Panel, defaults, validate


class PanelTests(unittest.TestCase):
    def test_planning_records_survive_stop_and_are_marked_archived(self):
        import json
        import time
        with tempfile.TemporaryDirectory() as directory:
            panel = Panel(directory)
            (Path(directory) / 'status.json').write_text(json.dumps(dict(
                received_at=time.time(), planning=dict(count=2, records=[]))))
            state = panel.state()
            self.assertIsNone(state['mission'])
            self.assertEqual(state['planning']['count'], 2)
            self.assertTrue(state['planning']['archived'])

    def test_default_and_variable_height_are_supported(self):
        config = defaults()
        config.update(height=1.5, hover_time=2., max_vel=.6)
        config['stations'] = config['stations'][:2]
        result = validate(config)
        self.assertEqual(result['height'], 1.5)
        self.assertEqual(len(result['stations']), 2)

    def test_rejects_nonfinite_boolean_out_of_bounds_and_bad_station_count(self):
        for key, value in [('max_vel', float('nan')), ('height', True), ('height', 3),
                           ('max_acc', -1), ('hover_time', 0), ('stations', []),
                           ('stations', defaults()['stations'] * 11)]:
            with self.subTest(key=key, value=value):
                config = defaults()
                config[key] = value
                with self.assertRaises(ValueError):
                    validate(config)
        config = defaults()
        config['stations'][0]['x'] = 20
        with self.assertRaises(ValueError):
            validate(config)

    def test_save_persists_draft_without_mutating_active_mission(self):
        with tempfile.TemporaryDirectory() as directory:
            panel = Panel(directory)
            panel.active_config = defaults()
            config = defaults()
            config['height'] = 1.5
            panel.save(config)
            self.assertEqual(Panel(directory).config['height'], 1.5)
            self.assertEqual(panel.active_config['height'], 1.)

    def test_launch_applies_all_parameters_and_extends_timeout_for_long_hover(self):
        with tempfile.TemporaryDirectory() as directory:
            panel = Panel(directory, ros_port=0)
            config = defaults()
            config.update(height=1.5, hover_time=60., max_vel=.6, max_acc=.7)
            config['stations'] = [dict(x=-16, y=0, yaw_deg=45), dict(x=-15, y=0, yaw_deg=-45)]
            process = Mock()
            process.poll.return_value = 0
            with patch('inspection_panel.subprocess.Popen', return_value=process) as spawn:
                panel.launch(config)
                command = spawn.call_args.args[0]
                self.assertIn('max_vel:=0.6', command)
                self.assertIn('max_acc:=0.7', command)
                self.assertIn('auto_start:=false', command)
                self.assertEqual(spawn.call_args.kwargs['env']['ROS_MASTER_URI'], 'http://127.0.0.1:0')
                mission = yaml.safe_load((Path(directory) / 'mission.yaml').read_text())
                self.assertEqual([s['position'][2] for s in mission['stations']], [1.5, 1.5])
                self.assertEqual([s['yaw_deg'] for s in mission['stations']], [45, -45])
                self.assertEqual(mission['hover_time'], 60)
                self.assertGreaterEqual(mission['station_timeout'], 81)
                panel.stop()

    def test_active_mission_cannot_be_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            panel = Panel(directory)
            for state in ['TRAVEL', 'RETURN_HOME', 'DESCEND']:
                with patch.object(panel, '_alive', return_value=True), patch.object(panel, '_mission', return_value={'state':state}), patch.object(panel, 'stop') as stop:
                    with self.assertRaises(RuntimeError):
                        panel.launch(defaults())
                    stop.assert_not_called()

    def test_start_requires_fresh_ready_mission(self):
        with tempfile.TemporaryDirectory() as directory:
            panel = Panel(directory)
            for state in [None, {'state':'TRAVEL'}, {'state':'ALIGN'}, {'state':'HOVER'}, {'state':'RETURN_HOME'}, {'state':'DESCEND'}]:
                with patch.object(panel, '_mission', return_value=state), patch('inspection_panel.subprocess.run') as run:
                    with self.assertRaises(RuntimeError):
                        panel.start_mission()
                    run.assert_not_called()

    def test_stop_waits_for_roslaunch_to_clean_up_core(self):
        with tempfile.TemporaryDirectory() as directory:
            panel = Panel(directory)
            panel.process = Mock(pid=1234)
            panel.process.poll.return_value = None
            with patch('inspection_panel.os.killpg') as kill:
                panel.stop()
                panel.process.send_signal.assert_called_once_with(signal.SIGINT)
                panel.process.wait.assert_called_once_with(timeout=25)
                kill.assert_not_called()

    def test_stop_escalates_only_owned_process_group_after_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            panel = Panel(directory)
            panel.process = Mock(pid=1234)
            panel.process.poll.return_value = None
            panel.process.wait.side_effect = [subprocess.TimeoutExpired('roslaunch', 25), 0]
            with patch('inspection_panel.os.killpg') as kill:
                panel.stop()
                kill.assert_called_once_with(1234, signal.SIGKILL)


if __name__ == '__main__':
    unittest.main()
