#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from inspection_panel_status import PlanningMetrics


def record(seq=1, duration=2., success=True, response=None):
    return dict(seq=seq, goal_id=1, time=100., stage='local_initial', success=success,
                duration_ms=duration, goal_response_ms=response, trajectory_id=seq if success else None)


class MetricsTests(unittest.TestCase):
    def test_failure_and_first_response_are_included(self):
        metrics = PlanningMetrics()
        metrics.add(record(duration=10, success=False))
        metrics.add(record(2, duration=2, response=35))
        result = metrics.snapshot()
        self.assertEqual((result['count'], result['failed']), (2, 1))
        self.assertEqual(result['average_ms'], 6)
        self.assertEqual(result['maximum_ms'], 10)
        self.assertEqual(result['goal_response_ms'], 35)

    def test_gap_duplicate_and_bounded_history(self):
        metrics = PlanningMetrics()
        metrics.add(record(2))
        self.assertFalse(metrics.add(record(2)))
        for seq in range(3, 110):
            metrics.add(record(seq))
        result = metrics.snapshot()
        self.assertEqual(result['count'], 108)
        self.assertEqual(result['missed'], 1)
        self.assertEqual(len(result['records']), 100)
        self.assertEqual(result['records'][0]['seq'], 10)

    def test_invalid_values_do_not_modify_aggregate(self):
        metrics = PlanningMetrics()
        for key, value in [('duration_ms', float('nan')), ('duration_ms', -1),
                           ('goal_response_ms', float('inf')), ('success', 1), ('seq', True)]:
            data = record()
            data[key] = value
            with self.assertRaises(ValueError):
                metrics.add(data)
        self.assertEqual(metrics.snapshot()['count'], 0)
        self.assertIsNone(metrics.snapshot()['average_ms'])


if __name__ == '__main__':
    unittest.main()
