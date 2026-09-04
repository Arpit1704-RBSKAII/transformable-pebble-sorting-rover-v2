"""
Unit tests for the DecisionMachine using MockComm to simulate hardware.

Run:
    pytest python/tests/test_decision.py -q

Reused as-is from the existing repo.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from communication import MockComm
from decision import DecisionMachine


def make_simple_detection():
    return {
        "detected": True,
        "bbox": (100, 120, 40, 35),
        "area": 1200,
        "perimeter": 140.0,
        "contour_solidity": 0.9
    }


def test_full_pick_flow():
    comm = MockComm(behavior_delay=0.01, auto_done=True)
    dm = DecisionMachine(comm=comm)
    dm.start_search()

    det = make_simple_detection()
    dm.process_detection(det, image=None)

    assert dm.state in (dm.STATE_READY, dm.STATE_ERROR) or dm.state == dm.STATE_READY
    assert any(cmd.startswith("ARM") or cmd.startswith("GRIP") or cmd.startswith("SORT") for cmd in comm.sent_commands)


def test_emergency_stops_immediately():
    comm = MockComm(auto_done=True)
    dm = DecisionMachine(comm=comm)
    dm.start_search()
    dm.emergency_stop()

    det = make_simple_detection()
    dm.process_detection(det, image=None)
    assert dm.state == dm.STATE_EMERGENCY
