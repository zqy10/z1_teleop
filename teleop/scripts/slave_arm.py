#!/usr/bin/env python3
"""
Slave entry point. Shared control stack lives in ``master_arm`` (``TeleopZ1Arm``).

  python3 ./teleop/scripts/slave_arm.py

Run with ``teleop/scripts`` on ``PYTHONPATH`` or from that directory so ``import master_arm`` works.
"""

from master_arm import teleop_main

if __name__ == "__main__":
    teleop_main("slave")
