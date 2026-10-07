"""
Timing and Path Output Script
Connected to test_maps.py and config.py.

Usage:
    python time_path.py
"""
import time
import os
import sys

# Ensure the script directory is in Python path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config
import test_maps
import planner

# validate_config() is no longer bypassed: this script now plans exactly as
# algo_server.py does, so a config error shows up here first.


def run_benchmark():
    obstacles = test_maps.OBSTACLES
    normalized_obstacles = planner.normalize_obstacles(obstacles)

    start_time = time.perf_counter()
    result = planner.plan(normalized_obstacles)
    elapsed_s = time.perf_counter() - start_time
    elapsed_ms = elapsed_s * 1000.0

    print("=" * 60)
    print("           PATH PLANNING BENCHMARK & SUMMARY           ")
    print("=" * 60)
    print(f"Total Obstacles in test_maps.py : {len(normalized_obstacles)}")
    print(f"Obstacles Visited               : {len(result['order'])} / {len(normalized_obstacles)}")
    print(f"Visit Order                     : {result['order']}")
    print(f"Total Cost                      : {result['total_cost']:.2f}")
    print(f"Time Taken (Python Elapsed)     : {elapsed_s:.4f} s ({elapsed_ms:.2f} ms)")
    print(f"Planner Internal Time           : {result['planning_ms'] / 1000.0:.4f} s ({result['planning_ms']:.2f} ms)")
    print("-" * 60)

    print("\n--- DETAILED PATH / COMMANDS ---")
    if result["commands"]:
        print("Full Hardware Command Sequence:")
        print(" ".join(result["commands"]))
    else:
        print("No commands generated.")

    if result["segments"]:
        print("\n--- SEGMENTS BREAKDOWN ---")
        for idx, seg in enumerate(result["segments"], start=1):
            oid = seg["obstacle_id"]
            start_p = tuple(seg["start_pose"])
            goal_p = tuple(seg["goal_pose"])
            cost = seg["cost"]
            cmds = " ".join(seg["hardware_commands"])
            print(f"Leg {idx} -> Target Obstacle {oid}:")
            print(f"   From: {start_p}  -->  To: {goal_p}  |  Cost: {cost:.2f}")
            print(f"   Commands: {cmds}")

    if result["skipped"]:
        print("\n--- SKIPPED OBSTACLES ---")
        for skip in result["skipped"]:
            print(f"   Obstacle {skip['obstacle_id']}: {skip['reason']}")

    print("=" * 60)


if __name__ == "__main__":
    run_benchmark()
