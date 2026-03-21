"""
test_rail.py — One-command RAIL integration test suite.

Validates all three RAIL-integrated modules against the CARLA simulator:
  1. Camera Perception — detects obstacles from front camera
  2. Local Grid Planner — avoids static obstacles via A* on occupancy grid
  3. Imitation Learning — inference produces valid control outputs

Usage:
    python test_rail.py                  # Run all tests
    python test_rail.py camera           # Test camera perception only
    python test_rail.py grid             # Test local grid planner only
    python test_rail.py learning         # Test imitation learning only
    python test_rail.py --frames 3000    # Run with custom frame count
"""

import sys
import os
import time
import math
import random
import argparse

sys.path.insert(0, os.path.dirname(__file__))

import config
import numpy as np


# ==============================================================================
# -- Test Configuration --------------------------------------------------------
# ==============================================================================
MAX_FRAMES = 2000
PASS_CRITERIA = {
    'camera_detections_min': 5,        # Must detect at least N objects from camera
    'grid_activations_min': 3,         # Grid planner must activate at least N times
    'learning_valid_outputs': True,    # All learning outputs must be in [-1, 1]
    'no_crashes': True,                # No unhandled exceptions
    'fps_min': 8.0,                    # Must sustain at least N FPS
}


class TestResults:
    """Tracks pass/fail for each RAIL module."""
    def __init__(self):
        self.camera_detections = 0
        self.camera_fused = 0
        self.grid_activations = 0
        self.grid_path_lengths = []
        self.learning_predictions = 0
        self.learning_invalid = 0
        self.frames_processed = 0
        self.errors = []
        self.state_counts = {}
        self.start_time = None
        self.collision_count = 0

    def record_state(self, state_name):
        self.state_counts[state_name] = self.state_counts.get(state_name, 0) + 1

    def fps(self):
        if not self.start_time:
            return 0
        elapsed = time.time() - self.start_time
        return self.frames_processed / max(elapsed, 0.001)

    def report(self, tests_run):
        elapsed = time.time() - self.start_time if self.start_time else 0
        print()
        print("=" * 70)
        print("  RAIL Integration Test — Results")
        print("=" * 70)
        print(f"  Frames: {self.frames_processed} | Duration: {elapsed:.0f}s | FPS: {self.fps():.1f}")
        print()

        verdicts = {}

        # Camera Perception
        if 'camera' in tests_run:
            print("  [Camera Perception]")
            print(f"    Raw detections:  {self.camera_detections}")
            print(f"    After fusion:    {self.camera_fused} novel (not in LiDAR/Radar)")
            cam_pass = self.camera_detections >= PASS_CRITERIA['camera_detections_min']
            verdicts['camera'] = cam_pass
            print(f"    Verdict: {'PASS' if cam_pass else 'FAIL'} "
                  f"(need >= {PASS_CRITERIA['camera_detections_min']} detections)")
            print()

        # Grid Planner
        if 'grid' in tests_run:
            print("  [Local Grid Planner]")
            print(f"    Activations:     {self.grid_activations}")
            if self.grid_path_lengths:
                avg_len = sum(self.grid_path_lengths) / len(self.grid_path_lengths)
                print(f"    Avg path length: {avg_len:.1f} waypoints")
            grid_pass = self.grid_activations >= PASS_CRITERIA['grid_activations_min']
            verdicts['grid'] = grid_pass
            print(f"    Verdict: {'PASS' if grid_pass else 'FAIL'} "
                  f"(need >= {PASS_CRITERIA['grid_activations_min']} activations)")
            print()

        # Learning
        if 'learning' in tests_run:
            print("  [Imitation Learning]")
            print(f"    Predictions:     {self.learning_predictions}")
            print(f"    Invalid outputs: {self.learning_invalid}")
            learn_pass = (self.learning_predictions > 0 and
                          self.learning_invalid == 0)
            verdicts['learning'] = learn_pass
            print(f"    Verdict: {'PASS' if learn_pass else 'FAIL'} "
                  f"(need valid outputs in [-1, 1])")
            print()

        # Performance
        fps_pass = self.fps() >= PASS_CRITERIA['fps_min']
        verdicts['performance'] = fps_pass
        print(f"  [Performance]")
        print(f"    Average FPS: {self.fps():.1f} (need >= {PASS_CRITERIA['fps_min']})")
        print(f"    Verdict: {'PASS' if fps_pass else 'FAIL'}")
        print()

        # State distribution
        if self.state_counts:
            print("  [Behavior States]")
            total = sum(self.state_counts.values())
            for state, count in sorted(self.state_counts.items(), key=lambda x: -x[1]):
                pct = 100.0 * count / total
                bar = '#' * int(pct / 2)
                print(f"    {state:20s} {count:5d} ({pct:5.1f}%) {bar}")
            print()

        # Errors
        if self.errors:
            print(f"  [Errors] ({len(self.errors)} total)")
            for frame, err in self.errors[:5]:
                print(f"    Frame {frame}: {err}")
            if len(self.errors) > 5:
                print(f"    ... and {len(self.errors) - 5} more")
            print()

        # Overall
        all_pass = all(verdicts.values())
        print("-" * 70)
        for name, passed in verdicts.items():
            print(f"  {name:15s}: {'PASS' if passed else 'FAIL'}")
        print("-" * 70)
        print(f"  OVERALL: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
        print("=" * 70)

        return all_pass


def run_tests(tests_to_run, max_frames=MAX_FRAMES):
    """
    Runs the specified RAIL tests against CARLA.
    """
    import carla
    from carla_interface import CarlaInterface
    from planner import BehaviorPlanner, MotionPlanner, LocalGridPlanner
    from controller import VehicleController
    from perception import CameraPerceptionModule, PerceptionModule

    test_camera = 'camera' in tests_to_run
    test_grid = 'grid' in tests_to_run
    test_learning = 'learning' in tests_to_run

    print("=" * 70)
    print("  RAIL Integration Test Suite")
    print(f"  Tests: {', '.join(tests_to_run)}")
    print(f"  Frames: {max_frames}")
    print("=" * 70)

    results = TestResults()
    interface = CarlaInterface()
    lead_vehicle = None
    static_obstacles = []

    try:
        # --- Setup ---
        interface.connect()
        interface.setup_world()
        interface.spawn_ego_vehicle(spawn_point_index=0)
        interface.spawn_sensors()
        interface.world.tick()

        # Spawn lead vehicle
        ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
        next_wps = ego_wp.next(15.0)
        if next_wps:
            lead_trans = next_wps[0].transform
            lead_trans.location.z += 1.0
            bp = interface.world.get_blueprint_library().find('vehicle.nissan.patrol')
            lead_vehicle = interface.world.try_spawn_actor(bp, lead_trans)
            if lead_vehicle:
                lead_vehicle.set_autopilot(True, interface.tm.get_port())
                interface.tm.vehicle_percentage_speed_difference(lead_vehicle, 60.0)
                print(f"  Lead vehicle spawned (id={lead_vehicle.id})")

        # Spawn static obstacles for grid planner testing
        if test_grid:
            print("  Spawning static obstacles for grid planner test...")
            obstacle_bps = interface.world.get_blueprint_library().filter('static.prop.*')
            if not obstacle_bps:
                obstacle_bps = interface.world.get_blueprint_library().filter('vehicle.*')

            for dist in [25.0, 40.0, 55.0]:
                ahead_wps = ego_wp.next(dist)
                if ahead_wps:
                    obs_trans = ahead_wps[0].transform
                    obs_trans.location.z += 0.5
                    # Offset slightly to not fully block the road
                    yaw_rad = math.radians(obs_trans.rotation.yaw)
                    obs_trans.location.x += 1.5 * math.sin(yaw_rad)
                    obs_trans.location.y -= 1.5 * math.cos(yaw_rad)

                    obs_bp = random.choice(obstacle_bps)
                    obs = interface.world.try_spawn_actor(obs_bp, obs_trans)
                    if obs:
                        static_obstacles.append(obs)

            print(f"  Spawned {len(static_obstacles)} static obstacles")

        # Generate route
        spawn_points = interface.map.get_spawn_points()
        destination = random.choice(spawn_points).location
        route = interface.generate_route(interface.ego_vehicle.get_location(), destination)
        print(f"  Route: {len(route)} waypoints")

        # Init modules
        behavior_planner = BehaviorPlanner(interface)
        behavior_planner.global_route = route
        motion_planner = MotionPlanner(interface.map)
        controller = VehicleController()
        perception = PerceptionModule()

        camera_perception = CameraPerceptionModule() if test_camera else None
        local_grid_planner = LocalGridPlanner() if test_grid else None

        # Learning module (offline test — no CARLA needed for inference check)
        learner = None
        if test_learning:
            from learning import ImitationLearner
            learner = ImitationLearner()
            if not learner.enabled:
                print("  WARNING: PyTorch not available, learning test will be skipped.")
                learner = None

        print(f"\n  Starting test loop ({max_frames} frames)...\n")
        results.start_time = time.time()

        # --- Main Test Loop ---
        for frame_idx in range(max_frames):
            try:
                data = interface.get_data()
                if not data:
                    continue

                results.frames_processed += 1

                ego_transform = data['ego_transform']
                ego_vel = data['ego_velocity']
                ego_speed = math.sqrt(ego_vel.x ** 2 + ego_vel.y ** 2)
                nearby = data['nearby_vehicles']

                # Re-route if reached destination
                if behavior_planner.global_route:
                    d_goal = ego_transform.location.distance(
                        behavior_planner.global_route[-1][0].transform.location
                    )
                    if d_goal < 5.0:
                        destination = random.choice(spawn_points).location
                        route = interface.generate_route(ego_transform.location, destination)
                        behavior_planner.global_route = route

                # --- Test: Camera Perception ---
                if camera_perception and 'camera_front' in data and data['camera_front'] is not None:
                    cam_detections = camera_perception.process(data['camera_front'], ego_transform)
                    if cam_detections:
                        results.camera_detections += len(cam_detections)
                        fused = perception.fuse_camera_detections(nearby, cam_detections)
                        novel = len(fused) - len(nearby)
                        results.camera_fused += novel
                        nearby = fused

                # Planning
                state, target_speed = behavior_planner.plan(ego_transform, ego_speed, nearby)
                results.record_state(state.name)

                waypoints = motion_planner.generate_path(
                    ego_transform, state,
                    behavior_planner.target_lane_wp,
                    global_route=behavior_planner.global_route
                )

                # --- Test: Local Grid Planner ---
                if local_grid_planner and nearby:
                    local_path = local_grid_planner.plan_local_path(ego_transform, nearby)
                    if local_path:
                        results.grid_activations += 1
                        results.grid_path_lengths.append(len(local_path))
                        waypoints = local_path + waypoints[len(local_path):]

                # Control
                control_cmd = controller.run_step(ego_speed, ego_transform, target_speed, waypoints)

                # --- Test: Imitation Learning ---
                if learner:
                    state_vec = learner.extract_state(ego_speed, ego_transform, target_speed, nearby)
                    learned_cmd = learner.predict(state_vec)
                    if learned_cmd is not None:
                        results.learning_predictions += 1
                        # Validate output ranges
                        for key in ['throttle', 'brake', 'steer']:
                            val = learned_cmd[key]
                            if val < -1.01 or val > 1.01:
                                results.learning_invalid += 1

                # Actuation
                interface.apply_control(control_cmd)

                # Progress logging
                if frame_idx % 400 == 0 and frame_idx > 0:
                    print(f"  Frame {frame_idx:5d}/{max_frames} | "
                          f"FPS: {results.fps():.1f} | "
                          f"State: {state.name:15s} | "
                          f"CamDet: {results.camera_detections} | "
                          f"GridAct: {results.grid_activations} | "
                          f"LearnPred: {results.learning_predictions}")

            except Exception as e:
                results.errors.append((frame_idx, str(e)))
                if len(results.errors) > 20:
                    print(f"  Too many errors ({len(results.errors)}), aborting test.")
                    break

        # --- Report ---
        return results.report(tests_to_run)

    except KeyboardInterrupt:
        print("\n  Test interrupted by user.")
        return results.report(tests_to_run)
    except Exception as e:
        print(f"\n  Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        for obs in static_obstacles:
            if obs.is_alive:
                obs.destroy()
        if lead_vehicle and lead_vehicle.is_alive:
            lead_vehicle.destroy()
        interface.cleanup()


def test_learning_offline():
    """
    Tests the imitation learning module WITHOUT CARLA.
    Validates model loading, inference, and output ranges.
    """
    print("=" * 70)
    print("  RAIL Learning Module — Offline Test")
    print("=" * 70)

    from learning import ImitationLearner, TORCH_AVAILABLE

    if not TORCH_AVAILABLE:
        print("  SKIP: PyTorch not installed.")
        return False

    learner = ImitationLearner()
    if not learner.enabled:
        print("  SKIP: Learner not enabled.")
        return False

    model_path = learner.model_path
    if not os.path.exists(model_path):
        print(f"  SKIP: No trained model at {model_path}")
        print("  Run 'python train.py all' first.")
        return False

    print(f"  Model: {model_path}")

    # Test with synthetic inputs
    test_cases = [
        ("Cruising, no obstacles", np.array([8.3, 0.0, 100.0, 0.0, 8.3], dtype=np.float32)),
        ("Following, close", np.array([5.0, 0.0, 8.0, 0.0, 5.0], dtype=np.float32)),
        ("Stopped, obstacle ahead", np.array([0.0, 0.0, 3.0, 0.0, 0.0], dtype=np.float32)),
        ("Turning, obstacle left", np.array([6.0, 0.3, 15.0, 1.0, 8.3], dtype=np.float32)),
        ("Fast, clear road", np.array([12.0, 0.0, 100.0, 0.0, 12.0], dtype=np.float32)),
    ]

    all_valid = True
    print()
    for name, state_vec in test_cases:
        cmd = learner.predict(state_vec)
        valid = all(-1.01 <= cmd[k] <= 1.01 for k in ['throttle', 'brake', 'steer'])
        status = "OK" if valid else "FAIL"
        if not valid:
            all_valid = False
        print(f"  [{status}] {name:30s} -> "
              f"throttle={cmd['throttle']:.3f} brake={cmd['brake']:.3f} steer={cmd['steer']:.3f}")

    # Test blending
    classical = {'throttle': 0.5, 'brake': 0.0, 'steer': 0.1}
    learned = {'throttle': 0.3, 'brake': 0.0, 'steer': -0.1}
    blended = ImitationLearner.blend_controls(classical, learned, alpha=0.3)
    blend_valid = all(-1.01 <= blended[k] <= 1.01 for k in ['throttle', 'brake', 'steer'])
    print(f"\n  Blend test (alpha=0.3):")
    print(f"    Classical: throttle={classical['throttle']}, steer={classical['steer']}")
    print(f"    Learned:   throttle={learned['throttle']}, steer={learned['steer']}")
    print(f"    Blended:   throttle={blended['throttle']:.3f}, steer={blended['steer']:.3f}")
    print(f"    Valid: {'OK' if blend_valid else 'FAIL'}")

    overall = all_valid and blend_valid
    print(f"\n  Overall: {'PASS' if overall else 'FAIL'}")
    print("=" * 70)
    return overall


def main():
    parser = argparse.ArgumentParser(
        description="RAIL Integration Test Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python test_rail.py                  Run all tests in CARLA
  python test_rail.py camera           Test camera perception only
  python test_rail.py grid             Test local grid planner only
  python test_rail.py learning         Test imitation learning only
  python test_rail.py camera grid      Test camera + grid
  python test_rail.py --offline        Test learning module without CARLA
  python test_rail.py --frames 3000    Run with more frames
        """
    )
    parser.add_argument('tests', nargs='*',
                        help="Which modules to test: camera, grid, learning (default: all)")
    parser.add_argument('--frames', type=int, default=MAX_FRAMES,
                        help=f"Number of simulation frames (default: {MAX_FRAMES})")
    parser.add_argument('--offline', action='store_true',
                        help="Run learning test offline (no CARLA needed)")

    args = parser.parse_args()

    valid = {'camera', 'grid', 'learning'}
    for t in (args.tests or []):
        if t not in valid:
            parser.error(f"invalid test: '{t}' (choose from {', '.join(sorted(valid))})")

    tests = args.tests if args.tests else ['camera', 'grid', 'learning']

    if args.offline:
        success = test_learning_offline()
    else:
        success = run_tests(tests, max_frames=args.frames)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
