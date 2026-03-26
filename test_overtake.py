"""
==========================================================================
 OVERTAKE FUNCTION TEST
==========================================================================
 Minimal scenario: Ego + one slow lead on a multi-lane road.

 Expected sequence:
   CRUISE → FOLLOW → PREPARE_OVERTAKE → LANE_CHANGE → PASSING
       → RETURN_TO_LANE → CRUISE

 Prints a clear PASS/FAIL verdict.
 Press Ctrl+C to abort early.
==========================================================================
"""

import time
import math
import random
import carla
import config
from carla_interface import CarlaInterface
from planner import BehaviorPlanner, MotionPlanner, BehaviorState
from controller import VehicleController


# How long to run before declaring failure (seconds of sim time)
MAX_SIM_SECONDS = 60
MAX_FRAMES = int(MAX_SIM_SECONDS / config.FIXED_DELTA_SECONDS)  # 1200 frames

# Track which states we've visited
OVERTAKE_SEQUENCE = [
    BehaviorState.FOLLOW,
    BehaviorState.PREPARE_OVERTAKE,
    BehaviorState.LANE_CHANGE,
    BehaviorState.PASSING,
    BehaviorState.RETURN_TO_LANE,
]


def main():
    print("=" * 60)
    print("  OVERTAKE FUNCTION TEST")
    print("  Spawning ego + slow lead on multi-lane road")
    print("=" * 60)

    interface = CarlaInterface()
    lead_vehicle = None

    try:
        # ── Setup ──
        interface.connect()
        interface.setup_world()
        interface.spawn_ego_vehicle(spawn_point_index=0)
        interface.spawn_sensors()
        # Ignore traffic lights for ego so overtake test isn't blocked
        interface.tm.ignore_lights_percentage(interface.ego_vehicle, 100.0)
        interface.world.tick()

        # ── Spawn lead vehicle 25m ahead, very slow ──
        ego_wp = interface.map.get_waypoint(
            interface.ego_vehicle.get_location()
        )
        next_wps = ego_wp.next(25.0)
        if not next_wps:
            print("FAIL: Road ends, cannot spawn lead.")
            return

        lead_trans = next_wps[0].transform
        lead_trans.location.z += 1.0

        bp = interface.world.get_blueprint_library().find(
            'vehicle.nissan.patrol'
        )
        lead_vehicle = interface.world.try_spawn_actor(bp, lead_trans)

        if not lead_vehicle:
            print("FAIL: Could not spawn lead vehicle.")
            return

        # Make lead very slow: 80% speed difference = 20% of speed limit
        lead_vehicle.set_autopilot(True, interface.tm.get_port())
        interface.tm.vehicle_percentage_speed_difference(lead_vehicle, 80.0)
        interface.tm.ignore_lights_percentage(lead_vehicle, 100.0)

        print(f"Lead vehicle {lead_vehicle.id} spawned 25m ahead (very slow)")

        # ── Route: pick a destination far ahead ──
        dest_wps = ego_wp.next(500.0)
        if dest_wps:
            destination = dest_wps[0].transform.location
        else:
            destination = random.choice(
                interface.map.get_spawn_points()
            ).location

        route = interface.generate_route(
            interface.ego_vehicle.get_location(), destination
        )
        print(f"Route: {len(route)} waypoints")

        # ── Init autonomy modules ──
        planner = BehaviorPlanner(interface)
        planner.global_route = route
        planner.ignore_traffic_lights = True
        motion = MotionPlanner(interface.map)
        controller = VehicleController()

        # ── Tracking ──
        states_seen = set()
        state_log = []  # (frame, state_name) for transitions
        prev_state = None
        overtake_complete = False

        # ── Main loop ──
        print("\nRunning...\n")
        for frame_num in range(MAX_FRAMES):
            data = interface.get_data()
            if not data:
                continue

            ego_transform = data['ego_transform']
            ego_vel = data['ego_velocity']
            ego_speed = math.sqrt(ego_vel.x ** 2 + ego_vel.y ** 2)

            # Plan
            state, target_speed = planner.plan(
                ego_transform, ego_speed, data['nearby_vehicles']
            )

            # Track transitions
            if state != prev_state:
                state_log.append((frame_num, state.name))
                states_seen.add(state)
                print(f"  [{frame_num:4d}] State: {state.name:20s} "
                      f"| Speed: {ego_speed * 3.6:5.1f} km/h "
                      f"| follow_ticks: {planner.follow_ticks}")
                prev_state = state

            # Check for successful overtake completion
            if (BehaviorState.RETURN_TO_LANE in states_seen and
                    state == BehaviorState.CRUISE and
                    BehaviorState.PASSING in states_seen):
                overtake_complete = True
                print(f"\n  [{frame_num:4d}] Overtake sequence COMPLETE!")
                break

            # Motion + Control
            waypoints = motion.generate_path(
                ego_transform, state, planner.target_lane_wp,
                global_route=planner.global_route,
                overtake_ticks=planner.overtake_ticks
            )

            control_cmd = controller.run_step(
                ego_speed, ego_transform, target_speed, waypoints
            )
            interface.apply_control(control_cmd)

        # ── Verdict ──
        print("\n" + "=" * 60)
        print("  TEST RESULT")
        print("=" * 60)
        print(f"  States visited: {[s.name for s in states_seen]}")
        print(f"  Transitions:")
        for f, name in state_log:
            print(f"    Frame {f:4d}: {name}")

        # Check each required state was hit
        all_hit = True
        for required in OVERTAKE_SEQUENCE:
            hit = required in states_seen
            mark = "OK" if hit else "MISSING"
            print(f"  {required.name:20s}: {mark}")
            if not hit:
                all_hit = False

        return_hit = BehaviorState.RETURN_TO_LANE in states_seen
        print(f"  {'RETURN_TO_LANE':20s}: {'OK' if return_hit else 'MISSING'}")

        if overtake_complete:
            print("\n  >>> PASS: Full overtake sequence completed! <<<")
        elif all_hit:
            print("\n  >>> PARTIAL: All states hit but didn't return to CRUISE <<<")
        else:
            print("\n  >>> FAIL: Overtake sequence incomplete <<<")

            # Diagnose why
            if BehaviorState.FOLLOW not in states_seen:
                print("  Diagnosis: Never entered FOLLOW — lead too far?")
            elif BehaviorState.PREPARE_OVERTAKE not in states_seen:
                print(f"  Diagnosis: Never triggered overtake.")
                print(f"    follow_ticks reached: {planner.follow_ticks}")
                print(f"    cooldown remaining: {planner.overtake_cooldown_ticks}")
            elif BehaviorState.LANE_CHANGE not in states_seen:
                print("  Diagnosis: PREPARE aborted — lane always blocked?")
            elif BehaviorState.PASSING not in states_seen:
                print("  Diagnosis: Stuck in LANE_CHANGE — never reached target lane?")

        print("=" * 60)

    except KeyboardInterrupt:
        print("\nAborted by user.")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if lead_vehicle and lead_vehicle.is_alive:
            lead_vehicle.destroy()
        interface.cleanup()


if __name__ == '__main__':
    main()
