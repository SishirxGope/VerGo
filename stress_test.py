"""
==========================================================================
 STRESS TEST - Infinite Autonomy Stack Endurance Runner
==========================================================================
 Runs the full autonomy pipeline (Perception -> Planning -> Control)
 in an infinite loop with escalating chaos to find roadblocks:

 - Infinite route cycling (re-routes to random destination on arrival)
 - Weather cycling (Clear -> Rain -> Storm -> Fog -> repeat)
 - Periodic NPC traffic spawning & cleanup
 - Lead vehicle respawns with random braking events
 - Sensor health monitoring (detects dropped frames / empty data)
 - Planner state tracking (detects stuck states)
 - Memory & frame-time logging

 Press Ctrl+C to stop. Full report printed on exit.
==========================================================================
"""

import time
import math
import random
import sys
import traceback

import carla
import config
from carla_interface import CarlaInterface
from planner import BehaviorPlanner, MotionPlanner, BehaviorState
from controller import VehicleController

# ==============================================================================
# -- Stress Test Configuration -------------------------------------------------
# ==============================================================================

# Weather presets to cycle through
WEATHER_CYCLE = [
    ("Clear Noon", carla.WeatherParameters.ClearNoon),
    ("Cloudy Noon", carla.WeatherParameters.CloudyNoon),
    ("Heavy Rain", carla.WeatherParameters(
        cloudiness=100.0, precipitation=100.0, sun_altitude_angle=10.0,
        fog_density=50.0, fog_distance=10.0, wetness=100.0
    )),
    ("Storm", carla.WeatherParameters.HardRainNoon),
    ("Dense Fog", carla.WeatherParameters(
        cloudiness=90.0, precipitation=0.0, sun_altitude_angle=15.0,
        fog_density=100.0, fog_distance=5.0, wetness=30.0
    )),
    ("Night Clear", carla.WeatherParameters(
        cloudiness=10.0, precipitation=0.0, sun_altitude_angle=-30.0,
        fog_density=0.0, fog_distance=0.0, wetness=0.0
    )),
    ("Night Rain", carla.WeatherParameters(
        cloudiness=100.0, precipitation=80.0, sun_altitude_angle=-50.0,
        fog_density=40.0, fog_distance=15.0, wetness=100.0
    )),
]

WARMUP_FRAMES            = 200   # frames to let ego cruise before spawning obstacles
WEATHER_CHANGE_INTERVAL  = 600   # frames between weather changes
NPC_SPAWN_INTERVAL       = 900   # frames between NPC spawns
LEAD_RESPAWN_INTERVAL    = 1200  # frames between lead vehicle respawns
LEAD_BRAKE_CHANCE        = 0.25  # 25% chance lead slams brakes on respawn
REPORT_INTERVAL          = 100   # frames between console status lines
STUCK_SPEED_THRESHOLD    = 0.3   # m/s - below this we consider "stuck"
STUCK_FRAME_LIMIT        = 400   # consecutive slow frames before flagging
MAX_NPC_COUNT            = 12    # cap on NPC actors to avoid overload


# ==============================================================================
# -- Stats Tracker -------------------------------------------------------------
# ==============================================================================

class StressStats:
    """Tracks everything interesting that happens during the run."""

    def __init__(self):
        self.start_time = time.time()
        self.total_frames = 0
        self.routes_completed = 0
        self.weather_changes = 0
        self.npcs_spawned = 0
        self.npcs_cleaned = 0
        self.lead_respawns = 0
        self.lead_brake_events = 0
        self.emergency_stops = 0
        self.overtakes_initiated = 0
        self.lane_changes = 0
        self.sensor_drops = 0        # frames where a sensor returned None
        self.stuck_events = 0        # times the car got stuck
        self.max_speed_kph = 0.0
        self.errors = []             # (frame, error_msg) tuples
        self.state_histogram = {}    # BehaviorState -> count
        self.stuck_counter = 0       # running counter for consecutive slow frames

    def record_state(self, state):
        name = state.name
        self.state_histogram[name] = self.state_histogram.get(name, 0) + 1

    def elapsed(self):
        return time.time() - self.start_time

    def report(self):
        elapsed = self.elapsed()
        hrs = int(elapsed // 3600)
        mins = int((elapsed % 3600) // 60)
        secs = int(elapsed % 60)

        print("\n" + "=" * 70)
        print("  STRESS TEST REPORT")
        print("=" * 70)
        print(f"  Runtime           : {hrs}h {mins}m {secs}s")
        print(f"  Total Frames      : {self.total_frames}")
        print(f"  Avg FPS           : {self.total_frames / max(elapsed, 1):.1f}")
        print(f"  Routes Completed  : {self.routes_completed}")
        print(f"  Weather Changes   : {self.weather_changes}")
        print(f"  NPCs Spawned      : {self.npcs_spawned}")
        print(f"  NPCs Cleaned Up   : {self.npcs_cleaned}")
        print(f"  Lead Respawns     : {self.lead_respawns}")
        print(f"  Lead Brake Events : {self.lead_brake_events}")
        print(f"  Emergency Stops   : {self.emergency_stops}")
        print(f"  Overtakes         : {self.overtakes_initiated}")
        print(f"  Lane Changes      : {self.lane_changes}")
        print(f"  Sensor Drops      : {self.sensor_drops}")
        print(f"  Stuck Events      : {self.stuck_events}")
        print(f"  Max Speed         : {self.max_speed_kph:.1f} km/h")
        print()
        print("  State Distribution:")
        total_states = max(sum(self.state_histogram.values()), 1)
        for name, count in sorted(self.state_histogram.items(), key=lambda x: -x[1]):
            pct = (count / total_states) * 100
            bar = "█" * int(pct / 2)
            print(f"    {name:25s} {count:>7d}  ({pct:5.1f}%)  {bar}")

        if self.errors:
            print()
            print(f"  Errors ({len(self.errors)}):")
            for frame, msg in self.errors[-10:]:  # last 10
                print(f"    [Frame {frame}] {msg}")
            if len(self.errors) > 10:
                print(f"    ... and {len(self.errors) - 10} more")

        print("=" * 70)


# ==============================================================================
# -- Helper Functions ----------------------------------------------------------
# ==============================================================================

def spawn_lead_vehicle(interface, dist=20.0):
    """Spawns a lead vehicle ahead of ego on the same road."""
    try:
        ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
        next_wps = ego_wp.next(dist)
        if not next_wps:
            print("[STRESS] Warning: Road ends, cannot spawn lead vehicle.")
            return None

        lead_wp = next_wps[0]
        lead_trans = lead_wp.transform
        lead_trans.location.z += 1.0

        bp = interface.world.get_blueprint_library().find('vehicle.nissan.patrol')
        lead = interface.world.try_spawn_actor(bp, lead_trans)

        if lead:
            lead.set_autopilot(True, interface.tm.get_port())
            interface.tm.vehicle_percentage_speed_difference(lead, 60.0)
            interface.tm.ignore_lights_percentage(lead, 0.0)
            print(f"[STRESS] Lead Vehicle {lead.id} spawned at {lead.get_location()}")
        return lead
    except Exception as e:
        print(f"[STRESS] Lead spawn error: {e}")
        return None


def spawn_npc_traffic(interface, count=3, existing_npcs=None):
    """Spawns random NPC vehicles around the map."""
    if existing_npcs is None:
        existing_npcs = []

    spawned = []
    spawn_points = interface.map.get_spawn_points()
    random.shuffle(spawn_points)

    bp_lib = interface.world.get_blueprint_library()
    vehicle_bps = bp_lib.filter('vehicle.*')

    for sp in spawn_points[:count * 2]:  # try double to account for failures
        if len(spawned) >= count:
            break

        bp = random.choice(vehicle_bps)
        # Skip bikes/motorcycles for stability
        if int(bp.get_attribute('number_of_wheels')) < 4:
            continue

        actor = interface.world.try_spawn_actor(bp, sp)
        if actor:
            actor.set_autopilot(True, interface.tm.get_port())
            interface.tm.vehicle_percentage_speed_difference(actor, random.uniform(20.0, 80.0))
            spawned.append(actor)

    return spawned


def cleanup_far_npcs(interface, npcs, max_dist=80.0):
    """Destroys NPCs that are too far behind the ego vehicle."""
    if not interface.ego_vehicle or not interface.ego_vehicle.is_alive:
        return npcs, 0

    ego_loc = interface.ego_vehicle.get_location()
    fwd = interface.ego_vehicle.get_transform().get_forward_vector()
    alive = []
    cleaned = 0

    for actor in npcs:
        if not actor.is_alive:
            continue
        vec = actor.get_location() - ego_loc
        dot = vec.x * fwd.x + vec.y * fwd.y
        dist = math.sqrt(vec.x**2 + vec.y**2)

        # Remove if far behind or very far in any direction
        if dot < -max_dist or dist > max_dist * 2:
            try:
                actor.destroy()
                cleaned += 1
            except:
                pass
        else:
            alive.append(actor)

    return alive, cleaned


def pick_random_destination(interface):
    """Returns a random spawn point location as destination."""
    spawn_points = interface.map.get_spawn_points()
    return random.choice(spawn_points).location


# ==============================================================================
# -- Main Stress Test Loop -----------------------------------------------------
# ==============================================================================

def main():
    print("=" * 70)
    print("  INDIA-MODE AUTONOMY STACK — INFINITE STRESS TEST")
    print("  Press Ctrl+C to stop. Report will be printed on exit.")
    print("=" * 70)

    interface = CarlaInterface()
    stats = StressStats()
    lead_vehicle = None
    lead_is_braked = False  # Track if CURRENT lead is in a brake event
    npc_actors = []
    weather_idx = 0
    warmup_done = False

    try:
        # ── 1. Setup ──────────────────────────────────────────────────────
        interface.connect()
        interface.setup_world()

        # Start with clear weather
        weather_name, weather_params = WEATHER_CYCLE[0]
        interface.world.set_weather(weather_params)
        print(f"[STRESS] Initial weather: {weather_name}")

        # ── 2. Spawn Ego ──────────────────────────────────────────────────
        interface.spawn_ego_vehicle(spawn_point_index=0)
        interface.spawn_sensors()
        interface.world.tick()

        # ── 3. NO lead/NPC spawn yet — warmup first ──────────────────────
        # Let the ego cruise alone for WARMUP_FRAMES to build speed
        print(f"[STRESS] Warmup: ego will cruise alone for {WARMUP_FRAMES} frames...")

        # ── 4. Generate Initial Route ─────────────────────────────────────
        destination = pick_random_destination(interface)
        route = interface.generate_route(interface.ego_vehicle.get_location(), destination)
        print(f"[STRESS] Initial route: {len(route)} waypoints")

        # ── 5. Initialize Autonomy Modules ────────────────────────────────
        behavior_planner = BehaviorPlanner(interface)
        behavior_planner.global_route = route
        motion_planner = MotionPlanner(interface.map)
        controller = VehicleController()

        prev_state = None

        # ==================================================================
        # INFINITE LOOP
        # ==================================================================
        print("\n[STRESS] >>> STARTING INFINITE LOOP <<<\n")

        while True:
            frame_start = time.time()

            # ── A. Perception / Tick ──────────────────────────────────────
            data = interface.get_data()
            if not data:
                stats.sensor_drops += 1
                continue

            stats.total_frames += 1
            frame = data['frame']

            ego_transform = data['ego_transform']
            ego_vel = data['ego_velocity']
            ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)
            ego_speed_kph = ego_speed * 3.6

            if ego_speed_kph > stats.max_speed_kph:
                stats.max_speed_kph = ego_speed_kph

            # ── B. Sensor Health Check ────────────────────────────────────
            for sensor_name in ['lidar', 'camera_front', 'radar', 'gnss', 'imu']:
                if data.get(sensor_name) is None:
                    stats.sensor_drops += 1
                    if stats.total_frames % REPORT_INTERVAL == 0:
                        print(f"[STRESS] WARNING: Sensor '{sensor_name}' dropped at frame {frame}")

            # ── C. Stuck Detection (skip during warmup) ────────────────────
            if ego_speed < STUCK_SPEED_THRESHOLD and warmup_done:
                stats.stuck_counter += 1
                if stats.stuck_counter >= STUCK_FRAME_LIMIT:
                    stats.stuck_events += 1
                    stats.stuck_counter = 0
                    stats.errors.append((frame, f"Vehicle stuck! Speed={ego_speed_kph:.1f} km/h for {STUCK_FRAME_LIMIT} frames"))
                    print(f"[STRESS] *** STUCK DETECTED at frame {frame}! Clearing blockers & re-routing... ***")

                    # Recovery Step 1: Destroy lead vehicle that may be blocking
                    if lead_vehicle and lead_vehicle.is_alive:
                        try:
                            lead_vehicle.destroy()
                            print("[STRESS] Destroyed blocking lead vehicle.")
                        except:
                            pass
                        lead_vehicle = None
                        lead_is_braked = False

                    # Recovery Step 2: Destroy any NPCs within 20m ahead
                    ego_loc = ego_transform.location
                    fwd = ego_transform.get_forward_vector()
                    surviving_npcs = []
                    for actor in npc_actors:
                        if not actor.is_alive:
                            continue
                        vec = actor.get_location() - ego_loc
                        dot = vec.x * fwd.x + vec.y * fwd.y
                        dist = math.sqrt(vec.x**2 + vec.y**2)
                        if dot > 0 and dist < 20.0:
                            try:
                                actor.destroy()
                                print(f"[STRESS] Destroyed blocking NPC {actor.id}")
                            except:
                                pass
                        else:
                            surviving_npcs.append(actor)
                    npc_actors = surviving_npcs

                    # Recovery Step 3: Re-route
                    destination = pick_random_destination(interface)
                    route = interface.generate_route(ego_transform.location, destination)
                    behavior_planner.global_route = route
                    behavior_planner.state = BehaviorState.CRUISE

                    # Recovery Step 4: Respawn lead far ahead
                    lead_vehicle = spawn_lead_vehicle(interface, 40.0)
                    if lead_vehicle:
                        stats.lead_respawns += 1
            else:
                stats.stuck_counter = 0

            # ── D. Route Cycling (Infinite Roaming) ───────────────────────
            if behavior_planner.global_route:
                d_goal = ego_transform.location.distance(
                    behavior_planner.global_route[-1][0].transform.location
                )
                if d_goal < 10.0:
                    stats.routes_completed += 1
                    destination = pick_random_destination(interface)
                    route = interface.generate_route(ego_transform.location, destination)
                    behavior_planner.global_route = route
                    print(f"[STRESS] Route #{stats.routes_completed} complete! New route: {len(route)} wps")
            elif stats.total_frames % 100 == 0:
                # Empty route fallback
                destination = pick_random_destination(interface)
                route = interface.generate_route(ego_transform.location, destination)
                behavior_planner.global_route = route

            # ── E. Warmup Check ────────────────────────────────────────────
            if not warmup_done and stats.total_frames >= WARMUP_FRAMES:
                warmup_done = True
                print(f"[STRESS] Warmup complete! Speed: {ego_speed_kph:.1f} km/h. Spawning obstacles...")

                # Now spawn lead vehicle far ahead
                lead_vehicle = spawn_lead_vehicle(interface, 35.0)
                if lead_vehicle:
                    stats.lead_respawns += 1

                # Spawn initial NPCs
                initial_npcs = spawn_npc_traffic(interface, count=5)
                npc_actors.extend(initial_npcs)
                stats.npcs_spawned += len(initial_npcs)
                print(f"[STRESS] Spawned {len(initial_npcs)} initial NPCs")

            # ── F. Weather Cycling ────────────────────────────────────────
            if frame % WEATHER_CHANGE_INTERVAL == 0 and warmup_done:
                weather_idx = (weather_idx + 1) % len(WEATHER_CYCLE)
                weather_name, weather_params = WEATHER_CYCLE[weather_idx]
                interface.world.set_weather(weather_params)
                stats.weather_changes += 1
                print(f"[STRESS] Weather changed to: {weather_name} (change #{stats.weather_changes})")

            # ── G. NPC Traffic Spawning ───────────────────────────────────
            if frame % NPC_SPAWN_INTERVAL == 0 and warmup_done:
                if len(npc_actors) < MAX_NPC_COUNT:
                    batch_size = min(3, MAX_NPC_COUNT - len(npc_actors))
                    new_npcs = spawn_npc_traffic(interface, count=batch_size, existing_npcs=npc_actors)
                    npc_actors.extend(new_npcs)
                    stats.npcs_spawned += len(new_npcs)
                    print(f"[STRESS] Spawned {len(new_npcs)} NPCs (total active: {len(npc_actors)})")

            # ── H. NPC Cleanup ────────────────────────────────────────────
            if frame % 300 == 0:
                npc_actors, cleaned = cleanup_far_npcs(interface, npc_actors)
                stats.npcs_cleaned += cleaned

            # ── I. Lead Vehicle Respawn ───────────────────────────────────
            if frame % LEAD_RESPAWN_INTERVAL == 0 and warmup_done and stats.total_frames > WARMUP_FRAMES + 10:
                # Destroy old lead
                if lead_vehicle and lead_vehicle.is_alive:
                    try:
                        lead_vehicle.destroy()
                    except:
                        pass

                lead_vehicle = spawn_lead_vehicle(interface, random.uniform(30.0, 50.0))
                stats.lead_respawns += 1

                # Random brake event (only after car has been driving a while)
                lead_is_braked = False  # Reset for new lead
                if lead_vehicle and stats.routes_completed >= 1 and random.random() < LEAD_BRAKE_CHANCE:
                    stats.lead_brake_events += 1
                    lead_is_braked = True
                    print(f"[STRESS] !!! Lead vehicle BRAKE EVENT #{stats.lead_brake_events} !!!")
                    lead_vehicle.set_autopilot(False, interface.tm.get_port())
                    lead_vehicle.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

            # Release braked lead after ~5 seconds (100 frames at 20fps)
            if lead_vehicle and warmup_done and lead_is_braked:
                try:
                    ctrl = lead_vehicle.get_control()
                    if not ctrl.throttle and frame % LEAD_RESPAWN_INTERVAL > 100 and frame % LEAD_RESPAWN_INTERVAL < 120:
                        if lead_vehicle.is_alive:
                            lead_vehicle.apply_control(carla.VehicleControl(brake=0.0, hand_brake=False))
                            lead_vehicle.set_autopilot(True, interface.tm.get_port())
                            lead_is_braked = False
                            print("[STRESS] Released braked lead vehicle.")
                except:
                    pass

            # ── I. Planning ───────────────────────────────────────────────
            try:
                state, target_speed = behavior_planner.plan(
                    ego_transform,
                    ego_speed,
                    data['nearby_vehicles']
                )
                stats.record_state(state)

                # Track state transitions
                if prev_state != state:
                    if state == BehaviorState.EMERGENCY_STOP:
                        stats.emergency_stops += 1
                    elif state == BehaviorState.PREPARE_OVERTAKE:
                        stats.overtakes_initiated += 1
                    elif state == BehaviorState.LANE_CHANGE:
                        stats.lane_changes += 1
                    prev_state = state

            except Exception as e:
                stats.errors.append((frame, f"Planner error: {e}"))
                state = BehaviorState.CRUISE
                target_speed = config.TARGET_SPEED_MPS
                if stats.total_frames % REPORT_INTERVAL == 0:
                    print(f"[STRESS] Planner error at frame {frame}: {e}")

            # ── J. Motion Planning ────────────────────────────────────────
            try:
                waypoints = motion_planner.generate_path(
                    ego_transform,
                    state,
                    behavior_planner.target_lane_wp,
                    global_route=behavior_planner.global_route,
                    overtake_ticks=behavior_planner.overtake_ticks
                )
            except Exception as e:
                stats.errors.append((frame, f"Motion planner error: {e}"))
                waypoints = []

            # ── K. Control ────────────────────────────────────────────────
            try:
                control_cmd = controller.run_step(
                    ego_speed,
                    ego_transform,
                    target_speed,
                    waypoints
                )
                interface.apply_control(control_cmd)
            except Exception as e:
                stats.errors.append((frame, f"Controller error: {e}"))
                # Emergency stop fallback
                interface.apply_control({
                    'throttle': 0.0, 'brake': 1.0, 'steer': 0.0
                })

            # ── L. Status Report ──────────────────────────────────────────
            frame_time = time.time() - frame_start

            if stats.total_frames % REPORT_INTERVAL == 0:
                elapsed = stats.elapsed()
                fps = stats.total_frames / max(elapsed, 1)
                weather_name_now = WEATHER_CYCLE[weather_idx][0]

                print(
                    f"[STRESS] F:{frame} | "
                    f"T:{int(elapsed)}s | "
                    f"FPS:{fps:.1f} | "
                    f"State:{state.name} | "
                    f"Spd:{ego_speed_kph:.1f}kph | "
                    f"Tgt:{target_speed*3.6:.1f}kph | "
                    f"Routes:{stats.routes_completed} | "
                    f"NPCs:{len(npc_actors)} | "
                    f"Weather:{weather_name_now} | "
                    f"Errs:{len(stats.errors)} | "
                    f"Stuck:{stats.stuck_events}"
                )

    except KeyboardInterrupt:
        print("\n[STRESS] User interrupted (Ctrl+C).")
    except Exception as e:
        stats.errors.append((stats.total_frames, f"FATAL: {e}"))
        print(f"\n[STRESS] FATAL ERROR: {e}")
        traceback.print_exc()
    finally:
        # ── Cleanup ───────────────────────────────────────────────────────
        print("\n[STRESS] Cleaning up all actors...")

        if lead_vehicle and lead_vehicle.is_alive:
            try:
                lead_vehicle.destroy()
            except:
                pass

        for actor in npc_actors:
            if actor.is_alive:
                try:
                    actor.destroy()
                except:
                    pass

        interface.cleanup()

        # ── Final Report ──────────────────────────────────────────────────
        stats.report()


if __name__ == '__main__':
    main()
