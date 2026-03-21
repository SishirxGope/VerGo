"""
train.py — One-command training pipeline for the RAIL Imitation Learning module.

Usage:
    python train.py collect              # Step 1: Collect driving data from CARLA autopilot
    python train.py collect --frames 5000   # Collect with custom frame count
    python train.py train                # Step 2: Train model on collected data
    python train.py train --epochs 200      # Train with custom epochs
    python train.py all                  # Collect + Train in one go
    python train.py info                 # Show data/model status
"""

import sys
import os
import time
import math
import random
import argparse

# Ensure module imports work
sys.path.insert(0, os.path.dirname(__file__))

import config

def collect_data(max_frames=6000, lead_dist=15.0):
    """
    Runs the full autonomy stack in CARLA with data collection enabled.
    Records (state, action) pairs from the classical controller for imitation learning.
    """
    import carla
    from carla_interface import CarlaInterface
    from planner import BehaviorPlanner, MotionPlanner
    from controller import VehicleController
    from learning import ImitationLearner

    print("=" * 60)
    print("  RAIL Training — Phase 1: Data Collection")
    print(f"  Target: {max_frames} frames")
    print("=" * 60)

    interface = CarlaInterface()
    learner = ImitationLearner()
    lead_vehicle = None

    if not learner.enabled:
        print("ERROR: PyTorch not available. Cannot collect data.")
        return False

    try:
        # --- Setup ---
        interface.connect()
        interface.setup_world()
        interface.spawn_ego_vehicle(spawn_point_index=0)
        interface.spawn_sensors()
        interface.world.tick()

        # Spawn a lead vehicle for interesting driving behavior
        ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
        next_wps = ego_wp.next(lead_dist)
        if next_wps:
            lead_trans = next_wps[0].transform
            lead_trans.location.z += 1.0
            bp = interface.world.get_blueprint_library().find('vehicle.nissan.patrol')
            lead_vehicle = interface.world.try_spawn_actor(bp, lead_trans)
            if lead_vehicle:
                lead_vehicle.set_autopilot(True, interface.tm.get_port())
                interface.tm.vehicle_percentage_speed_difference(lead_vehicle, 50.0)
                print(f"Lead vehicle spawned (id={lead_vehicle.id})")

        # Generate route
        spawn_points = interface.map.get_spawn_points()
        destination = random.choice(spawn_points).location
        route = interface.generate_route(interface.ego_vehicle.get_location(), destination)
        print(f"Route: {len(route)} waypoints to destination")

        # Init modules
        behavior_planner = BehaviorPlanner(interface)
        behavior_planner.global_route = route
        motion_planner = MotionPlanner(interface.map)
        controller = VehicleController()

        # --- Collection Loop ---
        collected = 0
        start_time = time.time()
        reroute_counter = 0

        for frame_idx in range(max_frames):
            data = interface.get_data()
            if not data:
                continue

            ego_transform = data['ego_transform']
            ego_vel = data['ego_velocity']
            ego_speed = math.sqrt(ego_vel.x ** 2 + ego_vel.y ** 2)
            nearby = data['nearby_vehicles']

            # Re-route if destination reached
            if behavior_planner.global_route:
                d_goal = ego_transform.location.distance(
                    behavior_planner.global_route[-1][0].transform.location
                )
                if d_goal < 5.0:
                    reroute_counter += 1
                    destination = random.choice(spawn_points).location
                    route = interface.generate_route(ego_transform.location, destination)
                    behavior_planner.global_route = route
                    behavior_planner.state = behavior_planner.state.__class__(1)  # Reset to CRUISE
                    print(f"  Re-routed #{reroute_counter} ({len(route)} wps)")

            # Planning
            state, target_speed = behavior_planner.plan(ego_transform, ego_speed, nearby)
            waypoints = motion_planner.generate_path(
                ego_transform, state,
                behavior_planner.target_lane_wp,
                global_route=behavior_planner.global_route
            )

            # Control (classical — this is the expert we learn from)
            control_cmd = controller.run_step(ego_speed, ego_transform, target_speed, waypoints)

            # Record
            state_vec = learner.extract_state(ego_speed, ego_transform, target_speed, nearby)
            learner.record_step(state_vec, control_cmd)
            collected += 1

            # Actuation
            interface.apply_control(control_cmd)

            # Progress
            if frame_idx % 500 == 0 and frame_idx > 0:
                elapsed = time.time() - start_time
                fps = frame_idx / elapsed
                pct = 100.0 * frame_idx / max_frames
                print(f"  [{pct:5.1f}%] Frame {frame_idx}/{max_frames} | "
                      f"Samples: {collected} | {fps:.0f} FPS | "
                      f"State: {state.name} | Speed: {ego_speed * 3.6:.1f} km/h")

            # Respawn lead periodically for variety
            if frame_idx % 1500 == 0 and frame_idx > 0:
                if lead_vehicle and lead_vehicle.is_alive:
                    lead_vehicle.destroy()
                ego_wp = interface.map.get_waypoint(interface.ego_vehicle.get_location())
                next_wps = ego_wp.next(lead_dist)
                if next_wps:
                    lead_trans = next_wps[0].transform
                    lead_trans.location.z += 1.0
                    bp = interface.world.get_blueprint_library().find('vehicle.nissan.patrol')
                    lead_vehicle = interface.world.try_spawn_actor(bp, lead_trans)
                    if lead_vehicle:
                        lead_vehicle.set_autopilot(True, interface.tm.get_port())
                        interface.tm.vehicle_percentage_speed_difference(lead_vehicle, random.uniform(30, 70))

        # --- Save ---
        elapsed = time.time() - start_time
        learner.save_data()

        print("=" * 60)
        print(f"  Collection Complete!")
        print(f"  Samples: {collected}")
        print(f"  Duration: {elapsed:.0f}s ({elapsed / 60:.1f} min)")
        print(f"  Avg FPS: {max_frames / elapsed:.0f}")
        print(f"  Re-routes: {reroute_counter}")
        print(f"  Data saved to: {learner.collector.save_path}")
        print("=" * 60)
        return True

    except KeyboardInterrupt:
        print("\nInterrupted by user. Saving collected data...")
        learner.save_data()
        return True
    except Exception as e:
        print(f"\nError during collection: {e}")
        import traceback
        traceback.print_exc()
        learner.save_data()
        return False
    finally:
        if lead_vehicle and lead_vehicle.is_alive:
            lead_vehicle.destroy()
        interface.cleanup()


def train_model(epochs=None, batch_size=64):
    """
    Trains the ImitationModel on previously collected data.
    No CARLA connection needed.
    """
    from learning import ImitationLearner, DataCollector, TORCH_AVAILABLE

    if not TORCH_AVAILABLE:
        print("ERROR: PyTorch not installed. Run: pip install torch")
        return False

    import torch

    epochs = epochs or config.LEARNING_EPOCHS

    print("=" * 60)
    print("  RAIL Training — Phase 2: Model Training")
    print(f"  Epochs: {epochs} | Batch: {batch_size} | LR: {config.LEARNING_LR}")
    print("=" * 60)

    # Load data
    collector = DataCollector()
    states, actions = collector.load()
    if states is None:
        print("ERROR: No training data found.")
        print(f"  Expected at: {collector.save_path}")
        print("  Run 'python train.py collect' first.")
        return False

    print(f"  Loaded {len(states)} samples")
    print(f"  State shape: {states.shape} | Action shape: {actions.shape}")
    print(f"  Speed range: [{states[:, 0].min():.1f}, {states[:, 0].max():.1f}] m/s")
    print(f"  Obs dist range: [{states[:, 2].min():.1f}, {states[:, 2].max():.1f}] m")

    # Split train/val (90/10)
    n = len(states)
    indices = torch.randperm(n)
    split = int(0.9 * n)
    train_idx, val_idx = indices[:split], indices[split:]

    train_states = torch.from_numpy(states[train_idx])
    train_actions = torch.from_numpy(actions[train_idx])
    val_states = torch.from_numpy(states[val_idx])
    val_actions = torch.from_numpy(actions[val_idx])

    train_loader = torch.utils.data.DataLoader(
        torch.utils.data.TensorDataset(train_states, train_actions),
        batch_size=batch_size, shuffle=True
    )

    print(f"  Train: {len(train_states)} | Val: {len(val_states)}")
    print("-" * 60)

    # Model
    learner = ImitationLearner()
    model = learner.model
    optimizer = learner.optimizer
    loss_fn = learner.loss_fn

    best_val_loss = float('inf')
    best_epoch = 0
    start_time = time.time()

    model.train()
    for epoch in range(epochs):
        # Train
        total_train_loss = 0
        batches = 0
        for batch_s, batch_a in train_loader:
            pred = model(batch_s)
            loss = loss_fn(pred, batch_a)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_train_loss += loss.item()
            batches += 1
        avg_train = total_train_loss / batches

        # Validate
        model.eval()
        with torch.no_grad():
            val_pred = model(val_states)
            val_loss = loss_fn(val_pred, val_actions).item()
        model.train()

        # Track best
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_epoch = epoch + 1
            torch.save(model.state_dict(), learner.model_path)

        # Log
        if (epoch + 1) % 10 == 0 or epoch == 0:
            elapsed = time.time() - start_time
            print(f"  Epoch {epoch + 1:4d}/{epochs} | "
                  f"Train: {avg_train:.6f} | Val: {val_loss:.6f} | "
                  f"Best: {best_val_loss:.6f} (ep {best_epoch}) | "
                  f"{elapsed:.0f}s")

    elapsed = time.time() - start_time
    print("-" * 60)
    print(f"  Training Complete!")
    print(f"  Best val loss: {best_val_loss:.6f} at epoch {best_epoch}")
    print(f"  Model saved to: {learner.model_path}")
    print(f"  Duration: {elapsed:.0f}s")
    print("=" * 60)

    # Quick inference test
    model.eval()
    with torch.no_grad():
        sample = torch.from_numpy(states[:1])
        pred = model(sample).numpy()[0]
        print(f"\n  Quick test — Input: speed={states[0, 0]:.1f} m/s, obs={states[0, 2]:.1f}m")
        print(f"  Predicted: throttle/brake={pred[0]:.3f}, steer={pred[1]:.3f}")
        print(f"  Actual:    throttle/brake={actions[0, 0]:.3f}, steer={actions[0, 1]:.3f}")

    return True


def show_info():
    """Shows status of collected data and trained model."""
    from learning import DataCollector, TORCH_AVAILABLE

    print("=" * 60)
    print("  RAIL Training — Status")
    print("=" * 60)

    collector = DataCollector()

    # Data status
    if os.path.exists(collector.save_path):
        states, actions = collector.load()
        size_mb = os.path.getsize(collector.save_path) / (1024 * 1024)
        print(f"  Data file: {collector.save_path}")
        print(f"  Samples:   {len(states)}")
        print(f"  File size: {size_mb:.1f} MB")
        print(f"  Features:  {states.shape[1]} (speed, heading_err, obs_dist, obs_angle, target_speed)")
        print(f"  Speed range:    [{states[:, 0].min():.2f}, {states[:, 0].max():.2f}] m/s")
        print(f"  Obs dist range: [{states[:, 2].min():.2f}, {states[:, 2].max():.2f}] m")
    else:
        print(f"  Data file: NOT FOUND ({collector.save_path})")
        print("  Run 'python train.py collect' to generate training data.")

    print()

    # Model status
    model_path = os.path.join(os.path.dirname(__file__), 'imitation_model.pt')
    if os.path.exists(model_path):
        size_kb = os.path.getsize(model_path) / 1024
        print(f"  Model file: {model_path}")
        print(f"  File size:  {size_kb:.1f} KB")
        if TORCH_AVAILABLE:
            import torch
            from learning import ImitationModel
            model = ImitationModel()
            model.load_state_dict(torch.load(model_path, weights_only=True))
            params = sum(p.numel() for p in model.parameters())
            print(f"  Parameters: {params:,}")
            print(f"  Architecture: {config.LEARNING_INPUT_DIM}->128->64->2")
        print("  Status: READY (set LEARNING_ENABLED=True in config.py to use)")
    else:
        print(f"  Model file: NOT FOUND ({model_path})")
        print("  Run 'python train.py train' to train the model.")

    print()
    print(f"  PyTorch: {'Available' if TORCH_AVAILABLE else 'NOT INSTALLED (pip install torch)'}")
    print(f"  Config — LEARNING_ENABLED: {config.LEARNING_ENABLED}")
    print(f"  Config — LEARNING_COLLECT_DATA: {config.LEARNING_COLLECT_DATA}")
    print(f"  Config — LEARNING_BLEND_ALPHA: {config.LEARNING_BLEND_ALPHA}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="RAIL Imitation Learning — Training Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python train.py collect               Collect driving data from CARLA (6000 frames)
  python train.py collect --frames 10000   Collect more data
  python train.py train                 Train model on collected data
  python train.py train --epochs 200    Train with more epochs
  python train.py all                   Collect data + train model
  python train.py info                  Show data/model status
        """
    )
    parser.add_argument('command', choices=['collect', 'train', 'all', 'info'],
                        help="collect=gather data, train=train model, all=both, info=status")
    parser.add_argument('--frames', type=int, default=6000,
                        help="Number of frames to collect (default: 6000)")
    parser.add_argument('--epochs', type=int, default=None,
                        help=f"Training epochs (default: {config.LEARNING_EPOCHS})")
    parser.add_argument('--batch-size', type=int, default=64,
                        help="Training batch size (default: 64)")

    args = parser.parse_args()

    if args.command == 'info':
        show_info()
    elif args.command == 'collect':
        collect_data(max_frames=args.frames)
    elif args.command == 'train':
        train_model(epochs=args.epochs, batch_size=args.batch_size)
    elif args.command == 'all':
        print("Running full pipeline: Collect → Train\n")
        ok = collect_data(max_frames=args.frames)
        if ok:
            print()
            train_model(epochs=args.epochs, batch_size=args.batch_size)
        else:
            print("Collection failed. Skipping training.")


if __name__ == '__main__':
    main()
