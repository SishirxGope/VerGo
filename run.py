"""
run.py — Unified entry point for the CARLA Autonomy Stack + RAIL modules.

One command to do everything:

    python run.py drive                  # Normal driving (camera + grid enabled)
    python run.py drive --no-rail        # Drive with RAIL modules disabled
    python run.py collect                # Collect imitation learning data
    python run.py collect --frames 10000 # Collect with custom frame count
    python run.py train                  # Train imitation model (no CARLA needed)
    python run.py train --epochs 200     # Train with custom epochs
    python run.py pipeline               # Full pipeline: collect -> train -> drive with learning
    python run.py test                   # Run RAIL integration tests
    python run.py test --offline         # Test learning module without CARLA
    python run.py test camera grid       # Test specific modules
    python run.py info                   # Show system status
"""

import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(__file__))


def cmd_drive(args):
    """Run the main autonomy stack."""
    import config

    if args.no_rail:
        config.CAMERA_PERCEPTION_ENABLED = False
        config.LOCAL_GRID_ENABLED = False
        config.LEARNING_ENABLED = False
        print("RAIL modules: DISABLED")
    else:
        print("RAIL modules: camera={}, grid={}, learning={}".format(
            config.CAMERA_PERCEPTION_ENABLED,
            config.LOCAL_GRID_ENABLED,
            config.LEARNING_ENABLED
        ))

    from main import main
    main()


def cmd_collect(args):
    """Collect training data."""
    from train import collect_data
    collect_data(max_frames=args.frames)


def cmd_train(args):
    """Train the imitation model."""
    from train import train_model
    train_model(epochs=args.epochs, batch_size=args.batch_size)


def cmd_pipeline(args):
    """Full pipeline: collect -> train -> enable learning -> drive."""
    import config
    from train import collect_data, train_model

    print("=" * 60)
    print("  Full RAIL Pipeline: Collect -> Train -> Drive")
    print("=" * 60)

    # Step 1: Collect
    print("\n--- Step 1/3: Collecting driving data ---\n")
    ok = collect_data(max_frames=args.frames)
    if not ok:
        print("Collection failed. Aborting pipeline.")
        return

    # Step 2: Train
    print("\n--- Step 2/3: Training imitation model ---\n")
    ok = train_model(epochs=args.epochs, batch_size=args.batch_size)
    if not ok:
        print("Training failed. Aborting pipeline.")
        return

    # Step 3: Drive with learning enabled
    print("\n--- Step 3/3: Driving with learned model ---\n")
    config.LEARNING_ENABLED = True
    config.LEARNING_COLLECT_DATA = False
    print(f"LEARNING_ENABLED = True (alpha={config.LEARNING_BLEND_ALPHA})")

    from main import main
    main()


def cmd_test(args):
    """Run RAIL integration tests."""
    from test_rail import run_tests, test_learning_offline

    if args.offline:
        success = test_learning_offline()
    else:
        # Validate module names
        valid = {'camera', 'grid', 'learning'}
        for t in (args.modules or []):
            if t not in valid:
                print(f"Error: invalid test module '{t}' (choose from {', '.join(sorted(valid))})")
                sys.exit(1)
        tests = args.modules if args.modules else ['camera', 'grid', 'learning']
        success = run_tests(tests, max_frames=args.frames)

    sys.exit(0 if success else 1)


def cmd_info(args):
    """Show system status."""
    from train import show_info
    import config

    print("=" * 60)
    print("  CARLA Autonomy Stack + RAIL — System Status")
    print("=" * 60)
    print()
    print("  [Core Stack]")
    print(f"    CARLA:  {config.CARLA_HOST}:{config.CARLA_PORT}")
    print(f"    Vehicle: {config.VEHICLE_MODEL}")
    print(f"    Speed:   {config.TARGET_SPEED_KPH} km/h")
    print(f"    India Mode: drive_left={config.LEFT_LANE_DRIVING}, overtake_right={config.OVERTAKE_ON_RIGHT}")
    print()
    print("  [RAIL Modules]")
    print(f"    Camera Perception:  {'ON' if config.CAMERA_PERCEPTION_ENABLED else 'OFF'}")
    print(f"    Local Grid Planner: {'ON' if config.LOCAL_GRID_ENABLED else 'OFF'}")
    print(f"    Imitation Learning: {'ON' if config.LEARNING_ENABLED else 'OFF'} (alpha={config.LEARNING_BLEND_ALPHA})")
    print(f"    Data Collection:    {'ON' if config.LEARNING_COLLECT_DATA else 'OFF'}")
    print()

    # Delegate to train.py for detailed learning info
    show_info()


def main():
    parser = argparse.ArgumentParser(
        prog='run.py',
        description='CARLA Autonomy Stack + RAIL — Unified CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Quick Start:
  python run.py drive               Drive with RAIL modules
  python run.py pipeline            Collect data + train + drive with learning
  python run.py test                Run all RAIL integration tests

Training:
  python run.py collect             Gather expert driving data from CARLA
  python run.py train               Train imitation model on collected data
  python run.py info                Check data/model/config status
        """
    )
    subparsers = parser.add_subparsers(dest='command', help='Command to run')

    # drive
    p_drive = subparsers.add_parser('drive', help='Run the autonomy stack')
    p_drive.add_argument('--no-rail', action='store_true', help='Disable all RAIL modules')

    # collect
    p_collect = subparsers.add_parser('collect', help='Collect training data from CARLA')
    p_collect.add_argument('--frames', type=int, default=6000, help='Frames to collect (default: 6000)')

    # train
    p_train = subparsers.add_parser('train', help='Train imitation model (no CARLA needed)')
    p_train.add_argument('--epochs', type=int, default=None, help='Training epochs')
    p_train.add_argument('--batch-size', type=int, default=64, help='Batch size')

    # pipeline
    p_pipe = subparsers.add_parser('pipeline', help='Full pipeline: collect -> train -> drive')
    p_pipe.add_argument('--frames', type=int, default=6000, help='Frames to collect')
    p_pipe.add_argument('--epochs', type=int, default=None, help='Training epochs')
    p_pipe.add_argument('--batch-size', type=int, default=64, help='Batch size')

    # test
    p_test = subparsers.add_parser('test', help='Run RAIL integration tests')
    p_test.add_argument('modules', nargs='*',
                        help='Modules to test: camera, grid, learning (default: all)')
    p_test.add_argument('--frames', type=int, default=2000, help='Test frames')
    p_test.add_argument('--offline', action='store_true', help='Test learning without CARLA')

    # info
    subparsers.add_parser('info', help='Show system status')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    commands = {
        'drive': cmd_drive,
        'collect': cmd_collect,
        'train': cmd_train,
        'pipeline': cmd_pipeline,
        'test': cmd_test,
        'info': cmd_info,
    }
    commands[args.command](args)


if __name__ == '__main__':
    main()
