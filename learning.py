"""
Imitation Learning Module — adapted from RAIL/learning_module.py.

Expanded from the RAIL skeleton (2→64→2 MLP) into a production-ready module with:
- Richer input features (ego speed, heading error, nearest obstacles)
- Data collection during autopilot driving
- Training loop
- Inference with blending against classical controller output
"""

import os
import math
import numpy as np
import config

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class ImitationModel(nn.Module if TORCH_AVAILABLE else object):
    """
    Neural network that maps driving state to control commands.
    Input:  [ego_speed, heading_error, nearest_obs_dist, nearest_obs_angle, target_speed]
    Output: [throttle_brake, steer]  (throttle_brake: positive=throttle, negative=brake)
    """
    def __init__(self):
        if not TORCH_AVAILABLE:
            return
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(config.LEARNING_INPUT_DIM, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 2),
            nn.Tanh()  # Output bounded [-1, 1]
        )

    def forward(self, x):
        return self.net(x)


class DataCollector:
    """
    Records (state, action) pairs during driving for later training.
    Saves to a numpy file.
    """
    def __init__(self, save_path=None):
        self.save_path = save_path or os.path.join(
            os.path.dirname(__file__), 'collected_data.npz'
        )
        self.states = []
        self.actions = []

    def record(self, state_vector, control_cmd):
        """
        state_vector: np array [ego_speed, heading_error, obs_dist, obs_angle, target_speed]
        control_cmd: dict with 'throttle', 'brake', 'steer'
        """
        throttle_brake = control_cmd.get('throttle', 0) - control_cmd.get('brake', 0)
        steer = control_cmd.get('steer', 0)
        self.states.append(state_vector)
        self.actions.append([throttle_brake, steer])

    def save(self):
        if not self.states:
            return
        np.savez(
            self.save_path,
            states=np.array(self.states, dtype=np.float32),
            actions=np.array(self.actions, dtype=np.float32)
        )
        print(f"Saved {len(self.states)} samples to {self.save_path}")

    def load(self):
        if os.path.exists(self.save_path):
            data = np.load(self.save_path)
            return data['states'], data['actions']
        return None, None


class ImitationLearner:
    """
    Manages training and inference of the ImitationModel.
    """
    def __init__(self):
        if not TORCH_AVAILABLE:
            print("WARNING: PyTorch not available. Learning module disabled.")
            self.enabled = False
            return

        self.enabled = True
        self.model = ImitationModel()
        self.optimizer = optim.Adam(self.model.parameters(), lr=config.LEARNING_LR)
        self.loss_fn = nn.MSELoss()
        self.collector = DataCollector()
        self.model_path = os.path.join(os.path.dirname(__file__), 'imitation_model.pt')

        # Try loading a pretrained model
        if os.path.exists(self.model_path):
            self.model.load_state_dict(torch.load(self.model_path, weights_only=True))
            self.model.eval()
            print(f"Loaded imitation model from {self.model_path}")

    def extract_state(self, ego_speed, ego_transform, target_speed, nearby_vehicles):
        """
        Builds the state vector from driving context.
        Returns np.array of shape (LEARNING_INPUT_DIM,)
        """
        # Heading error: angle between ego heading and direction to nearest route point
        heading_error = 0.0  # Simplified — full version would use route

        # Nearest obstacle distance and angle
        nearest_dist = 100.0
        nearest_angle = 0.0
        ego_loc = ego_transform.location
        yaw_rad = math.radians(ego_transform.rotation.yaw)

        for v in nearby_vehicles:
            v_loc = v.get_location()
            dx = v_loc.x - ego_loc.x
            dy = v_loc.y - ego_loc.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < nearest_dist:
                nearest_dist = dist
                # Angle relative to ego heading
                angle = math.atan2(dy, dx) - yaw_rad
                nearest_angle = math.atan2(math.sin(angle), math.cos(angle))  # Normalize

        return np.array([
            ego_speed,
            heading_error,
            nearest_dist,
            nearest_angle,
            target_speed
        ], dtype=np.float32)

    def record_step(self, state_vector, control_cmd):
        """Record a single step during data collection mode."""
        if self.enabled:
            self.collector.record(state_vector, control_cmd)

    def save_data(self):
        if self.enabled:
            self.collector.save()

    def train(self, epochs=None):
        """Train the model on collected data."""
        if not self.enabled:
            return

        epochs = epochs or config.LEARNING_EPOCHS
        states, actions = self.collector.load()
        if states is None:
            print("No training data found.")
            return

        dataset = torch.utils.data.TensorDataset(
            torch.from_numpy(states),
            torch.from_numpy(actions)
        )
        loader = torch.utils.data.DataLoader(dataset, batch_size=64, shuffle=True)

        self.model.train()
        for epoch in range(epochs):
            total_loss = 0
            for batch_states, batch_actions in loader:
                pred = self.model(batch_states)
                loss = self.loss_fn(pred, batch_actions)
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                total_loss += loss.item()

            if (epoch + 1) % 10 == 0:
                print(f"Epoch {epoch+1}/{epochs}, Loss: {total_loss/len(loader):.4f}")

        # Save trained model
        torch.save(self.model.state_dict(), self.model_path)
        self.model.eval()
        print(f"Model saved to {self.model_path}")

    def predict(self, state_vector):
        """
        Run inference. Returns blended control suggestion.
        Output: dict with 'throttle', 'brake', 'steer'
        """
        if not self.enabled:
            return None

        self.model.eval()
        with torch.no_grad():
            inp = torch.from_numpy(state_vector).unsqueeze(0)
            out = self.model(inp).squeeze(0).numpy()

        throttle_brake = float(out[0])
        steer = float(out[1])

        return {
            'throttle': max(0.0, throttle_brake),
            'brake': max(0.0, -throttle_brake),
            'steer': np.clip(steer, -1.0, 1.0)
        }

    @staticmethod
    def blend_controls(classical_cmd, learned_cmd, alpha=None):
        """
        Blends classical PID/PurePursuit output with learned model output.
        alpha: weight for learned model (0 = full classical, 1 = full learned)
        """
        if learned_cmd is None:
            return classical_cmd

        alpha = alpha if alpha is not None else config.LEARNING_BLEND_ALPHA
        blended = {}
        for key in ['throttle', 'brake', 'steer']:
            c_val = classical_cmd.get(key, 0.0)
            l_val = learned_cmd.get(key, 0.0)
            blended[key] = (1.0 - alpha) * c_val + alpha * l_val

        return blended
