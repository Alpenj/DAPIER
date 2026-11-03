"""
Upgraded Step 2 Visualization — Waypoint Convergence Playback (Fixed Target)
=============================================================================
- Loads the trained model from './101_output/act_policy_fixed.pt' (generated in Step 1)
- Fixes the destination (Target) to FIXED_TARGET [0.25, 0.0, 0.2]
- Animates how the predicted Action Chunk converges to the fixed target from random starts
- All Matplotlib text has been converted to English to prevent font rendering issues.
"""

import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa
import os
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────
# Configurations (Matching Step 1 specs)
# ─────────────────────────────────────────
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"
CHUNK_SIZE    = 20
WORKSPACE_MIN = np.array([0.0, -0.3, 0.0])
WORKSPACE_MAX = np.array([0.5,  0.3, 0.4])

# ★ Fixed Target & Model path (aligned with '101_act_pytorch_2.py')
FIXED_TARGET  = np.array([0.25, 0.0, 0.2]) 
MODEL_PATH    = "./101_output/act_policy_fixed.pt"

# Output Directory Setup
os.makedirs("./102_output", exist_ok=True)
SAVE_GIF      = "./102_output/viz_fixed_playback.gif"
SAVE_PNG      = "./102_output/viz_fixed_final_frame.png"

# ─────────────────────────────────────────
# Model Definition & Loader
# ─────────────────────────────────────────
class ActionChunkingPolicy(nn.Module):
    def __init__(self, obs_dim=6, chunk_dim=CHUNK_SIZE*3,
                 hidden=256, n_layers=3, dropout=0.1):
        super().__init__()
        layers, in_dim = [], obs_dim
        for _ in range(n_layers):
            layers += [nn.Linear(in_dim, hidden),
                       nn.LayerNorm(hidden), nn.ReLU(), nn.Dropout(dropout)]
            in_dim = hidden
        layers.append(nn.Linear(hidden, chunk_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)

    @torch.no_grad()
    def predict(self, start, target):
        obs = torch.tensor(np.concatenate([start, target]),
                           dtype=torch.float32).unsqueeze(0).to(DEVICE)
        out = self(obs).squeeze(0).cpu().numpy()
        return out.reshape(CHUNK_SIZE, 3)


def load_model(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"❌ Model file not found: '{path}'\n"
            f"Please run '101_act_pytorch_2.py' first to train and save the model!"
        )
    # Safely load the custom configuration dictionary
    ckpt  = torch.load(path, map_location=DEVICE, weights_only=False)
    model = ActionChunkingPolicy(**ckpt["config"])
    model.load_state_dict(ckpt["state_dict"])
    print(f" Successfully loaded Model Weight from: {path}")
    return model.to(DEVICE).eval()


# ─────────────────────────────────────────
# Trajectory Generation (For GT comparison)
# ─────────────────────────────────────────
def generate_trajectory(start, target, n_steps=CHUNK_SIZE, curvature=0.15):
    t       = np.linspace(0, 1, n_steps).reshape(-1, 1)
    linear  = start + t * (target - start)
    d       = target - start
    perp    = np.cross(d, [0,0,1]) if abs(d[2]) < 1e-6 else np.cross(d, [1,0,0])
    norm    = np.linalg.norm(perp)
    perp    = perp / norm if norm > 1e-6 else np.array([0,0,1])
    offset  = curvature * 4 * t * (1-t) * perp.reshape(1,3)
    return linear + offset


# ─────────────────────────────────────────
# Main Visualization Pipeline
# ─────────────────────────────────────────
def main():
    # ── 1. Model Load & Inference ───────────────────────
    model = load_model(MODEL_PATH)

    # Pick a random start point for testing
    rng    = np.random.default_rng(77)  # Seed fixed for reproducibility
    start  = rng.uniform(WORKSPACE_MIN, WORKSPACE_MAX)
    target = FIXED_TARGET  # Destination is strictly FIXED

    # Predict future 20 steps (Action Chunk) in a single forward pass
    chunk  = model.predict(start, target)          # shape: (20, 3)
    print("Prediction result chunk shape: ", chunk.shape, type(chunk), '\n', chunk)
    gt     = generate_trajectory(start, target, CHUNK_SIZE, curvature=0.15)  # shape: (20, 3)

    # Compute metric curves
    rmse_per_step = np.sqrt(np.sum((chunk - gt)**2, axis=1))  # (20,)
    dist_to_goal  = np.linalg.norm(chunk - target, axis=1)    # (20,)

    print(f"  Start (s) : {start.round(3)}")
    print(f"  Target (d): {target.round(3)}")
    print(f"  Total RMSE: {np.sqrt(np.mean((chunk-gt)**2)):.4f} m ({np.sqrt(np.mean((chunk-gt)**2))*1000:.2f} mm)")

    # ── 2. Figure Layout Setup ─────────────────────────
    fig = plt.figure(figsize=(13, 7), facecolor="#0d0d0d")
    fig.suptitle(
        "ACT (Fixed Target) — Waypoint Chunk Playback  |  Inference: 1 Time  →  Execution Steps: 20",
        color="white", fontsize=13, fontweight="bold", y=0.97
    )

    gs  = gridspec.GridSpec(2, 2,
                            left=0.05, right=0.97,
                            top=0.91, bottom=0.10,
                            wspace=0.30, hspace=0.45)

    # Left Big Panel: 3D Workspace
    ax3d = fig.add_subplot(gs[:, 0], projection='3d')
    ax3d.set_facecolor("#111111")
    for pane in [ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane]:
        pane.fill = False
        pane.set_edgecolor("#333333")

    ax3d.set_xlim(WORKSPACE_MIN[0], WORKSPACE_MAX[0])
    ax3d.set_ylim(WORKSPACE_MIN[1], WORKSPACE_MAX[1])
    ax3d.set_zlim(WORKSPACE_MIN[2], WORKSPACE_MAX[2])
    ax3d.set_xlabel("X (m)", color="#aaaaaa", labelpad=4)
    ax3d.set_ylabel("Y (m)", color="#aaaaaa", labelpad=4)
    ax3d.set_zlabel("Z (m)", color="#aaaaaa", labelpad=4)
    ax3d.tick_params(colors="#666666", labelsize=7)
    ax3d.set_title("3D Workspace (Fixed Target Convergence)", color="white", fontsize=10, pad=6)

    # GT Reference Path (Dashed gray line)
    ax3d.plot(gt[:,0], gt[:,1], gt[:,2],
              '--', color="#444444", linewidth=1.2, label="Ground Truth (Expert)")

    # Predicted Whole Path Preview (Cyan dotted line representing plan)
    ax3d.plot(chunk[:,0], chunk[:,1], chunk[:,2],
              ':', color="#00ffcc", linewidth=1.0, alpha=0.35,
              label="Predicted Chunk (Planned Path)")

    # Plot Start (Green Square) & Target (Red Star)
    ax3d.scatter(*start,  color="#44ff44", s=90,  marker="s",  zorder=5, label="Start Point (s)")
    ax3d.scatter(*target, color="#ff4444", s=180, marker="*",  zorder=5, label="Fixed Target (d)")

    ax3d.legend(loc="upper left", fontsize=7,
                facecolor="#1a1a1a", edgecolor="#444", labelcolor="white")

    # Right Top Panel: Euclidean Distance to Fixed Target
    ax_dist = fig.add_subplot(gs[0, 1])
    ax_dist.set_facecolor("#111111")
    ax_dist.plot(range(CHUNK_SIZE), dist_to_goal,
                 color="#888888", linewidth=1.0, alpha=0.4)
    ax_dist.set_xlim(-0.5, CHUNK_SIZE - 0.5)
    ax_dist.set_ylim(0, dist_to_goal.max() * 1.15)
    ax_dist.set_xlabel("Waypoint Index", color="#aaaaaa", fontsize=9)
    ax_dist.set_ylabel("Distance to Target (m)", color="#aaaaaa", fontsize=9)
    ax_dist.set_title("Distance to Target (d) Trend", color="white", fontsize=10)
    ax_dist.tick_params(colors="#666666")
    ax_dist.spines[:].set_edgecolor("#333333")

    # Right Bottom Panel: Step-wise Prediction RMSE (vs GT)
    ax_err = fig.add_subplot(gs[1, 1])
    ax_err.set_facecolor("#111111")
    ax_err.bar(range(CHUNK_SIZE), rmse_per_step,
               color="#ff6666", alpha=0.3, width=0.7)
    ax_err.set_xlim(-0.5, CHUNK_SIZE - 0.5)
    ax_err.set_xlabel("Waypoint Index", color="#aaaaaa", fontsize=9)
    ax_err.set_ylabel("RMSE (m)", color="#aaaaaa", fontsize=9)
    ax_err.set_title("Step-by-step Prediction Error (vs GT)", color="white", fontsize=10)
    ax_err.tick_params(colors="#666666")
    ax_err.spines[:].set_edgecolor("#333333")

    # Educational Footnote Banner (English translation)
    edu_text = (
        "💡 Action Chunking Philosophy: Model plans a smooth 3D trajectory (Chunk) towards the fixed target in ONE forward pass.  |  "
        "Notice that as execution moves further into the future (higher index), minor compounding errors accumulate."
    )
    fig.text(0.5, 0.02, edu_text,
             ha="center", va="bottom", fontsize=8.5,
             color="#aaaaaa",
             bbox=dict(boxstyle="round,pad=0.4",
                       facecolor="#1a1a1a", edgecolor="#444444"))

    # ── 3. Animation Elements Initialization ─────────────
    # Blue solid line tracing actual movement
    traj_line, = ax3d.plot([], [], [], '-',
                           color="#00ccff", linewidth=2.0, alpha=0.9,
                           zorder=8)
    # White sphere representing the physical gripper
    gripper_pt, = ax3d.plot([], [], [], 'o',
                            color="white", markersize=10,
                            markeredgecolor="#00ccff", markeredgewidth=2,
                            zorder=10)
    # Sync visual lines & error bars for the current step
    vline_dist = ax_dist.axvline(x=-1, color="#00ccff", linewidth=1.5, alpha=0.8)
    highlight_bars = ax_err.bar(range(CHUNK_SIZE), rmse_per_step,
                                color="#00ccff", alpha=0.0, width=0.7)
    
    # 3D Screen Overlay HUD
    step_text = ax3d.text2D(
        0.02, 0.97, "", transform=ax3d.transAxes,
        color="white", fontsize=9, va="top",
        bbox=dict(boxstyle="round,pad=0.3",
                  facecolor="#000000aa", edgecolor="#444444")
    )

    # ── 4. Animation Callback Design ─────────────────────
    PAUSE_FRAMES = 8   # Pauses at the beginning and the end for looping comfort
    total_frames = PAUSE_FRAMES + CHUNK_SIZE + PAUSE_FRAMES

    def update(frame):
        if frame < PAUSE_FRAMES:
            idx = 0
        elif frame < PAUSE_FRAMES + CHUNK_SIZE:
            idx = frame - PAUSE_FRAMES
        else:
            idx = CHUNK_SIZE - 1

        # Render current executed trajectory
        xs = chunk[:idx+1, 0]
        ys = chunk[:idx+1, 1]
        zs = chunk[:idx+1, 2]
        traj_line.set_data(xs, ys)
        traj_line.set_3d_properties(zs)

        gripper_pt.set_data([chunk[idx,0]], [chunk[idx,1]])
        gripper_pt.set_3d_properties([chunk[idx,2]])

        # Update Right Graphs sync line
        vline_dist.set_xdata([idx, idx])

        # Highlight current error bar
        for bi, bar in enumerate(highlight_bars):
            bar.set_alpha(0.85 if bi == idx else 0.0)

        # Update HUD text
        dist_now = dist_to_goal[idx]
        err_now  = rmse_per_step[idx]
        step_text.set_text(
            f"Step {idx+1:2d} / {CHUNK_SIZE}\n"
            f"Curr Pos: ({chunk[idx,0]:.3f}, {chunk[idx,1]:.3f}, {chunk[idx,2]:.3f})\n"
            f"Dist To Target: {dist_now:.4f} m\n"
            f"RMSE Error    : {err_now:.4f} m"
        )
        return traj_line, gripper_pt, vline_dist, step_text

    anim = FuncAnimation(fig, update,
                         frames=total_frames,
                         interval=160, blit=False, repeat=True)

    # ── 5. File Export ──────────────────────────────────
    print("Saving convergence animation to GIF...")
    anim.save(SAVE_GIF, writer="pillow", fps=7, dpi=120)
    print(f" Saved -> {SAVE_GIF}")

    # Export final frame as PNG
    update(PAUSE_FRAMES + CHUNK_SIZE - 1)
    plt.savefig(SAVE_PNG, dpi=150, facecolor=fig.get_facecolor())
    print(f" Saved -> {SAVE_PNG}")

    plt.show()

if __name__ == "__main__":
    main()