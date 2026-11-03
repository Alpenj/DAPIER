"""
Idea E Upgrade — Single-Shot vs Receding Horizon Side-by-Side (Fixed Target)
===========================================================================
Left Panel  : Single-Shot (Inference once, execute all 20 chunk waypoints)
Right Panel : Receding Horizon (Re-predict every k steps, execute only front k)
Target      : Fixed at [0.25, 0.0, 0.2] for intuitive convergence visualization.
"""

import os
import numpy as np
import torch
import torch.nn as nn
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D   # noqa
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"
CHUNK_SIZE    = 20
K             = 5          # Receding Horizon: Re-predict every k steps
WORKSPACE_MIN = np.array([0.0, -0.3, 0.0])
WORKSPACE_MAX = np.array([0.5,  0.3, 0.4])

# Fixed Target for intuitive convergence visualization
FIXED_TARGET  = np.array([0.25, 0.0, 0.2])

# Load the fixed-target policy trained in Step 1 (101_output)
os.makedirs("./103_output", exist_ok=True)
MODEL_PATH    = "./101_output/act_policy_fixed.pt"
SAVE_GIF      = "./103_output/viz_E_fixed_receding.gif"
SAVE_PNG      = "./103_output/viz_E_fixed_final_frame.png"

# ─────────────────────────────────────────
# Model Architecture
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
    def predict(self, start: np.ndarray, target: np.ndarray) -> np.ndarray:
        obs = torch.tensor(np.concatenate([start, target]),
                           dtype=torch.float32).unsqueeze(0).to(DEVICE)
        return self(obs).squeeze(0).cpu().numpy().reshape(CHUNK_SIZE, 3)


def load_model(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Could not find model weights at '{path}'. "
            f"Please run the step 1 training (fixed target) first to save the model."
        )
    ckpt  = torch.load(path, map_location=DEVICE, weights_only=False)
    # Compatibility with older ckpt structures
    config = ckpt.get("config", {"obs_dim": 6, "chunk_dim": CHUNK_SIZE*3, "hidden": 256, "n_layers": 3, "dropout": 0.1})
    model = ActionChunkingPolicy(**config)
    model.load_state_dict(ckpt["state_dict"])
    return model.to(DEVICE).eval()


# ─────────────────────────────────────────
# Trajectory Generation (Ground Truth Reference)
# ─────────────────────────────────────────
def make_gt(start, target, n=CHUNK_SIZE, curv=0.15):
    t      = np.linspace(0, 1, n).reshape(-1, 1)
    linear = start + t * (target - start)
    d      = target - start
    perp   = np.cross(d, [0,0,1]) if abs(d[2]) < 1e-6 else np.cross(d, [1,0,0])
    nm     = np.linalg.norm(perp)
    perp   = perp/nm if nm > 1e-6 else np.array([0,0,1])
    return linear + curv * 4 * t * (1-t) * perp.reshape(1,3)


# ─────────────────────────────────────────
# Simulation Methods
# ─────────────────────────────────────────
def simulate_single_shot(model, start, target):
    """
    Predicts once -> executes all 20 waypoints sequentially.
    """
    chunk  = model.predict(start, target)
    steps  = list(chunk)
    return steps, 1, [chunk] * len(steps)


def simulate_receding_horizon(model, start, target, k=K):
    """
    Re-predicts every k steps, moving along the freshly predicted path.
    """
    steps          = []
    replan_indices = []      # Indices where re-planning occurred
    active_chunks  = []      # Store predicted chunk at each step
    n_inferences   = 0
    current_pos    = start.copy()
    MAX_STEPS      = CHUNK_SIZE * 2

    while len(steps) < MAX_STEPS:
        # ── Re-plan ─────────────────────────────────────────────
        replan_indices.append(len(steps))
        chunk        = model.predict(current_pos, target)
        n_inferences += 1

        # ── Execute only first k steps ──────────────────────────
        for i in range(k):
            if len(steps) >= MAX_STEPS:
                break
            wp = chunk[i]
            steps.append(wp.copy())
            active_chunks.append(chunk.copy())
            current_pos = wp.copy()

            # Target arrival check
            if np.linalg.norm(current_pos - target) < 0.025:
                return steps, replan_indices, n_inferences, active_chunks

    return steps, replan_indices, n_inferences, active_chunks


# ─────────────────────────────────────────
# Main Visualization
# ─────────────────────────────────────────
def main():
    print("Loading model weights...")
    model = load_model(MODEL_PATH)

    # Randomly select a starting position, target is strictly FIXED
    rng    = np.random.default_rng(42)  # Seed for reproducible comparison
    start  = rng.uniform(WORKSPACE_MIN + 0.05, WORKSPACE_MAX - 0.05)
    target = FIXED_TARGET

    print(f"  Start  Pos : {start.round(3)}")
    print(f"  Target Pos (Fixed): {target.round(3)}")
    gt = make_gt(start, target)

    # ── Run Simulations ─────────────────────────────────────────
    print("\nSimulating Single-Shot...")
    ss_steps, ss_ninf, ss_chunks = simulate_single_shot(model, start, target)
    ss_pos = np.array(ss_steps)

    print("Simulating Receding Horizon...")
    rh_steps, rh_replan, rh_ninf, rh_chunks = simulate_receding_horizon(
        model, start, target, k=K)
    rh_pos = np.array(rh_steps)

    print(f"\n  Single-Shot: Inferences = {ss_ninf}, Total Steps = {len(ss_steps)}")
    print(f"  Receding (k={K}): Inferences = {rh_ninf}, Total Steps = {len(rh_steps)}")

    # Compute Cumulative RMSE compared to GT
    def cumulative_rmse(pos_arr, ref_arr):
        n    = len(pos_arr)
        rmse = []
        for i in range(1, n+1):
            t_idx = np.linspace(0, len(ref_arr)-1, i).astype(int)
            diff  = pos_arr[:i] - ref_arr[t_idx]
            rmse.append(float(np.sqrt(np.mean(diff**2))))
        return rmse

    ss_rmse = cumulative_rmse(ss_pos, gt)
    rh_rmse = cumulative_rmse(rh_pos, gt)

    # ── Plot Layout ─────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 7.5), facecolor="#0d0d0d")
    fig.suptitle(
        "Single-Shot (1 Inference)  vs  Receding Horizon (Re-predict every k=5 steps)",
        color="white", fontsize=14, fontweight="bold", y=0.97
    )

    gs = gridspec.GridSpec(
        2, 3,
        left=0.04, right=0.98, top=0.91, bottom=0.09,
        wspace=0.28, hspace=0.40,
        width_ratios=[2, 2, 1.4]
    )

    def make_3d_ax(pos, title):
        ax = fig.add_subplot(pos, projection='3d')
        ax.set_facecolor("#111111")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#2a2a2a")
        ax.set_xlim(WORKSPACE_MIN[0], WORKSPACE_MAX[0])
        ax.set_ylim(WORKSPACE_MIN[1], WORKSPACE_MAX[1])
        ax.set_zlim(WORKSPACE_MIN[2], WORKSPACE_MAX[2])
        ax.set_xlabel("x", color="#666", labelpad=3, fontsize=8)
        ax.set_ylabel("y", color="#666", labelpad=3, fontsize=8)
        ax.set_zlabel("z", color="#666", labelpad=3, fontsize=8)
        ax.tick_params(colors="#444", labelsize=6)
        ax.set_title(title, color="white", fontsize=11, pad=5)
        # Ground Truth Reference
        ax.plot(gt[:,0], gt[:,1], gt[:,2],
                '--', color="#333333", lw=1.2, label="GT Ref Trajectory")
        return ax

    ax_ss = make_3d_ax(gs[0, 0], "Single-Shot Control  |  Inference: 0")
    ax_rh = make_3d_ax(gs[0, 1], f"Receding Horizon (k={K})  |  Inference: 0")

    # Single-Shot: static preview of the full predicted chunk
    ax_ss.plot(ss_pos[:,0], ss_pos[:,1], ss_pos[:,2],
               ':', color="#00ffcc", lw=0.8, alpha=0.3, label="Predicted Chunk")

    # Add Markers for Start and Target on both axes
    for ax in [ax_ss, ax_rh]:
        ax.scatter(*start,  color="#44ff44", s=80,  marker="s", zorder=5, label="Start (s)")
        ax.scatter(*target, color="#ff4444", s=130, marker="*", zorder=5, label="Target (d)")
        ax.legend(loc="upper left", fontsize=7,
                  facecolor="#1a1a1a", edgecolor="#444", labelcolor="white")

    # ── Right Panel: RMSE curves ───────────────────────────────
    ax_rmse = fig.add_subplot(gs[:, 2])
    ax_rmse.set_facecolor("#111111")
    ax_rmse.set_title("Cumulative RMSE Comparison", color="white", fontsize=11)
    ax_rmse.set_xlabel("Execution Steps", color="#aaa", fontsize=9)
    ax_rmse.set_ylabel("RMSE (meters)", color="#aaa", fontsize=9)
    ax_rmse.tick_params(colors="#555")
    ax_rmse.spines[:].set_edgecolor("#333")

    # Background reference lines
    max_step = max(len(ss_rmse), len(rh_rmse))
    ax_rmse.plot(range(1, len(ss_rmse)+1), ss_rmse, color="#00ccff", lw=0.8, alpha=0.1)
    ax_rmse.plot(range(1, len(rh_rmse)+1), rh_rmse, color="#ff9944", lw=0.8, alpha=0.1)
    ax_rmse.set_xlim(0, max_step + 1)
    ax_rmse.set_ylim(0, max(max(ss_rmse), max(rh_rmse)) * 1.15)

    # Dynamic line plots
    line_ss_rmse, = ax_rmse.plot([], [], '-', color="#00ccff", lw=2.0,
                                  label=f"Single-Shot (Inf: {ss_ninf})")
    line_rh_rmse, = ax_rmse.plot([], [], '-', color="#ff9944", lw=2.0,
                                  label=f"Receding (Inf: {rh_ninf})")
    ax_rmse.legend(fontsize=8, facecolor="#1a1a1a", edgecolor="#444", labelcolor="white")

    # Add Vertical dashed markers at planning intervals on the RMSE chart
    for ri in rh_replan[1:]:
        if ri < len(rh_rmse):
            ax_rmse.axvline(x=ri, color="#ff9944", lw=0.8, alpha=0.3, linestyle=":")

    # ── Dynamic Elements for Animation ───────────────────────
    # Single-Shot Elements
    ss_traj, = ax_ss.plot([], [], [], '-',  color="#00ccff", lw=2.2, alpha=0.9)
    ss_grip, = ax_ss.plot([], [], [], 'o',  color="white", markersize=9,
                          markeredgecolor="#00ccff", markeredgewidth=2, zorder=10)
    ss_info  = ax_ss.text2D(0.02, 0.97, "", transform=ax_ss.transAxes,
                            color="white", fontsize=8.5, va="top",
                            bbox=dict(boxstyle="round,pad=0.3",
                                      facecolor="#000000bb", edgecolor="#444"))

    # Receding Horizon Elements
    rh_traj,     = ax_rh.plot([], [], [], '-',  color="#ff9944", lw=2.2, alpha=0.9)
    rh_grip,     = ax_rh.plot([], [], [], 'o',  color="white", markersize=9,
                              markeredgecolor="#ff9944", markeredgewidth=2, zorder=10)
    rh_preview,  = ax_rh.plot([], [], [], ':',  color="#ff9944", lw=1.0, alpha=0.45)
    rh_info      = ax_rh.text2D(0.02, 0.97, "", transform=ax_rh.transAxes,
                                color="white", fontsize=8.5, va="top",
                                bbox=dict(boxstyle="round,pad=0.3",
                                          facecolor="#000000bb", edgecolor="#444"))
    rh_replan_txt = ax_rh.text2D(0.50, 0.90, "", transform=ax_rh.transAxes,
                                 color="#ff9944", fontsize=11, va="top",
                                 ha="center", fontweight="bold")

    # Core Education Banner (Bottom)
    edu_text = (
        "💡 Receding Horizon Control (RHC) prevents error compounding by executing only first k steps "
        "and re-planning from current state.\n"
        "Observe how the Receding Horizon path smoothly converges to the Target * (Red Star) with near-zero final RMSE."
    )
    fig.text(0.5, 0.02, edu_text, ha="center", va="bottom", fontsize=9,
             color="#aaaaaa",
             bbox=dict(boxstyle="round,pad=0.4", facecolor="#1a1a1a", edgecolor="#444"))

    # ── Animation Update Function ────────────────────────────
    PAUSE    = 6
    n_ss     = len(ss_steps)
    n_rh     = len(rh_steps)
    n_common = max(n_ss, n_rh)
    total_frames = PAUSE + n_common + PAUSE

    def update(frame):
        # Handle start/end pauses
        if frame < PAUSE:
            fi = 0
        elif frame < PAUSE + n_common:
            fi = frame - PAUSE
        else:
            fi = n_common - 1

        ss_idx = min(fi, n_ss - 1)
        rh_idx = min(fi, n_rh - 1)

        # ── Update Single-Shot Panel ──
        ss_traj.set_data(ss_pos[:ss_idx+1, 0], ss_pos[:ss_idx+1, 1])
        ss_traj.set_3d_properties(ss_pos[:ss_idx+1, 2])
        ss_grip.set_data([ss_pos[ss_idx, 0]], [ss_pos[ss_idx, 1]])
        ss_grip.set_3d_properties([ss_pos[ss_idx, 2]])
        dist_ss = np.linalg.norm(ss_pos[ss_idx] - target)
        
        ax_ss.set_title(f"Single-Shot Control  |  Inference: {ss_ninf}", color="white", fontsize=11, pad=5)
        ss_info.set_text(
            f"Step {ss_idx+1:2d}/{n_ss}\n"
            f"Distance: {dist_ss:.4f} m\n"
            f"Inferences: {ss_ninf} (Once)"
        )

        # ── Update Receding Horizon Panel ──
        rh_cur = rh_pos[rh_idx]
        rh_traj.set_data(rh_pos[:rh_idx+1, 0], rh_pos[:rh_idx+1, 1])
        rh_traj.set_3d_properties(rh_pos[:rh_idx+1, 2])
        rh_grip.set_data([rh_cur[0]], [rh_cur[1]])
        rh_grip.set_3d_properties([rh_cur[2]])

        # Update the active look-ahead prediction chunk
        cur_chunk = rh_chunks[rh_idx]
        rh_preview.set_data(cur_chunk[:,0], cur_chunk[:,1])
        rh_preview.set_3d_properties(cur_chunk[:,2])

        # Track re-planning steps
        is_replan = rh_idx in rh_replan
        infer_count = sum(1 for r in rh_replan if r <= rh_idx)
        dist_rh = np.linalg.norm(rh_cur - target)

        ax_rh.set_title(f"Receding Horizon (k={K})  |  Inference: {infer_count}", color="white", fontsize=11, pad=5)
        rh_info.set_text(
            f"Step {rh_idx+1:2d}/{n_rh}\n"
            f"Distance: {dist_rh:.4f} m\n"
            f"Inferences: {infer_count} (Re-plan)"
        )
        rh_replan_txt.set_text("🔄 Re-plan!" if is_replan and fi > 0 else "")

        # ── Update RMSE Graphs ──
        line_ss_rmse.set_data(range(1, ss_idx+2), ss_rmse[:ss_idx+1])
        line_rh_rmse.set_data(range(1, rh_idx+2), rh_rmse[:rh_idx+1])

        return (ss_traj, ss_grip, rh_traj, rh_grip,
                rh_preview, rh_replan_txt,
                line_ss_rmse, line_rh_rmse,
                ss_info, rh_info)

    # ── Save Output ──
    anim = FuncAnimation(fig, update, frames=total_frames, interval=180, blit=False, repeat=True)

    print("\nSaving animation as GIF (Please wait)...")
    anim.save(SAVE_GIF, writer="pillow", fps=7, dpi=120)
    print(f"[Success] Saved: {SAVE_GIF}")

    # Save final frame as PNG
    update(PAUSE + n_common - 1)
    plt.savefig(SAVE_PNG, dpi=150, facecolor=fig.get_facecolor())
    print(f"[Success] Saved: {SAVE_PNG}")


if __name__ == "__main__":
    main()