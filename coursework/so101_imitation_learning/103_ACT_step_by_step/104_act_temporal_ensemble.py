"""
Temporal Ensemble Implementation & Visualization (Fixed Target Version)
======================================================================
Comparing Three Execution Modes Side-by-Side with a Fixed Target:
  Left  : Single-Shot      (Infer once, execute all 20 steps)
  Center: Receding Horizon (Re-infer every k=5 steps, has boundaries)
  Right : Temporal Ensemble(Re-infer every k=1 step + weighted average)
"""

import numpy as np
import torch
import torch.nn as nn
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
import warnings
import os
warnings.filterwarnings("ignore")

0# ─────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"
CHUNK_SIZE    = 20
K_RH          = 5        
K_TE          = 1        
TE_WINDOW     = 8        
TE_LAMBDA     = 0.15     
WORKSPACE_MIN = np.array([0.0, -0.3, 0.0])
WORKSPACE_MAX = np.array([0.5,  0.3, 0.4])

# CRITICAL: Using the fixed policy from step 1 & Fixing Target
MODEL_PATH    = "./101_output/act_policy_fixed.pt"
FIXED_TARGET  = np.array([0.25, 0.0, 0.2])

os.makedirs("./104_output", exist_ok=True)
SAVE_GIF      = "./104_output/viz_TE_fixed_comparison.gif"
SAVE_PNG      = "./104_output/viz_TE_fixed_final_frame.png"


# ─────────────────────────────────────────
# MODEL DEFINITION
# ─────────────────────────────────────────
class ActionChunkingPolicy(nn.Module):
    def __init__(self, obs_dim=6, chunk_dim=CHUNK_SIZE*3, hidden=256, n_layers=3, dropout=0.1):
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
        obs = torch.tensor(
            np.concatenate([start, target]), dtype=torch.float32
        ).unsqueeze(0).to(DEVICE)
        return self(obs).squeeze(0).cpu().numpy().reshape(CHUNK_SIZE, 3)


def load_model(path):
    if not os.path.exists(path):
        # Fallback to local pt if directory structure differs
        path = "./act_policy.pt"
    ckpt  = torch.load(path, map_location=DEVICE, weights_only=False)
    model = ActionChunkingPolicy(**ckpt["config"])
    model.load_state_dict(ckpt["state_dict"])
    return model.to(DEVICE).eval()


# ─────────────────────────────────────────
# GROUND TRUTH TRAJECTORY GENERATION
# ─────────────────────────────────────────
def make_gt(start, target, n=CHUNK_SIZE, curv=0.15):
    t      = np.linspace(0, 1, n).reshape(-1, 1)
    linear = start + t * (target - start)
    d      = target - start
    perp   = np.cross(d, [0,0,1]) if abs(d[2]) < 1e-6 else np.cross(d, [1,0,0])
    nm     = np.linalg.norm(perp)
    perp   = perp / nm if nm > 1e-6 else np.array([0,0,1])
    return linear + curv * 4 * t * (1-t) * perp.reshape(1,3)


# ─────────────────────────────────────────
# SIMULATION MODES
# ─────────────────────────────────────────
def simulate_single_shot(model, start, target):
    chunk = model.predict(start, target)
    steps = list(chunk)
    return np.array(steps), 1


def simulate_receding_horizon(model, start, target, k=K_RH):
    steps, replan_idx = [], []
    n_infer, current_pos = 0, start.copy()
    MAX = CHUNK_SIZE * 4

    while len(steps) < MAX:
        replan_idx.append(len(steps))
        chunk   = model.predict(current_pos, target)
        n_infer += 1
        for i in range(k):
            if len(steps) >= MAX: break
            wp = chunk[i]
            steps.append(wp.copy())
            current_pos = wp.copy()
            if np.linalg.norm(current_pos - target) < 0.03:
                return np.array(steps), n_infer, replan_idx
    return np.array(steps), n_infer, replan_idx


def simulate_temporal_ensemble(model, start, target, window=TE_WINDOW, lam=TE_LAMBDA):
    steps          = []
    chunk_buffer   = []   
    weight_history = []   
    n_infer        = 0
    current_pos    = start.copy()
    MAX            = CHUNK_SIZE * 4

    while len(steps) < MAX:
        new_chunk   = model.predict(current_pos, target)   
        n_infer    += 1

        chunk_buffer.insert(0, new_chunk)
        if len(chunk_buffer) > window:
            chunk_buffer.pop()           

        n_buf   = len(chunk_buffer)
        raw_w   = np.array([np.exp(-i * lam) for i in range(n_buf)])
        weights = raw_w / raw_w.sum()             

        weighted_wp = np.zeros(3)
        for i, (ch, w) in enumerate(zip(chunk_buffer, weights)):
            if i < len(ch):
                weighted_wp += w * ch[i]

        weight_history.append(weights.copy())
        steps.append(weighted_wp.copy())
        current_pos = weighted_wp.copy()

        if np.linalg.norm(current_pos - target) < 0.03:
            break

    return np.array(steps), n_infer, weight_history


def cumulative_rmse(pos_arr, ref_arr):
    rmse = []
    for i in range(1, len(pos_arr)+1):
        t_idx = np.linspace(0, len(ref_arr)-1, i).astype(int)
        diff  = pos_arr[:i] - ref_arr[t_idx]
        rmse.append(float(np.sqrt(np.mean(diff**2))))
    return rmse


# ─────────────────────────────────────────
# MAIN VISUALIZATION PIPELINE
# ─────────────────────────────────────────
def main():
    print("Loading ACT Model...")
    model = load_model(MODEL_PATH)

    rng    = np.random.default_rng(42)
    start  = rng.uniform(WORKSPACE_MIN + 0.05, WORKSPACE_MAX - 0.05)
    target = FIXED_TARGET # Fixed target enforced
    
    print(f"  Start Node : {start.round(3)}")
    print(f"  Fixed Target: {target.round(3)}")
    gt = make_gt(start, target)

    # Simulating Modes
    ss_pos, ss_ninf         = simulate_single_shot(model, start, target)
    rh_pos, rh_ninf, rh_idx = simulate_receding_horizon(model, start, target)
    te_pos, te_ninf, te_w   = simulate_temporal_ensemble(model, start, target)

    ss_rmse = cumulative_rmse(ss_pos, gt)
    rh_rmse = cumulative_rmse(rh_pos, gt)
    te_rmse = cumulative_rmse(te_pos, gt)

    # Plot Layout Setup
    fig = plt.figure(figsize=(16, 8.5), facecolor="#0d0d0d")
    fig.suptitle(
        "ACT Optimization: Single-Shot vs Receding Horizon vs Temporal Ensemble (Fixed Target)",
        color="white", fontsize=13, fontweight="bold", y=0.97
    )

    gs = gridspec.GridSpec(2, 4, left=0.03, right=0.98, top=0.92, bottom=0.09, wspace=0.30, hspace=0.42, width_ratios=[2, 2, 2, 1.6])

    def make_ax(subplot_spec, title, color):
        ax = fig.add_subplot(subplot_spec, projection='3d')
        ax.set_facecolor("#111111")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#2a2a2a")
        ax.set_xlim(*WORKSPACE_MIN[[0]], *WORKSPACE_MAX[[0]])
        ax.set_ylim(*WORKSPACE_MIN[[1]], *WORKSPACE_MAX[[1]])
        ax.set_zlim(*WORKSPACE_MIN[[2]], *WORKSPACE_MAX[[2]])
        for axis, label in zip([ax.set_xlabel, ax.set_ylabel, ax.set_zlabel], ["X","Y","Z"]):
            axis(label, color="#555", labelpad=2, fontsize=7)
        ax.tick_params(colors="#333", labelsize=5)
        ax.set_title(title, color=color, fontsize=9, pad=4, fontweight='bold')
        ax.plot(gt[:,0], gt[:,1], gt[:,2], '--', color="#2a2a2a", lw=1.0)
        ax.scatter(*start,  color="#44ff44", s=70,  marker="s", zorder=5)
        ax.scatter(*target, color="#ff4444", s=120, marker="*", zorder=5)
        return ax

    ax_ss = make_ax(gs[:, 0], f"Single-Shot | Inf: {ss_ninf}", "#00ccff")
    ax_rh = make_ax(gs[:, 1], f"Receding Horizon (k={K_RH}) | Inf: {rh_ninf}", "#ff9944")
    ax_te = make_ax(gs[:, 2], f"Temporal Ensemble (W={TE_WINDOW}) | Inf: {te_ninf}", "#cc88ff")

    # Right Panel: RMSE Comparison
    ax_rmse = fig.add_subplot(gs[0, 3])
    ax_rmse.set_facecolor("#111111")
    ax_rmse.set_title("Cumulative RMSE Comparison", color="white", fontsize=9)
    ax_rmse.set_xlabel("Execution Steps", color="#888", fontsize=8)
    ax_rmse.set_ylabel("RMSE (m)", color="#888", fontsize=8)
    ax_rmse.tick_params(colors="#555", labelsize=7)
    ax_rmse.spines[:].set_edgecolor("#333")
    max_step = max(len(ss_rmse), len(rh_rmse), len(te_rmse))
    max_val  = max(max(ss_rmse), max(rh_rmse), max(te_rmse))
    ax_rmse.set_xlim(0, max_step+1)
    ax_rmse.set_ylim(0, max_val * 1.15)

    line_ss_r, = ax_rmse.plot([], [], color="#00ccff", lw=1.8, label="Single-Shot")
    line_rh_r, = ax_rmse.plot([], [], color="#ff9944", lw=1.8, label="Receding H.")
    line_te_r, = ax_rmse.plot([], [], color="#cc88ff", lw=2.2, label="Temp Ensemble")
    ax_rmse.legend(fontsize=7, facecolor="#1a1a1a", edgecolor="#444", labelcolor="white")

    # Right Panel: TE Weights Dynamic Bar
    ax_w = fig.add_subplot(gs[1, 3])
    ax_w.set_facecolor("#111111")
    ax_w.set_title(f"TE Weights (Recent {TE_WINDOW} Chunks)", color="white", fontsize=9)
    ax_w.set_xlabel("Chunk Index (0 = Newest)", color="#888", fontsize=8)
    ax_w.set_ylabel("Weight Value", color="#888", fontsize=8)
    ax_w.tick_params(colors="#555", labelsize=7)
    ax_w.spines[:].set_edgecolor("#333")
    ax_w.set_xlim(-0.5, TE_WINDOW - 0.5)
    ax_w.set_ylim(0, 1.05)

    ref_w_raw = np.array([np.exp(-i * TE_LAMBDA) for i in range(TE_WINDOW)])
    ref_w     = ref_w_raw / ref_w_raw.sum()
    ax_w.bar(range(TE_WINDOW), ref_w, color="#555555", alpha=0.3, width=0.6)

    bars_w = ax_w.bar(range(TE_WINDOW), [0]*TE_WINDOW, color="#cc88ff", alpha=0.85, width=0.6)
    w_text = ax_w.text(TE_WINDOW/2, 0.85, "", ha="center", fontsize=8, color="#cc88ff")

    # Interactive Elements Setup
    def make_elements(ax, color):
        traj, = ax.plot([], [], [], '-',  color=color, lw=2.2, alpha=0.9)
        grip, = ax.plot([], [], [], 'o',  color="white", markersize=8, markeredgecolor=color, markeredgewidth=2, zorder=10)
        info  = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, color="white", fontsize=7, va="top", bbox=dict(boxstyle="round,pad=0.25", facecolor="#000000bb", edgecolor="#444"))
        return traj, grip, info

    ss_traj, ss_grip, ss_info = make_elements(ax_ss, "#00ccff")
    rh_traj, rh_grip, rh_info = make_elements(ax_rh, "#ff9944")
    te_traj, te_grip, te_info = make_elements(ax_te, "#cc88ff")

    rh_replan_txt = ax_rh.text2D(0.50, 0.88, "", transform=ax_rh.transAxes, color="#ff9944", fontsize=9, ha="center", fontweight="bold")
    te_avg_txt = ax_te.text2D(0.50, 0.88, "", transform=ax_te.transAxes, color="#cc88ff", fontsize=9, ha="center", fontweight="bold")

    edu = "💡 Temporal Ensemble: Re-predicts at every step & blends overlapping paths smoothly, eliminating sudden boundary jumps."
    fig.text(0.5, 0.02, edu, ha="center", va="bottom", fontsize=8.5, color="#aaaaaa", bbox=dict(boxstyle="round,pad=0.35", facecolor="#1a1a1a", edgecolor="#444"))

    PAUSE    = 5
    n_ss, n_rh, n_te = len(ss_pos), len(rh_pos), len(te_pos)
    n_common = max(n_ss, n_rh, n_te)
    total    = PAUSE + n_common + PAUSE

    def update(frame):
        fi = max(0, min(frame - PAUSE, n_common - 1))
        si, ri, ti = min(fi, n_ss - 1), min(fi, n_rh - 1), min(fi, n_te - 1)

        # Update Single-Shot
        ss_traj.set_data(ss_pos[:si+1,0], ss_pos[:si+1,1])
        ss_traj.set_3d_properties(ss_pos[:si+1,2])
        ss_grip.set_data([ss_pos[si,0]], [ss_pos[si,1]])
        ss_grip.set_3d_properties([ss_pos[si,2]])
        ss_info.set_text(f"Step {si+1}/{n_ss}\nDist: {np.linalg.norm(ss_pos[si]-target):.3f}m")

        # Update Receding Horizon
        rh_traj.set_data(rh_pos[:ri+1,0], rh_pos[:ri+1,1])
        rh_traj.set_3d_properties(rh_pos[:ri+1,2])
        rh_grip.set_data([rh_pos[ri,0]], [rh_pos[ri,1]])
        rh_grip.set_3d_properties([rh_pos[ri,2]])
        infer_rh = sum(1 for r in rh_idx if r <= ri)
        rh_info.set_text(f"Step {ri+1}/{n_rh}\nDist: {np.linalg.norm(rh_pos[ri]-target):.3f}m\nInfer: {infer_rh}")
        rh_replan_txt.set_text("🔄 Re-plan!" if (ri in rh_idx and fi > 0) else "")

        # Update Temporal Ensemble
        te_traj.set_data(te_pos[:ti+1,0], te_pos[:ti+1,1])
        te_traj.set_3d_properties(te_pos[:ti+1,2])
        te_grip.set_data([te_pos[ti,0]], [te_pos[ti,1]])
        te_grip.set_3d_properties([te_pos[ti,2]])
        te_info.set_text(f"Step {ti+1}/{n_te}\nDist: {np.linalg.norm(te_pos[ti]-target):.3f}m\nInfer: {ti+1}")
        te_avg_txt.set_text("⚖ Blending..." if fi > 0 else "")

        # Update TE weights bars
        if ti < len(te_w):
            w = te_w[ti]
            for bi, bar in enumerate(bars_w):
                bar.set_height(w[bi] if bi < len(w) else 0)
            w_text.set_text(f"Latest w₀={w[0]:.3f}")

        # Update Curves
        line_ss_r.set_data(range(1, si+2), ss_rmse[:si+1])
        line_rh_r.set_data(range(1, ri+2), rh_rmse[:ri+1])
        line_te_r.set_data(range(1, ti+2), te_rmse[:ti+1])

        return (ss_traj, ss_grip, rh_traj, rh_grip, te_traj, te_grip, line_ss_r, line_rh_r, line_te_r, rh_replan_txt, te_avg_txt, ss_info, rh_info, te_info, *bars_w)

    anim = FuncAnimation(fig, update, frames=total, interval=150, blit=False, repeat=True)

    print("Saving Comparison GIF...")
    anim.save(SAVE_GIF, writer="pillow", fps=7, dpi=120)
    print(f"[Saved] {SAVE_GIF}")

    update(PAUSE + n_common - 1)
    plt.savefig(SAVE_PNG, dpi=150, facecolor=fig.get_facecolor())
    print(f"[Saved] {SAVE_PNG}")


if __name__ == "__main__":
    main() 