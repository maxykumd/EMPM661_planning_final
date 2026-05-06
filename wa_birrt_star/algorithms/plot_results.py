# plot_results.py
import csv
import numpy as np
import matplotlib.pyplot as plt
import os

# Load results
csv_path = os.path.join(os.path.dirname(__file__), 'results.csv')
results  = []
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        results.append(row)

algorithms = ['RRT', 'Bi-RRT', 'Bi-RRT*']
colors     = ['#2196F3', '#4CAF50', '#FF5722']

def get_metric(algo, metric):
    runs = [r for r in results
            if r['algorithm'] == algo and r['success'] == 'True']
    return [float(r[metric]) for r in runs]

# ── Figure 1: Planning Time ────────────────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
fig.suptitle('Algorithm Comparison — 20 runs each', fontsize=14)

# Plot 1 — Planning Time
ax = axes[0]
means = [np.mean(get_metric(a, 'time')) for a in algorithms]
stds  = [np.std(get_metric(a,  'time')) for a in algorithms]
bars  = ax.bar(algorithms, means, color=colors, alpha=0.8,
               yerr=stds, capsize=5)
ax.set_title('Average Planning Time')
ax.set_ylabel('Time (seconds)')
ax.set_ylim(0, max(means) * 1.3)
for bar, mean in zip(bars, means):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.002,
            f'{mean:.3f}s', ha='center', va='bottom', fontsize=10)
ax.grid(True, alpha=0.3, axis='y')

# Plot 2 — Path Length
ax = axes[1]
means = [np.mean(get_metric(a, 'path_length')) for a in algorithms]
stds  = [np.std(get_metric(a,  'path_length')) for a in algorithms]
bars  = ax.bar(algorithms, means, color=colors, alpha=0.8,
               yerr=stds, capsize=5)
ax.set_title('Average Path Length')
ax.set_ylabel('Length (meters)')
ax.set_ylim(0, max(means) * 1.3)
for bar, mean in zip(bars, means):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
            f'{mean:.2f}m', ha='center', va='bottom', fontsize=10)
ax.grid(True, alpha=0.3, axis='y')

# Plot 3 — Waypoints
ax = axes[2]
means = [np.mean(get_metric(a, 'waypoints')) for a in algorithms]
stds  = [np.std(get_metric(a,  'waypoints')) for a in algorithms]
bars  = ax.bar(algorithms, means, color=colors, alpha=0.8,
               yerr=stds, capsize=5)
ax.set_title('Average Waypoints')
ax.set_ylabel('Number of Waypoints')
ax.set_ylim(0, max(means) * 1.3)
for bar, mean in zip(bars, means):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
            f'{mean:.1f}', ha='center', va='bottom', fontsize=10)
ax.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig('comparison_plots.png', dpi=150, bbox_inches='tight')
plt.show()

# ── Figure 2: Distribution Box Plots ──────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle('Performance Distribution — 20 runs each', fontsize=14)

# Box plot — Planning Time
ax = axes[0]
data = [get_metric(a, 'time') for a in algorithms]
bp   = ax.boxplot(data, labels=algorithms, patch_artist=True)
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)
ax.set_title('Planning Time Distribution')
ax.set_ylabel('Time (seconds)')
ax.grid(True, alpha=0.3, axis='y')

# Box plot — Path Length
ax = axes[1]
data = [get_metric(a, 'path_length') for a in algorithms]
bp   = ax.boxplot(data, labels=algorithms, patch_artist=True)
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)
ax.set_title('Path Length Distribution')
ax.set_ylabel('Length (meters)')
ax.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig('distribution_plots.png', dpi=150, bbox_inches='tight')
plt.show()

print("✅ Plots saved:")
print("   comparison_plots.png")
print("   distribution_plots.png")