# plot_results.py
import csv
import numpy as np
import matplotlib.pyplot as plt
import os

csv_path = os.path.join(os.path.dirname(__file__), 'results.csv')
results  = []
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        results.append(row)

algorithms = ['RRT', 'Bi-RRT', 'Bi-RRT*', 'WA*+Bi-RRT*']
colors     = ['#2196F3', '#4CAF50', '#FF5722', '#9C27B0']
labels     = ['RRT', 'Bi-RRT', 'Bi-RRT*', 'WA*+\nBi-RRT*']  # wrap long label

def get_metric(algo, metric):
    runs = [r for r in results
            if r['algorithm'] == algo and r['success'] == 'True']
    return [float(r[metric]) for r in runs]

def success_rate(algo):
    runs = [r for r in results if r['algorithm'] == algo]
    succ = [r for r in runs    if r['success'] == 'True']
    return len(succ) / len(runs) * 100 if runs else 0

# ── Figure 1: Bar Charts ───────────────────────────────────────
fig, axes = plt.subplots(1, 4, figsize=(18, 5))
fig.suptitle('Algorithm Comparison — 50 runs each', fontsize=14)

# Plot 1 — Success Rate
ax = axes[0]
rates = [success_rate(a) for a in algorithms]
bars  = ax.bar(labels, rates, color=colors, alpha=0.8)
ax.set_title('Success Rate')
ax.set_ylabel('Success (%)')
ax.set_ylim(0, 115)
for bar, rate in zip(bars, rates):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
            f'{rate:.0f}%', ha='center', va='bottom', fontsize=10, fontweight='bold')
ax.grid(True, alpha=0.3, axis='y')

# Plot 2 — Planning Time
ax = axes[1]
means = [np.mean(get_metric(a, 'time')) for a in algorithms]
stds  = [np.std(get_metric(a,  'time')) for a in algorithms]
bars  = ax.bar(labels, means, color=colors, alpha=0.8, yerr=stds, capsize=5)
ax.set_title('Average Planning Time')
ax.set_ylabel('Time (seconds)')
ax.set_ylim(0, max(means) * 1.4)
for bar, mean in zip(bars, means):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.002,
            f'{mean:.3f}s', ha='center', va='bottom', fontsize=9)
ax.grid(True, alpha=0.3, axis='y')

# Plot 3 — Path Length
ax = axes[2]
means = [np.mean(get_metric(a, 'path_length')) for a in algorithms]
stds  = [np.std(get_metric(a,  'path_length')) for a in algorithms]
bars  = ax.bar(labels, means, color=colors, alpha=0.8, yerr=stds, capsize=5)
ax.set_title('Average Path Length')
ax.set_ylabel('Length (meters)')
ax.set_ylim(0, max(means) * 1.4)
for bar, mean in zip(bars, means):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
            f'{mean:.2f}m', ha='center', va='bottom', fontsize=9)
ax.grid(True, alpha=0.3, axis='y')

# Plot 4 — Waypoints
ax = axes[3]
means = [np.mean(get_metric(a, 'waypoints')) for a in algorithms]
stds  = [np.std(get_metric(a,  'waypoints')) for a in algorithms]
bars  = ax.bar(labels, means, color=colors, alpha=0.8, yerr=stds, capsize=5)
ax.set_title('Average Waypoints')
ax.set_ylabel('Number of Waypoints')
ax.set_ylim(0, max(means) * 1.4)
for bar, mean in zip(bars, means):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
            f'{mean:.1f}', ha='center', va='bottom', fontsize=9)
ax.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig('comparison_plots.png', dpi=150, bbox_inches='tight')
plt.show()

# ── Figure 2: Box Plots ────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle('Performance Distribution — 50 runs each', fontsize=14)

# Box plot — Planning Time
ax = axes[0]
data = [get_metric(a, 'time') for a in algorithms]
bp   = ax.boxplot(data, labels=labels, patch_artist=True)
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)
ax.set_title('Planning Time Distribution')
ax.set_ylabel('Time (seconds)')
ax.grid(True, alpha=0.3, axis='y')

# Box plot — Path Length
ax = axes[1]
data = [get_metric(a, 'path_length') for a in algorithms]
bp   = ax.boxplot(data, labels=labels, patch_artist=True)
for patch, color in zip(bp['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)
ax.set_title('Path Length Distribution')
ax.set_ylabel('Length (meters)')
ax.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig('distribution_plots.png', dpi=150, bbox_inches='tight')
plt.show()

# ── Print Summary ──────────────────────────────────────────────
print("\n" + "=" * 65)
print("SUMMARY")
print("=" * 65)
print(f"{'Algorithm':<14} {'Success':>8} {'Avg Time':>10} "
      f"{'Avg Length':>12} {'Avg WPs':>9}")
print("-" * 65)
for algo, label in zip(algorithms, ['RRT', 'Bi-RRT', 'Bi-RRT*', 'WA*+Bi-RRT*']):
    succ     = get_metric(algo, 'time')
    rate     = success_rate(algo)
    avg_time = np.mean(get_metric(algo, 'time'))        if succ else 0
    avg_len  = np.mean(get_metric(algo, 'path_length')) if succ else 0
    avg_wps  = np.mean(get_metric(algo, 'waypoints'))   if succ else 0
    print(f"{label:<14} {rate:>7.0f}%  "
          f"{avg_time:>9.3f}s  "
          f"{avg_len:>11.2f}m  "
          f"{avg_wps:>8.1f}")
print("=" * 65)

print("\n✅ Plots saved:")
print("   comparison_plots.png")
print("   distribution_plots.png")