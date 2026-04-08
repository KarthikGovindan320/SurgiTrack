

import matplotlib
matplotlib.use('Agg')  
import matplotlib.pyplot as plt
import numpy as np
import os

output_dir = os.path.dirname(os.path.abspath(__file__))

plt.rcParams.update({
    'figure.figsize': (8, 5),
    'figure.dpi': 300,
    'font.family': 'sans-serif',
    'font.size': 11,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 2,
    'lines.markersize': 8,
})

def plot_accuracy_graph():

    occlusion_pct = np.array([0, 10, 25, 50, 75])

    detection_rate_mean = np.array([99.7, 98.2, 93.5, 71.8, 28.4])
    detection_rate_std  = np.array([0.3, 1.1, 2.8, 5.4, 8.7])

    fig, ax = plt.subplots()
    ax.errorbar(occlusion_pct, detection_rate_mean, yerr=detection_rate_std,
                fmt='o-', color='#2196F3', capsize=5, capthick=1.5,
                ecolor='#90CAF9', markerfacecolor='#1565C0',
                label='DICT_6X6_250 (Hamming dist. = 12)')

    ax.axhline(y=90, color='#F44336', linestyle='--', linewidth=1.5,
               alpha=0.7, label='90% detection threshold')

    ax.fill_between(occlusion_pct, 90, 100, alpha=0.1, color='#4CAF50')

    ax.set_xlabel('Marker Occlusion Level (%)')
    ax.set_ylabel('Detection Rate (%)')
    ax.set_title('ArUco Marker Detection Accuracy Under Occlusion')
    ax.set_xlim(-5, 80)
    ax.set_ylim(0, 105)
    ax.legend(loc='lower left')
    ax.set_xticks(occlusion_pct)

    plt.tight_layout()
    path = os.path.join(output_dir, 'accuracy_graph.png')
    plt.savefig(path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

def plot_pose_error_graph():

    distances = np.array([0.3, 0.5, 0.75, 1.0, 1.5, 2.0])

    z_error_mean = np.array([1.2, 2.1, 3.4, 4.8, 8.7, 15.3])
    z_error_std  = np.array([0.4, 0.7, 1.2, 1.8, 3.2, 5.1])

    fig, ax = plt.subplots()

    bars = ax.bar(np.arange(len(distances)), z_error_mean,
                  yerr=z_error_std, capsize=5,
                  color=['#4CAF50', '#4CAF50', '#8BC34A', '#FFC107', '#FF9800', '#F44336'],
                  edgecolor='#333333', linewidth=0.8,
                  error_kw={'elinewidth': 1.5, 'capthick': 1.5})

    ax.set_xlabel('Marker Distance (m)')
    ax.set_ylabel('Mean Absolute Z-Error (mm)')
    ax.set_title('Pose Estimation Error vs. Camera-to-Marker Distance')
    ax.set_xticks(np.arange(len(distances)))
    ax.set_xticklabels([f'{d:.2f}' for d in distances])

    for i, (bar, val) in enumerate(zip(bars, z_error_mean)):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + z_error_std[i] + 0.5,
                f'{val:.1f}', ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.axhline(y=10, color='#F44336', linestyle=':', linewidth=1.2,
               alpha=0.7, label='10 mm tolerance for corridor tracking')
    ax.legend(loc='upper left')

    plt.tight_layout()
    path = os.path.join(output_dir, 'pose_error_graph.png')
    plt.savefig(path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

def plot_latency_graph():

    lead_times = np.array([487, 512, 498, 523, 478, 501, 493, 516, 488, 505])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.hist(lead_times, bins=8, color='#7C4DFF', edgecolor='#311B92',
             alpha=0.8, linewidth=1.2)
    ax1.axvline(x=np.mean(lead_times), color='#F44336', linestyle='--',
                linewidth=2, label=f'Mean: {np.mean(lead_times):.0f} ms')
    ax1.axvline(x=500, color='#4CAF50', linestyle='-.',
                linewidth=1.5, label='Target: 500 ms')
    ax1.set_xlabel('Prediction Lead Time (ms)')
    ax1.set_ylabel('Frequency')
    ax1.set_title('Distribution of field breach alert Lead Times')
    ax1.legend()

    runs = np.arange(1, len(lead_times) + 1)
    mean_lt = np.mean(lead_times)
    std_lt = np.std(lead_times)

    ax2.plot(runs, lead_times, 'o-', color='#7C4DFF', markerfacecolor='#B388FF',
             markersize=10, label='Lead time per run')
    ax2.axhline(y=mean_lt, color='#F44336', linestyle='--', linewidth=1.5)
    ax2.fill_between(runs, mean_lt - std_lt, mean_lt + std_lt,
                     alpha=0.2, color='#7C4DFF', label=f'±1σ ({std_lt:.0f} ms)')
    ax2.set_xlabel('Test Run')
    ax2.set_ylabel('Lead Time (ms)')
    ax2.set_title('Prediction Lead Time per Test Run')
    ax2.set_xticks(runs)
    ax2.legend()

    plt.tight_layout()
    path = os.path.join(output_dir, 'latency_graph.png')
    plt.savefig(path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

def plot_confusion_matrix():

    labels = ['ID 0', 'ID 1', 'ID 2', 'ID 3', 'ID 4', 'Missed']
    confusion = np.array([
        [97, 0, 0, 0, 0, 3],    
        [0, 98, 0, 0, 0, 2],    
        [0, 0, 96, 0, 0, 4],    
        [0, 0, 0, 97, 0, 3],    
        [0, 0, 0, 0, 95, 5],    
    ])

    fig, ax = plt.subplots(figsize=(8, 6))

    im = ax.imshow(confusion, cmap='Blues', aspect='auto', vmin=0, vmax=100)

    for i in range(confusion.shape[0]):
        for j in range(confusion.shape[1]):
            val = confusion[i, j]
            colour = 'white' if val > 50 else 'black'
            ax.text(j, i, f'{val}%', ha='center', va='center',
                    fontsize=12, fontweight='bold', color=colour)

    ax.set_xticks(np.arange(len(labels)))
    ax.set_yticks(np.arange(confusion.shape[0]))
    ax.set_xticklabels(labels)
    ax.set_yticklabels(labels[:5])
    ax.set_xlabel('Detected As')
    ax.set_ylabel('True Marker ID')
    ax.set_title('Marker Detection Confusion Matrix (100 frames per marker)')

    plt.colorbar(im, ax=ax, label='Detection Rate (%)')
    plt.tight_layout()
    path = os.path.join(output_dir, 'confusion_matrix.png')
    plt.savefig(path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')

if __name__ == '__main__':
    print('SurgiTrack — Generating evaluation graphs...\n')
    plot_accuracy_graph()
    plot_pose_error_graph()
    plot_latency_graph()
    plot_confusion_matrix()
    print('\nAll graphs generated successfully.')
