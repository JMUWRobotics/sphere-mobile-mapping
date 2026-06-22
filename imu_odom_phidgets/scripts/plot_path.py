#!/usr/bin/env python3
"""
Plot the pose path from /posePub_merged topic
"""
import sys
import os
import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def extract_poses(bag_file, topic="/posePub_merged"):
    """Extract x, y, z positions from topic in bag"""
    x_data = []
    y_data = []
    z_data = []
    timestamps = []
    
    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            for topic_name, msg, t in bag.read_messages(topics=[topic]):
                x_data.append(msg.pose.pose.position.x)
                y_data.append(msg.pose.pose.position.y)
                z_data.append(msg.pose.pose.position.z)
                timestamps.append(t.to_sec())
    except Exception as e:
        print(f"Error reading bag: {e}")
        return None, None, None, None
    
    if not x_data:
        print(f"No messages found on topic {topic}")
        return None, None, None, None
    
    return np.array(x_data), np.array(y_data), np.array(z_data), np.array(timestamps)

def plot_paths(bag_files, title="Pose Paths"):
    """Plot pose paths from one or more bag files."""
    fig = plt.figure(figsize=(16, 10))
    
    # 3D plot
    ax3d = fig.add_subplot(2, 2, 1, projection='3d')
    
    # 2D XY plot
    ax_xy = fig.add_subplot(2, 2, 2)
    
    # XZ plot
    ax_xz = fig.add_subplot(2, 2, 3)
    
    # YZ plot
    ax_yz = fig.add_subplot(2, 2, 4)
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(bag_files)))
    
    stats = {}
    
    for idx, (bag_file, color) in enumerate(zip(bag_files, colors)):
        x, y, z, t = extract_poses(bag_file)
        
        if x is None:
            print(f"Skipping {bag_file}")
            continue
        
        label = bag_file.split('/')[-1]
        
        # 3D plot
        ax3d.plot(x, y, z, color=color, label=label, linewidth=2)
        ax3d.scatter(x[0], y[0], z[0], color='g', s=100, marker='o', edgecolors='black', linewidths=2)  # start
        ax3d.scatter(x[-1], y[-1], z[-1], color='r', s=100, marker='s', edgecolors='black', linewidths=2)  # end
        
        # 2D XY plot
        ax_xy.plot(x, y, color=color, label=label, linewidth=2)
        ax_xy.scatter(x[0], y[0], color='g', s=100, marker='o', edgecolors='black', linewidths=2)
        ax_xy.scatter(x[-1], y[-1], color='r', s=100, marker='s', edgecolors='black', linewidths=2)
        
        # XZ plot
        ax_xz.plot(x, z, color=color, label=label, linewidth=2)
        ax_xz.scatter(x[0], z[0], color='g', s=100, marker='o', edgecolors='black', linewidths=2)
        ax_xz.scatter(x[-1], z[-1], color='r', s=100, marker='s', edgecolors='black', linewidths=2)
        
        # YZ plot
        ax_yz.plot(y, z, color=color, label=label, linewidth=2)
        ax_yz.scatter(y[0], z[0], color='g', s=100, marker='o', edgecolors='black', linewidths=2)
        ax_yz.scatter(y[-1], z[-1], color='r', s=100, marker='s', edgecolors='black', linewidths=2)
        
        # Calculate statistics
        distance = np.sqrt(np.sum(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2))
        displacement = np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2 + (z[-1] - z[0])**2)
        duration = t[-1] - t[0]
        
        stats[label] = {
            'distance': distance,
            'displacement': displacement,
            'duration': duration,
            'samples': len(x),
            'start': (x[0], y[0], z[0]),
            'end': (x[-1], y[-1], z[-1]),
        }
    
    # Configure axes
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('3D Path')
    ax3d.legend()
    ax3d.grid(True)
    
    ax_xy.set_xlabel('X (m)')
    ax_xy.set_ylabel('Y (m)')
    ax_xy.set_title('XY Projection (Top View)')
    ax_xy.legend()
    ax_xy.grid(True)
    ax_xy.axis('equal')
    
    ax_xz.set_xlabel('X (m)')
    ax_xz.set_ylabel('Z (m)')
    ax_xz.set_title('XZ Projection (Side View)')
    ax_xz.legend()
    ax_xz.grid(True)
    ax_xz.axis('equal')
    
    ax_yz.set_xlabel('Y (m)')
    ax_yz.set_ylabel('Z (m)')
    ax_yz.set_title('YZ Projection (Front View)')
    ax_yz.legend()
    ax_yz.grid(True)
    ax_yz.axis('equal')
    
    fig.suptitle(title, fontsize=16, fontweight='bold')
    plt.tight_layout()
    
    # Save fig in the same directory as script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = title.replace(" vs ", "_vs_").replace(" ", "_") + ".png"
    filepath = os.path.join(script_dir, filename)
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    print(f"\nFigure saved to: {filepath}")
    
    print("\n" + "="*80)
    print(f"Path Stats for {title}")
    print("="*80)
    for label, stat in stats.items():
        print(f"\nFile: {label}")
        print(f"  Samples: {stat['samples']}")
        print(f"  Duration: {stat['duration']:.2f} seconds")
        print(f"  Path distance: {stat['distance']:.3f} m")
        print(f"  Displacement: {stat['displacement']:.3f} m")
        print(f"  Start: ({stat['start'][0]:.3f}, {stat['start'][1]:.3f}, {stat['start'][2]:.3f})")
        print(f"  End:   ({stat['end'][0]:.3f}, {stat['end'][1]:.3f}, {stat['end'][2]:.3f})")
    
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: plot_path.py <bag_file> [<bag_file2> ...]")
        print("\nPlots the pose path from /posePub_merged topic in bag file(s).")
        sys.exit(1)
    
    bag_files = sys.argv[1:]
    title = " vs ".join([f.split('/')[-1] for f in bag_files])
    plot_paths(bag_files, title=title)
