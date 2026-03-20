"""Visualization utilities for datasets."""

import matplotlib.pyplot as plt
import numpy as np


def get_color_for_class(class_id):
    """Get consistent color for a given class ID.

    Args:
        class_id: Integer class ID.

    Returns:
        RGB color tuple.
    """
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    return colors[int(class_id) % 10]


def draw_bbox_bev(ax, x, y, yaw, length, width, color="r", linewidth=2):
    """Draw a 2D bounding box in bird's eye view.

    Args:
        ax: Matplotlib axis object.
        x: X coordinate of box center (forward).
        y: Y coordinate of box center (left).
        yaw: Rotation angle in radians.
        length: Box length.
        width: Box width.
        color: Line color for the box.
        linewidth: Line width for the box.
    """
    # Calculate corners
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    corners = np.array(
        [
            [length / 2, width / 2],
            [length / 2, -width / 2],
            [-length / 2, -width / 2],
            [-length / 2, width / 2],
            [length / 2, width / 2],  # Close the box
        ]
    )

    # Rotate and translate
    rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    rotated_corners = corners @ rotation_matrix.T
    rotated_corners[:, 0] += x
    rotated_corners[:, 1] += y

    ax.plot(rotated_corners[:, 0], rotated_corners[:, 1], color=color, linewidth=linewidth)
    # Draw direction indicator
    ax.arrow(x, y, cos_yaw * length / 3, sin_yaw * length / 3, head_width=0.3, head_length=0.2, fc="blue", ec="blue")


def draw_2d_bbox(ax, xmin, ymin, xmax, ymax, class_id, color="red", linewidth=2):
    """Draw a 2D bounding box on an image.

    Args:
        ax: Matplotlib axis object.
        xmin: Minimum x coordinate (left).
        ymin: Minimum y coordinate (top).
        xmax: Maximum x coordinate (right).
        ymax: Maximum y coordinate (bottom).
        class_id: Class ID of the object.
        color: Box color.
        linewidth: Line width.
    """
    width = xmax - xmin
    height = ymax - ymin
    rect = plt.Rectangle((xmin, ymin), width, height, fill=False, edgecolor=color, linewidth=linewidth)
    ax.add_patch(rect)
    ax.text(
        xmin,
        ymin - 5,
        f"Class {int(class_id)}",
        color=color,
        fontsize=8,
        weight="bold",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7),
    )


def visualize_lidar_sample(point_cloud, objects, title="Lidar Sample", save_path=None):
    """Visualize lidar point cloud with 3D bounding boxes.

    Args:
        point_cloud: Array of points with shape (N, 4) containing [x, y, z, intensity].
        objects: Array of objects with shape (M, 8) containing
                 [class_id, x, y, z, yaw, length, width, height].
        title: Title for the plot.
        save_path: Optional path to save the figure.

    Returns:
        Matplotlib figure object.
    """

    fig = plt.figure(figsize=(15, 10))

    # Bird's eye view
    ax1 = fig.add_subplot(211)
    ax1.scatter(point_cloud[:, 0], point_cloud[:, 1], c=point_cloud[:, 2], s=0.1, cmap="viridis")
    ax1.set_xlabel("X (forward)")
    ax1.set_xlim([-40, 40])
    ax1.set_ylabel("Y (left)")
    ax1.set_ylim([-15, 15])
    ax1.set_title(f"Bird's Eye View - Point Cloud\n{len(objects)} objects, {len(point_cloud)} points")
    ax1.set_aspect("equal", adjustable="box")

    # Use only relevant object attributes
    objects = objects[:, :8]

    # Draw bounding boxes with color coding by class
    for obj in objects:
        class_id, x, y, z, yaw, length, width, height = obj
        color = get_color_for_class(class_id)
        draw_bbox_bev(ax1, x, y, yaw, length, width, color=color)

    # Front view - only show points in front of vehicle (X > 0)
    ax2 = fig.add_subplot(212)
    # Filter points with x > 0 (in front of vehicle)
    front_mask = point_cloud[:, 0] > 0
    filtered_points = point_cloud[front_mask]
    ax2.scatter(filtered_points[:, 1], filtered_points[:, 2], c=filtered_points[:, 3], s=0.1, cmap="plasma")
    ax2.set_xlabel("Y (left)")
    ax2.set_ylabel("Z (up)")
    ax2.set_title(f"Front View - Point Cloud (X > 0)\n{len(objects)} objects, {len(filtered_points)} points")
    ax2.set_xlim([15, -15])
    ax2.set_ylim([-2, 3])
    ax2.set_aspect("equal", adjustable="box")

    # Draw bounding boxes in front view (show width and height)
    for obj in objects:
        class_id, x, y, z, yaw, length, width, height = obj
        # draw only if in front of vehicle
        if x < 0:
            continue
        color = get_color_for_class(class_id)
        # Project box to y-z plane (front view)
        # Draw rectangle for width and height
        y_corners = [y - width / 2, y + width / 2, y + width / 2, y - width / 2, y - width / 2]
        z_corners = [z - height / 2, z - height / 2, z + height / 2, z + height / 2, z - height / 2]
        ax2.plot(y_corners, z_corners, color=color, linewidth=2)

    plt.suptitle(f"{title}\nObjects: {len(objects)}")
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)

    return fig
