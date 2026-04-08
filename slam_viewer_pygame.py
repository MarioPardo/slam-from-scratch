#!/usr/bin/env python3
"""
SLAM Pygame Visualization Tool
Optimized: persistent map surface, ZMQ CONFLATE, cached stats, no dead code.
"""

import pygame
import zmq
import json
import math
import sys
import argparse
import os

# 5 cm² gap threshold for connecting consecutive scan points
MAP_GAP_SQ    = 0.05 * 0.05
# Frames between stat text re-renders
STATS_REFRESH = 15
SHOW_ODOM_TRAJECTORY = True


class SLAMViewer:
    def __init__(self, width=1400, height=1000, scale=50.0):
        pygame.init()

        self.width   = width
        self.height  = height
        self.scale   = scale
        self.screen  = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.display.set_caption("SLAM Viewer — Odometry (Blue)  ICP/SLAM (Green)  GT (White)")

        # Colors
        self.C_BG          = (0,   0,   0)
        self.C_GRID        = (40,  40,  40)
        self.C_MAP         = (60,  60,  60)
        self.C_ODOM        = (0,   100, 255)
        self.C_ICP         = (0,   255, 100)
        self.C_SCAN        = (255, 50,  50)
        self.C_TEXT        = (255, 255, 255)
        self.C_AXIS_X_ODOM = (150, 100, 255)
        self.C_AXIS_Y_ODOM = (100, 150, 255)
        self.C_AXIS_X_ICP  = (255, 0,   0)
        self.C_AXIS_Y_ICP  = (0,   255, 0)
        self.C_GRAPH_EDGE         = (255, 20,  147)  # bright pink  – sequential
        self.C_LOOP_CLOSURE_EDGE  = (0,   255, 255)   # cyan         – loop closure
        self.C_GRAPH_NODE  = (255, 130, 200)  # lighter pink for dots
        self.C_MEAS_EDGE   = (255, 255, 0)    # yellow for reported measurement endpoint
        self.C_GT          = (255, 255, 255)   # white for ground truth trajectory

        # Camera
        self.origin_x        = width  // 2
        self.origin_y        = height // 2
        self.camera_offset_x = 0
        self.camera_offset_y = 0
        self.auto_center     = True
        self.show_odom_trajectory = SHOW_ODOM_TRAJECTORY

        # Live data (latest message only)
        self.odom_trajectory = []   # [(x,y), ...]
        self.icp_trajectory  = []
        self.gt_trajectory   = []   # [(x,y), ...] ground truth in SLAM frame
        self.odom_poses      = []   # [(x,y,theta), ...]
        self.icp_poses       = []
        self.current_scan    = []   # [(x,y), ...] world frame

        # Full accumulated history – kept only for surface rebuild on zoom/pan
        self.all_lidar_points = []  # [(x,y), ...] half-density, world frame

        # Pose graph
        self.pose_graph_nodes = []  # [(id, x, y, theta), ...]
        self.pose_graph_edges = []  # [{'from':id,'to':id,'type':int,'meas':{dx,dy,dth}}, ...]

        # ── Persistent off-screen surfaces ──────────────────────────────
        # grid_surface  – opaque background, rebuilt on zoom/pan
        self.grid_surface = pygame.Surface((width, height))
        self._rebuild_grid_surface()

        # map_surface   – transparent, new scan points painted incrementally
        self.map_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        self.map_surface.fill((0, 0, 0, 0))

        # traj_surface  – transparent, new trajectory segments painted incrementally
        self.traj_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        self.traj_surface.fill((0, 0, 0, 0))

        # graph_surface – transparent, pose graph nodes + edges painted incrementally
        self.graph_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        self.graph_surface.fill((0, 0, 0, 0))

        # Set when zoom/pan changes – triggers a full surface rebuild next render
        self._view_dirty = False

        # Stats text cache
        self.font          = pygame.font.Font(None, 18)
        self.message_count = 0
        self._stats_cache  = []   # list of (Surface, (x, y))

    # ─────────────────────────────────────────
    #  Coordinate helper
    # ─────────────────────────────────────────
    def w2s(self, x, y):
        """World metres -> screen pixel tuple (int)."""
        ox = self.origin_x + self.camera_offset_x
        oy = self.origin_y + self.camera_offset_y
        return (int(x * self.scale + ox),
                int(-y * self.scale + oy))

    # ─────────────────────────────────────────
    #  Off-screen surface builders (called on init + zoom/pan)
    # ─────────────────────────────────────────
    def _rebuild_grid_surface(self):
        self.grid_surface.fill(self.C_BG)
        for i in range(-20, 21):
            x1, y1 = self.w2s(i, -20);  x2, y2 = self.w2s(i,  20)
            pygame.draw.line(self.grid_surface, self.C_GRID, (x1,y1), (x2,y2), 1)
            x1, y1 = self.w2s(-20, i);  x2, y2 = self.w2s( 20, i)
            pygame.draw.line(self.grid_surface, self.C_GRID, (x1,y1), (x2,y2), 1)
        pygame.draw.line(self.grid_surface, (100,0,0),
                         self.w2s(-20,0), self.w2s(20,0), 2)
        pygame.draw.line(self.grid_surface, (0,100,0),
                         self.w2s(0,-20), self.w2s(0,20), 2)

    def _rebuild_map_surface(self):
        """Full redraw of accumulated lidar onto map_surface (infrequent)."""
        self.map_surface.fill((0, 0, 0, 0))
        self._paint_connected(self.map_surface, self.all_lidar_points, self.C_MAP)

    def _rebuild_traj_surface(self):
        """Full redraw of all trajectories onto traj_surface (infrequent)."""
        self.traj_surface.fill((0, 0, 0, 0))
        if self.show_odom_trajectory:
            self._paint_polyline(self.traj_surface, self.odom_trajectory, self.C_ODOM, 2)
        self._paint_polyline(self.traj_surface, self.icp_trajectory,  self.C_ICP,  3)
        self._paint_polyline(self.traj_surface, self.gt_trajectory,   self.C_GT,   2)

    def _rebuild_graph_surface(self):
        """Full redraw of pose graph onto graph_surface (infrequent)."""
        self.graph_surface.fill((0, 0, 0, 0))
        node_pos = {nid: (x, y, theta) for nid, x, y, theta in self.pose_graph_nodes}
        for edge in self.pose_graph_edges:
            from_id = edge.get('from')
            to_id = edge.get('to')
            if from_id in node_pos and to_id in node_pos:
                fx, fy, _ = node_pos[from_id]
                tx, ty, _ = node_pos[to_id]
                color = self.C_LOOP_CLOSURE_EDGE if edge.get('type', 0) == 1 else self.C_GRAPH_EDGE
                pygame.draw.line(self.graph_surface, color, self.w2s(fx, fy), self.w2s(tx, ty), 2)
        # draw node dots at their current poses
        for _, x, y, _ in self.pose_graph_nodes:
            pygame.draw.circle(self.graph_surface, self.C_GRAPH_NODE, self.w2s(x, y), 4)

    # ─────────────────────────────────────────
    #  Low-level drawing helpers (surface-agnostic)
    # ─────────────────────────────────────────
    def _paint_connected(self, surface, points, color):
        """Draw points onto surface; connect consecutive ones if < 5 cm apart."""
        prev_s = prev_w = None
        for x, y in points:
            sx, sy = self.w2s(x, y)
            if prev_w is not None:
                dx = x - prev_w[0];  dy = y - prev_w[1]
                if dx*dx + dy*dy < MAP_GAP_SQ:
                    pygame.draw.line(surface, color, prev_s, (sx, sy), 1)
            prev_s = (sx, sy)
            prev_w = (x, y)

    def _paint_polyline(self, surface, traj, color, thickness):
        if len(traj) < 2:
            return
        pygame.draw.lines(surface, color, False,
                          [self.w2s(x, y) for x, y in traj], thickness)

    # ─────────────────────────────────────────
    #  Data update  (called once per received message)
    # ─────────────────────────────────────────
    def _update_graph(self, graph_data):
        """Incrementally paint any new keyframe nodes/edges onto graph_surface."""
        new_nodes = [(n['id'], n['x'], n['y'], n.get('theta', 0.0)) for n in graph_data.get('nodes', [])]
        new_edges = []
        for e in graph_data.get('edges', []):
            new_edges.append({'from': e['from'], 'to': e['to'], 'type': e.get('type', 0), 'meas': e.get('meas')})

        # Optimization can move existing node positions without changing node count.
        # In that case we must redraw the full graph layer; incremental append is insufficient.
        changed = (new_nodes != self.pose_graph_nodes or new_edges != self.pose_graph_edges)
        if changed:
            grew_only = (
                len(new_nodes) >= len(self.pose_graph_nodes)
                and len(new_edges) >= len(self.pose_graph_edges)
                and new_nodes[:len(self.pose_graph_nodes)] == self.pose_graph_nodes
                and new_edges[:len(self.pose_graph_edges)] == self.pose_graph_edges
            )

            if not grew_only:
                self.pose_graph_nodes = new_nodes
                self.pose_graph_edges = new_edges
                self._rebuild_graph_surface()
                return

        if len(new_nodes) > len(self.pose_graph_nodes):
            node_pos = {nid: (x, y, theta) for nid, x, y, theta in new_nodes}
            # Paint only newly added edges using node endpoints
            for i in range(len(self.pose_graph_edges), len(new_edges)):
                edge = new_edges[i]
                from_id = edge['from']
                to_id = edge['to']
                if from_id in node_pos and to_id in node_pos:
                    fx, fy, _ = node_pos[from_id]
                    tx, ty, _ = node_pos[to_id]
                    color = self.C_LOOP_CLOSURE_EDGE if edge.get('type', 0) == 1 else self.C_GRAPH_EDGE
                    pygame.draw.line(self.graph_surface, color, self.w2s(fx, fy), self.w2s(tx, ty), 2)
            # Paint only newly added node dots
            for i in range(len(self.pose_graph_nodes), len(new_nodes)):
                _, x, y, _ = new_nodes[i]
                pygame.draw.circle(self.graph_surface, self.C_GRAPH_NODE, self.w2s(x, y), 4)

        self.pose_graph_nodes = new_nodes
        self.pose_graph_edges = new_edges

    def reset_map(self, full_map_points):
        """Wipe the accumulated map and repaint from optimized projected scans."""
        self.all_lidar_points = []
        self.map_surface.fill((0, 0, 0, 0))
        pts = [(p['x'], p['y']) for p in full_map_points]
        self._paint_connected(self.map_surface, pts, self.C_MAP)
        self.all_lidar_points.extend(pts)

    def update_data(self, odom_pose, icp_pose, lidar_points, pose_graph, gt_pose=None, map_update=False):
        self.message_count += 1

        # Accumulate trajectory history from single-pose deltas
        if odom_pose:
            pt = (odom_pose['x'], odom_pose['y'])
            if self.show_odom_trajectory and len(self.odom_trajectory) >= 2:
                pygame.draw.line(self.traj_surface, self.C_ODOM,
                                 self.w2s(*self.odom_trajectory[-1]), self.w2s(*pt), 2)
            self.odom_trajectory.append(pt)
            self.odom_poses = [(odom_pose['x'], odom_pose['y'], odom_pose.get('theta', 0))]

        if icp_pose:
            pt = (icp_pose['x'], icp_pose['y'])
            if len(self.icp_trajectory) >= 2:
                pygame.draw.line(self.traj_surface, self.C_ICP,
                                 self.w2s(*self.icp_trajectory[-1]), self.w2s(*pt), 3)
            self.icp_trajectory.append(pt)
            self.icp_poses = [(icp_pose['x'], icp_pose['y'], icp_pose.get('theta', 0))]

        if gt_pose:
            pt = (gt_pose['x'], gt_pose['y'])
            if len(self.gt_trajectory) >= 2:
                pygame.draw.line(self.traj_surface, self.C_GT,
                                 self.w2s(*self.gt_trajectory[-1]), self.w2s(*pt), 2)
            self.gt_trajectory.append(pt)

        self.current_scan = [(p['x'], p['y']) for p in lidar_points]

        # Only accumulate map points on keyframe scans — avoids smear from ICP drift
        if map_update:
            new_pts = self.current_scan[::2]
            if new_pts:
                join = ([self.all_lidar_points[-1]] + new_pts
                        if self.all_lidar_points else new_pts)
                self._paint_connected(self.map_surface, join, self.C_MAP)
                self.all_lidar_points.extend(new_pts)

        self._update_graph(pose_graph)

        # Auto-center: follow ICP pose, fall back to odom
        if self.auto_center:
            ref = self.icp_trajectory or self.odom_trajectory
            if ref:
                lx, ly = ref[-1]
                new_ox = -int(lx * self.scale)
                new_oy =  int(ly * self.scale)
                if new_ox != self.camera_offset_x or new_oy != self.camera_offset_y:
                    self.camera_offset_x = new_ox
                    self.camera_offset_y = new_oy
                    self._view_dirty = True

    # ─────────────────────────────────────────
    #  Render  (called every frame)
    # ─────────────────────────────────────────
    def _draw_latest_dot(self, trajectory, color):
        if trajectory:
            pygame.draw.circle(self.screen, color, self.w2s(*trajectory[-1]), 4)

    def _refresh_stats(self):
        ref = self.icp_trajectory or self.odom_trajectory
        px, py = ref[-1] if ref else (0.0, 0.0)
        lines = [
            f"Messages : {self.message_count}",
            f"Map pts  : {len(self.all_lidar_points)}",
            f"Scan pts : {len(self.current_scan)}",
            f"Scale    : {self.scale:.1f} px/m",
            f"Pose     : ({px:.2f}, {py:.2f})",
            "",
            "+/-  Zoom      Arrows  Pan",
            "C  auto-center    R  reset    Q  quit",
        ]
        self._stats_cache = []
        y = 10
        for line in lines:
            if not line:
                y += 6;  continue
            surf = self.font.render(line, True, self.C_TEXT)
            self._stats_cache.append((surf, (10, y)))
            y += 18

    def render(self, frame: int):
        # Full rebuild if zoom/pan changed
        if self._view_dirty:
            self._rebuild_grid_surface()
            self._rebuild_map_surface()
            self._rebuild_traj_surface()
            self._rebuild_graph_surface()
            self._view_dirty = False

        # Blit pre-rendered layers (effectively O(1) each)
        self.screen.blit(self.grid_surface, (0, 0))
        self.screen.blit(self.map_surface,  (0, 0))
        self.screen.blit(self.traj_surface, (0, 0))
        self.screen.blit(self.graph_surface, (0, 0))

        # Current scan: live red ring, redrawn each frame (~180 pts)
        self._paint_connected(self.screen, self.current_scan, self.C_SCAN)

        if self.show_odom_trajectory:
            self._draw_latest_dot(self.odom_trajectory, self.C_ODOM)
        self._draw_latest_dot(self.icp_trajectory, self.C_ICP)
        self._draw_latest_dot(self.gt_trajectory, self.C_GT)

        # Stats overlay (re-rendered every STATS_REFRESH frames)
        if frame % STATS_REFRESH == 0:
            self._refresh_stats()
        for surf, pos in self._stats_cache:
            self.screen.blit(surf, pos)

        pygame.display.flip()

    # ─────────────────────────────────────────
    #  Input
    # ─────────────────────────────────────────
    def _handle_resize(self, new_w, new_h):
        self.width   = new_w
        self.height  = new_h
        self.origin_x = new_w // 2
        self.origin_y = new_h // 2
        self.screen = pygame.display.set_mode((new_w, new_h), pygame.RESIZABLE)
        self.grid_surface  = pygame.Surface((new_w, new_h))
        self.map_surface   = pygame.Surface((new_w, new_h), pygame.SRCALPHA)
        self.traj_surface  = pygame.Surface((new_w, new_h), pygame.SRCALPHA)
        self.graph_surface = pygame.Surface((new_w, new_h), pygame.SRCALPHA)
        self.map_surface.fill((0, 0, 0, 0))
        self.traj_surface.fill((0, 0, 0, 0))
        self.graph_surface.fill((0, 0, 0, 0))
        self._view_dirty = True

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.VIDEORESIZE:
                self._handle_resize(event.w, event.h)
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    return False
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    self.scale *= 1.2;  self._view_dirty = True
                elif event.key == pygame.K_MINUS:
                    self.scale /= 1.2;  self._view_dirty = True
                elif event.key == pygame.K_LEFT:
                    self.camera_offset_x += 50;  self.auto_center = False;  self._view_dirty = True
                elif event.key == pygame.K_RIGHT:
                    self.camera_offset_x -= 50;  self.auto_center = False;  self._view_dirty = True
                elif event.key == pygame.K_UP:
                    self.camera_offset_y += 50;  self.auto_center = False;  self._view_dirty = True
                elif event.key == pygame.K_DOWN:
                    self.camera_offset_y -= 50;  self.auto_center = False;  self._view_dirty = True
                elif event.key == pygame.K_c:
                    self.auto_center = not self.auto_center
                elif event.key == pygame.K_r:
                    self.scale = 50.0
                    self.camera_offset_x = 0;  self.camera_offset_y = 0
                    self.auto_center = True;   self._view_dirty = True
        return True


# ─────────────────────────────────────────────
#  Trajectory image export
# ─────────────────────────────────────────────
def save_trajectory_image(icp_trajectory, gt_trajectory, filename="slam_output.png"):
    """
    Render trajectories to a PNG using matplotlib.

    Parameters
    ----------
    icp_trajectory : list of (x, y)  – ICP/SLAM path (pink)
    gt_trajectory  : list of (x, y)  – ground truth path (black)
    filename       : output PNG path
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 10), facecolor="white")
    ax.set_aspect("equal")
    ax.set_facecolor("white")

    def _plot_traj(traj, color, label, lw=1.5, zorder=2):
        if len(traj) >= 2:
            xs, ys = zip(*traj)
            ax.plot(xs, ys, color=color, linewidth=lw, label=label, zorder=zorder)

    _plot_traj(gt_trajectory,  "black",   "Ground Truth", lw=1.5, zorder=3)
    _plot_traj(icp_trajectory, "hotpink", "ICP / SLAM",   lw=2.0, zorder=4)

    for traj, color in ((gt_trajectory, "black"), (icp_trajectory, "hotpink")):
        if traj:
            ax.plot(traj[0][0], traj[0][1], "o", color=color, markersize=7, zorder=6)

    # Fit axes tightly to all data with a small margin
    all_x, all_y = [], []
    for traj in (icp_trajectory, gt_trajectory):
        if traj:
            xs, ys = zip(*traj)
            all_x.extend(xs);  all_y.extend(ys)
    if all_x:
        margin = max((max(all_x) - min(all_x)) * 0.05,
                     (max(all_y) - min(all_y)) * 0.05, 0.5)
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    ax.set_xlabel("X (m)");  ax.set_ylabel("Y (m)")
    ax.set_title("SLAM Trajectories")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, color="#cccccc", linewidth=0.5)

    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    plt.close(fig)
    print(f"[save] Trajectory image written to: {filename}")


def save_localization_error_linegraph(
    icp_trajectory,
    gt_trajectory,
    filename="slam_error_over_time.png",
    max_dist_m=None,
    sample_every_frames=1,
):
    """Render XY localization error as a line graph normalised by max_dist_m.

    Parameters
    ----------
    max_dist_m : float or None
        Denominator for normalisation.  Pass the map diagonal for square/rect
        environments, or the diameter for circular ones.  Defaults to
        hypot(2.5, 2.5) — the diagonal of the 2.5 x 2.5 m default world.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if sample_every_frames <= 0:
        sample_every_frames = 1

    paired_count = min(len(icp_trajectory), len(gt_trajectory))
    if paired_count == 0:
        print("[save] No paired GT/ICP samples available for error line graph")
        return

    if max_dist_m is None:
        max_dist_m = math.hypot(2.5, 2.5)  # default: 2.5 x 2.5 m map diagonal
    if max_dist_m <= 1e-9:
        print("[save] max_dist_m must be positive")
        return

    frame_indices = []
    normalized_errors = []

    for i in range(0, paired_count, sample_every_frames):
        ix, iy = icp_trajectory[i]
        gx, gy = gt_trajectory[i]
        err_m = math.hypot(ix - gx, iy - gy)
        frame_indices.append(i)
        normalized_errors.append(err_m / max_dist_m)

    fig, ax = plt.subplots(figsize=(12, 5), facecolor="white")
    ax.set_facecolor("white")
    ax.plot(frame_indices, normalized_errors, color="steelblue", linewidth=1.2)
    ax.fill_between(frame_indices, normalized_errors, alpha=0.15, color="steelblue")
    ax.set_xlabel("Frame Index")
    ax.set_ylabel(f"Normalised XY Error  (error / {max_dist_m:.3f} m)")
    ax.set_title("Localisation Error Over Time")
    ax.set_ylim(bottom=0)
    ax.grid(True, axis="y", color="#cccccc", linewidth=0.5)
    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    plt.close(fig)
    print(f"[save] Error line graph written to: {filename}")


def save_ate_summary(
    icp_trajectory,
    gt_trajectory,
    filename="slam_ate_summary.png",
):
    """Compute and save ATE (Absolute Trajectory Error).

    Produces a two-row figure:
      - Top:    per-frame ATE in metres over time (line graph)
      - Bottom: aggregate stats bar chart (left) + table (right)
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    paired_count = min(len(icp_trajectory), len(gt_trajectory))
    if paired_count == 0:
        print("[save] No paired GT/ICP samples available for ATE summary")
        return

    errors = [
        math.hypot(icp_trajectory[i][0] - gt_trajectory[i][0],
                   icp_trajectory[i][1] - gt_trajectory[i][1])
        for i in range(paired_count)
    ]

    rmse   = math.sqrt(sum(e * e for e in errors) / len(errors))
    mean   = sum(errors) / len(errors)
    sorted_e = sorted(errors)
    n = len(sorted_e)
    median = (sorted_e[n // 2] if n % 2 else
              (sorted_e[n // 2 - 1] + sorted_e[n // 2]) / 2)
    std    = math.sqrt(sum((e - mean) ** 2 for e in errors) / len(errors))
    maximum = max(errors)
    final   = errors[-1]

    labels = ["RMSE", "Mean", "Median", "Std", "Max", "Final"]
    values = [rmse, mean, median, std, maximum, final]

    fig = plt.figure(figsize=(12, 8), facecolor="white")
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.4, wspace=0.3)

    ax_line  = fig.add_subplot(gs[0, :])   # top row: full width
    ax_bar   = fig.add_subplot(gs[1, 0])   # bottom left
    ax_table = fig.add_subplot(gs[1, 1])   # bottom right

    # ── Top: ATE over time ──────────────────────────────────────────────
    ax_line.set_facecolor("white")
    ax_line.plot(range(paired_count), errors, color="steelblue", linewidth=1.2)
    ax_line.fill_between(range(paired_count), errors, alpha=0.15, color="steelblue")
    ax_line.axhline(rmse,   color="crimson",     linewidth=1.0, linestyle="--", label=f"RMSE {rmse:.3f} m")
    ax_line.axhline(mean,   color="darkorange",  linewidth=1.0, linestyle=":",  label=f"Mean {mean:.3f} m")
    ax_line.axhline(maximum, color="grey",       linewidth=0.8, linestyle="-.", label=f"Max {maximum:.3f} m")
    ax_line.set_xlabel("Frame Index")
    ax_line.set_ylabel("ATE (m)")
    ax_line.set_title("ATE Over Time")
    ax_line.set_ylim(bottom=0)
    ax_line.legend(fontsize=8, loc="upper left")
    ax_line.grid(True, axis="y", color="#cccccc", linewidth=0.5)

    # ── Bottom left: aggregate bar chart ────────────────────────────────
    ax_bar.set_facecolor("white")
    bars = ax_bar.bar(labels, values, color="steelblue", width=0.5)
    ax_bar.set_ylabel("Error (m)")
    ax_bar.set_title("ATE Summary Statistics")
    ax_bar.set_ylim(bottom=0)
    ax_bar.grid(True, axis="y", color="#cccccc", linewidth=0.5)
    for bar, val in zip(bars, values):
        ax_bar.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + maximum * 0.01,
                    f"{val:.4f}", ha="center", va="bottom", fontsize=8)

    # ── Bottom right: stats table ────────────────────────────────────────
    ax_table.axis("off")
    tbl = ax_table.table(
        cellText=[[f"{v:.4f} m"] for v in values],
        rowLabels=labels,
        colLabels=["Value"],
        loc="center",
        cellLoc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1.2, 1.6)

    fig.savefig(filename, dpi=150)
    plt.close(fig)

    print(f"[save] ATE summary written to: {filename}")
    print(f"       RMSE={rmse:.4f} m  mean={mean:.4f} m  median={median:.4f} m  "
          f"std={std:.4f} m  max={maximum:.4f} m  final={final:.4f} m")


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="SLAM Pygame Viewer")
    parser.add_argument("--save", metavar="FILE", nargs="?",
                        const="slam_output.png", default=None,
                        help="On exit, save a trajectory PNG (default: slam_output.png)")
    parser.add_argument("--maxdist", metavar="METRES", type=float, default=None,
                        help="Max distance used to normalise the error graph. "
                             "Use the map diagonal for rectangular worlds, or the "
                             "diameter for circular ones. "
                             "Default: hypot(2.5, 2.5) ≈ 3.54 m.")
    args = parser.parse_args()

    print("=== SLAM Pygame Viewer (optimized) ===")
    print("Connecting to tcp://localhost:5556 ...")

    context    = zmq.Context()
    subscriber = context.socket(zmq.SUB)

    # CONFLATE: only the most-recent message is kept in the ZMQ buffer.
    # This prevents the viewer falling behind when SLAM publishes faster than we render.
    subscriber.setsockopt(zmq.CONFLATE, 1)

    subscriber.connect("tcp://localhost:5556")
    subscriber.subscribe(b"visualization")

    viewer = SLAMViewer()
    clock  = pygame.time.Clock()
    frame  = 0

    print("Running. Q / ESC to quit.")

    while True:
        if not viewer.handle_input():
            break

        # Drain any buffered messages; with CONFLATE there will be at most one,
        # but drain defensively to always get the very latest.
        # C++ publishes as a single frame: "topic {json}"  (space-separated)
        latest_data = None
        while True:
            try:
                msg = subscriber.recv(flags=zmq.NOBLOCK)
                full = msg.decode('utf-8')
                space = full.find(' ')
                if space != -1:
                    latest_data = json.loads(full[space + 1:])
            except zmq.Again:
                break
            except Exception as e:
                print(f"[Recv] {e}")
                break

        if latest_data is not None:
            if latest_data.get('clear_map', False):
                viewer.reset_map(latest_data.get('full_map_points', []))
            viewer.update_data(
                latest_data.get('odom_pose',    None),
                latest_data.get('icp_pose',     None),
                latest_data.get('lidar_points', []),
                latest_data.get('pose_graph',   {'nodes': [], 'edges': []}),
                latest_data.get('gt_pose',      None),
                latest_data.get('map_update',   False),
            )

        viewer.render(frame)
        clock.tick(30)
        frame += 1

    subscriber.close()
    context.term()
    pygame.quit()

    if args.save:
        save_trajectory_image(
            viewer.icp_trajectory,
            viewer.gt_trajectory,
            filename=args.save,
        )
        base, ext = os.path.splitext(args.save)
        error_file = f"{base}_error_line{ext or '.png'}"
        save_localization_error_linegraph(
            viewer.icp_trajectory,
            viewer.gt_trajectory,
            filename=error_file,
            max_dist_m=args.maxdist,
            sample_every_frames=1,
        )
        ate_file = f"{base}_ate_summary{ext or '.png'}"
        save_ate_summary(
            viewer.icp_trajectory,
            viewer.gt_trajectory,
            filename=ate_file,
        )

    sys.exit(0)


if __name__ == "__main__":
    main()
