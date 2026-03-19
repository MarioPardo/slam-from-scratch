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

# 5 cm² gap threshold for connecting consecutive scan points
MAP_GAP_SQ    = 0.05 * 0.05
# Frames between stat text re-renders
STATS_REFRESH = 15


class SLAMViewer:
    def __init__(self, width=1400, height=1000, scale=50.0):
        pygame.init()

        self.width   = width
        self.height  = height
        self.scale   = scale
        self.screen  = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SLAM Viewer - Odometry (Blue) vs ICP (Green)")

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

        # Camera
        self.origin_x        = width  // 2
        self.origin_y        = height // 2
        self.camera_offset_x = 0
        self.camera_offset_y = 0
        self.auto_center     = True

        # Live data (latest message only)
        self.odom_trajectory = []   # [(x,y), ...]
        self.icp_trajectory  = []
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
        """Full redraw of both trajectories onto traj_surface (infrequent)."""
        self.traj_surface.fill((0, 0, 0, 0))
        self._paint_polyline(self.traj_surface, self.odom_trajectory, self.C_ODOM, 2)
        self._paint_polyline(self.traj_surface, self.icp_trajectory,  self.C_ICP,  3)

    def _rebuild_graph_surface(self):
        """Full redraw of pose graph onto graph_surface (infrequent)."""
        self.graph_surface.fill((0, 0, 0, 0))
        node_pos = {nid: (x, y, theta) for nid, x, y, theta in self.pose_graph_nodes}
        for edge in self.pose_graph_edges:
            # Draw ONLY the reported measurement transform: from_node ⊕ meas -> measured endpoint
            from_id = edge.get('from'); meas = edge.get('meas') if isinstance(edge, dict) else None
            if meas and from_id in node_pos:
                fx, fy, ftheta = node_pos[from_id]
                dx = meas.get('dx', 0.0); dy = meas.get('dy', 0.0)
                cos_t = math.cos(ftheta); sin_t = math.sin(ftheta)
                gx = fx + (cos_t*dx - sin_t*dy)
                gy = fy + (sin_t*dx + cos_t*dy)
                color = self.C_LOOP_CLOSURE_EDGE if edge.get('type', 0) == 1 else self.C_GRAPH_EDGE
                # draw measured transform as the primary edge (thin)
                pygame.draw.line(self.graph_surface, self.C_MEAS_EDGE, self.w2s(fx, fy), self.w2s(gx, gy), 2)
                pygame.draw.circle(self.graph_surface, self.C_MEAS_EDGE, self.w2s(gx, gy), 3)
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

        if len(new_nodes) > len(self.pose_graph_nodes):
            node_pos = {nid: (x, y, theta) for nid, x, y, theta in new_nodes}
            # Paint only newly added edges using reported measurements
            for i in range(len(self.pose_graph_edges), len(new_edges)):
                edge = new_edges[i]
                from_id = edge['from']
                meas = edge.get('meas')
                if meas and from_id in node_pos:
                    fx, fy, ftheta = node_pos[from_id]
                    dx = meas.get('dx', 0.0); dy = meas.get('dy', 0.0)
                    cos_t = math.cos(ftheta); sin_t = math.sin(ftheta)
                    gx = fx + (cos_t*dx - sin_t*dy)
                    gy = fy + (sin_t*dx + cos_t*dy)
                    pygame.draw.line(self.graph_surface, self.C_MEAS_EDGE, self.w2s(fx, fy), self.w2s(gx, gy), 2)
                    pygame.draw.circle(self.graph_surface, self.C_MEAS_EDGE, self.w2s(gx, gy), 3)
            # Paint only newly added node dots
            for i in range(len(self.pose_graph_nodes), len(new_nodes)):
                _, x, y, _ = new_nodes[i]
                pygame.draw.circle(self.graph_surface, self.C_GRAPH_NODE, self.w2s(x, y), 4)

        self.pose_graph_nodes = new_nodes
        self.pose_graph_edges = new_edges

    def update_data(self, odom_traj, icp_traj, lidar_points, pose_graph):
        self.message_count += 1

        self.odom_trajectory = [(p['x'], p['y']) for p in odom_traj]
        self.icp_trajectory  = [(p['x'], p['y']) for p in icp_traj]
        self.odom_poses      = [(p['x'], p['y'], p.get('theta', 0)) for p in odom_traj]
        self.icp_poses       = [(p['x'], p['y'], p.get('theta', 0)) for p in icp_traj]
        self.current_scan    = [(p['x'], p['y']) for p in lidar_points]

        # Paint only NEW half-density map points onto map_surface (O(n_new) not O(n_total))
        new_pts = self.current_scan[::2]
        if new_pts:
            join = ([self.all_lidar_points[-1]] + new_pts
                    if self.all_lidar_points else new_pts)
            self._paint_connected(self.map_surface, join, self.C_MAP)
            self.all_lidar_points.extend(new_pts)

        # Paint only the newest trajectory segment onto traj_surface
        if len(self.odom_trajectory) >= 2:
            pygame.draw.line(self.traj_surface, self.C_ODOM,
                             self.w2s(*self.odom_trajectory[-2]),
                             self.w2s(*self.odom_trajectory[-1]), 2)
        if len(self.icp_trajectory) >= 2:
            pygame.draw.line(self.traj_surface, self.C_ICP,
                             self.w2s(*self.icp_trajectory[-2]),
                             self.w2s(*self.icp_trajectory[-1]), 3)

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
    def _draw_robot_axes(self, x, y, theta, length, cx, cy, thick):
        ox, oy = self.w2s(x, y)
        ex, ey = self.w2s(x + length * math.cos(theta),
                          y + length * math.sin(theta))
        lx, ly = self.w2s(x + length * math.cos(theta + math.pi/2),
                          y + length * math.sin(theta + math.pi/2))
        pygame.draw.line(self.screen, cx, (ox, oy), (ex, ey), thick)
        pygame.draw.line(self.screen, cy, (ox, oy), (lx, ly), thick)

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

        # Robot axes
        if self.odom_poses:
            x, y, th = self.odom_poses[-1]
            self._draw_robot_axes(x, y, th, 0.3,  self.C_AXIS_X_ODOM, self.C_AXIS_Y_ODOM, 2)
        if self.icp_poses:
            x, y, th = self.icp_poses[-1]
            self._draw_robot_axes(x, y, th, 0.35, self.C_AXIS_X_ICP,  self.C_AXIS_Y_ICP,  3)

        # Stats overlay (re-rendered every STATS_REFRESH frames)
        if frame % STATS_REFRESH == 0:
            self._refresh_stats()
        for surf, pos in self._stats_cache:
            self.screen.blit(surf, pos)

        pygame.display.flip()

    # ─────────────────────────────────────────
    #  Input
    # ─────────────────────────────────────────
    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
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
#  Entry point
# ─────────────────────────────────────────────
def main():
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
            viewer.update_data(
                latest_data.get('odom_trajectory', []),
                latest_data.get('icp_trajectory',  []),
                latest_data.get('lidar_points',     []),
                latest_data.get('pose_graph',       {'nodes': [], 'edges': []}),
            )

        viewer.render(frame)
        clock.tick(30)
        frame += 1

    subscriber.close()
    context.term()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
