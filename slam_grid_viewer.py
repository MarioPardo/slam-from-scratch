#!/usr/bin/env python3
"""Occupancy grid viewer for slam_core (with zoom/pan).

- Subscribes to ZMQ topic 'grid_raw' on tcp://localhost:5556
- Expects JSON: {"width": int, "height": int, "resolution": float, "cells": [[p00, ...]]}
- Renders one occupancy grid at a time (choose via --icp or --odom)

Controls
--------
+ / - : zoom in/out
Arrows: pan
C     : toggle auto-fit (re-center + fit to panel on new grids)
R     : reset view
S     : screenshot
Q/ESC : quit

CLI
---
--icp : show optimized_cells (default)
--odom: show cells
"""

import sys
import json
from datetime import datetime
from typing import Tuple, Optional
import argparse

import pygame
import zmq

import numpy as np


WINDOW_WIDTH = 1800
WINDOW_HEIGHT = 960
MARGIN = 40
BG_COLOR = (0, 0, 0)
GRID_LINE_COLOR = (40, 40, 40)
PANEL_BORDER_COLOR = (90, 90, 90)
TEXT_COLOR = (220, 220, 220)

ZOOM_STEP = 1.2
PAN_STEP_PX = 60
MIN_ZOOM = 0.05
MAX_ZOOM = 60.0

HUD_HEIGHT = 36
OOB_PROB = 0.5


def prob_to_color(p: float) -> Tuple[int, int, int]:
    """Map occupancy probability to grayscale.

    p=0.0 -> black (free), p=0.5 -> mid gray (unknown), p=1.0 -> white (occupied)
    """
    p = max(0.0, min(1.0, p))
    value = int(255 * p)
    return (value, value, value)


class ViewState:
    def __init__(self) -> None:
        # zoom multiplier applied on top of a per-grid "fit-to-panel" base scale
        self.zoom: float = 1.0
        # center in grid cell coordinates (gx, gy). gy=0 is bottom row.
        self.center_x: Optional[float] = None
        self.center_y: Optional[float] = None
        self.auto_fit: bool = True

    def reset(self) -> None:
        self.zoom = 1.0
        self.center_x = None
        self.center_y = None
        self.auto_fit = True

    def ensure_center(self, width: int, height: int) -> None:
        if self.center_x is None or self.center_y is None:
            self.center_x = (width - 1) / 2.0 if width > 0 else 0.0
            self.center_y = (height - 1) / 2.0 if height > 0 else 0.0


def _effective_scale(panel_w: int, panel_h: int, grid_width: int, grid_height: int, zoom: float) -> float:
    if panel_w <= 0 or panel_h <= 0 or grid_width <= 0 or grid_height <= 0:
        return 1e-6
    base_scale = min(panel_w / grid_width, panel_h / grid_height)
    return max(1e-6, base_scale * float(zoom))


def _cells_to_numpy(cells, width: int, height: int) -> np.ndarray:
    """Convert JSON cells to a (height,width) float32 array.

    Defensive about ragged lists; missing values default to 0.5 (unknown).
    """
    arr = np.full((max(0, height), max(0, width)), 0.5, dtype=np.float32)
    if width <= 0 or height <= 0 or not isinstance(cells, list):
        return arr
    h = min(height, len(cells))
    for gy in range(h):
        row = cells[gy]
        if not isinstance(row, list):
            continue
        w = min(width, len(row))
        if w <= 0:
            continue
        # Use a local conversion to avoid numpy trying to ingest the whole ragged list.
        try:
            arr[gy, :w] = np.asarray(row[:w], dtype=np.float32)
        except Exception:
            # Fall back to per-element conversion if any values are non-numeric.
            for gx in range(w):
                try:
                    arr[gy, gx] = float(row[gx])
                except Exception:
                    pass
    np.clip(arr, 0.0, 1.0, out=arr)
    return arr


def render_grid_to_panel_surface(
    panel_surface: pygame.Surface,
    cells_np: np.ndarray,
    grid_width: int,
    grid_height: int,
    view: ViewState,
) -> None:
    """Rasterize a grid into panel_surface for the current view.

    This path supports very large maps and sub-pixel scales by sampling
    per-screen-pixel (O(panel_size^2)).
    """
    panel_w = panel_surface.get_width()
    panel_h = panel_surface.get_height()
    if panel_w <= 0 or panel_h <= 0 or grid_width <= 0 or grid_height <= 0:
        panel_surface.fill(BG_COLOR)
        return

    # base scale: pixels per cell that would fit the full map
    base_scale = min(panel_w / grid_width, panel_h / grid_height)
    scale = base_scale * float(view.zoom)
    if scale <= 1e-6:
        scale = 1e-6

    view.ensure_center(grid_width, grid_height)
    cx = float(view.center_x or 0.0)
    cy = float(view.center_y or 0.0)

    half_x = (panel_w - 1) / 2.0
    half_y = (panel_h - 1) / 2.0

    # 1D mapping from panel pixels to grid indices, then broadcast sampling.
    xs = (np.arange(panel_w, dtype=np.float32) - half_x) / scale + cx
    ys = (half_y - np.arange(panel_h, dtype=np.float32)) / scale + cy
    gx = xs.astype(np.int32)
    gy = ys.astype(np.int32)

    # Clamp for safe sampling; use mask to black out areas outside the grid.
    gx_c = np.clip(gx, 0, grid_width - 1)
    gy_c = np.clip(gy, 0, grid_height - 1)

    samples = cells_np[gy_c[:, None], gx_c[None, :]]  # (panel_h,panel_w)
    in_bounds = (
        (gx[None, :] >= 0)
        & (gx[None, :] < grid_width)
        & (gy[:, None] >= 0)
        & (gy[:, None] < grid_height)
    )
    # Out-of-bounds pixels: treat as unknown (mid-gray) rather than black.
    samples = np.where(in_bounds, samples, float(OOB_PROB))

    gray = (samples * 255.0).astype(np.uint8)
    rgb_yx = np.stack((gray, gray, gray), axis=2)  # (H,W,3)
    rgb_xy = np.transpose(rgb_yx, (1, 0, 2))       # (W,H,3) for surfarray
    pygame.surfarray.blit_array(panel_surface, rgb_xy)

    # Draw coarse grid lines (every 50 cells) in panel coordinates.
    # Keep it lightweight: only if lines are visually separated.
    if scale >= 2.0:
        major = 50
        # visible grid range in cell coords at panel extents
        min_x = int(((0 - half_x) / scale + cx) // major * major)
        max_x = int((((panel_w - 1) - half_x) / scale + cx) // major * major) + major
        min_y = int(((0 - half_y) / scale + cy) // major * major)
        max_y = int((((panel_h - 1) - half_y) / scale + cy) // major * major) + major

        for gx_line in range(min_x, max_x + 1, major):
            x = int((gx_line - cx) * scale + half_x)
            if 0 <= x < panel_w:
                pygame.draw.line(panel_surface, GRID_LINE_COLOR, (x, 0), (x, panel_h - 1), 1)
        for gy_line in range(min_y, max_y + 1, major):
            y = int(half_y - (gy_line - cy) * scale)
            if 0 <= y < panel_h:
                pygame.draw.line(panel_surface, GRID_LINE_COLOR, (0, y), (panel_w - 1, y), 1)


def draw_grid_panel(
    screen: pygame.Surface,
    font: pygame.font.Font,
    title: str,
    left: int,
    top: int,
    width: int,
    height: int,
    panel_surface: pygame.Surface,
    cells_np: np.ndarray,
    view: ViewState,
) -> None:
    panel_w = panel_surface.get_width()
    panel_h = panel_surface.get_height()
    pygame.draw.rect(screen, PANEL_BORDER_COLOR, (left, top, panel_w, panel_h), 1)
    # Include current zoom in the title for quick debugging.
    title_surf = font.render(f"{title}   (zoom {view.zoom:.2f}x)", True, TEXT_COLOR)
    screen.blit(title_surf, (left, top - 22))

    render_grid_to_panel_surface(panel_surface, cells_np, width, height, view)
    screen.blit(panel_surface, (left, top))


def main() -> None:
    parser = argparse.ArgumentParser(description="SLAM occupancy grid viewer (zoom/pan)")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--icp", action="store_true", help="Show ICP-optimized grid (optimized_cells)")
    group.add_argument("--odom", action="store_true", help="Show odometry grid (cells)")
    args = parser.parse_args()

    mode = "icp" if args.icp or (not args.icp and not args.odom) else "odom"

    print("=== SLAM Occupancy Grid Viewer ===")
    print(f"Mode: {mode.upper()}  (use --icp or --odom)")
    print("Connecting to tcp://localhost:5556 (topic: grid_raw)...")

    # ZMQ subscriber setup
    context = zmq.Context()
    sub = context.socket(zmq.SUB)

    # Only keep the most recent grid
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.connect("tcp://localhost:5556")
    sub.subscribe(b"grid_raw")

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption(f"SLAM Occupancy Grid ({mode.upper()})")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 28)
    font_small = pygame.font.Font(None, 22)

    panel_w = WINDOW_WIDTH - (2 * MARGIN)
    panel_h = WINDOW_HEIGHT - (2 * MARGIN) - HUD_HEIGHT
    if panel_h < 50:
        panel_h = max(50, WINDOW_HEIGHT - (2 * MARGIN))
    panel_surface = pygame.Surface((panel_w, panel_h)).convert()

    view = ViewState()

    latest_grid = None
    latest_raw_np: Optional[np.ndarray] = None
    latest_opt_np: Optional[np.ndarray] = None
    latest_dims: Tuple[int, int] = (0, 0)

    latest_has_optimized: bool = False
    latest_diff_mean_abs: float = 0.0
    latest_diff_max_abs: float = 0.0
    diff_tmp: Optional[np.ndarray] = None

    debug_text: str = ""
    debug_surf: Optional[pygame.Surface] = None

    running = True
    while running:
        # Basic event handling: only quit; no zoom/pan.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                view.zoom = min(MAX_ZOOM, view.zoom * ZOOM_STEP)
                view.auto_fit = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_MINUS:
                view.zoom = max(MIN_ZOOM, view.zoom / ZOOM_STEP)
                view.auto_fit = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                w, h = latest_dims
                if w > 0 and h > 0:
                    scale = _effective_scale(panel_w, panel_h, w, h, view.zoom)
                    view.ensure_center(w, h)
                    view.center_x = float(view.center_x or 0.0) - (PAN_STEP_PX / scale)
                    view.auto_fit = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                w, h = latest_dims
                if w > 0 and h > 0:
                    scale = _effective_scale(panel_w, panel_h, w, h, view.zoom)
                    view.ensure_center(w, h)
                    view.center_x = float(view.center_x or 0.0) + (PAN_STEP_PX / scale)
                    view.auto_fit = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                w, h = latest_dims
                if w > 0 and h > 0:
                    scale = _effective_scale(panel_w, panel_h, w, h, view.zoom)
                    view.ensure_center(w, h)
                    view.center_y = float(view.center_y or 0.0) + (PAN_STEP_PX / scale)
                    view.auto_fit = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                w, h = latest_dims
                if w > 0 and h > 0:
                    scale = _effective_scale(panel_w, panel_h, w, h, view.zoom)
                    view.ensure_center(w, h)
                    view.center_y = float(view.center_y or 0.0) - (PAN_STEP_PX / scale)
                    view.auto_fit = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                view.auto_fit = not view.auto_fit
                # If turning auto-fit back on, re-center immediately.
                if view.auto_fit:
                    w, h = latest_dims
                    view.center_x = (w - 1) / 2.0 if w > 0 else 0.0
                    view.center_y = (h - 1) / 2.0 if h > 0 else 0.0
                    view.zoom = 1.0
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                view.reset()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                filename = f"slam_grid_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                pygame.image.save(screen, filename)
                print(f"[Screenshot] Saved {filename}")

        # Non-blocking receive: take the newest grid if available
        while True:
            try:
                msg = sub.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception as e:  # pragma: no cover - defensive
                print(f"[ZMQ] Error: {e}")
                break
            else:
                text = msg.decode("utf-8")
                space = text.find(" ")
                if space == -1:
                    continue
                try:
                    payload = json.loads(text[space + 1 :])
                except json.JSONDecodeError:
                    continue
                latest_grid = payload

                # Cache numpy conversions so drawing doesn't re-parse giant lists every frame.
                width = int(latest_grid.get("width", 0) or 0)
                height = int(latest_grid.get("height", 0) or 0)
                raw_cells = latest_grid.get("cells", [])
                latest_has_optimized = (
                    ("optimized_cells" in latest_grid) and (latest_grid.get("optimized_cells") is not None)
                )
                optimized_cells = latest_grid.get("optimized_cells", raw_cells)
                latest_raw_np = _cells_to_numpy(raw_cells, width, height)
                latest_opt_np = _cells_to_numpy(optimized_cells, width, height)
                latest_dims = (width, height)

                # Cache diff stats once per message (avoid per-frame NumPy work).
                # If no optimized grid is present, report 0 diffs.
                latest_diff_mean_abs = 0.0
                latest_diff_max_abs = 0.0
                if (
                    latest_has_optimized
                    and latest_raw_np is not None
                    and latest_opt_np is not None
                    and latest_raw_np.shape == latest_opt_np.shape
                    and latest_raw_np.size > 0
                ):
                    if diff_tmp is None or diff_tmp.shape != latest_raw_np.shape:
                        diff_tmp = np.empty_like(latest_raw_np, dtype=np.float32)
                    np.subtract(latest_raw_np, latest_opt_np, out=diff_tmp)
                    np.abs(diff_tmp, out=diff_tmp)
                    latest_diff_mean_abs = float(diff_tmp.mean())
                    latest_diff_max_abs = float(diff_tmp.max())

                new_debug_text = (
                    f"mode={mode.upper()}  dims={width}x{height}  optimized_cells={int(latest_has_optimized)}  "
                    f"mean|Δ|={latest_diff_mean_abs:.4f}  max|Δ|={latest_diff_max_abs:.4f}"
                )
                if new_debug_text != debug_text:
                    debug_text = new_debug_text
                    debug_surf = font_small.render(debug_text, True, TEXT_COLOR, BG_COLOR)

                if view.auto_fit:
                    view.center_x = (width - 1) / 2.0 if width > 0 else 0.0
                    view.center_y = (height - 1) / 2.0 if height > 0 else 0.0
                    view.zoom = 1.0

        # Draw
        screen.fill(BG_COLOR)

        if latest_grid is not None:
            width, height = latest_dims
            if latest_raw_np is None or latest_opt_np is None:
                raw_cells = latest_grid.get("cells", [])
                optimized_cells = latest_grid.get("optimized_cells", raw_cells)
                latest_raw_np = _cells_to_numpy(raw_cells, width, height)
                latest_opt_np = _cells_to_numpy(optimized_cells, width, height)

            panel_x = MARGIN
            panel_y = MARGIN + HUD_HEIGHT

            # Small help overlay
            help_surf = font.render(
                "+/- zoom  arrows pan  C auto-fit  R reset  S screenshot  Q quit",
                True,
                TEXT_COLOR,
            )
            screen.blit(help_surf, (MARGIN, 10))

            # Debug overlay (cached; blit-only per frame)
            if debug_surf is not None:
                screen.blit(debug_surf, (panel_x + 8, panel_y + 8))

            if mode == "icp":
                title = "ICP Corrected (sharper)"
                active_np = latest_opt_np
            else:
                title = "Odometry Only (smeared)"
                active_np = latest_raw_np

            draw_grid_panel(
                screen,
                font,
                title,
                panel_x,
                panel_y,
                width,
                height,
                panel_surface,
                active_np,
                view,
            )

        pygame.display.flip()
        clock.tick(20)

    sub.close()
    context.term()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
