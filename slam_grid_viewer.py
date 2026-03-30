#!/usr/bin/env python3
"""Minimal occupancy grid viewer for slam_core

- Subscribes to ZMQ topic 'grid_raw' on tcp://localhost:5556
- Expects JSON: {"width": int, "height": int, "resolution": float, "cells": [[p00, ...]]}
- Renders a static 20x20 m grid (0.1 m resolution) with colors based on occupancy probability.
"""

import sys
import json
from datetime import datetime
from typing import Tuple

import pygame
import zmq


WINDOW_WIDTH = 1800
WINDOW_HEIGHT = 960
MARGIN = 40
PANEL_GAP = 30
STATIC_ZOOM = 1.00
BG_COLOR = (0, 0, 0)
GRID_LINE_COLOR = (40, 40, 40)
PANEL_BORDER_COLOR = (90, 90, 90)
TEXT_COLOR = (220, 220, 220)


def prob_to_color(p: float) -> Tuple[int, int, int]:
    """Map occupancy probability to grayscale.

    p=0.0 -> black (free), p=0.5 -> mid gray (unknown), p=1.0 -> white (occupied)
    """
    p = max(0.0, min(1.0, p))
    value = int(255 * p)
    return (value, value, value)


def draw_grid_panel(
    screen: pygame.Surface,
    font: pygame.font.Font,
    title: str,
    left: int,
    top: int,
    panel_size: int,
    width: int,
    height: int,
    cells,
) -> None:
    pygame.draw.rect(screen, PANEL_BORDER_COLOR, (left, top, panel_size, panel_size), 1)
    title_surf = font.render(title, True, TEXT_COLOR)
    screen.blit(title_surf, (left, top - 22))

    if width <= 0 or height <= 0:
        return

    cell_size = int(min(panel_size / width, panel_size / height) * STATIC_ZOOM)
    if cell_size <= 0:
        cell_size = 1

    map_w = width * cell_size
    map_h = height * cell_size
    x0 = left + (panel_size - map_w) // 2
    y0 = top + (panel_size - map_h) // 2

    for gy in range(height):
        if gy >= len(cells):
            break
        row = cells[gy]
        for gx in range(width):
            if gx >= len(row):
                break
            color = prob_to_color(float(row[gx]))
            sx = x0 + gx * cell_size
            sy = y0 + (height - gy - 1) * cell_size
            pygame.draw.rect(screen, color, (sx, sy, cell_size, cell_size))

    for gx in range(0, width + 1, 50):
        x = x0 + gx * cell_size
        pygame.draw.line(screen, GRID_LINE_COLOR, (x, y0), (x, y0 + map_h), 1)
    for gy in range(0, height + 1, 50):
        y = y0 + gy * cell_size
        pygame.draw.line(screen, GRID_LINE_COLOR, (x0, y), (x0 + map_w, y), 1)


def main() -> None:
    print("=== Minimal SLAM Occupancy Grid Viewer ===")
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
    pygame.display.set_caption("SLAM Occupancy Grid: Odometry vs ICP")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 28)

    latest_grid = None

    running = True
    while running:
        # Basic event handling: only quit; no zoom/pan.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False
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

        # Draw
        screen.fill(BG_COLOR)

        if latest_grid is not None:
            width = latest_grid.get("width", 0)
            height = latest_grid.get("height", 0)
            raw_cells = latest_grid.get("cells", [])
            optimized_cells = latest_grid.get("optimized_cells", raw_cells)

            panel_size = min(
                (WINDOW_WIDTH - (2 * MARGIN) - PANEL_GAP) // 2,
                WINDOW_HEIGHT - (2 * MARGIN),
            )
            left_panel_x = MARGIN
            right_panel_x = MARGIN + panel_size + PANEL_GAP
            panel_y = (WINDOW_HEIGHT - panel_size) // 2

            draw_grid_panel(
                screen,
                font,
                "Odometry Only (smeared)",
                left_panel_x,
                panel_y,
                panel_size,
                width,
                height,
                raw_cells,
            )

            draw_grid_panel(
                screen,
                font,
                "ICP Corrected (sharper)",
                right_panel_x,
                panel_y,
                panel_size,
                width,
                height,
                optimized_cells,
            )

        pygame.display.flip()
        clock.tick(20)

    sub.close()
    context.term()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
