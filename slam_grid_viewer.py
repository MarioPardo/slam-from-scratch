#!/usr/bin/env python3
"""Minimal occupancy grid viewer for slam_core

- Subscribes to ZMQ topic 'grid_raw' on tcp://localhost:5556
- Expects JSON: {"width": int, "height": int, "resolution": float, "cells": [[p00, ...]]}
- Renders a static 20x20 m grid (0.1 m resolution) with colors based on occupancy probability.
"""

import sys
import json
from typing import Tuple

import pygame
import zmq


WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 1000
MARGIN = 30
STATIC_ZOOM = 0.90
BG_COLOR = (0, 0, 0)
GRID_LINE_COLOR = (40, 40, 40)


def prob_to_color(p: float) -> Tuple[int, int, int]:
    """Map occupancy probability to grayscale.

    p=0.0 -> black (free), p=0.5 -> mid gray (unknown), p=1.0 -> white (occupied)
    """
    p = max(0.0, min(1.0, p))
    value = int(255 * p)
    return (value, value, value)


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
    pygame.display.set_caption("SLAM Occupancy Grid (raw)")
    clock = pygame.time.Clock()

    latest_grid = None

    running = True
    while running:
        # Basic event handling: only quit; no zoom/pan.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False

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
            resolution = float(latest_grid.get("resolution", 0.1))
            origin_x = float(latest_grid.get("origin_x", 0.0))
            origin_y = float(latest_grid.get("origin_y", 0.0))
            cells = latest_grid.get("cells", [])

            # Compute cell size so that full 20x20 m fits with margins
            if width > 0 and height > 0:
                cell_size_x = (WINDOW_WIDTH - 2 * MARGIN) / width
                cell_size_y = (WINDOW_HEIGHT - 2 * MARGIN) / height
                cell_size = int(min(cell_size_x, cell_size_y))
                cell_size = int(cell_size * STATIC_ZOOM)
                if cell_size <= 0:
                    cell_size = 1

                # Draw occupancy cells
                for gy in range(height):
                    if gy >= len(cells):
                        break
                    row = cells[gy]
                    for gx in range(width):
                        if gx >= len(row):
                            break
                        p = float(row[gx])
                        color = prob_to_color(p)
                        # origin (0,0) at bottom-left visually
                        sx = MARGIN + gx * cell_size
                        sy = WINDOW_HEIGHT - MARGIN - (gy + 1) * cell_size
                        pygame.draw.rect(screen, color, (sx, sy, cell_size, cell_size))

                # Optional coarse grid lines only (every 10 cells = 1 meter)
                for gx in range(0, width + 1, 10):
                    x = MARGIN + gx * cell_size
                    pygame.draw.line(screen, GRID_LINE_COLOR, (x, MARGIN), (x, WINDOW_HEIGHT - MARGIN), 1)
                for gy in range(0, height + 1, 10):
                    y = WINDOW_HEIGHT - MARGIN - gy * cell_size
                    pygame.draw.line(screen, GRID_LINE_COLOR, (MARGIN, y), (WINDOW_WIDTH - MARGIN, y), 1)

                # Static display: no center-axis emphasis

        pygame.display.flip()
        clock.tick(20)

    sub.close()
    context.term()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
