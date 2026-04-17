#!/usr/bin/env python3
"""
CARMEN log replay for SLAM pipeline.

Parses Intel CARMEN .log files (ROBOTLASER1 / FLASER format) and feeds them
into slam_subscriber via ZMQ using the same JSON schema as TurtleBotGoForward.py.

Usage:
    python carmen_replay.py intel.log --config configs/intel.json --speed 5.0

Controls (while running):
    Enter       Skip current loop-closure pause early
    Ctrl+C      Quit (metrics written on exit)

Viewers (run in parallel):
    python slam_grid_viewer.py                       # occupancy: S to screenshot
    python slam_viewer_pygame.py --save out.png      # trajectory: zoom out with -
"""

import argparse
import gzip
import json
import math
import select
import sys
import time

import zmq


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def load_config(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def open_log(path: str):
    """Open a plain or gzip-compressed log file for text reading."""
    if path.endswith(".gz"):
        return gzip.open(path, "rt", encoding="utf-8")
    return open(path, encoding="utf-8")


def count_laser_lines(path: str) -> int:
    """Pre-scan to count ROBOTLASER1/FLASER lines for ETA display."""
    n = 0
    with open_log(path) as f:
        for line in f:
            tag = line[:12]
            if tag.startswith("ROBOTLASER1") or tag.startswith("FLASER"):
                n += 1
    return n


# ---------------------------------------------------------------------------
# CARMEN line parsers
# ---------------------------------------------------------------------------

def parse_robotlaser1(tokens):
    """
    ROBOTLASER1 laser_type start_angle fov angular_res max_range accuracy
                remission_mode nbeams [ranges*nbeams] [remissions*nbeams]
                robot_x robot_y robot_theta odom_x odom_y odom_theta
                safety_dist max_num_scans timestamp hostname logger_ts
    """
    try:
        # idx 0 = "ROBOTLASER1"
        nbeams = int(tokens[8])
        ranges = [float(tokens[9 + i]) for i in range(nbeams)]
        base   = 9 + nbeams          # start of remissions block
        # skip remission values (same count as beams)
        base  += nbeams
        robot_x     = float(tokens[base])
        robot_y     = float(tokens[base + 1])
        robot_theta = float(tokens[base + 2])
        odom_x      = float(tokens[base + 3])
        odom_y      = float(tokens[base + 4])
        odom_theta  = float(tokens[base + 5])
        timestamp   = float(tokens[base + 8])
        start_angle = float(tokens[1])
        fov         = float(tokens[3])
        max_range   = float(tokens[5])
        return {
            "ranges":     ranges,
            "nbeams":     nbeams,
            "start_angle": start_angle,
            "fov":         fov,
            "max_range":   max_range,
            "odom_x":      odom_x,
            "odom_y":      odom_y,
            "odom_theta":  odom_theta,
            "timestamp":   timestamp,
        }
    except (IndexError, ValueError):
        return None


def parse_flaser(tokens):
    """
    FLASER nbeams [ranges*nbeams] x y theta odom_x odom_y odom_theta timestamp
    """
    try:
        nbeams = int(tokens[1])
        ranges = [float(tokens[2 + i]) for i in range(nbeams)]
        base   = 2 + nbeams
        odom_x     = float(tokens[base + 3])
        odom_y     = float(tokens[base + 4])
        odom_theta = float(tokens[base + 5])
        timestamp  = float(tokens[base + 6])
        # FLASER is always 180 beams from -pi/2 to +pi/2
        return {
            "ranges":      ranges,
            "nbeams":      nbeams,
            "start_angle": -math.pi / 2,
            "fov":          math.pi,
            "max_range":    80.0,
            "odom_x":       odom_x,
            "odom_y":       odom_y,
            "odom_theta":   odom_theta,
            "timestamp":    timestamp,
        }
    except (IndexError, ValueError):
        return None


# ---------------------------------------------------------------------------
# JSON message builder
# ---------------------------------------------------------------------------

def build_json(
    step: int,
    timestamp: float,
    left_enc: float,
    right_enc: float,
    gyro_z: float,
    compass: float,
    ranges: list[float],
    angle_min: float,
    angle_max: float,
    range_min: float,
    range_max: float,
) -> str:
    ranges_str = ",".join(f"{r:.4f}" for r in ranges)
    return (
        '{"header":{"timestamp":' + f"{timestamp:.6f}" +
        ',"step_count":' + str(step) + '}' +
        ',"odometry":{"left_encoder":' + f"{left_enc:.6f}" +
        ',"right_encoder":' + f"{right_enc:.6f}" +
        ',"imu_gyro_z":' + f"{gyro_z:.6f}" +
        ',"imu_accel":[0.0,0.0,0.0]' +
        ',"compass_heading":' + f"{compass:.6f}" + '}' +
        ',"lidar":{"count":' + str(len(ranges)) +
        ',"angle_min":' + f"{angle_min:.7f}" +
        ',"angle_max":' + f"{angle_max:.7f}" +
        ',"range_min":' + f"{range_min:.4f}" +
        ',"range_max":' + f"{range_max:.4f}" +
        ',"ranges":[' + ranges_str + ']}}'
    )


# ---------------------------------------------------------------------------
# Loop-closure pause (Enter to skip)
# ---------------------------------------------------------------------------

def pause_for_screenshot(duration: float, label: str) -> None:
    print()
    print("=" * 60)
    print(f"  *** {label} ***")
    print(f"  Screenshot now (S in either viewer).")
    print(f"  Resuming in {duration:.0f}s  |  Press Enter to skip")
    print("=" * 60)
    deadline = time.time() + duration
    while time.time() < deadline:
        remaining = deadline - time.time()
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            sys.stdin.readline()
            print("  [Skipped pause]")
            return
        print(f"\r  Resuming in {remaining:.1f}s ...   ", end="", flush=True)
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="CARMEN log replay for SLAM pipeline")
    parser.add_argument("log_file",                        help="Path to .log / .clf CARMEN file")
    parser.add_argument("--config",    default="configs/intel.json", help="SLAMConfig JSON (default: configs/intel.json)")
    parser.add_argument("--speed",     type=float, default=5.0,      help="Replay speed multiplier (default: 5.0)")
    parser.add_argument("--loop-pause",type=float, default=8.0,      dest="loop_pause",
                                                                       help="Seconds to pause on loop closure (default: 8)")
    parser.add_argument("--zmq-pub",   default="tcp://localhost:5555",dest="zmq_pub",
                                                                       help="ZMQ address to publish sensor data")
    parser.add_argument("--zmq-sub",   default="tcp://localhost:5556",dest="zmq_sub",
                                                                       help="ZMQ address to subscribe for slam output")
    parser.add_argument("--metrics",   default="replay_metrics.json", help="Output metrics file (default: replay_metrics.json)")
    parser.add_argument("--sync",      action="store_true",
                                                                       help="Block after each frame until SLAM acks (no dropped frames)")
    args = parser.parse_args()

    # --- Config ---
    cfg = load_config(args.config)
    wheel_radius = cfg.get("wheel_radius", 0.097)
    wheelbase    = cfg.get("wheelbase",    0.38)
    range_min    = cfg.get("lidar_range_min", 0.1)

    # --- Pre-scan ---
    print(f"[Replay] Pre-scanning {args.log_file} ...")
    total = count_laser_lines(args.log_file)
    if total == 0:
        print("[Replay] No ROBOTLASER1/FLASER lines found. Check file path.")
        sys.exit(1)
    eta_s = total * 0.143 / args.speed   # ~7 Hz average
    if args.sync:
        print(f"[Replay] {total} laser scans  |  sync mode (SLAM-paced, no dropped frames)")
    else:
        print(f"[Replay] {total} laser scans  |  ETA at {args.speed}x speed: {eta_s/60:.1f} min")

    # --- ZMQ ---
    ctx = zmq.Context()

    pub = ctx.socket(zmq.PUB)
    pub.bind(args.zmq_pub)

    sub = ctx.socket(zmq.SUB)
    if not args.sync:
        sub.setsockopt(zmq.CONFLATE, 1)   # real-time mode: only care about latest viz
    sub.connect(args.zmq_sub)
    sub.subscribe(b"visualization")

    # Allow subscribers to connect before first message
    time.sleep(0.5)

    print()
    print("  S  = screenshot in either viewer")
    print("  Enter = skip loop-closure pause")
    print("  Ctrl+C = quit (metrics written on exit)")
    print()

    # --- State ---
    prev_odom_x     = None
    prev_odom_y     = None
    prev_odom_theta = None
    prev_ts         = None
    left_accum      = 0.0
    right_accum     = 0.0

    step              = 0
    loop_count        = 0
    clear_map_seen    = False
    seen_loop_edges   = 0   # count of type:1 edges seen so far in viz messages
    before_taken      = False  # tracks whether we've already paused for "before" on this event

    loop_closure_log: list[dict] = []
    t_start = time.time()

    # --- Metrics on exit ---
    def write_metrics() -> None:
        elapsed = time.time() - t_start
        data = {
            "total_frames":        step,
            "loop_closure_count":  loop_count,
            "loop_closures":       loop_closure_log,
            "replay_duration_secs": round(elapsed, 2),
        }
        with open(args.metrics, "w") as f:
            json.dump(data, f, indent=2)
        print(f"\n[Replay] Metrics written to {args.metrics}")
        print(f"[Replay] Frames: {step}  |  Loop closures: {loop_count}  |  Time: {elapsed:.1f}s")

    # --- Main loop ---
    try:
        with open_log(args.log_file) as log:
            for raw_line in log:
                line = raw_line.strip()
                if not line:
                    continue

                tokens = line.split()
                tag    = tokens[0]

                if tag == "ROBOTLASER1":
                    parsed = parse_robotlaser1(tokens)
                elif tag == "FLASER":
                    parsed = parse_flaser(tokens)
                else:
                    continue

                if parsed is None:
                    continue

                curr_x     = parsed["odom_x"]
                curr_y     = parsed["odom_y"]
                curr_theta = parsed["odom_theta"]
                curr_ts    = parsed["timestamp"]
                max_range  = parsed["max_range"]
                start_ang  = parsed["start_angle"]
                fov        = parsed["fov"]
                nbeams     = parsed["nbeams"]
                angle_min  = start_ang
                angle_max  = start_ang + fov

                # Clamp ranges
                ranges = [
                    max_range if (r <= 0.0 or not math.isfinite(r) or r > max_range)
                    else r
                    for r in parsed["ranges"]
                ]

                # Encoder synthesis
                if prev_odom_x is None:
                    # First message: initialise accumulators
                    gyro_z = 0.0
                    dt     = 0.0
                else:
                    dx      = curr_x - prev_odom_x
                    dy      = curr_y - prev_odom_y
                    d_dist  = math.hypot(dx, dy)
                    if math.cos(prev_odom_theta) * dx + math.sin(prev_odom_theta) * dy < 0:
                        d_dist = -d_dist
                    d_theta = normalize_angle(curr_theta - prev_odom_theta)
                    dt      = max(curr_ts - prev_ts, 1e-6)

                    dR = (d_dist + wheelbase / 2 * d_theta) / wheel_radius
                    dL = (d_dist - wheelbase / 2 * d_theta) / wheel_radius
                    left_accum  += dL
                    right_accum += dR
                    gyro_z = d_theta / dt

                prev_odom_x     = curr_x
                prev_odom_y     = curr_y
                prev_odom_theta = curr_theta
                prev_ts         = curr_ts
                step           += 1

                # Build and publish
                msg = build_json(
                    step, curr_ts,
                    left_accum, right_accum, gyro_z, curr_theta,
                    ranges, angle_min, angle_max, range_min, max_range,
                )
                pub.send_string(f"robot_state {msg}")

                # Wait for viz response — blocking in sync mode, non-blocking otherwise
                recv_flags = 0 if args.sync else zmq.NOBLOCK
                try:
                    raw = sub.recv(flags=recv_flags)
                    text = raw.decode("utf-8")
                    space = text.find(" ")
                    if space != -1:
                        viz = json.loads(text[space + 1:])

                        # --- BEFORE signal: new loop closure edge appeared ---
                        # Edges with type:1 are loop closure edges. They are added
                        # to the graph ~10 keyframes before GTSAM optimization runs,
                        # giving a genuine "before optimization" snapshot.
                        edges = viz.get("pose_graph", {}).get("edges", [])
                        current_loop_edges = sum(1 for e in edges if e.get("type", 0) == 1)
                        if current_loop_edges > seen_loop_edges and not before_taken:
                            seen_loop_edges = current_loop_edges
                            before_taken = True
                            loop_count += 1
                            loop_closure_log.append({"frame": step, "timestamp": curr_ts})
                            pause_for_screenshot(
                                args.loop_pause,
                                f"LOOP CLOSURE #{loop_count} DETECTED — BEFORE screenshot"
                            )
                        else:
                            seen_loop_edges = max(seen_loop_edges, current_loop_edges)

                        # --- AFTER signal: optimization fired, map rebuilt ---
                        is_clear = viz.get("clear_map", False)
                        if is_clear and not clear_map_seen:
                            clear_map_seen = True
                            before_taken = False  # ready for next event's "before" pause
                            pause_for_screenshot(
                                args.loop_pause,
                                f"LOOP CLOSURE #{loop_count} OPTIMIZED — AFTER screenshot"
                            )
                        elif not is_clear:
                            clear_map_seen = False
                except zmq.Again:
                    pass
                except Exception:
                    pass

                # Progress
                if step % 500 == 0:
                    pct = step / total * 100
                    print(f"[Replay] {step}/{total}  ({pct:.1f}%)  loops={loop_count}", flush=True)

                # Rate limiting — skipped in sync mode (SLAM pace drives timing)
                if not args.sync and dt > 0:
                    sleep_s = min(dt / args.speed, 0.5)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\n[Replay] Interrupted.")
    finally:
        write_metrics()
        pub.close()
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
