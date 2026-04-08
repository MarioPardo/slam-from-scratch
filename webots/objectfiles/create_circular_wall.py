import math

# --- Wall Parameters ---
radius = 2.5       # Outer radius in meters
degrees = 360.0     # Arc span in degrees (360 = full circle)
thickness = 0.03     # Wall thickness in meters
height = 2.0        # Wall height in meters
segments = 128       # Higher = smoother curve

inner_radius = radius - thickness
arc = math.radians(degrees)
closed = abs(degrees - 360.0) < 1e-6

r_str = f"{radius:g}"
d_str = f"{degrees:g}"
output_file = f"circular_wall_R{r_str}_D{d_str}.obj"

with open(output_file, "w") as f:
    f.write("# Webots-compatible Circular Wall\n")
    f.write("o CircularWall\n\n")

    # Closed circle: segments rings with last wrapping to first
    # Arc: segments+1 rings with open ends
    num_rings = segments if closed else segments + 1

    for i in range(num_rings):
        angle = arc * i / segments
        x_out = radius * math.cos(angle)
        z_out = radius * math.sin(angle)
        x_in  = inner_radius * math.cos(angle)
        z_in  = inner_radius * math.sin(angle)

        # Per ring: Bottom Outer, Top Outer, Bottom Inner, Top Inner
        f.write(f"v {x_out:.4f} 0.0000 {z_out:.4f}\n")
        f.write(f"v {x_out:.4f} {height:.4f} {z_out:.4f}\n")
        f.write(f"v {x_in:.4f}  0.0000 {z_in:.4f}\n")
        f.write(f"v {x_in:.4f}  {height:.4f} {z_in:.4f}\n")

    f.write("\n")

    # Side, top, and bottom faces
    for i in range(segments):
        curr = i * 4 + 1
        # Closed circle wraps the last segment back to ring 0
        nxt = ((i + 1) % segments) * 4 + 1 if closed else (i + 1) * 4 + 1

        # Outer wall
        f.write(f"f {curr}   {nxt}   {nxt+1} {curr+1}\n")
        # Inner wall (reversed so normal faces inward)
        f.write(f"f {curr+2} {curr+3} {nxt+3} {nxt+2}\n")
        # Top cap
        f.write(f"f {curr+1} {nxt+1} {nxt+3} {curr+3}\n")
        # Bottom cap
        f.write(f"f {curr}   {curr+2} {nxt+2} {nxt}\n")

    # End caps for arcs only
    if not closed:
        # Start cap (ring 0)
        s = 1
        f.write(f"f {s} {s+2} {s+3} {s+1}\n")

        # End cap (last ring)
        e = segments * 4 + 1
        f.write(f"f {e} {e+1} {e+3} {e+2}\n")

print(f"Successfully generated '{output_file}'!")
