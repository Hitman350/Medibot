#!/usr/bin/env python3
"""Generate hospital_ward.pgm map -- original layout (0-14, 0-10)."""

import os

# Map parameters
RESOLUTION = 0.05  # meters per pixel
ORIGIN_X = -1.0    # 1m border around hospital (0..14)
ORIGIN_Y = -1.0    # 1m border around hospital (0..10)
MAP_WIDTH_M = 16.0
MAP_HEIGHT_M = 12.0

WIDTH = int(MAP_WIDTH_M / RESOLUTION)   # 320
HEIGHT = int(MAP_HEIGHT_M / RESOLUTION)  # 240

def m2px(x, y):
    px = int((x - ORIGIN_X) / RESOLUTION)
    py = int((y - ORIGIN_Y) / RESOLUTION)
    return px, py

def draw_rect(grid, x1, y1, x2, y2, val):
    px1, py1 = m2px(x1, y1)
    px2, py2 = m2px(x2, y2)
    px1, px2 = min(px1, px2), max(px1, px2)
    py1, py2 = min(py1, py2), max(py1, py2)
    for y in range(max(0, py1), min(HEIGHT, py2)):
        for x in range(max(0, px1), min(WIDTH, px2)):
            grid[y][x] = val

grid = [[205 for _ in range(WIDTH)] for _ in range(HEIGHT)]

# Outside boundaries -> unknown
draw_rect(grid, -1, -1, 15, 0, 128)
draw_rect(grid, -1, 10, 15, 11, 128)
draw_rect(grid, -1, -1, 0, 11, 128)
draw_rect(grid, 14, -1, 15, 11, 128)

W = 0  # wall color (black)
H = 0.075  # half wall thickness

# Outer walls
draw_rect(grid, 0-H, 0-H, 14+H, 0+H, W)   # south
draw_rect(grid, 0-H, 10-H, 14+H, 10+H, W)  # north
draw_rect(grid, 0-H, 0, 0+H, 10, W)         # west
draw_rect(grid, 14-H, 0, 14+H, 10, W)       # east

# Corridor walls y=4 (south side of corridor)
draw_rect(grid, 0, 4-H, 1.4, 4+H, W)
# gap 1.4 to 2.6 (pharmacy door)
draw_rect(grid, 2.6, 4-H, 3.5, 4+H, W)
draw_rect(grid, 3.5, 4-H, 5.25, 4+H, W)
# gap 5.25 to 6.4 (room 4 door)
draw_rect(grid, 6.4, 4-H, 8.75, 4+H, W)
draw_rect(grid, 8.75, 4-H, 10.5, 4+H, W)
draw_rect(grid, 10.5, 4-H, 14, 4+H, W)

# Corridor walls y=6 (north side of corridor)
draw_rect(grid, 0, 6-H, 1.4, 6+H, W)
# gap 1.4 to 2.6 (room 1 door)
draw_rect(grid, 2.6, 6-H, 3.5, 6+H, W)
draw_rect(grid, 3.5, 6-H, 5.25, 6+H, W)
# gap 5.25 to 6.4 (room 2 door)
draw_rect(grid, 6.4, 6-H, 8.75, 6+H, W)
draw_rect(grid, 8.75, 6-H, 10.5, 6+H, W)
# gap 10.5 to 11.7 (room 3 door)
draw_rect(grid, 11.7, 6-H, 14, 6+H, W)

# Room dividers (vertical)
# Upper rooms
draw_rect(grid, 3.5-H, 6, 3.5+H, 10, W)
draw_rect(grid, 5.25-H, 6, 5.25+H, 10, W)
draw_rect(grid, 8.75-H, 6, 8.75+H, 10, W)
draw_rect(grid, 10.5-H, 6, 10.5+H, 10, W)
# Lower rooms
draw_rect(grid, 3.5-H, 0, 3.5+H, 4, W)
draw_rect(grid, 5.25-H, 0, 5.25+H, 4, W)
draw_rect(grid, 8.75-H, 0, 8.75+H, 4, W)
draw_rect(grid, 10.5-H, 0, 10.5+H, 4, W)

# Furniture
draw_rect(grid, 0.75, 8.0, 2.75, 9.0, W)    # bed 01
draw_rect(grid, 6.0,  8.0, 8.0,  9.0, W)     # bed 02
draw_rect(grid, 11.25, 8.0, 13.25, 9.0, W)   # bed 03
draw_rect(grid, 6.0,  1.0, 8.0,  2.0, W)     # bed 04
draw_rect(grid, 0.5,  0.7, 3.0,  1.3, W)     # pharmacy counter
draw_rect(grid, 11.35, 1.6, 13.15, 2.4, W)   # nurse desk
draw_rect(grid, 0.25, 8.25, 0.75, 8.75, W)   # table 01
draw_rect(grid, 5.25, 8.25, 5.75, 8.75, W)   # table 02
draw_rect(grid, 10.55, 8.25, 11.05, 8.75, W) # table 03

# Write PGM
script_dir = os.path.dirname(os.path.abspath(__file__))
out = os.path.normpath(os.path.join(script_dir, '..', 'maps', 'hospital_ward.pgm'))
with open(out, 'wb') as f:
    f.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode())
    for row in reversed(grid):
        f.write(bytes(row))
print(f"Generated {out} ({WIDTH}x{HEIGHT})")
