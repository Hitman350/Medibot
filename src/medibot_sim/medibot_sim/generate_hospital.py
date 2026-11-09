#!/usr/bin/env python3
"""
Generate the hospital ward SDF world file and matching Nav2 map.

Hospital Layout (14m x 10m):
    ┌─────────┐  ┌─────────┐  ┌─────────┐
    │ Room 1  │  │ Room 2  │  │ Room 3  │
    │ Bed 01  │  │ Bed 02  │  │ Bed 03  │
    │         D  │         D  │         D
    ├────D────┘  └────D────┘  └────D────┤
    │                                    │
    │         Main Corridor              │
    │                                    │
    ├────D────┐  ┌────D────┐  ┌────D────┤
    │         D  │         D  │         D
    │Pharmacy │  │ Room 4  │  │  Nurse  │
    │ Station │  │ Bed 04  │  │ Station │
    └─────────┘  └─────────┘  └─────────┘

D = Door opening
"""

import struct
import os

# --- Configuration ---
WORLD_W = 14.0   # meters
WORLD_H = 10.0
WALL_H = 2.5     # wall height
WALL_T = 0.15    # wall thickness
CORRIDOR_Y_LO = 4.0
CORRIDOR_Y_HI = 6.0
DOOR_W = 1.2     # door opening width

# Room boundaries [x_min, x_max]
COL1 = (0.0, 3.5)
COL2 = (5.25, 8.75)
COL3 = (10.5, 14.0)

# Map config
MAP_RESOLUTION = 0.05  # m/pixel
MAP_ORIGIN_X = -1.0
MAP_ORIGIN_Y = -1.0


def wall_model(name, cx, cy, sx, sy, sz=WALL_H, color="0.85 0.85 0.80"):
    """Generate SDF for a single wall box."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {sz/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{color} 1</ambient>
            <diffuse>{color} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def bed_model(name, cx, cy, label=""):
    """Generate SDF for a hospital bed (simplified box)."""
    # Bed: 2.0m x 0.9m x 0.5m (mattress height)
    bed_w, bed_d, bed_h = 2.0, 0.9, 0.5
    frame_h = 0.45
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} 0 0 0 0</pose>
      <link name="frame">
        <pose>0 0 {frame_h/2} 0 0 0</pose>
        <collision name="col"><geometry><box><size>{bed_w} {bed_d} {frame_h}</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{bed_w} {bed_d} {frame_h}</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>
        </visual>
      </link>
      <link name="mattress">
        <pose>0 0 {frame_h + 0.05} 0 0 0</pose>
        <collision name="col"><geometry><box><size>{bed_w - 0.1} {bed_d - 0.05} 0.1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{bed_w - 0.1} {bed_d - 0.05} 0.1</size></box></geometry>
          <material><ambient>0.9 0.9 0.95 1</ambient><diffuse>0.9 0.9 0.95 1</diffuse></material>
        </visual>
      </link>
      <link name="headboard">
        <pose>{-bed_w/2 + 0.05} 0 {frame_h + 0.3} 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.05 {bed_d} 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.05 {bed_d} 0.5</size></box></geometry>
          <material><ambient>0.45 0.3 0.2 1</ambient><diffuse>0.45 0.3 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def table_model(name, cx, cy, size_x=0.5, size_y=0.5):
    """Small bedside table or cabinet."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} 0.3 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{size_x} {size_y} 0.6</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{size_x} {size_y} 0.6</size></box></geometry>
          <material><ambient>0.5 0.35 0.2 1</ambient><diffuse>0.5 0.35 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def pharmacy_counter(name, cx, cy):
    """Pharmacy counter/station."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} 0.45 0 0 0</pose>
      <link name="counter">
        <collision name="col"><geometry><box><size>2.5 0.6 0.9</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>2.5 0.6 0.9</size></box></geometry>
          <material><ambient>0.3 0.5 0.7 1</ambient><diffuse>0.3 0.5 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def generate_world(output_path):
    """Generate the hospital ward SDF world."""

    walls = []
    wn = 0  # wall name counter

    def add_wall(cx, cy, sx, sy, color="0.85 0.85 0.80"):
        nonlocal wn
        walls.append(wall_model(f"wall_{wn}", cx, cy, sx, sy, color=color))
        wn += 1

    # === OUTER WALLS ===
    # Bottom wall (y=0)
    add_wall(WORLD_W/2, 0, WORLD_W, WALL_T)
    # Top wall (y=10)
    add_wall(WORLD_W/2, WORLD_H, WORLD_W, WALL_T)
    # Left wall (x=0)
    add_wall(0, WORLD_H/2, WALL_T, WORLD_H)
    # Right wall (x=14)
    add_wall(WORLD_W, WORLD_H/2, WALL_T, WORLD_H)

    # === CORRIDOR WALLS (y=4 and y=6) with door gaps ===

    # --- Lower corridor wall (y=4) ---
    # Pharmacy door at x=1.5-2.7 → wall segments: [0, 1.5] and [2.7, 5.25] and [5.25+gap...] etc.
    # Pharmacy wall: x=0 to COL1[1]=3.5
    # Door centered at x=2.0
    door_cx = 2.0
    add_wall((0 + (door_cx - DOOR_W/2))/2, CORRIDOR_Y_LO,
             door_cx - DOOR_W/2, WALL_T)
    add_wall((door_cx + DOOR_W/2 + COL1[1])/2, CORRIDOR_Y_LO,
             COL1[1] - (door_cx + DOOR_W/2), WALL_T)

    # Gap between Col1 and Col2: x=3.5 to 5.25 → open corridor wall
    add_wall((COL1[1] + COL2[0])/2, CORRIDOR_Y_LO,
             COL2[0] - COL1[1], WALL_T)

    # Room 4 door at x=7.0
    door_cx = 7.0
    add_wall((COL2[0] + (door_cx - DOOR_W/2))/2, CORRIDOR_Y_LO,
             (door_cx - DOOR_W/2) - COL2[0], WALL_T)
    add_wall((door_cx + DOOR_W/2 + COL2[1])/2, CORRIDOR_Y_LO,
             COL2[1] - (door_cx + DOOR_W/2), WALL_T)

    # Gap between Col2 and Col3
    add_wall((COL2[1] + COL3[0])/2, CORRIDOR_Y_LO,
             COL3[0] - COL2[1], WALL_T)

    # Nurse station door at x=12.0
    door_cx = 12.0
    add_wall((COL3[0] + (door_cx - DOOR_W/2))/2, CORRIDOR_Y_LO,
             (door_cx - DOOR_W/2) - COL3[0], WALL_T)
    add_wall((door_cx + DOOR_W/2 + COL3[1])/2, CORRIDOR_Y_LO,
             COL3[1] - (door_cx + DOOR_W/2), WALL_T)

    # --- Upper corridor wall (y=6) ---
    # Room 1 door at x=2.0
    door_cx = 2.0
    add_wall((0 + (door_cx - DOOR_W/2))/2, CORRIDOR_Y_HI,
             door_cx - DOOR_W/2, WALL_T)
    add_wall((door_cx + DOOR_W/2 + COL1[1])/2, CORRIDOR_Y_HI,
             COL1[1] - (door_cx + DOOR_W/2), WALL_T)

    # Gap between Col1 and Col2
    add_wall((COL1[1] + COL2[0])/2, CORRIDOR_Y_HI,
             COL2[0] - COL1[1], WALL_T)

    # Room 2 door at x=7.0
    door_cx = 7.0
    add_wall((COL2[0] + (door_cx - DOOR_W/2))/2, CORRIDOR_Y_HI,
             (door_cx - DOOR_W/2) - COL2[0], WALL_T)
    add_wall((door_cx + DOOR_W/2 + COL2[1])/2, CORRIDOR_Y_HI,
             COL2[1] - (door_cx + DOOR_W/2), WALL_T)

    # Gap between Col2 and Col3
    add_wall((COL2[1] + COL3[0])/2, CORRIDOR_Y_HI,
             COL3[0] - COL2[1], WALL_T)

    # Room 3 door at x=12.0
    door_cx = 12.0
    add_wall((COL3[0] + (door_cx - DOOR_W/2))/2, CORRIDOR_Y_HI,
             (door_cx - DOOR_W/2) - COL3[0], WALL_T)
    add_wall((door_cx + DOOR_W/2 + COL3[1])/2, CORRIDOR_Y_HI,
             COL3[1] - (door_cx + DOOR_W/2), WALL_T)

    # === ROOM DIVIDER WALLS (vertical, between rooms) ===
    # Between Col1 and Col2 — upper rooms
    add_wall(COL1[1], (CORRIDOR_Y_HI + WORLD_H)/2, WALL_T,
             WORLD_H - CORRIDOR_Y_HI)
    add_wall(COL2[0], (CORRIDOR_Y_HI + WORLD_H)/2, WALL_T,
             WORLD_H - CORRIDOR_Y_HI)
    # Between Col2 and Col3 — upper rooms
    add_wall(COL2[1], (CORRIDOR_Y_HI + WORLD_H)/2, WALL_T,
             WORLD_H - CORRIDOR_Y_HI)
    add_wall(COL3[0], (CORRIDOR_Y_HI + WORLD_H)/2, WALL_T,
             WORLD_H - CORRIDOR_Y_HI)

    # Between Col1 and Col2 — lower rooms
    add_wall(COL1[1], CORRIDOR_Y_LO/2, WALL_T, CORRIDOR_Y_LO)
    add_wall(COL2[0], CORRIDOR_Y_LO/2, WALL_T, CORRIDOR_Y_LO)
    # Between Col2 and Col3 — lower rooms
    add_wall(COL2[1], CORRIDOR_Y_LO/2, WALL_T, CORRIDOR_Y_LO)
    add_wall(COL3[0], CORRIDOR_Y_LO/2, WALL_T, CORRIDOR_Y_LO)

    # === FURNITURE ===
    furniture = []

    # Beds (oriented along X axis, positioned against walls)
    furniture.append(bed_model("bed_01", 1.75, 8.5))
    furniture.append(bed_model("bed_02", 7.0, 8.5))
    furniture.append(bed_model("bed_03", 12.25, 8.5))
    furniture.append(bed_model("bed_04", 7.0, 1.5))

    # Bedside tables
    furniture.append(table_model("table_01", 0.5, 8.5))
    furniture.append(table_model("table_02", 5.5, 8.5))
    furniture.append(table_model("table_03", 10.8, 8.5))
    furniture.append(table_model("table_04", 5.5, 1.5))

    # Pharmacy counter
    furniture.append(pharmacy_counter("pharmacy_counter", 1.75, 1.0))

    # Nurse station desk
    furniture.append(f"""
    <model name="nurse_desk">
      <static>true</static>
      <pose>12.25 2.0 0.35 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>1.8 0.8 0.7</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.8 0.8 0.7</size></box></geometry>
          <material><ambient>0.4 0.3 0.25 1</ambient><diffuse>0.4 0.3 0.25 1</diffuse></material>
        </visual>
      </link>
    </model>""")

    # Chairs in corridor (obstacles)
    furniture.append(table_model("corridor_chair_1", 4.5, 5.0, 0.4, 0.4))
    furniture.append(table_model("corridor_chair_2", 9.5, 5.0, 0.4, 0.4))

    # === FLOOR ===
    floor = f"""
    <model name="hospital_floor">
      <static>true</static>
      <pose>{WORLD_W/2} {WORLD_H/2} -0.005 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{WORLD_W + 2} {WORLD_H + 2} 0.01</size></box></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="vis"><geometry><box><size>{WORLD_W + 2} {WORLD_H + 2} 0.01</size></box></geometry>
          <material><ambient>0.92 0.92 0.90 1</ambient><diffuse>0.92 0.92 0.90 1</diffuse></material>
        </visual>
      </link>
    </model>"""

    # === ASSEMBLE WORLD ===
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="hospital_ward">

    <!-- Physics -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>7 5 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.3 0.1 -0.9</direction>
    </light>

    <light type="point" name="corridor_light_1">
      <pose>3 5 2.4 0 0 0</pose>
      <diffuse>0.8 0.8 0.75 1</diffuse>
      <intensity>0.5</intensity>
      <attenuation>
        <range>10</range>
        <constant>0.5</constant>
        <linear>0.1</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>

    <light type="point" name="corridor_light_2">
      <pose>10 5 2.4 0 0 0</pose>
      <diffuse>0.8 0.8 0.75 1</diffuse>
      <intensity>0.5</intensity>
      <attenuation>
        <range>10</range>
        <constant>0.5</constant>
        <linear>0.1</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>

    <!-- Gravity -->
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <!-- Scene -->
    <scene>
      <ambient>0.6 0.6 0.58 1</ambient>
      <background>0.7 0.75 0.8 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- ==================== -->
    <!--        FLOOR          -->
    <!-- ==================== -->
    {floor}

    <!-- ==================== -->
    <!--        WALLS          -->
    <!-- ==================== -->
    {"".join(walls)}

    <!-- ==================== -->
    <!--      FURNITURE        -->
    <!-- ==================== -->
    {"".join(furniture)}

  </world>
</sdf>
"""
    with open(output_path, 'w') as f:
        f.write(sdf)
    print(f"World saved to {output_path}")


def generate_map(pgm_path, yaml_path):
    """Generate a PGM occupancy grid map matching the hospital layout."""

    # Map dimensions in pixels
    pad = 1.0  # padding around the world
    map_w_m = WORLD_W + 2 * pad
    map_h_m = WORLD_H + 2 * pad
    w_px = int(map_w_m / MAP_RESOLUTION)
    h_px = int(map_h_m / MAP_RESOLUTION)

    # Create white (free) image
    pixels = [[254] * w_px for _ in range(h_px)]

    def world_to_pixel(wx, wy):
        px = int((wx - MAP_ORIGIN_X) / MAP_RESOLUTION)
        py = int((wy - MAP_ORIGIN_Y) / MAP_RESOLUTION)
        return px, py

    def draw_wall_h(y, x_start, x_end):
        """Draw horizontal wall."""
        px_s, py = world_to_pixel(x_start, y)
        px_e, _ = world_to_pixel(x_end, y)
        for t in range(-1, 2):  # wall thickness
            row = py + t
            if 0 <= row < h_px:
                for px in range(max(0, px_s), min(w_px, px_e)):
                    pixels[row][px] = 0

    def draw_wall_v(x, y_start, y_end):
        """Draw vertical wall."""
        px, py_s = world_to_pixel(x, y_start)
        _, py_e = world_to_pixel(x, y_end)
        for t in range(-1, 2):
            col = px + t
            if 0 <= col < w_px:
                for py in range(max(0, py_s), min(h_px, py_e)):
                    pixels[py][col] = 0

    def draw_box(cx, cy, sx, sy):
        """Draw a filled obstacle box."""
        px_s, py_s = world_to_pixel(cx - sx/2, cy - sy/2)
        px_e, py_e = world_to_pixel(cx + sx/2, cy + sy/2)
        for py in range(max(0, py_s), min(h_px, py_e)):
            for px in range(max(0, px_s), min(w_px, px_e)):
                pixels[py][px] = 0

    # Outer walls
    draw_wall_h(0, 0, WORLD_W)
    draw_wall_h(WORLD_H, 0, WORLD_W)
    draw_wall_v(0, 0, WORLD_H)
    draw_wall_v(WORLD_W, 0, WORLD_H)

    # Corridor walls with door gaps
    for y_wall in [CORRIDOR_Y_LO, CORRIDOR_Y_HI]:
        for col in [COL1, COL2, COL3]:
            # Each column has a door centered: col1@2.0, col2@7.0, col3@12.0
            if col == COL1:
                door_cx = 2.0
            elif col == COL2:
                door_cx = 7.0
            else:
                door_cx = 12.0

            # Left of door
            draw_wall_h(y_wall, col[0], door_cx - DOOR_W/2)
            # Right of door
            draw_wall_h(y_wall, door_cx + DOOR_W/2, col[1])

        # Walls between columns
        draw_wall_h(y_wall, COL1[1], COL2[0])
        draw_wall_h(y_wall, COL2[1], COL3[0])

    # Room dividers (vertical walls between rooms)
    for x_wall in [COL1[1], COL2[0], COL2[1], COL3[0]]:
        draw_wall_v(x_wall, 0, CORRIDOR_Y_LO)
        draw_wall_v(x_wall, CORRIDOR_Y_HI, WORLD_H)

    # Furniture obstacles
    draw_box(1.75, 8.5, 2.0, 0.9)    # bed_01
    draw_box(7.0, 8.5, 2.0, 0.9)     # bed_02
    draw_box(12.25, 8.5, 2.0, 0.9)   # bed_03
    draw_box(7.0, 1.5, 2.0, 0.9)     # bed_04
    draw_box(0.5, 8.5, 0.5, 0.5)     # table_01
    draw_box(5.5, 8.5, 0.5, 0.5)     # table_02
    draw_box(10.8, 8.5, 0.5, 0.5)    # table_03
    draw_box(5.5, 1.5, 0.5, 0.5)     # table_04
    draw_box(1.75, 1.0, 2.5, 0.6)    # pharmacy counter
    draw_box(12.25, 2.0, 1.8, 0.8)   # nurse desk
    draw_box(4.5, 5.0, 0.4, 0.4)     # corridor chair 1
    draw_box(9.5, 5.0, 0.4, 0.4)     # corridor chair 2

    # Write PGM (P5 binary format)
    # PGM is stored top-to-bottom, but our pixel array is bottom-to-top
    with open(pgm_path, 'wb') as f:
        header = f"P5\n{w_px} {h_px}\n255\n"
        f.write(header.encode())
        # Flip vertically: row 0 in PGM = top of image = highest Y
        for row in reversed(pixels):
            f.write(bytes(row))

    # Write YAML
    yaml_content = f"""image: hospital_ward.pgm
resolution: {MAP_RESOLUTION}
origin: [{MAP_ORIGIN_X}, {MAP_ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f"Map saved to {pgm_path} ({w_px}x{h_px} pixels)")
    print(f"Map YAML saved to {yaml_path}")


if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.join(script_dir, '..')

    world_path = os.path.join(pkg_dir, 'worlds', 'hospital_ward.world')
    pgm_path = os.path.join(pkg_dir, 'maps', 'hospital_ward.pgm')
    yaml_path = os.path.join(pkg_dir, 'maps', 'hospital_ward.yaml')

    generate_world(world_path)
    generate_map(pgm_path, yaml_path)
    print("\nDone! Hospital ward generated successfully.")

