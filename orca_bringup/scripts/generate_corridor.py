#!/usr/bin/env python3

"""
Generate models/corredor, a long underwater corridor to test the Rolling Bonxai global map.

The inpetu pool (about 5.8 x 3.6 m) fits inside the window Rolling Bonxai keeps in memory (chunks of 2.5 m,
radius 2: 12.5 m), so no chunk ever leaves memory there. Going from one end of this corridor to the other moves
chunks out of memory (saved to disk), and coming back loads them again.

Model frame: x runs along the corridor from the inner face of the start wall, y = 0 is the center line and
z = 0 is the water surface (the buoyancy plugin puts water below z = 0).

- Collisions are a few long boxes: floor, two side walls and two end walls.
- Visuals are panels of about 2 m with the concrete texture of the inpetu pool. Each panel uses one of a few crops
  of the texture, so the walls don't repeat the same image every 2 m.
- Landmarks (pillars, boxes and cylinders in different colours) every 2.5 m on alternating sides, because a smooth
  corridor looks the same everywhere to ICP and SLAM.

Typical usage:
scripts/generate_corridor.py
"""

import math
import os
import random

from PIL import Image

MODEL_NAME = 'corredor'
MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models')
MODEL_DIR = os.path.join(MODELS_DIR, MODEL_NAME)

# Corridor size (m)
LENGTH = 36.0       # inside, along x
WIDTH = 3.0         # inside, along y
FLOOR_Z = -4.5      # top of the floor
TOP_Z = 0.5         # top of the walls, above the water
THICKNESS = 0.2     # floor and walls
PANEL_SIZE = 2.0    # largest side of a visual panel

# Texture crops: NUM_TEXTURES squares of CROP_SIZE px from the pool texture, saved with TEXTURE_SIZE px
TEXTURE_SOURCE = os.path.join(MODELS_DIR, 'inpetu_pool', 'meshes', '002_RV_CONCRETE_TEXTURE.jpg')
NUM_TEXTURES = 6
CROP_SIZE = 2048
TEXTURE_SIZE = 1024
TEXTURE_QUALITY = 90

# Fixed seed: the same model every time the script runs
SEED = 4

LEFT = 1.0          # wall at y = +WIDTH / 2
RIGHT = -1.0        # wall at y = -WIDTH / 2

CONCRETE = (0.85, 0.85, 0.85)
RED = (0.9, 0.12, 0.08)
ORANGE = (1.0, 0.5, 0.05)
YELLOW = (1.0, 0.9, 0.1)
LIME = (0.55, 0.95, 0.1)
GREEN = (0.1, 0.65, 0.2)
BLUE = (0.15, 0.3, 1.0)
PURPLE = (0.55, 0.2, 0.85)
MAGENTA = (0.95, 0.15, 0.7)
WHITE = (1.0, 1.0, 1.0)
BLACK = (0.08, 0.08, 0.08)


def box_geometry(size):
    return f'<box><size>{size[0]:g} {size[1]:g} {size[2]:g}</size></box>'


def box_landmark(center, size, colour):
    return center, box_geometry(size), colour


def pillar(x, side, colour, size=0.3):
    """Square pillar against a side wall, from the floor to the top of the wall."""
    y = side * (WIDTH / 2 - size / 2)
    return box_landmark((x, y, (FLOOR_Z + TOP_Z) / 2), (size, size, TOP_Z - FLOOR_Z), colour)


def wall_box(x, side, z, length, depth, height, colour):
    """Box sticking out of a side wall, centered at height z."""
    y = side * (WIDTH / 2 - depth / 2)
    return box_landmark((x, y, z), (length, depth, height), colour)


def floor_box(x, y, top, size, colour):
    """Square box standing on the floor up to height top."""
    return box_landmark((x, y, (FLOOR_Z + top) / 2), (size, size, top - FLOOR_Z), colour)


def floor_cylinder(x, y, top, radius, colour):
    """Vertical cylinder standing on the floor up to height top."""
    length = top - FLOOR_Z
    geometry = f'<cylinder><radius>{radius:g}</radius><length>{length:g}</length></cylinder>'
    return (x, y, (FLOOR_Z + top) / 2), geometry, colour


def end_mark(at_end, z, size, colour):
    """Thin square on an end wall, facing into the corridor."""
    depth = 0.05
    x = LENGTH - depth / 2 if at_end else depth / 2
    return box_landmark((x, 0.0, z), (depth, size, size), colour)


# x from the inner face of the start wall (the world puts the ROV at x = 2). Every 2.5 m on alternating sides, each
# one different from the others in shape, size or colour, plus a square on each end wall.
LANDMARKS = [
    end_mark(False, -1.0, 1.2, GREEN),
    wall_box(4.0, RIGHT, -1.0, 0.8, 0.4, 0.6, RED),
    pillar(6.5, LEFT, YELLOW),
    floor_cylinder(9.0, -0.9, -1.8, 0.25, GREEN),
    wall_box(11.5, LEFT, -1.5, 1.2, 0.35, 0.4, BLUE),
    pillar(14.0, RIGHT, ORANGE),
    floor_box(16.5, 0.9, -2.0, 0.6, MAGENTA),
    wall_box(19.0, RIGHT, -0.8, 0.5, 0.4, 1.0, WHITE),
    pillar(21.5, LEFT, PURPLE),
    floor_cylinder(24.0, -0.9, -2.4, 0.35, LIME),
    wall_box(26.5, LEFT, -1.0, 0.6, 0.4, 0.6, BLACK),
    pillar(29.0, RIGHT, BLUE, size=0.4),
    floor_box(31.5, 0.9, -1.6, 0.5, YELLOW),
    end_mark(True, -1.0, 1.2, RED),
]


def split(start, end):
    """Split [start, end] into equal parts no larger than PANEL_SIZE. Returns (center, size) of each part."""
    count = math.ceil((end - start) / PANEL_SIZE - 1e-9)
    size = (end - start) / count
    return [(start + (i + 0.5) * size, size) for i in range(count)]


def pose(center):
    return f'{center[0]:g} {center[1]:g} {center[2]:g} 0 0 0'


def material(colour, texture):
    r, g, b = colour
    return f"""        <material>
          <ambient>{r:g} {g:g} {b:g} 1</ambient>
          <diffuse>{r:g} {g:g} {b:g} 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://{MODEL_NAME}/materials/textures/{texture}</albedo_map>
              <roughness>0.9</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>"""


def visual(name, center, geometry, colour, texture):
    return f"""      <visual name="{name}">
        <pose>{pose(center)}</pose>
        <geometry>{geometry}</geometry>
{material(colour, texture)}
      </visual>"""


def collision(name, center, geometry):
    # SDF wants unique names among all the elements of a link, visuals included
    return f"""      <collision name="{name}_collision">
        <pose>{pose(center)}</pose>
        <geometry>{geometry}</geometry>
      </collision>"""


def write_textures(model_dir):
    """Crop NUM_TEXTURES squares from the pool texture into model_dir/materials/textures: (0,0), the other corners,
    the center and a rotated one. Returns the file names."""
    source = Image.open(TEXTURE_SOURCE).convert('RGB')
    side = source.size[0] - CROP_SIZE
    corners = [(0, 0), (side, 0), (0, side), (side, side), (side // 2, side // 2), (side // 2, 0)]

    os.makedirs(os.path.join(model_dir, 'materials', 'textures'), exist_ok=True)
    names = []
    for i, (left, top) in enumerate(corners[:NUM_TEXTURES]):
        crop = source.crop((left, top, left + CROP_SIZE, top + CROP_SIZE))
        if i == len(corners) - 1:
            crop = crop.transpose(Image.Transpose.ROTATE_90)
        crop = crop.resize((TEXTURE_SIZE, TEXTURE_SIZE), Image.Resampling.LANCZOS)
        name = f'concrete_{i}.jpg'
        crop.save(os.path.join(model_dir, 'materials', 'textures', name), quality=TEXTURE_QUALITY)
        names.append(name)
    return names


def corridor():
    rng = random.Random(SEED)
    textures = write_textures(MODEL_DIR)
    elements = []

    def panel(name, center, size):
        # Slightly different shades of grey so the panels don't look all the same
        shade = rng.uniform(0.85, 1.0)
        colour = tuple(c * shade for c in CONCRETE)
        elements.append(visual(name, center, box_geometry(size), colour, rng.choice(textures)))

    wall_z = (FLOOR_Z + TOP_Z) / 2
    wall_h = TOP_Z - FLOOR_Z
    outer_w = WIDTH + 2 * THICKNESS

    # Collisions: floor under everything, side walls on the floor, end walls closing the ends
    elements.append(collision('floor', (LENGTH / 2, 0, FLOOR_Z - THICKNESS / 2),
                              box_geometry((LENGTH + 2 * THICKNESS, outer_w, THICKNESS))))
    for side, label in ((LEFT, 'left'), (RIGHT, 'right')):
        elements.append(collision(f'wall_{label}', (LENGTH / 2, side * (WIDTH + THICKNESS) / 2, wall_z),
                                  box_geometry((LENGTH, THICKNESS, wall_h))))
    for x, label in ((-THICKNESS / 2, 'start'), (LENGTH + THICKNESS / 2, 'end')):
        elements.append(collision(f'wall_{label}', (x, 0, wall_z), box_geometry((THICKNESS, outer_w, wall_h))))

    # Visual panels
    for i, (x, sx) in enumerate(split(0, LENGTH)):
        for j, (y, sy) in enumerate(split(-WIDTH / 2, WIDTH / 2)):
            panel(f'floor_{i}_{j}', (x, y, FLOOR_Z - THICKNESS / 2), (sx, sy, THICKNESS))
        for side, label in ((LEFT, 'left'), (RIGHT, 'right')):
            for j, (z, sz) in enumerate(split(FLOOR_Z, TOP_Z)):
                panel(f'wall_{label}_{i}_{j}', (x, side * (WIDTH + THICKNESS) / 2, z), (sx, THICKNESS, sz))
    for x, label in ((-THICKNESS / 2, 'start'), (LENGTH + THICKNESS / 2, 'end')):
        for i, (y, sy) in enumerate(split(-outer_w / 2, outer_w / 2)):
            for j, (z, sz) in enumerate(split(FLOOR_Z, TOP_Z)):
                panel(f'wall_{label}_{i}_{j}', (x, y, z), (THICKNESS, sy, sz))

    # Landmarks: the same texture tinted with their colour, so they have features for the stereo and the SLAM
    for i, (center, geometry, colour) in enumerate(LANDMARKS):
        elements.append(visual(f'landmark_{i}', center, geometry, colour, rng.choice(textures)))
        elements.append(collision(f'landmark_{i}', center, geometry))

    body = '\n'.join(elements)
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_corridor.py, edit the script instead of this file -->
<sdf version="1.6">
  <model name="{MODEL_NAME}">
    <static>true</static>
    <link name="corridor">
{body}
    </link>
  </model>
</sdf>
"""


MODEL_CONFIG = f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_corridor.py -->
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Henrique Schneider</name>
    <email>schneider.henrique1706@gmail.com</email>
  </author>

  <description>
    Underwater corridor of {LENGTH:g} x {WIDTH:g} m with landmarks, longer than the Rolling Bonxai memory window
  </description>
</model>
"""


def main():
    os.makedirs(MODEL_DIR, exist_ok=True)
    with open(os.path.join(MODEL_DIR, 'model.sdf'), 'w') as f:
        f.write(corridor())
    with open(os.path.join(MODEL_DIR, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG)
    print(f'Wrote {os.path.normpath(MODEL_DIR)}')


if __name__ == '__main__':
    main()
