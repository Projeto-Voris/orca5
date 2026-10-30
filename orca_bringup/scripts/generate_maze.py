#!/usr/bin/env python3

"""
Generate models/labirinto, a 100 x 100 m underwater maze to test the Rolling Bonxai global map over a large area.

Same depth, walls and concrete texture as the corridor (scripts/generate_corridor.py), without the coloured
landmarks. The maze is a grid of CELLS x CELLS cells carved by a depth-first search, so every cell can be reached,
plus a few extra openings (LOOP_FRACTION) so that some paths make loops.

Model frame: origin at the center of the start cell, x and y along the grid, z = 0 at the water surface (the buoyancy
plugin puts water below z = 0). The start cell opens towards +x, where the ROV faces at the start.

The finish is the cell farthest from the start along the maze, marked by flat chequered squares (no collision) on its
floor and on the wall facing the way in. solution.png shows the maze from above with the shortest path to it.

- Collisions are boxes: the floor and one box per straight run of wall.
- Visuals are NUM_TEXTURES meshes (meshes/maze_<n>.obj), one per crop of the texture, holding panels of about 2 m.
  A few meshes instead of thousands of boxes keep Gazebo fast; each panel gets a random crop and orientation, so
  the walls don't repeat the same image every 2 m.

Typical usage:
scripts/generate_maze.py
"""

import os
import random
from collections import deque

from PIL import Image, ImageDraw, ImageFont

from generate_corridor import (CONCRETE, FLOOR_Z, MODELS_DIR, NUM_TEXTURES, PANEL_SIZE, THICKNESS, TOP_Z, split,
                               write_textures)

MODEL_NAME = 'labirinto'
MODEL_DIR = os.path.join(MODELS_DIR, MODEL_NAME)

# Maze size
SIZE = 100.0            # between the center lines of the outer walls (m)
CELLS = 30              # cells per side: corridors of SIZE / CELLS - THICKNESS = 3.13 m
LOOP_FRACTION = 0.05    # share of the inner walls left after carving that are opened to make loops

# Finish: chequered squares of FINISH_SIZE m, the one on the wall centered at FINISH_Z (the ROV camera is near -1.3 m)
FINISH_SIZE = 2.0
FINISH_Z = -1.5
FINISH_SQUARES = 8      # squares per side of the chequered texture

# Fixed seed: the same maze every time the script runs
SEED = 4

PITCH = SIZE / CELLS


def grid(k):
    """Coordinate of grid line k (0..CELLS): the cell (0, 0) is centered on the origin."""
    return -PITCH / 2 + k * PITCH


def carve(rng):
    """
    Walls left after carving the maze.
    h[i][j]: wall on the line y = grid(j) between x = grid(i) and grid(i + 1), i.e. between cells (i, j - 1) and (i, j).
    v[i][j]: wall on the line x = grid(i) between y = grid(j) and grid(j + 1), i.e. between cells (i - 1, j) and (i, j).
    """
    h = [[True] * (CELLS + 1) for _ in range(CELLS)]
    v = [[True] * CELLS for _ in range(CELLS + 1)]

    def open_between(a, b):
        (i, j), (k, l) = a, b
        if i == k:
            h[i][max(j, l)] = False
        else:
            v[max(i, k)][j] = False

    # Depth-first search (recursive backtracker), with a list instead of recursion for the 900 cells
    visited = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        i, j = stack[-1]
        neighbours = [(i + di, j + dj) for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
                      if 0 <= i + di < CELLS and 0 <= j + dj < CELLS and (i + di, j + dj) not in visited]
        if not neighbours:
            stack.pop()
            continue
        cell = rng.choice(neighbours)
        open_between((i, j), cell)
        visited.add(cell)
        stack.append(cell)

    # Loops: open some of the inner walls left
    inner = ([(h, i, j) for i in range(CELLS) for j in range(1, CELLS) if h[i][j]] +
             [(v, i, j) for i in range(1, CELLS) for j in range(CELLS) if v[i][j]])
    for walls, i, j in rng.sample(inner, round(LOOP_FRACTION * len(inner))):
        walls[i][j] = False

    # The ROV starts in the cell (0, 0) looking towards +x
    v[1][0] = False
    return h, v


def cell_walls(h, v, i, j):
    """Walls around the cell (i, j), by the direction from its center: True where there is a wall."""
    return {(1, 0): v[i + 1][j], (-1, 0): v[i][j], (0, 1): h[i][j + 1], (0, -1): h[i][j]}


def solve(h, v):
    """The finish is the cell farthest from the start along the maze. Returns it and the shortest path to it."""
    parent = {(0, 0): None}
    queue = deque([(0, 0)])
    while queue:
        cell = queue.popleft()
        for (di, dj), wall in cell_walls(h, v, *cell).items():
            neighbour = (cell[0] + di, cell[1] + dj)
            if not wall and neighbour not in parent:
                parent[neighbour] = cell
                queue.append(neighbour)
    # The breadth-first search reaches the cells in order of distance: the last one is the farthest
    finish = cell
    path = []
    while cell is not None:
        path.append(cell)
        cell = parent[cell]
    return finish, path[::-1]


def finish_marks(h, v, finish):
    """Flat chequered squares at the finish, without collision: on the floor, and on the wall facing the way in (or
    another wall of the cell). Returns (name, center, size) of each."""
    i, j = finish
    x, y = i * PITCH, j * PITCH
    depth = 0.02
    marks = [('finish_floor', (x, y, FLOOR_Z + 0.75 * depth), (FINISH_SIZE, FINISH_SIZE, depth))]

    walls = cell_walls(h, v, i, j)
    way_in = next(d for d, wall in walls.items() if not wall)
    back = (-way_in[0], -way_in[1])
    present = [d for d, wall in walls.items() if wall]
    if present:
        dx, dy = back if walls[back] else present[0]
        gap = PITCH / 2 - THICKNESS / 2 - 0.75 * depth   # a little off the wall face, so they don't flicker
        size = (depth if dx else FINISH_SIZE, depth if dy else FINISH_SIZE, FINISH_SIZE)
        marks.append(('finish_wall', (x + dx * gap, y + dy * gap, FINISH_Z), size))
    return marks


def write_finish_texture(folder):
    side = 64 * FINISH_SQUARES
    image = Image.new('L', (side, side), 255)
    draw = ImageDraw.Draw(image)
    for a in range(FINISH_SQUARES):
        for b in range(FINISH_SQUARES):
            if (a + b) % 2:
                draw.rectangle([a * 64, b * 64, a * 64 + 63, b * 64 + 63], fill=0)
    image.save(os.path.join(folder, 'finish.png'))


def draw_solution(h, v, path, filename):
    """Maze seen from above, x to the right and y up, with the shortest path from the start to the finish."""
    scale_px = 8                        # pixels per m
    margin = 50
    side = int(SIZE * scale_px) + 2 * margin
    image = Image.new('RGB', (side, side + 50), 'white')
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype('DejaVuSans.ttf', 16)
    except OSError:
        font = ImageFont.load_default(size=16)    # without accents

    def px(x, y):
        return margin + (x - grid(0)) * scale_px, margin + (grid(CELLS) - y) * scale_px

    def center(cell):
        return px(cell[0] * PITCH, cell[1] * PITCH)

    for i in range(CELLS):
        for j in range(CELLS + 1):
            if h[i][j]:
                draw.line([px(grid(i), grid(j)), px(grid(i + 1), grid(j))], fill=(40, 50, 60), width=3)
    for i in range(CELLS + 1):
        for j in range(CELLS):
            if v[i][j]:
                draw.line([px(grid(i), grid(j)), px(grid(i), grid(j + 1))], fill=(40, 50, 60), width=3)

    draw.line([center(c) for c in path], fill=(220, 40, 40), width=4, joint='curve')
    r = 9
    sx, sy = center(path[0])
    draw.ellipse([sx - r, sy - r, sx + r, sy + r], fill=(30, 160, 60))
    fx, fy = center(path[-1])
    for a in range(2):
        for b in range(2):
            draw.rectangle([fx - r + a * r, fy - r + b * r, fx + a * r, fy + b * r],
                           fill=(0, 0, 0) if (a + b) % 2 else (255, 255, 255), outline=(0, 0, 0))

    # Axes in metres, from the start cell
    for k in range(0, int(SIZE), 10):
        x, y = px(k, grid(0))
        draw.text((x, y + 6), f'{k}', fill='black', font=font, anchor='mt')
        x, y = px(grid(0), k)
        draw.text((x - 6, y), f'{k}', fill='black', font=font, anchor='rm')
    finish_x, finish_y = path[-1][0] * PITCH, path[-1][1] * PITCH
    draw.text((margin, side - 10),
              f'início (0, 0) em verde · chegada ({finish_x:.1f}, {finish_y:.1f}) m · x e y em metros\n'
              f'caminho mais curto em vermelho: {(len(path) - 1) * PITCH:.0f} m',
              fill='black', font=font, spacing=8)
    image.save(filename)


def runs(h, v):
    """Straight runs of wall: (axis, line, start, end) in grid indices, along x on y = grid(line) or along y."""
    found = []
    for axis, line_count, length, present in (('x', CELLS + 1, CELLS, lambda line, k: h[k][line]),
                                              ('y', CELLS + 1, CELLS, lambda line, k: v[line][k])):
        for line in range(line_count):
            k = 0
            while k < length:
                if not present(line, k):
                    k += 1
                    continue
                start = k
                while k < length and present(line, k):
                    k += 1
                found.append((axis, line, start, k))
    return found


class Meshes:
    """Quads grouped by texture, written as one OBJ file per texture."""

    def __init__(self, rng):
        self.rng = rng
        self.quads = [[] for _ in range(NUM_TEXTURES)]

    def quad(self, origin, u, v, uv, texture):
        """Corners origin, +u, +u+v, +v: counterclockwise seen from the side u x v points to (the visible side)."""
        corners = [origin, add(origin, u), add(add(origin, u), v), add(origin, v)]
        self.quads[texture].append((corners, uv, normalize(cross(u, v))))

    def panel(self, origin, u, v):
        """Quad with a random crop of the texture, rotated by a multiple of 90 degrees and maybe mirrored."""
        uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
        turn = self.rng.randrange(4)
        uv = uv[turn:] + uv[:turn]
        if self.rng.random() < 0.5:
            uv = [(1 - s, t) for s, t in uv]
        self.quad(origin, u, v, uv, self.rng.randrange(NUM_TEXTURES))

    def strip(self, origin, u, v):
        """Thin quad (top or end of a wall): the texture keeps its scale of one image per PANEL_SIZE, up to 1."""
        s = min(1.0, length(u) / PANEL_SIZE)
        t = min(1.0, length(v) / PANEL_SIZE)
        self.quad(origin, u, v, [(0, 0), (s, 0), (s, t), (0, t)], self.rng.randrange(NUM_TEXTURES))

    def write(self, folder):
        os.makedirs(folder, exist_ok=True)
        for texture, quads in enumerate(self.quads):
            normals = sorted({n for _, _, n in quads})
            lines = ['# Generated by scripts/generate_maze.py, edit the script instead of this file']
            lines += [f'vn {n[0]:g} {n[1]:g} {n[2]:g}' for n in normals]
            for corners, uv, _ in quads:
                lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for x, y, z in corners]
                lines += [f'vt {s:g} {t:.4f}' for s, t in uv]
            for q, (_, _, n) in enumerate(quads):
                ni = normals.index(n) + 1
                lines.append('f ' + ' '.join(f'{4 * q + c + 1}/{4 * q + c + 1}/{ni}' for c in range(4)))
            with open(os.path.join(folder, f'maze_{texture}.obj'), 'w') as f:
                f.write('\n'.join(lines) + '\n')


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def length(a):
    return sum(x * x for x in a) ** 0.5


def normalize(a):
    return tuple(round(x / length(a)) for x in a)  # only axis-aligned faces here


def build(rng, h, v):
    meshes = Meshes(rng)
    collisions = []
    half = THICKNESS / 2
    up = (0.0, 0.0, 1.0)
    rows = split(FLOOR_Z, TOP_Z)

    def degree(i, j):
        """Number of wall pieces touching the grid vertex (i, j)."""
        return sum([i > 0 and h[i - 1][j], i < CELLS and h[i][j], j > 0 and v[i][j - 1], j < CELLS and v[i][j]])

    def top_edge(i, j, run_end, inward):
        """Where the top of a wall along y stops at the vertex (i, j): short of a wall along x touching the vertex,
        past the vertex at the end of the run, on the vertex between two pieces of the same run."""
        if (i > 0 and h[i - 1][j]) or (i < CELLS and h[i][j]):
            return grid(j) + inward
        return grid(j) - inward if run_end else grid(j)

    def wall_side(fixed_axis, fixed, a, b, normal):
        """Panels of the vertical face from a to b along the run, on the plane fixed_axis = fixed."""
        along = cross(up, normal)                 # along x up = normal, so the panel faces the normal
        sign = sum(along)                         # +1 or -1 along the run axis
        for center, size in split(a, b):
            start = center - sign * size / 2
            for z, sz in rows:
                origin = [0.0, 0.0, z - sz / 2]
                origin[fixed_axis] = fixed
                origin[1 - fixed_axis] = start
                meshes.panel(tuple(origin), scale(along, size), (0.0, 0.0, sz))

    def wall_end(fixed_axis, fixed, across_center, normal):
        """Face closing a run that ends without touching another wall."""
        along = cross(up, normal)
        sign = sum(along)
        for z, sz in rows:
            origin = [0.0, 0.0, z - sz / 2]
            origin[fixed_axis] = fixed
            origin[1 - fixed_axis] = across_center - sign * half
            meshes.strip(tuple(origin), scale(along, THICKNESS), (0.0, 0.0, sz))

    for axis, line, start, end in runs(h, v):
        c = grid(line)
        a, b = grid(start) - half, grid(end) + half
        if axis == 'x':
            collisions.append((f'wall_{len(collisions)}', ((a + b) / 2, c, (FLOOR_Z + TOP_Z) / 2),
                               (b - a, THICKNESS, TOP_Z - FLOOR_Z)))
            wall_side(1, c + half, a, b, (0, 1, 0))
            wall_side(1, c - half, a, b, (0, -1, 0))
            if degree(start, line) == 1:
                wall_end(0, a, c, (-1, 0, 0))
            if degree(end, line) == 1:
                wall_end(0, b, c, (1, 0, 0))
            for center, size in split(a, b):
                meshes.strip((center - size / 2, c - half, TOP_Z), (size, 0.0, 0.0), (0.0, THICKNESS, 0.0))
        else:
            collisions.append((f'wall_{len(collisions)}', (c, (a + b) / 2, (FLOOR_Z + TOP_Z) / 2),
                               (THICKNESS, b - a, TOP_Z - FLOOR_Z)))
            wall_side(0, c + half, a, b, (1, 0, 0))
            wall_side(0, c - half, a, b, (-1, 0, 0))
            if degree(line, start) == 1:
                wall_end(1, a, c, (0, -1, 0))
            if degree(line, end) == 1:
                wall_end(1, b, c, (0, 1, 0))
            # Tops one piece at a time, stopping at the tops of the walls along x so no two tops overlap
            for k in range(start, end):
                y0 = top_edge(line, k, k == start, half)
                y1 = top_edge(line, k + 1, k + 1 == end, -half)
                meshes.strip((c + half, y0, TOP_Z), (0.0, y1 - y0, 0.0), (-THICKNESS, 0.0, 0.0))

    # Floor under the whole maze
    low, high = grid(0) - half, grid(CELLS) + half
    for x, sx in split(low, high):
        for y, sy in split(low, high):
            meshes.panel((x - sx / 2, y - sy / 2, FLOOR_Z), (sx, 0.0, 0.0), (0.0, sy, 0.0))
    middle = (low + high) / 2
    collisions.append(('floor', (middle, middle, FLOOR_Z - half), (high - low, high - low, THICKNESS)))

    return meshes, collisions


def model_sdf(collisions, marks):
    r, g, b = CONCRETE
    visuals = '\n'.join(f"""      <visual name="maze_{n}">
        <geometry>
          <mesh>
            <uri>model://{MODEL_NAME}/meshes/maze_{n}.obj</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>{r:g} {g:g} {b:g} 1</ambient>
          <diffuse>{r:g} {g:g} {b:g} 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://{MODEL_NAME}/materials/textures/concrete_{n}.jpg</albedo_map>
              <roughness>0.9</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>""" for n in range(NUM_TEXTURES))
    finish = '\n'.join(f"""      <visual name="{name}">
        <pose>{c[0]:.4f} {c[1]:.4f} {c[2]:g} 0 0 0</pose>
        <geometry><box><size>{s[0]:g} {s[1]:g} {s[2]:g}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://{MODEL_NAME}/materials/textures/finish.png</albedo_map>
              <roughness>0.9</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>""" for name, c, s in marks)
    boxes = '\n'.join(f"""      <collision name="{name}_collision">
        <pose>{c[0]:.4f} {c[1]:.4f} {c[2]:g} 0 0 0</pose>
        <geometry><box><size>{s[0]:.4f} {s[1]:.4f} {s[2]:g}</size></box></geometry>
      </collision>""" for name, c, s in collisions)
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_maze.py, edit the script instead of this file -->
<sdf version="1.6">
  <model name="{MODEL_NAME}">
    <static>true</static>
    <link name="maze">
{visuals}
{finish}
{boxes}
    </link>
  </model>
</sdf>
"""


MODEL_CONFIG = f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_maze.py -->
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Henrique Schneider</name>
    <email>schneider.henrique1706@gmail.com</email>
  </author>

  <description>
    Underwater maze of {SIZE:g} x {SIZE:g} m ({CELLS} x {CELLS} cells), without landmarks
  </description>
</model>
"""


def main():
    rng = random.Random(SEED)
    h, v = carve(rng)
    meshes, collisions = build(rng, h, v)
    finish, path = solve(h, v)

    meshes.write(os.path.join(MODEL_DIR, 'meshes'))
    write_textures(MODEL_DIR)
    write_finish_texture(os.path.join(MODEL_DIR, 'materials', 'textures'))
    draw_solution(h, v, path, os.path.join(MODEL_DIR, 'solution.png'))
    with open(os.path.join(MODEL_DIR, 'model.sdf'), 'w') as f:
        f.write(model_sdf(collisions, finish_marks(h, v, finish)))
    with open(os.path.join(MODEL_DIR, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG)
    quads = sum(len(q) for q in meshes.quads)
    print(f'Wrote {os.path.normpath(MODEL_DIR)}: {quads} panels, {len(collisions)} collision boxes, '
          f'finish at cell {finish} = ({finish[0] * PITCH:.2f}, {finish[1] * PITCH:.2f}) m, '
          f'shortest path {(len(path) - 1) * PITCH:.0f} m')


if __name__ == '__main__':
    main()
