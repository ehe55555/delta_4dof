#!/usr/bin/env python3

from dataclasses import dataclass
from collections import deque

import numpy as np
from scipy.ndimage import binary_erosion, label

from delta_control.kinematics import DeltaKinematics2


@dataclass
class WorkspaceMesh:
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    volume_x: np.ndarray
    volume_y: np.ndarray
    volume_z: np.ndarray
    scan_path: np.ndarray
    bounds_mm: tuple[float, float, float, float, float, float]
    grid_step_mm: float


def calculate_workspace_surface(
    solver: DeltaKinematics2,
    grid_step: float = 0.004,
    scan_divisions: int = 12,
) -> WorkspaceMesh:
    """Return surface voxels of the IK workspace connected to HOME."""
    g = solver.g
    x_values = np.arange(-0.35, 0.35 + grid_step * 0.5, grid_step)
    y_values = np.arange(-0.33, 0.33 + grid_step * 0.5, grid_step)
    z_values = np.arange(-0.50, -0.18 + grid_step * 0.5, grid_step)
    x_grid, y_grid = np.meshgrid(x_values, y_values, indexing="xy")

    valid = np.zeros(
        (len(z_values), len(y_values), len(x_values)),
        dtype=bool,
    )

    for z_index, z_world in enumerate(z_values):
        z = z_world - g.motor_plane_z
        layer = np.ones(x_grid.shape, dtype=bool)

        for phi in solver.phis:
            ux = np.cos(phi)
            uy = np.sin(phi)
            vx = -np.sin(phi)
            vy = np.cos(phi)

            s = ux * x_grid + uy * y_grid - g.d_base
            h = vx * x_grid + vy * y_grid
            a = -s
            c = (
                g.l2 * g.l2
                - s * s
                - h * h
                - z * z
                - g.l1 * g.l1
            ) / (2.0 * g.l1)
            radius = np.sqrt(a * a + z * z)
            layer &= (radius > 1e-12) & (np.abs(c / radius) <= 1.0)

        valid[z_index] = layer

    labels, _ = label(valid)
    home_index = (
        int(np.argmin(np.abs(z_values - g.home_z))),
        int(np.argmin(np.abs(y_values - g.home_y))),
        int(np.argmin(np.abs(x_values - g.home_x))),
    )
    home_label = labels[home_index]
    if home_label == 0:
        raise RuntimeError("HOME is not inside the calculated workspace grid.")

    connected = labels == home_label
    surface = connected & ~binary_erosion(connected)
    z_index, y_index, x_index = np.where(surface)

    inside_z, inside_y, inside_x = np.where(connected)
    volume_sample = connected[::2, ::2, ::2]
    volume_z_index, volume_y_index, volume_x_index = np.where(volume_sample)
    sampled_x = x_values[::2]
    sampled_y = y_values[::2]
    sampled_z = z_values[::2]
    bounds_mm = (
        float(x_values[inside_x].min() * 1000.0),
        float(x_values[inside_x].max() * 1000.0),
        float(y_values[inside_y].min() * 1000.0),
        float(y_values[inside_y].max() * 1000.0),
        float(z_values[inside_z].min() * 1000.0),
        float(z_values[inside_z].max() * 1000.0),
    )

    scan_path = _create_volume_scan_path(solver, scan_divisions)

    return WorkspaceMesh(
        x=x_values[x_index] * 1000.0,
        y=y_values[y_index] * 1000.0,
        z=z_values[z_index] * 1000.0,
        volume_x=sampled_x[volume_x_index] * 1000.0,
        volume_y=sampled_y[volume_y_index] * 1000.0,
        volume_z=sampled_z[volume_z_index] * 1000.0,
        scan_path=scan_path,
        bounds_mm=bounds_mm,
        grid_step_mm=grid_step * 1000.0,
    )


def _point_is_reachable(solver, x, y, z_world):
    g = solver.g
    z = z_world - g.motor_plane_z
    for phi in solver.phis:
        ux = np.cos(phi)
        uy = np.sin(phi)
        vx = -np.sin(phi)
        vy = np.cos(phi)
        s = ux * x + uy * y - g.d_base
        h = vx * x + vy * y
        a = -s
        c = (
            g.l2 * g.l2
            - s * s
            - h * h
            - z * z
            - g.l1 * g.l1
        ) / (2.0 * g.l1)
        radius = np.hypot(a, z)
        if radius <= 1e-12 or abs(c / radius) > 1.0:
            return False
    return True


def _create_volume_scan_path(solver, divisions):
    """Visit a safe grid of cells throughout the HOME-connected workspace."""
    divisions = max(5, min(24, int(divisions)))
    home = np.array(
        [solver.g.home_x, solver.g.home_y, solver.g.home_z],
        dtype=float,
    )
    x_values = np.linspace(-0.33365 * 0.95, 0.33365 * 0.95, divisions)
    y_values = np.linspace(-0.31512 * 0.95, 0.31512 * 0.95, divisions)
    z_values = np.linspace(-0.48284 + 0.0142, -0.19910 - 0.0142, divisions)

    valid = np.zeros((divisions, divisions, divisions), dtype=bool)
    for iz, z_world in enumerate(z_values):
        for iy, y in enumerate(y_values):
            for ix, x in enumerate(x_values):
                valid[iz, iy, ix] = _point_is_reachable(
                    solver, x, y, z_world
                )

    home_index = (
        int(np.argmin(np.abs(z_values - home[2]))),
        int(np.argmin(np.abs(y_values - home[1]))),
        int(np.argmin(np.abs(x_values - home[0]))),
    )
    connectivity = np.zeros((3, 3, 3), dtype=int)
    connectivity[1, 1, 1] = 1
    connectivity[0, 1, 1] = 1
    connectivity[2, 1, 1] = 1
    connectivity[1, 0, 1] = 1
    connectivity[1, 2, 1] = 1
    connectivity[1, 1, 0] = 1
    connectivity[1, 1, 2] = 1
    labels, _ = label(valid, structure=connectivity)
    home_label = labels[home_index]
    if home_label == 0:
        nearest = np.argwhere(valid)
        if not len(nearest):
            raise RuntimeError("No reachable workspace cells were found.")
        home_index = tuple(
            nearest[
                np.argmin(
                    np.sum((nearest - np.asarray(home_index)) ** 2, axis=1)
                )
            ]
        )
        home_label = labels[home_index]

    connected = labels == home_label
    cells = []
    for iz in range(divisions):
        y_order = range(divisions) if iz % 2 == 0 else range(divisions - 1, -1, -1)
        for row_index, iy in enumerate(y_order):
            x_indices = np.where(connected[iz, iy])[0]
            if not len(x_indices):
                continue
            if (iz + row_index) % 2:
                x_indices = x_indices[::-1]
            cells.extend((iz, iy, int(ix)) for ix in x_indices)

    path_indices = [home_index]
    for target in cells:
        if target == path_indices[-1]:
            continue
        path_indices.extend(
            _grid_shortest_path(connected, path_indices[-1], target)[1:]
        )
    path_indices.extend(
        _grid_shortest_path(connected, path_indices[-1], home_index)[1:]
    )

    points = [home]
    points.extend(
        np.array([x_values[ix], y_values[iy], z_values[iz]])
        for iz, iy, ix in path_indices
    )
    points.append(home)
    return np.asarray(points)


def _grid_shortest_path(valid, start, goal):
    if start == goal:
        return [start]

    queue = deque([start])
    previous = {start: None}
    shape = valid.shape
    neighbors = (
        (-1, 0, 0),
        (1, 0, 0),
        (0, -1, 0),
        (0, 1, 0),
        (0, 0, -1),
        (0, 0, 1),
    )

    while queue:
        current = queue.popleft()
        for delta in neighbors:
            nxt = tuple(current[i] + delta[i] for i in range(3))
            if any(nxt[i] < 0 or nxt[i] >= shape[i] for i in range(3)):
                continue
            if not valid[nxt] or nxt in previous:
                continue
            previous[nxt] = current
            if nxt == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(previous[path[-1]])
                return path[::-1]
            queue.append(nxt)

    raise RuntimeError("Unable to connect two neighboring workspace cells.")
