#!/usr/bin/env python3
"""Theoretical upper bound on shelf coverage given arm-workspace constraints.

Question: given the arm base at world (0,0,0) and camera reach in
[reach_min, reach_max], what fraction of the shelf AABB is fundamentally
observable? This is a per-geometry upper bound to compare against actual
exploration test coverage (e.g. shelf_coverage_monitor's FINAL %).

Simple algorithm:

  1. Define a 3D voxel grid over the shelf AABB at resolution `res`.
     Each voxel is classified as MATERIAL (shelf panel/wall) or AIR.

  2. Sample K camera positions uniformly in the arm's reachable shell
     {p: reach_min <= ||p - base|| <= reach_max, z_min <= p.z <= z_max}.
     Restrict to the outward-facing hemisphere (dot(p - base, +x) > 0)
     to mirror `require_outward_look=True` in hppt_boundary_ros.

  3. For each voxel V, mark it VISIBLE if there exists at least one
     camera C such that the straight line C -> V does not pierce any
     other MATERIAL voxel before reaching V. (We do not simulate FOV:
     the arm can rotate the wrist to point the camera at any voxel, so
     the theoretical bound is orientation-free.)

  4. Report:
       coverage_% = |visible| / |total|
       occ_%      = |visible ∩ material| / |total|
       free_%     = |visible ∩ air| / |total|
       unknown_%  = 100 - coverage_%

     matching shelf_coverage_monitor's FINAL line format so numbers are
     directly comparable.

Shelf geometry: the SDF only tells us the collision box (0.84 x 0.39 x
1.11, solid). The visual mesh (RealShelf.obj) is hollow with internal
panels, but its topology is opaque without OBJ parsing. This script
supports three configurable models, from most pessimistic to most
optimistic:

  --model solid           : collision-box style (solid). Only the AABB
                            surface shell can be visible.
  --model hollow          : 5 outer panels (bottom, top, back, left,
                            right); front face (-X, facing arm) is open.
                            Interior is air.
  --model hollow_shelves  : same 5 outer panels plus N horizontal
                            internal shelves at evenly-spaced z levels.
                            (default N=3; guess for RealShelf.)

Use --model hollow_shelves --n-internal 3 as a realistic default.
"""
import argparse
import math
import sys
import time

import numpy as np


# ---------------------- Shelf AABB (from exploration_scene_shelf.sdf) ----------------------
# Model pose (0.80, 0.0, 0.0) + yaw = -pi/2, collision box offset (0, 0.195, 0.555)
# with size (0.84, 0.39, 1.11). After the yaw rotation and translation the
# axis-aligned world bbox is:
SHELF_AABB_MIN = np.array([0.80, -0.42, 0.00])
SHELF_AABB_MAX = np.array([1.19, +0.42, 1.11])
# Which face of the shelf is the opening. From the SDF: yaw was set so
# the visual mesh's local +y (front) face points toward world -X, i.e.
# the open face is at world x = SHELF_AABB_MIN[0].
OPEN_FACE_AXIS = 0    # x-axis
OPEN_FACE_SIDE = "-"  # min-x side is open


def build_material_grid(res, model, panel_thickness, n_internal):
    """Return a 3D bool array `is_material[i,j,k]` sampled at `res`,
    plus grid_shape and origin. True = shelf panel/wall.
    """
    size = SHELF_AABB_MAX - SHELF_AABB_MIN
    shape = tuple(int(round(s / res)) for s in size)
    is_material = np.zeros(shape, dtype=bool)
    # Cell-center coordinates along each axis.
    xs = SHELF_AABB_MIN[0] + (np.arange(shape[0]) + 0.5) * res
    ys = SHELF_AABB_MIN[1] + (np.arange(shape[1]) + 0.5) * res
    zs = SHELF_AABB_MIN[2] + (np.arange(shape[2]) + 0.5) * res

    if model == "solid":
        is_material[:, :, :] = True
        return is_material, shape

    # panel thickness distances (per axis).
    tx_min = SHELF_AABB_MIN[0] + panel_thickness
    tx_max = SHELF_AABB_MAX[0] - panel_thickness
    ty_min = SHELF_AABB_MIN[1] + panel_thickness
    ty_max = SHELF_AABB_MAX[1] - panel_thickness
    tz_min = SHELF_AABB_MIN[2] + panel_thickness
    tz_max = SHELF_AABB_MAX[2] - panel_thickness

    # Outer walls: 5 of 6 faces (front open).
    # Broadcast masks.
    X = xs[:, None, None]
    Y = ys[None, :, None]
    Z = zs[None, None, :]

    is_material |= (Z < tz_min)                    # bottom
    is_material |= (Z > tz_max)                    # top
    is_material |= (Y < ty_min)                    # -Y side
    is_material |= (Y > ty_max)                    # +Y side
    is_material |= (X > tx_max)                    # +X back
    # -X face (front) intentionally open

    if model == "hollow_shelves" and n_internal > 0:
        # Evenly space N horizontal internal shelves between top and
        # bottom (excluding the bottom/top panels themselves).
        # z levels chosen at fractions (k+1)/(N+1) between tz_min and
        # tz_max, each of panel thickness.
        for k in range(n_internal):
            zc = tz_min + (k + 1) * (tz_max - tz_min) / (n_internal + 1)
            zlo = zc - panel_thickness * 0.5
            zhi = zc + panel_thickness * 0.5
            shelf_mask = (Z >= zlo) & (Z <= zhi)
            # Only material where within the outer walls' interior.
            interior_x = (X >= tx_min) & (X <= tx_max)
            interior_y = (Y >= ty_min) & (Y <= ty_max)
            is_material |= (shelf_mask & interior_x & interior_y)

    return is_material, shape


def sample_cameras(n, base, reach_min, reach_max, z_min, z_max, seed):
    """Uniform random points in the arm's reachable shell, restricted to
    the +X hemisphere (outward-facing toward the shelf).
    """
    rng = np.random.default_rng(seed)
    cams = np.empty((n, 3), dtype=np.float64)
    got = 0
    tries = 0
    max_tries = n * 50
    while got < n and tries < max_tries:
        # Sample a batch to fill remaining.
        need = n - got
        # Cube [-reach_max, reach_max]^3 rejection sample.
        p = rng.uniform(-reach_max, reach_max, size=(need * 4, 3))
        rel = p - base
        r = np.linalg.norm(rel, axis=1)
        ok = (
            (r >= reach_min)
            & (r <= reach_max)
            & (p[:, 2] >= z_min)
            & (p[:, 2] <= z_max)
            & (rel[:, 0] > 0.0)         # +X hemisphere (outward-look)
        )
        picks = p[ok]
        take = min(len(picks), need)
        cams[got:got + take] = picks[:take]
        got += take
        tries += need * 4
    if got < n:
        cams = cams[:got]
    return cams


def voxel_centers(shape, res, origin):
    ii, jj, kk = np.meshgrid(
        np.arange(shape[0]), np.arange(shape[1]), np.arange(shape[2]),
        indexing="ij",
    )
    centers = np.stack([
        origin[0] + (ii + 0.5) * res,
        origin[1] + (jj + 0.5) * res,
        origin[2] + (kk + 0.5) * res,
    ], axis=-1)
    return centers.reshape(-1, 3)


def visibility_from_camera(cam, voxel_centers_flat, material_grid, res,
                           origin, target_idx):
    """Return (V,) bool: for each voxel, True iff the ray cam -> voxel
    is not occluded by any material voxel BEFORE reaching the target's
    own cell. Samples that fall inside the target cell (including near
    t=1) are ignored — the target voxel itself is what we are trying
    to observe, and a depth camera CAN see its first material hit.

    Ray sampled at 2 samples per voxel edge (Nyquist for the material
    grid).
    """
    V = voxel_centers_flat.shape[0]
    directions = voxel_centers_flat - cam           # (V, 3)
    dist = np.linalg.norm(directions, axis=1)       # (V,)
    max_dist = float(dist.max())
    n_samples = max(2, int(math.ceil(max_dist / (res * 0.5))))
    t = np.linspace(0.0, 1.0, n_samples + 2)[1:-1]  # skip both endpoints

    ni, nj, nk = material_grid.shape
    blocked_any = np.zeros(V, dtype=bool)
    for tt in t:
        pos = cam[None, :] + tt * directions        # (V, 3)
        idx = ((pos - origin[None, :]) / res).astype(np.int32)
        in_bounds = (
            (idx[:, 0] >= 0) & (idx[:, 0] < ni) &
            (idx[:, 1] >= 0) & (idx[:, 1] < nj) &
            (idx[:, 2] >= 0) & (idx[:, 2] < nk)
        )
        # Skip samples inside the target cell — those are the endpoint
        # region and MUST NOT count as blocking (a material target voxel
        # would otherwise always occlude itself).
        not_target = (
            (idx[:, 0] != target_idx[:, 0])
            | (idx[:, 1] != target_idx[:, 1])
            | (idx[:, 2] != target_idx[:, 2])
        )
        active = in_bounds & not_target & ~blocked_any
        if not np.any(active):
            continue
        act_where = np.where(active)[0]
        idx_a = idx[act_where]
        hit = material_grid[idx_a[:, 0], idx_a[:, 1], idx_a[:, 2]]
        blocked_any[act_where[hit]] = True
    return ~blocked_any


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--res", type=float, default=0.02,
                   help="Voxel edge length in meters (default 0.02). "
                        "Actual test uses 0.01 but 0.02 is much faster.")
    p.add_argument("--model", choices=("solid", "hollow", "hollow_shelves"),
                   default="hollow_shelves",
                   help="Shelf geometry model (default hollow_shelves).")
    p.add_argument("--panel-thickness", type=float, default=0.02,
                   help="Thickness of walls and internal shelves in m "
                        "(default 0.02).")
    p.add_argument("--n-internal", type=int, default=3,
                   help="Number of horizontal internal shelves for the "
                        "hollow_shelves model (default 3).")
    p.add_argument("--n-cameras", type=int, default=200,
                   help="Number of sampled camera positions (default 200).")
    p.add_argument("--reach-min", type=float, default=0.60)
    p.add_argument("--reach-max", type=float, default=0.85)
    p.add_argument("--z-min", type=float, default=0.05)
    p.add_argument("--z-max", type=float, default=1.20)
    p.add_argument("--base", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    print(f"Shelf AABB: x=[{SHELF_AABB_MIN[0]:.3f},{SHELF_AABB_MAX[0]:.3f}] "
          f"y=[{SHELF_AABB_MIN[1]:.3f},{SHELF_AABB_MAX[1]:.3f}] "
          f"z=[{SHELF_AABB_MIN[2]:.3f},{SHELF_AABB_MAX[2]:.3f}] "
          f"vol={np.prod(SHELF_AABB_MAX - SHELF_AABB_MIN):.4f} m^3")
    print(f"Model: {args.model}  panel_thickness={args.panel_thickness}  "
          f"n_internal={args.n_internal if args.model == 'hollow_shelves' else 0}")
    print(f"Grid res={args.res}m")

    t0 = time.perf_counter()
    material, shape = build_material_grid(
        args.res, args.model, args.panel_thickness, args.n_internal
    )
    n_total = material.size
    n_material = int(material.sum())
    n_air = n_total - n_material
    print(f"Grid: {shape}  total_voxels={n_total}  "
          f"material={n_material} ({100 * n_material / n_total:.2f}%)  "
          f"air={n_air} ({100 * n_air / n_total:.2f}%)")

    base = np.array(args.base, dtype=np.float64)
    cams = sample_cameras(
        args.n_cameras, base,
        args.reach_min, args.reach_max, args.z_min, args.z_max,
        args.seed,
    )
    print(f"Sampled {len(cams)}/{args.n_cameras} cameras in shell "
          f"r=[{args.reach_min:.2f},{args.reach_max:.2f}], "
          f"z=[{args.z_min:.2f},{args.z_max:.2f}], "
          f"+X hemisphere. build_material_grid took {time.perf_counter()-t0:.2f}s")
    if len(cams) == 0:
        print("ERROR: no camera positions sampled; check reach/z bounds.")
        sys.exit(1)

    centers = voxel_centers(shape, args.res, SHELF_AABB_MIN)  # (V, 3)
    V = centers.shape[0]
    # Precompute each voxel's own grid index for the "skip target cell" test.
    target_idx = ((centers - SHELF_AABB_MIN[None, :]) / args.res).astype(np.int32)

    t1 = time.perf_counter()
    visible = np.zeros(V, dtype=bool)
    report_every = max(1, len(cams) // 10)
    for i, cam in enumerate(cams):
        vis_i = visibility_from_camera(cam, centers, material, args.res,
                                       SHELF_AABB_MIN, target_idx)
        visible |= vis_i
        if (i + 1) % report_every == 0 or i + 1 == len(cams):
            n_vis = int(visible.sum())
            print(f"  camera {i+1:4d}/{len(cams)}: cumulative_visible="
                  f"{100*n_vis/V:.2f}%  ({time.perf_counter()-t1:.1f}s)")

    material_flat = material.ravel()
    vis_material = int((visible & material_flat).sum())
    vis_air      = int((visible & ~material_flat).sum())
    n_visible    = vis_material + vis_air
    n_unknown    = V - n_visible

    coverage = 100.0 * n_visible / V
    occ_pct  = 100.0 * vis_material / V
    free_pct = 100.0 * vis_air / V
    unk_pct  = 100.0 * n_unknown / V
    ku = (n_visible / n_unknown) if n_unknown > 0 else float("inf")

    print()
    print("=================================================================")
    print(" THEORETICAL COVERAGE UPPER BOUND (shelf AABB, arm-workspace)")
    print("=================================================================")
    print(f"  coverage  = {coverage:6.2f}%   ({n_visible}/{V} voxels)")
    print(f"  occ (mat) = {occ_pct:6.2f}%   ({vis_material}/{V})")
    print(f"  free (air)= {free_pct:6.2f}%   ({vis_air}/{V})")
    print(f"  unknown   = {unk_pct:6.2f}%   ({n_unknown}/{V})")
    print(f"  known/unknown = {ku:.3f}")
    print("=================================================================")
    print("Format matches shelf_coverage_monitor's FINAL line for direct")
    print("comparison against actual exploration results.")


if __name__ == "__main__":
    main()
