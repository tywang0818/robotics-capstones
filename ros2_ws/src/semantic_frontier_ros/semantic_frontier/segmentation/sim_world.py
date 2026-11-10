"""Procedural Gazebo worlds for v2 simulation semantic data collection.

The generator deliberately starts with simple geometric proxy assets instead of
external model downloads. That keeps labels, scale, collisions, and placement
rules deterministic while the data pipeline is still being validated.
"""

from __future__ import annotations

import json
import math
import random
import xml.sax.saxutils as xml_utils
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

from .labels import CLASS_NAMES, TASK_SOURCE_OTHER_LABEL


@dataclass(frozen=True)
class CategoryAsset:
    label_id: int
    name: str
    kind: str
    size: tuple[float, float, float]
    color: tuple[float, float, float]
    is_support: bool = False
    place_on_support: bool = False


@dataclass(frozen=True)
class SupportSurface:
    id: str
    parent_object_id: str
    center: tuple[float, float, float]
    size: tuple[float, float]
    yaw: float


@dataclass(frozen=True)
class ObjectInstance:
    id: str
    category: str
    label_id: int
    kind: str
    pose: tuple[float, float, float, float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float]
    support_parent: str | None = None


@dataclass(frozen=True)
class Viewpoint:
    id: str
    pose: tuple[float, float, float, float, float, float]
    target: tuple[float, float, float]


@dataclass(frozen=True)
class SceneSpec:
    scene_id: str
    seed: int
    scene_type: str
    categories: tuple[str, ...]
    objects: tuple[ObjectInstance, ...]
    support_surfaces: tuple[SupportSurface, ...]
    viewpoints: tuple[Viewpoint, ...]
    placement_issues: tuple[str, ...]


@dataclass(frozen=True)
class GeneratorConfig:
    dataset_id: str
    seed: int = 42
    scene_count: int = 3
    viewpoints_per_scene: int = 10
    min_categories_per_scene: int = 2
    max_categories_per_scene: int = 4
    other_clutter_min: int = 0
    other_clutter_max: int = 4
    scene_types: tuple[str, ...] = (
        "bookshelf_clutter",
        "tabletop_clutter",
        "cabinet_shelf_storage",
        "mixed_indoor_manipulation_workspace",
    )


TASK_CATEGORY_NAMES = tuple(CLASS_NAMES[1:])
SUPPORT_CATEGORY_NAMES = {
    "cabinet",
    "table",
    "shelf",
    "desk",
    "wardrobe",
    "chest_of_drawers",
    "counter",
    "bookcase",
    "coffee_table",
    "countertop",
}
PLACE_ON_SUPPORT_NAMES = {"box", "basket", "bag"}


CATEGORY_ASSETS: dict[str, CategoryAsset] = {
    "cabinet": CategoryAsset(1, "cabinet", "cabinet", (0.9, 0.45, 1.05), (0.62, 0.55, 0.45), True),
    "table": CategoryAsset(2, "table", "table", (1.2, 0.8, 0.72), (0.55, 0.35, 0.18), True),
    "shelf": CategoryAsset(3, "shelf", "shelf", (0.9, 0.35, 1.1), (0.50, 0.38, 0.22), True),
    "desk": CategoryAsset(4, "desk", "desk", (1.25, 0.7, 0.75), (0.48, 0.34, 0.24), True),
    "wardrobe": CategoryAsset(5, "wardrobe", "cabinet", (0.85, 0.55, 1.55), (0.48, 0.43, 0.36), True),
    "box": CategoryAsset(6, "box", "box", (0.28, 0.24, 0.25), (0.08, 0.22, 0.85), False, True),
    "chest_of_drawers": CategoryAsset(7, "chest_of_drawers", "drawers", (0.85, 0.45, 0.8), (0.58, 0.45, 0.34), True),
    "counter": CategoryAsset(8, "counter", "counter", (1.35, 0.55, 0.9), (0.62, 0.62, 0.55), True),
    "bookcase": CategoryAsset(9, "bookcase", "shelf", (0.9, 0.35, 1.35), (0.45, 0.31, 0.18), True),
    "coffee_table": CategoryAsset(10, "coffee_table", "table", (0.9, 0.55, 0.42), (0.50, 0.29, 0.16), True),
    "countertop": CategoryAsset(11, "countertop", "slab", (1.2, 0.55, 0.12), (0.75, 0.72, 0.66), True),
    "basket": CategoryAsset(12, "basket", "basket", (0.32, 0.28, 0.24), (0.78, 0.58, 0.25), False, True),
    "bag": CategoryAsset(13, "bag", "bag", (0.32, 0.24, 0.28), (0.10, 0.65, 0.25), False, True),
    "other": CategoryAsset(TASK_SOURCE_OTHER_LABEL, "other", "box", (0.16, 0.16, 0.14), (0.45, 0.45, 0.48), False, True),
}

SCENE_TYPE_PRIORS: dict[str, tuple[str, ...]] = {
    "bookshelf_clutter": ("bookcase", "shelf", "box", "basket", "bag", "cabinet"),
    "tabletop_clutter": ("table", "coffee_table", "box", "basket", "bag", "countertop"),
    "cabinet_shelf_storage": ("cabinet", "shelf", "wardrobe", "chest_of_drawers", "box", "basket", "bag"),
    "mixed_indoor_manipulation_workspace": tuple(TASK_CATEGORY_NAMES),
}


def validate_generator_config(config: GeneratorConfig) -> None:
    if config.scene_count < 1:
        raise ValueError("scene_count must be positive")
    if config.viewpoints_per_scene < 1:
        raise ValueError("viewpoints_per_scene must be positive")
    if config.min_categories_per_scene < 1:
        raise ValueError("min_categories_per_scene must be positive")
    if config.max_categories_per_scene < config.min_categories_per_scene:
        raise ValueError("max_categories_per_scene must be >= min_categories_per_scene")
    if config.max_categories_per_scene > len(TASK_CATEGORY_NAMES):
        raise ValueError("max_categories_per_scene cannot exceed task class count")
    if config.other_clutter_max < config.other_clutter_min:
        raise ValueError("other_clutter_max must be >= other_clutter_min")


def category_catalog() -> dict[str, dict[str, object]]:
    return {name: asdict(asset) for name, asset in CATEGORY_ASSETS.items()}


def _category_schedule(config: GeneratorConfig) -> list[tuple[str, ...]]:
    rng = random.Random(config.seed)
    shuffled = list(TASK_CATEGORY_NAMES)
    rng.shuffle(shuffled)
    cursor = 0
    schedule: list[tuple[str, ...]] = []
    for scene_index in range(config.scene_count):
        scene_type = config.scene_types[scene_index % len(config.scene_types)]
        candidates = list(SCENE_TYPE_PRIORS.get(scene_type, TASK_CATEGORY_NAMES))
        count = rng.randint(config.min_categories_per_scene, config.max_categories_per_scene)
        forced = [shuffled[(cursor + offset) % len(shuffled)] for offset in range(min(count, len(shuffled)))]
        cursor += count
        selected: list[str] = []
        for name in forced + candidates:
            if name not in selected and name in CATEGORY_ASSETS and name != "other":
                selected.append(name)
            if len(selected) >= count:
                break
        if not any(name in SUPPORT_CATEGORY_NAMES for name in selected):
            selected[0] = rng.choice(tuple(SUPPORT_CATEGORY_NAMES))
        schedule.append(tuple(selected))
    return schedule


def generate_dataset_specs(config: GeneratorConfig) -> list[SceneSpec]:
    validate_generator_config(config)
    schedule = _category_schedule(config)
    scenes = []
    for scene_index, categories in enumerate(schedule):
        scene_type = config.scene_types[scene_index % len(config.scene_types)]
        scene_seed = config.seed * 1000 + scene_index
        scene_id = f"scene_{scene_index:04d}"
        scenes.append(generate_scene_spec(config, scene_id, scene_seed, scene_type, categories))
    return scenes


def generate_scene_spec(
    config: GeneratorConfig,
    scene_id: str,
    scene_seed: int,
    scene_type: str,
    categories: Iterable[str],
) -> SceneSpec:
    rng = random.Random(scene_seed)
    selected = tuple(categories)
    objects: list[ObjectInstance] = []
    surfaces: list[SupportSurface] = []
    issues: list[str] = []

    support_names = [name for name in selected if name in SUPPORT_CATEGORY_NAMES]
    if not support_names:
        support_names = [rng.choice(tuple(SUPPORT_CATEGORY_NAMES))]
    floor_slots = [
        (-0.95, -0.55),
        (0.95, -0.55),
        (-0.95, 0.55),
        (0.95, 0.55),
        (0.0, 0.0),
    ]
    for support_index, name in enumerate(support_names):
        slot_x, slot_y = floor_slots[support_index % len(floor_slots)]
        x = slot_x + rng.uniform(-0.08, 0.08)
        y = slot_y + rng.uniform(-0.08, 0.08)
        obj = _make_floor_object(name, f"{scene_id}_{name}_{support_index}", x, y, rng.uniform(-0.25, 0.25))
        objects.append(obj)
        surfaces.extend(_support_surfaces_for(obj))

    placeable_names = [name for name in selected if name in PLACE_ON_SUPPORT_NAMES]
    if not placeable_names:
        placeable_names = ["box"]
    for index, name in enumerate(placeable_names):
        surface = surfaces[index % len(surfaces)]
        objects.append(_make_supported_object(name, f"{scene_id}_{name}_{index}", surface, rng))

    clutter_count = rng.randint(config.other_clutter_min, config.other_clutter_max)
    for index in range(clutter_count):
        surface = surfaces[(index + len(placeable_names)) % len(surfaces)]
        objects.append(_make_supported_object("other", f"{scene_id}_other_{index}", surface, rng))

    issues.extend(validate_placements(objects, surfaces))
    viewpoints = tuple(_generate_viewpoints(scene_id, config.viewpoints_per_scene, rng))
    return SceneSpec(
        scene_id=scene_id,
        seed=scene_seed,
        scene_type=scene_type,
        categories=selected,
        objects=tuple(objects),
        support_surfaces=tuple(surfaces),
        viewpoints=viewpoints,
        placement_issues=tuple(issues),
    )


def _make_floor_object(name: str, object_id: str, x: float, y: float, yaw: float) -> ObjectInstance:
    asset = CATEGORY_ASSETS[name]
    z = asset.size[2] / 2.0
    return ObjectInstance(
        id=object_id,
        category=name,
        label_id=asset.label_id,
        kind=asset.kind,
        pose=(x, y, z, 0.0, 0.0, yaw),
        size=asset.size,
        color=asset.color,
    )


def _support_surfaces_for(obj: ObjectInstance) -> list[SupportSurface]:
    x, y, z, _, _, yaw = obj.pose
    sx, sy, sz = obj.size
    top_z = z + sz / 2.0
    if obj.kind in {"shelf", "bookcase"}:
        return [
            SupportSurface(f"{obj.id}_shelf_{i}", obj.id, (x, y, top_z - 0.2 * i), (sx * 0.82, sy * 0.82), yaw)
            for i in range(3)
        ]
    return [SupportSurface(f"{obj.id}_top", obj.id, (x, y, top_z), (sx * 0.82, sy * 0.82), yaw)]


def _make_supported_object(name: str, object_id: str, surface: SupportSurface, rng: random.Random) -> ObjectInstance:
    asset = CATEGORY_ASSETS[name]
    sx, sy, sz = asset.size
    surf_x, surf_y, surf_z = surface.center
    avail_x = max(surface.size[0] / 2.0 - sx / 2.0 - 0.03, 0.0)
    avail_y = max(surface.size[1] / 2.0 - sy / 2.0 - 0.03, 0.0)
    local_x = rng.uniform(-avail_x, avail_x) if avail_x > 0 else 0.0
    local_y = rng.uniform(-avail_y, avail_y) if avail_y > 0 else 0.0
    cy, syaw = math.cos(surface.yaw), math.sin(surface.yaw)
    x = surf_x + cy * local_x - syaw * local_y
    y = surf_y + syaw * local_x + cy * local_y
    z = surf_z + sz / 2.0
    yaw = surface.yaw + rng.uniform(-0.45, 0.45)
    return ObjectInstance(object_id, name, asset.label_id, asset.kind, (x, y, z, 0.0, 0.0, yaw), asset.size, asset.color, surface.id)


def _aabb(obj: ObjectInstance) -> tuple[float, float, float, float, float, float]:
    x, y, z, _, _, _ = obj.pose
    sx, sy, sz = obj.size
    return (x - sx / 2.0, x + sx / 2.0, y - sy / 2.0, y + sy / 2.0, z - sz / 2.0, z + sz / 2.0)


def validate_placements(objects: Iterable[ObjectInstance], surfaces: Iterable[SupportSurface]) -> list[str]:
    surface_by_id = {surface.id: surface for surface in surfaces}
    object_list = list(objects)
    issues: list[str] = []
    for obj in object_list:
        bottom_z = obj.pose[2] - obj.size[2] / 2.0
        if obj.support_parent is None:
            if abs(bottom_z) > 1e-6:
                issues.append(f"{obj.id}: floor object bottom_z={bottom_z:.4f}")
        else:
            surface = surface_by_id.get(obj.support_parent)
            if surface is None:
                issues.append(f"{obj.id}: missing support {obj.support_parent}")
            elif abs(bottom_z - surface.center[2]) > 1e-6:
                issues.append(f"{obj.id}: floating bottom_z={bottom_z:.4f} support_z={surface.center[2]:.4f}")
    for left_index, left in enumerate(object_list):
        if left.support_parent is not None:
            continue
        lx0, lx1, ly0, ly1, lz0, lz1 = _aabb(left)
        for right in object_list[left_index + 1:]:
            if right.support_parent is not None:
                continue
            rx0, rx1, ry0, ry1, rz0, rz1 = _aabb(right)
            overlaps = lx0 < rx1 and rx0 < lx1 and ly0 < ry1 and ry0 < ly1 and lz0 < rz1 and rz0 < lz1
            if overlaps:
                issues.append(f"{left.id}/{right.id}: floor support overlap")
    return issues


def _generate_viewpoints(scene_id: str, count: int, rng: random.Random) -> list[Viewpoint]:
    viewpoints = []
    target = (0.0, 0.0, 0.65)
    for index in range(count):
        angle = 2 * math.pi * index / max(count, 1) + rng.uniform(-0.18, 0.18)
        radius = rng.uniform(1.35, 2.05)
        x = target[0] + math.cos(angle) * radius
        y = target[1] + math.sin(angle) * radius
        z = rng.uniform(0.85, 1.35)
        dx, dy = target[0] - x, target[1] - y
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(z - target[2], math.hypot(dx, dy))
        viewpoints.append(Viewpoint(f"{index:06d}", (x, y, z, 0.0, pitch, yaw), target))
    return viewpoints


def write_dataset_worlds(config: GeneratorConfig, raw_root: Path) -> list[SceneSpec]:
    scenes = generate_dataset_specs(config)
    scene_root = raw_root / "scenes"
    scene_root.mkdir(parents=True, exist_ok=True)
    for scene in scenes:
        directory = scene_root / scene.scene_id
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "scene_metadata.json").write_text(json.dumps(scene_to_json(scene), indent=2) + "\n", encoding="utf-8")
        write_jsonl(directory / "viewpoints.jsonl", (asdict(viewpoint) for viewpoint in scene.viewpoints))
        (directory / "world.sdf").write_text(render_world_sdf(scene, scene.viewpoints[0]), encoding="utf-8")
        view_world_root = directory / "view_worlds"
        view_world_root.mkdir(exist_ok=True)
        for viewpoint in scene.viewpoints:
            (view_world_root / f"{viewpoint.id}.sdf").write_text(render_world_sdf(scene, viewpoint), encoding="utf-8")
    write_jsonl(raw_root / "scene_manifest.jsonl", (scene_to_json(scene) for scene in scenes))
    return scenes


def scene_to_json(scene: SceneSpec) -> dict[str, object]:
    return {
        "scene_id": scene.scene_id,
        "seed": scene.seed,
        "scene_type": scene.scene_type,
        "categories": list(scene.categories),
        "objects": [asdict(obj) for obj in scene.objects],
        "support_surfaces": [asdict(surface) for surface in scene.support_surfaces],
        "viewpoints": [asdict(viewpoint) for viewpoint in scene.viewpoints],
        "placement_issues": list(scene.placement_issues),
    }


def write_jsonl(path: Path, rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def render_world_sdf(scene: SceneSpec, viewpoint: Viewpoint) -> str:
    models = "\n".join(_render_object_model(obj) for obj in scene.objects)
    camera_pose = _pose_text(viewpoint.pose)
    return f'''<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="{xml_utils.escape(scene.scene_id)}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <light name="key_light" type="directional">
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1</direction>
    </light>
{_ground_plane()}
{models}
{_camera_model(camera_pose)}
  </world>
</sdf>
'''


def _pose_text(pose: tuple[float, float, float, float, float, float]) -> str:
    return " ".join(f"{value:.6f}" for value in pose)


def _color_text(color: tuple[float, float, float]) -> str:
    return f"{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1"


def _ground_plane() -> str:
    return '''    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry><material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material></visual>
      </link>
    </model>'''


def _render_object_model(obj: ObjectInstance) -> str:
    name = xml_utils.escape(obj.id)
    pose = _pose_text(obj.pose)
    color = _color_text(obj.color)
    size = " ".join(f"{value:.4f}" for value in obj.size)
    geometry = _geometry_xml(obj.kind, obj.size)
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{obj.label_id}</label></plugin>
      <link name="link">
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual"><geometry>{geometry}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>'''


def _geometry_xml(kind: str, size: tuple[float, float, float]) -> str:
    sx, sy, sz = size
    if kind == "bag":
        radius = max(sx, sy, sz) / 2.0
        return f"<sphere><radius>{radius:.4f}</radius></sphere>"
    if kind == "basket":
        return f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"
    return f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"


def _camera_model(camera_pose: str) -> str:
    return f'''    <model name="sim_camera">
      <static>true</static>
      <pose>{camera_pose}</pose>
      <link name="camera_link">
        <sensor name="color" type="camera">
          <always_on>true</always_on><update_rate>5</update_rate><topic>/sim_camera/color/image_raw</topic>
          <camera><horizontal_fov>1.047</horizontal_fov><image><width>320</width><height>240</height><format>R8G8B8</format></image><clip><near>0.05</near><far>10.0</far></clip></camera>
        </sensor>
        <sensor name="depth" type="depth_camera">
          <always_on>true</always_on><update_rate>5</update_rate><topic>/sim_camera/depth/image_rect_raw</topic>
          <camera><horizontal_fov>1.047</horizontal_fov><image><width>320</width><height>240</height><format>R_FLOAT32</format></image><clip><near>0.05</near><far>10.0</far></clip></camera>
        </sensor>
        <sensor name="semantic" type="segmentation">
          <always_on>true</always_on><update_rate>5</update_rate><topic>/sim_camera/semantic/image_raw</topic><segmentation_type>semantic</segmentation_type>
          <camera><horizontal_fov>1.047</horizontal_fov><image><width>320</width><height>240</height></image><clip><near>0.05</near><far>10.0</far></clip></camera>
        </sensor>
      </link>
    </model>'''