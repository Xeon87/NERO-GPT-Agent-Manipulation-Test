#!/usr/bin/env python3
"""NERO V2-6 public-release unified MuJoCo/ROS2 manipulation backend.

The backend owns the deterministic side of the architecture: simulation state,
gripper control, MoveIt/BioIK/TRAC-IK motion skills, skill contracts, physical
postcondition checks, safety state, bounded recovery, and the Tk control panel.
The LLM is *not* trusted as a low-level controller; it only chooses validated
skills and arguments.

V2-6 is intentionally derived from the validated V2-5.2 stack.  The frozen V69
motion core remains separate so the public-release cleanup does not silently
change proven kinematics or trajectory behavior.
"""
import sys
import os
import time
import math
import traceback
import threading
import json
import multiprocessing as mp
import tempfile
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import mujoco

# ------------------------------------------------------------
# NERO V2-6 Public Release — multi-object, failure-aware manipulation on frozen V69 motion stack
# - Tkinter GUI: avoids Qt/PySide6 event-loop issues
# - multiprocessing "spawn": clean isolation from MuJoCo viewer
# - verified joint-name based mapping: joint1 ... joint7
# - per-joint speed / Wn / Zeta / accel limit
# - gravity ON/OFF, default OFF
# ------------------------------------------------------------

MODEL_FILE = str(Path(__file__).resolve().with_name("nero_motor.xml"))

JOINT_LIMITS_RAD = [
    (-2.70526,  2.70526),
    (-1.74,     1.74),
    (-2.75,     2.75),
    (-1.01,     2.14),
    (-2.75,     2.75),
    (-0.73,     0.95),
    (-1.5708,   1.5708),
]

# V2 reference speeds: defined as 20% in the new GUI.
V2_MAX_REF_VEL = np.array(
    [0.35, 0.35, 0.35, 0.35, 0.40, 0.40, 0.40],
    dtype=float,
)
FULL_SPEED_REF_VEL = V2_MAX_REF_VEL / 0.20

DEFAULT_WN = np.array([4.0] * 7, dtype=float)
DEFAULT_ZETA = np.array([1.25] * 7, dtype=float)
DEFAULT_MAX_QACC = np.array(
    [5.0, 5.0, 5.0, 5.0, 6.0, 6.0, 6.0],
    dtype=float,
)
DEFAULT_SPEED_PERCENT = np.array([20.0] * 7, dtype=float)


def shared_array(shared):
    return np.array(shared[:], dtype=float)




def _find_official_gripper_assets():
    """
    Locate AgileX official NERO gripper STL files.
    Search common local locations only; no download/network is required at runtime.
    """
    here = Path(__file__).resolve().parent
    home = Path.home()

    candidate_dirs = [
        here / "assets",
        here / "agilex_nero" / "assets",
        home / "mujoco_ws" / "nero" / "assets",
        home / "agx_arm_sim" / "mujoco" / "agilex_arm" / "agilex_nero" / "assets",
        home / "agx_arm_ws" / "src" / "agx_arm_sim" / "mujoco" / "agilex_arm" / "agilex_nero" / "assets",
    ]

    required = [
        "gripper_flange.stl",
        "gripper_base.stl",
        "gripper_link1.stl",
        "gripper_link2.stl",
    ]

    for d in candidate_dirs:
        if all((d / name).exists() for name in required):
            return d

    # Fallback: bounded recursive searches in likely workspaces.
    roots = [
        home / "mujoco_ws",
        home / "agx_arm_ws",
        home / "agx_arm_sim",
    ]
    for root in roots:
        if not root.exists():
            continue
        for flange in root.rglob("gripper_flange.stl"):
            d = flange.parent
            if all((d / name).exists() for name in required):
                return d

    raise FileNotFoundError(
        "\nAgileX official NERO gripper mesh files were not found.\n"
        "Need these four files in one assets directory:\n"
        "  gripper_flange.stl\n"
        "  gripper_base.stl\n"
        "  gripper_link1.stl\n"
        "  gripper_link2.stl\n"
        "\nThey are in the official agx_arm_sim repository under:\n"
        "mujoco/agilex_arm/agilex_nero/assets/\n"
    )


def add_official_gripper_to_xml(xml_path):
    """
    Add the AgileX official NERO two-finger gripper to an existing arm-only MJCF.

    Geometry / transforms / joint ranges are taken from AgileX's official
    mujoco/agilex_arm/agilex_nero/nero.xml model.

    If the XML already contains gripper_joint1, nothing is changed.
    """
    xml_path = Path(xml_path).resolve()
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Already present -> leave it alone.
    if root.find(".//joint[@name='gripper_joint1']") is not None:
        return str(xml_path)

    assets_dir = _find_official_gripper_assets()

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        worldbody = root.find("worldbody")
        insert_idx = list(root).index(worldbody) if worldbody is not None else 0
        root.insert(insert_idx, asset)

    existing_meshes = {
        m.get("name")
        for m in asset.findall("mesh")
        if m.get("name")
    }

    mesh_files = {
        "gripper_flange": "gripper_flange.stl",
        "gripper_base": "gripper_base.stl",
        "gripper_link1": "gripper_link1.stl",
        "gripper_link2": "gripper_link2.stl",
    }

    for mesh_name, file_name in mesh_files.items():
        if mesh_name not in existing_meshes:
            ET.SubElement(
                asset,
                "mesh",
                {
                    "name": mesh_name,
                    "file": str((assets_dir / file_name).resolve()),
                },
            )

    link7 = root.find(".//body[@name='link7']")
    if link7 is None:
        raise RuntimeError(
            "Could not find <body name='link7'> in nero_motor.xml; "
            "cannot attach the original gripper."
        )

    # Official flange visual + collision
    ET.SubElement(
        link7, "geom",
        {
            "name": "gripper_flange_visual",
            "pos": "0.032 0 -0.0235",
            "quat": "0.499998 -0.5 0.500002 -0.5",
            "type": "mesh",
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
            "density": "0",
            "mesh": "gripper_flange",
        },
    )
    ET.SubElement(
        link7, "geom",
        {
            "name": "gripper_flange_collision",
            "pos": "0.032 0 -0.0235",
            "quat": "0.499998 -0.5 0.500002 -0.5",
            "type": "mesh",
            "mesh": "gripper_flange",
            "friction": "1.2 0.02 0.001",
            "solref": "0.008 1",
            "solimp": "0.95 0.99 0.001",
            "margin": "0.001",
        },
    )

    # Official gripper base visual + collision
    ET.SubElement(
        link7, "geom",
        {
            "name": "gripper_base_visual",
            "pos": "0.0375 -2.02026e-08 -0.0235",
            "quat": "0.499998 -0.5 0.500002 -0.5",
            "type": "mesh",
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
            "density": "0",
            "mesh": "gripper_base",
        },
    )
    ET.SubElement(
        link7, "geom",
        {
            "name": "gripper_base_collision",
            "pos": "0.0375 -2.02026e-08 -0.0235",
            "quat": "0.499998 -0.5 0.500002 -0.5",
            "type": "mesh",
            "mesh": "gripper_base",
            "friction": "1.2 0.02 0.001",
            "solref": "0.008 1",
            "solimp": "0.95 0.99 0.001",
            "margin": "0.001",
        },
    )

    # Finger 1 - official transform.
    f1 = ET.SubElement(
        link7, "body",
        {
            "name": "gripper_link1",
            "pos": "0.1733 -5.19024e-07 -0.0235005",
            "quat": "-1.27026e-06 0.707108 0.707105 -1.30815e-06",
        },
    )
    ET.SubElement(
        f1, "inertial",
        {
            "pos": "0.000651232 -0.049193 0.00972259",
            "quat": "0.477798 0.572974 -0.518028 0.418398",
            "mass": "0.025",
            "diaginertia": "7.74531e-05 7.36783e-05 5.0886e-06",
        },
    )
    ET.SubElement(
        f1, "joint",
        {
            "name": "gripper_joint1",
            "pos": "0 0 0",
            "axis": "0 0 1",
            "type": "slide",
            "range": "0 0.05",
            "limited": "true",
        },
    )
    ET.SubElement(
        f1, "geom",
        {
            "type": "mesh",
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
            "density": "0",
            "mesh": "gripper_link1",
        },
    )
    # The original finger mesh is visual only for grasping in V15.
    # Actual object contact is handled by the visible rubber pad below.
    ET.SubElement(
        f1, "geom",
        {
            "name": "gripper_finger1_collision_disabled",
            "type": "mesh",
            "mesh": "gripper_link1",
            "contype": "0",
            "conaffinity": "0",
            "density": "0",
            "group": "3",
            "rgba": "0 0 0 0",
        },
    )

    # --------------------------------------------------------------
    # V19 ATTACHED RUBBER PAD - FINGER 1
    #
    # Keep the pad on the original finger region.  Only the front edge
    # extends toward +Y.  This avoids the detached/floating V18 pad.
    #
    # Local-Y span: about -48 mm .. +12 mm
    # Full pad length: 60 mm
    # --------------------------------------------------------------
    ET.SubElement(
        f1, "geom",
        {
            "name": "finger1_rubber_pad",
            "type": "box",
            "pos": "0 -0.018 -0.0020",
            "size": "0.0145 0.030 0.0020",
            "rgba": "0.90 0.08 0.03 1",
            "density": "0",
            "friction": "10.0 0.30 0.05",
            "condim": "6",
            "solref": "0.003 1",
            "solimp": "0.97 0.997 0.0003",
            "margin": "0.00025",
        },
    )

    # 5 x 8 = 40 raised studs per finger.
    stud_x = (-0.0115, -0.0058, 0.0, 0.0058, 0.0115)
    stud_y = (-0.043, -0.035, -0.027, -0.019,
              -0.011, -0.003, 0.005, 0.010)
    stud_index = 0
    for sx in stud_x:
        for sy in stud_y:
            stud_index += 1
            ET.SubElement(
                f1, "geom",
                {
                    "name": f"finger1_stud_{stud_index:02d}",
                    "type": "sphere",
                    "pos": f"{sx:.4f} {sy:.4f} -0.0052",
                    "size": "0.0017",
                    "rgba": "1.00 0.65 0.00 1",
                    "density": "0",
                    "friction": "10.0 0.30 0.05",
                    "condim": "6",
                    "solref": "0.003 1",
                    "solimp": "0.97 0.997 0.0003",
                    "margin": "0.00020",
                },
            )

    # Finger 2 - official transform.
    f2 = ET.SubElement(
        link7, "body",
        {
            "name": "gripper_link2",
            "pos": "0.1733 -5.19024e-07 -0.0235005",
            "quat": "0.707105 -1.30815e-06 1.30815e-06 -0.707108",
        },
    )
    ET.SubElement(
        f2, "inertial",
        {
            "pos": "0.000651232 -0.049193 0.00972259",
            "quat": "0.477798 0.572974 -0.518028 0.418398",
            "mass": "0.025",
            "diaginertia": "7.74531e-05 7.36783e-05 5.0886e-06",
        },
    )
    ET.SubElement(
        f2, "joint",
        {
            "name": "gripper_joint2",
            "pos": "0 0 0",
            "axis": "0 0 -1",
            "type": "slide",
            "range": "-0.05 0",
            "limited": "true",
        },
    )
    ET.SubElement(
        f2, "geom",
        {
            "type": "mesh",
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
            "density": "0",
            "mesh": "gripper_link2",
        },
    )
    # The original finger mesh is visual only for grasping in V15.
    ET.SubElement(
        f2, "geom",
        {
            "name": "gripper_finger2_collision_disabled",
            "type": "mesh",
            "mesh": "gripper_link2",
            "contype": "0",
            "conaffinity": "0",
            "density": "0",
            "group": "3",
            "rgba": "0 0 0 0",
        },
    )

    # --------------------------------------------------------------
    # V19 ATTACHED RUBBER PAD - FINGER 2
    # --------------------------------------------------------------
    ET.SubElement(
        f2, "geom",
        {
            "name": "finger2_rubber_pad",
            "type": "box",
            "pos": "0 -0.018 -0.0020",
            "size": "0.0145 0.030 0.0020",
            "rgba": "0.90 0.08 0.03 1",
            "density": "0",
            "friction": "10.0 0.30 0.05",
            "condim": "6",
            "solref": "0.003 1",
            "solimp": "0.97 0.997 0.0003",
            "margin": "0.00025",
        },
    )

    stud_index = 0
    for sx in stud_x:
        for sy in stud_y:
            stud_index += 1
            ET.SubElement(
                f2, "geom",
                {
                    "name": f"finger2_stud_{stud_index:02d}",
                    "type": "sphere",
                    "pos": f"{sx:.4f} {sy:.4f} -0.0052",
                    "size": "0.0017",
                    "rgba": "1.00 0.65 0.00 1",
                    "density": "0",
                    "friction": "10.0 0.30 0.05",
                    "condim": "6",
                    "solref": "0.003 1",
                    "solimp": "0.97 0.997 0.0003",
                    "margin": "0.00020",
                },
            )

    # Exclude direct finger-to-finger self-contact.
    # The pads can close through their nominal zero-gap pose without
    # generating a huge artificial internal collision impulse.
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")

    ET.SubElement(
        contact,
        "exclude",
        {
            "name": "gripper_finger_self_exclude",
            "body1": "gripper_link1",
            "body2": "gripper_link2",
        },
    )

    # Official coupling: q(gripper_joint2) = -q(gripper_joint1)
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")

    ET.SubElement(
        equality,
        "joint",
        {
            "name": "gripper_coupling",
            "joint1": "gripper_joint2",
            "joint2": "gripper_joint1",
            "polycoef": "0 -1 0 0 0",
        },
    )

    # Official AgileX gripper servo:
    # <position name="gripper" joint="gripper_joint1"
    #           kp="40" kv="5"; V12 simulation force limit = +/-20 N
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")

    if root.find(".//actuator/*[@name='gripper']") is None:
        ET.SubElement(
            actuator,
            "position",
            {
                "name": "gripper",
                "joint": "gripper_joint1",
                "kp": "40",
                "kv": "5",
                "forcerange": "-20 20",
                "forcelimited": "true",
                "ctrlrange": "0 0.05",
                "ctrllimited": "true",
            },
        )

    # V16 direct-force actuator.
    # q(gripper_joint1) increases when opening, so NEGATIVE force closes.
    # This actuator is separate from the position actuator above.
    if root.find(".//actuator/*[@name='gripper_force']") is None:
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "gripper_force",
                "joint": "gripper_joint1",
                "gear": "1",
                "ctrlrange": "-50 50",
                "ctrllimited": "true",
                "forcerange": "-50 50",
                "forcelimited": "true",
            },
        )

    out = xml_path.with_name("_nero_motor_v10_stable_arm_safe_gripper_tmp.xml")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return str(out)



def make_grid_world_xml(original_xml_path):
    """
    Create a temporary MJCF file in the SAME directory as the original model.
    This preserves all relative mesh paths while adding:
      - a bright sky gradient
      - a checker/grid floor plane
      - a soft overhead light

    The original nero_motor.xml is not modified.
    """
    src_path = Path(original_xml_path).resolve()
    xml = src_path.read_text(encoding="utf-8")

    # ---- Add / extend <asset> ------------------------------------------------
    asset_block = """
    <!-- V5 visual environment -->
    <texture name="v5_sky"
             type="skybox"
             builtin="gradient"
             rgb1="0.72 0.82 0.95"
             rgb2="0.95 0.97 1.00"
             width="512"
             height="3072"/>

    <texture name="v5_grid_tex"
             type="2d"
             builtin="checker"
             mark="edge"
             rgb1="0.86 0.86 0.86"
             rgb2="0.98 0.98 0.98"
             markrgb="0.55 0.55 0.55"
             width="1024"
             height="1024"/>

    <material name="v5_grid_mat"
              texture="v5_grid_tex"
              texrepeat="12 12"
              texuniform="true"
              reflectance="0.08"
              rgba="1 1 1 1"/>
"""

    if "<asset" in xml:
        xml = re.sub(
            r"(<asset\b[^>]*>)",
            r"\1\n" + asset_block,
            xml,
            count=1,
        )
    else:
        # Insert asset section before worldbody
        xml = xml.replace(
            "<worldbody>",
            "<asset>\n" + asset_block + "\n</asset>\n\n<worldbody>",
            1,
        )

    # ---- Add floor + light to worldbody --------------------------------------
    world_block = """
    <!-- ============================================================
         V53 SORTING WORKCELL

         Coordinate convention:
           robot mounting/base plane = z = 0.000 m
           tabletop top surface      = z = 0.000 m

         The arm is intended to represent a real NERO clamped at the
         near table edge. There is no raised corner pedestal in V52.
         ============================================================ -->

    <!-- Visual ground below the table; not the robot mounting plane. -->
    <geom name="v5_grid_floor"
          type="plane"
          pos="0 0 -0.28"
          size="6 6 0.1"
          material="v5_grid_mat"
          contype="1"
          conaffinity="1"
          friction="1.2 0.01 0.001"/>

    <!-- Single-level manipulation table. Top surface exactly z=0. -->
    <geom name="play_table"
          type="box"
          pos="-0.385 0 -0.03"
          size="0.335 0.38 0.03"
          rgba="0.62 0.64 0.68 1"
          friction="1.2 0.01 0.001"
          solref="0.008 1"
          solimp="0.96 0.995 0.0005"
          margin="0.001"
          contype="1"
          conaffinity="1"/>

    <!-- Simple visual table-edge clamp / support for the NERO base.
         Visual only (no collision) so it cannot disturb MoveIt or MuJoCo
         contact planning. V69 extends the tabletop toward the fixed NERO base:
         near edge x=-0.05, robot mounting origin near x=0. -->
    <geom name="nero_support_top"
          type="box"
          pos="-0.025 0 -0.008"
          size="0.025 0.085 0.008"
          rgba="0.20 0.22 0.25 1"
          contype="0" conaffinity="0"/>
    <geom name="nero_support_vertical"
          type="box"
          pos="-0.055 0 -0.075"
          size="0.008 0.085 0.067"
          rgba="0.20 0.22 0.25 1"
          contype="0" conaffinity="0"/>
    <geom name="nero_support_lower_jaw"
          type="box"
          pos="-0.080 0 -0.134"
          size="0.030 0.085 0.008"
          rgba="0.20 0.22 0.25 1"
          contype="0" conaffinity="0"/>

    <!-- ============================================================
         V69 CONSERVATIVE INITIAL-OBJECT ZONE (VISUAL ONLY)

         Center: (-0.405, 0.000) m
         Bounds: x=[-0.475,-0.335], y=[-0.110,+0.110] m
         Size:   140 x 220 mm

         The translucent fill + green border are non-colliding. Future
         object initial states should stay inside these bounds. Bins keep
         a 50 mm clear gap from the zone's +/-Y edges.
         ============================================================ -->
    <geom name="object_zone_fill" type="box"
          pos="-0.405 0 0.0005" size="0.070 0.110 0.0005"
          rgba="0.10 0.78 0.22 0.10" contype="0" conaffinity="0"/>
    <geom name="object_zone_xmin" type="box"
          pos="-0.475 0 0.0012" size="0.0015 0.110 0.0012"
          rgba="0.05 0.72 0.16 0.85" contype="0" conaffinity="0"/>
    <geom name="object_zone_xmax" type="box"
          pos="-0.335 0 0.0012" size="0.0015 0.110 0.0012"
          rgba="0.05 0.72 0.16 0.85" contype="0" conaffinity="0"/>
    <geom name="object_zone_ymin" type="box"
          pos="-0.405 -0.110 0.0012" size="0.070 0.0015 0.0012"
          rgba="0.05 0.72 0.16 0.85" contype="0" conaffinity="0"/>
    <geom name="object_zone_ymax" type="box"
          pos="-0.405 0.110 0.0012" size="0.070 0.0015 0.0012"
          rgba="0.05 0.72 0.16 0.85" contype="0" conaffinity="0"/>

    <!-- ============================================================
         Four graspable objects only.
         Cubes: 45 mm. Spheres: 52 mm diameter.
         V69: every initial object center lies inside the green object zone,
         with the larger 52 mm spheres still retaining edge clearance.
         ============================================================ -->

    <body name="obj_red_cube" pos="-0.365 -0.060 0.0225">
        <freejoint/>
        <geom name="obj_red_cube_geom"
              type="box"
              size="0.0225 0.0225 0.0225"
              rgba="0.92 0.12 0.12 1"
              mass="0.060"
              friction="4.0 0.12 0.02"
              solref="0.004 1"
              solimp="0.96 0.995 0.0005"
              margin="0.0004"
              condim="6"/>
    </body>

    <body name="obj_blue_cube" pos="-0.445 -0.060 0.0225">
        <freejoint/>
        <geom name="obj_blue_cube_geom"
              type="box"
              size="0.0225 0.0225 0.0225"
              rgba="0.12 0.35 0.92 1"
              mass="0.060"
              friction="4.0 0.12 0.02"
              solref="0.004 1"
              solimp="0.96 0.995 0.0005"
              margin="0.0004"
              condim="6"/>
    </body>

    <body name="obj_red_ball" pos="-0.365 0.060 0.026">
        <freejoint/>
        <geom name="obj_red_ball_geom"
              type="sphere"
              size="0.026"
              rgba="0.92 0.12 0.12 1"
              mass="0.055"
              friction="4.0 0.12 0.02"
              solref="0.004 1"
              solimp="0.96 0.995 0.0005"
              margin="0.0004"
              condim="6"/>
    </body>

    <body name="obj_blue_ball" pos="-0.445 0.060 0.026">
        <freejoint/>
        <geom name="obj_blue_ball_geom"
              type="sphere"
              size="0.026"
              rgba="0.12 0.35 0.92 1"
              mass="0.055"
              friction="4.0 0.12 0.02"
              solref="0.004 1"
              solimp="0.96 0.995 0.0005"
              margin="0.0004"
              condim="6"/>
    </body>

    <!-- ============================================================
         Two low sorting bins moved inward for V69 to keep carried-object
         transfers inside a more comfortable arm workspace. Internal opening is
         approximately 144 x 124 mm; wall top is z=40 mm.
         ============================================================ -->

    <!-- RED BIN, center (-0.455, -0.230) -->
    <geom name="red_bin_bottom" type="box"
          pos="-0.455 -0.230 0.004" size="0.080 0.070 0.004"
          rgba="0.55 0.10 0.10 0.65" friction="1.5 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="red_bin_wall_xneg" type="box"
          pos="-0.531 -0.230 0.020" size="0.004 0.070 0.020"
          rgba="0.78 0.14 0.14 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="red_bin_wall_xpos" type="box"
          pos="-0.379 -0.230 0.020" size="0.004 0.070 0.020"
          rgba="0.78 0.14 0.14 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="red_bin_wall_yneg" type="box"
          pos="-0.455 -0.296 0.020" size="0.072 0.004 0.020"
          rgba="0.78 0.14 0.14 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="red_bin_wall_ypos" type="box"
          pos="-0.455 -0.164 0.020" size="0.072 0.004 0.020"
          rgba="0.78 0.14 0.14 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>

    <!-- BLUE BIN, center (-0.455, +0.230) -->
    <geom name="blue_bin_bottom" type="box"
          pos="-0.455 0.230 0.004" size="0.080 0.070 0.004"
          rgba="0.10 0.25 0.58 0.65" friction="1.5 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="blue_bin_wall_xneg" type="box"
          pos="-0.531 0.230 0.020" size="0.004 0.070 0.020"
          rgba="0.14 0.35 0.82 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="blue_bin_wall_xpos" type="box"
          pos="-0.379 0.230 0.020" size="0.004 0.070 0.020"
          rgba="0.14 0.35 0.82 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="blue_bin_wall_yneg" type="box"
          pos="-0.455 0.164 0.020" size="0.072 0.004 0.020"
          rgba="0.14 0.35 0.82 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>
    <geom name="blue_bin_wall_ypos" type="box"
          pos="-0.455 0.296 0.020" size="0.072 0.004 0.020"
          rgba="0.14 0.35 0.82 0.82" friction="1.2 0.02 0.002"
          contype="1" conaffinity="1"/>

    <!-- Soft world light -->
    <light name="v5_world_light"
           pos="1.5 -1.5 3.5"
           dir="-0.3 0.3 -1"
           directional="true"
           diffuse="0.75 0.75 0.75"
           specular="0.15 0.15 0.15"/>
"""

    xml = xml.replace(
        "<worldbody>",
        "<worldbody>\n" + world_block,
        1,
    )

    # --------------------------------------------------------------
    # V26 runtime HOLD equalities.
    #
    # Use 3-DOF CONNECT rather than 6-DOF WELD:
    #   - locks object translation relative to link7
    #   - leaves object rotation free
    #   - less over-constrained than a rigid weld
    #
    # They start inactive with a schema-required placeholder anchor. At runtime we overwrite the connect anchors
    # to match the CURRENT grasp pose before activating one constraint.
    # --------------------------------------------------------------
    hold_object_names = [
                "obj_red_cube",
                "obj_blue_cube",
                "obj_red_ball",
                "obj_blue_ball",
            ]

    hold_eq_lines = []
    for obj_name in hold_object_names:
        hold_eq_lines.append(
            f'<connect name="hold_{obj_name}" '
            f'body1="link7" body2="{obj_name}" '
            f'anchor="0 0 0" '
            f'active="false" '
            f'solref="0.008 1" '
            f'solimp="0.98 0.995 0.001"/>'
        )

    hold_eq_xml = "\n        ".join(hold_eq_lines)

    if "</equality>" in xml:
        xml = xml.replace(
            "</equality>",
            "        " + hold_eq_xml + "\n    </equality>",
            1,
        )
    else:
        xml = xml.replace(
            "</mujoco>",
            "    <equality>\n        "
            + hold_eq_xml
            + "\n    </equality>\n</mujoco>",
            1,
        )

    # Put patched file next to original, so relative assets still resolve.
    out_path = src_path.with_name("_nero_motor_v56_unified_moveit_backend_tmp.xml")
    out_path.write_text(xml, encoding="utf-8")
    return str(out_path)



def run_sim(
    target_shared,
    actual_shared,
    speed_shared,
    wn_shared,
    zeta_shared,
    qacc_shared,
    gravity_enabled,
    gripper_target_mm,
    gripper_actual_mm,
    gripper_force_mode,
    gripper_force_N,
    clamp_speed_mult_shared,
    anti_slip_boost_shared,
    anti_slip_slip_mm_s_shared,
    anti_slip_mu_shared,
    contact_left_n_shared,
    contact_right_n_shared,
    contact_tangent_n_shared,
    contact_motor_cmd_shared,
    hold_enabled_shared,
    hold_active_shared,
    hold_name_shared,
    hold_ready_shared,
    hold_now_request,
    hold_off_request,
    hold_status_code_shared,
    reset_zero_request,
    recover_request,
    safety_trip_count,
    safety_state,
    sim_ready,
    sim_error_code,
    ros_bridge_ready,
    ros_cmd_count,
    ros_feedback_count,
    ros_scene_count,
    ros_last_cmd_age,
    ros_last_cmd_joint_count,
    ros_controller_state_rx_shared,
    ros_fallback_joint_state_rx_shared,
    ros_last_command_source_shared,
    ros_arm_command_gate_shared,
    stop_event,
):
    """
    MuJoCo process.
    Deliberately imports mujoco.viewer INSIDE this spawned child,
    so Tk/Tcl and GLFW never share the same GUI process.
    """
    try:
        import mujoco.viewer

        visual_model_file = make_grid_world_xml(MODEL_FILE)
        visual_model_file = add_official_gripper_to_xml(visual_model_file)
        model = mujoco.MjModel.from_xml_path(visual_model_file)
        data = mujoco.MjData(model)

        model.opt.timestep = 0.001
        dt = float(model.opt.timestep)

        joint_names = [f"joint{i}" for i in range(1, 8)]
        qpos_addr = []
        dof_addr = []

        # IMPORTANT: keep the verified mapping by joint NAME,
        # never assume qpos[i] == joint i.
        for name in joint_names:
            jid = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            if jid < 0:
                raise RuntimeError(f"Joint not found in model: {name}")

            qpos_addr.append(int(model.jnt_qposadr[jid]))
            dof_addr.append(int(model.jnt_dofadr[jid]))

        # Original AgileX two-finger gripper.
        gripper_jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint1"
        )
        if gripper_jid < 0:
            raise RuntimeError("gripper_joint1 not found after gripper injection")

        gripper_qpos_addr = int(model.jnt_qposadr[gripper_jid])
        gripper_dof_addr = int(model.jnt_dofadr[gripper_jid])

        gripper2_jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint2"
        )
        if gripper2_jid < 0:
            raise RuntimeError("gripper_joint2 not found after gripper injection")

        gripper2_qpos_addr = int(model.jnt_qposadr[gripper2_jid])
        gripper2_dof_addr = int(model.jnt_dofadr[gripper2_jid])


        # ==============================================================
        # V39 ROS2 / MOVEIT2 BRIDGE
        #
        # Official AgileX MoveIt2 publishes interpolated commands on:
        #   /control/joint_states
        #
        # This MuJoCo backend publishes actual state on:
        #   /feedback/joint_states
        #
        # With:
        #   demo.launch.py arm_type:=nero effector_type:=agx_gripper
        #                  follow:=true
        #
        # MoveIt/OMPL owns IK, planning, self/environment collision.
        # V31 remains the low-level MuJoCo execution controller.
        # ==============================================================
        ros_node = None
        ros_feedback_pub = None
        ros_scene_pub = None
        ros_joint_state_type = None
        ros_planning_scene_type = None
        ros_ready = False
        ros_last_spin_wall = 0.0
        ros_last_feedback_wall = 0.0
        ros_last_scene_wall = 0.0
        ros_last_command_wall = 0.0
        ros_cmd_seen = False
        # V41 safety latch:
        # if the arm penetrates a protected surface, stop accepting
        # further MoveIt arm commands until the user presses
        # STOP+RECOVER or HARD RESET.  This prevents the trajectory
        # controller from repeatedly driving back into the collision
        # after V14 rollback.
        ros_motion_fault_latched = False

        # Latest MoveIt command is consumed directly into V31 shared
        # targets. Callback runs in the SAME spawned MuJoCo process via
        # spin_once(), so no extra controller process touches MuJoCo.
        def _apply_arm_command(name_to_pos, source):
            nonlocal ros_last_command_wall, ros_cmd_seen
            nonlocal ros_motion_fault_latched

            updated = 0

            if (
                not ros_motion_fault_latched
                and bool(ros_arm_command_gate_shared.value)
            ):
                for i, jname in enumerate(joint_names):
                    if jname in name_to_pos:
                        qcmd = float(
                            np.clip(
                                name_to_pos[jname],
                                JOINT_LIMITS_RAD[i][0],
                                JOINT_LIMITS_RAD[i][1],
                            )
                        )
                        target_shared[i] = qcmd
                        updated += 1

            if updated > 0:
                ros_cmd_count.value += 1
                ros_last_cmd_joint_count.value = int(updated)
                ros_last_command_wall = time.perf_counter()
                ros_cmd_seen = True
                ros_last_command_source_shared.value = (
                    1 if source == "/arm_controller/controller_state" else 2
                )

            return updated

        def _moveit_joint_state_callback(msg):
            ros_fallback_joint_state_rx_shared.value += 1
            name_to_pos = {
                str(n): float(p)
                for n, p in zip(msg.name, msg.position)
            }
            _apply_arm_command(
                name_to_pos,
                "/control/joint_states",
            )

        def _controller_state_callback(msg):
            ros_controller_state_rx_shared.value += 1

            names = list(getattr(msg, "joint_names", []))
            point = None
            if hasattr(msg, "desired"):
                point = msg.desired
            elif hasattr(msg, "reference"):
                point = msg.reference

            if point is None:
                return

            positions = list(getattr(point, "positions", []))
            if not names or len(positions) < len(names):
                return

            name_to_pos = {
                str(n): float(p)
                for n, p in zip(names, positions)
            }
            _apply_arm_command(
                name_to_pos,
                "/arm_controller/controller_state",
            )

        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import (
                QoSProfile,
                ReliabilityPolicy,
                DurabilityPolicy,
            )
            from sensor_msgs.msg import JointState
            from control_msgs.msg import JointTrajectoryControllerState
            from moveit_msgs.msg import PlanningScene
            from moveit_msgs.msg import CollisionObject
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose
            from geometry_msgs.msg import TransformStamped
            from tf2_ros import TransformBroadcaster

            if not rclpy.ok():
                rclpy.init(args=None)

            ros_node = Node("nero_mujoco_v62_backend")
            object_tf_broadcaster = TransformBroadcaster(
                ros_node
            )

            object_tf_names = [
                "obj_red_cube",
                "obj_blue_cube",
                "obj_red_ball",
                "obj_blue_ball",
            ]
            object_tf_bids = {
                name: mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    name,
                )
                for name in object_tf_names
            }

            ros_feedback_pub = ros_node.create_publisher(
                JointState,
                "/feedback/joint_states",
                20,
            )

            ros_node.create_subscription(
                JointTrajectoryControllerState,
                "/arm_controller/controller_state",
                _controller_state_callback,
                50,
            )

            ros_node.create_subscription(
                JointState,
                "/control/joint_states",
                _moveit_joint_state_callback,
                20,
            )

            scene_qos = QoSProfile(depth=1)
            scene_qos.reliability = ReliabilityPolicy.RELIABLE
            scene_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

            ros_scene_pub = ros_node.create_publisher(
                PlanningScene,
                "/planning_scene",
                scene_qos,
            )

            ros_joint_state_type = JointState
            ros_planning_scene_type = PlanningScene

            def _make_box_collision(
                object_id,
                center_xyz,
                size_xyz,
            ):
                obj = CollisionObject()
                obj.header.frame_id = "base_link"
                obj.id = str(object_id)

                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [
                    float(size_xyz[0]),
                    float(size_xyz[1]),
                    float(size_xyz[2]),
                ]

                pose = Pose()
                pose.position.x = float(center_xyz[0])
                pose.position.y = float(center_xyz[1])
                pose.position.z = float(center_xyz[2])
                pose.orientation.w = 1.0

                obj.primitives = [primitive]
                obj.primitive_poses = [pose]
                obj.operation = CollisionObject.ADD
                return obj

            def _publish_mujoco_scene():
                if ros_scene_pub is None:
                    return

                scene = PlanningScene()
                scene.is_diff = True

                # V69 contact-aware planning geometry.
                # IMPORTANT: tabletop grasp/approach is an intended near-contact
                # operation, so the table must use its REAL geometry.  The V58
                # global +30 mm table inflation intruded into the grasp workspace
                # and made the final ~5% of otherwise-valid approaches look like
                # collisions.  Keep runtime safety monitoring for model mismatch.
                table_obj = _make_box_collision(
                    "mujoco_play_table",
                    (-0.385, 0.0, -0.03),
                    (0.67, 0.76, 0.06),   # V69 exact table; near edge x=-0.05, top=0.000 m
                )

                static_objects = [table_obj]

                # V69 bins move 35 mm toward NERO while keeping the same +/-Y
                # lanes. Their inner physical Y wall faces remain at +/-0.160 m,
                # leaving 50 mm clear space to the object-zone edges (+/-0.110 m).
                # MoveIt keeps only a modest +10 mm vertical safety margin,
                # so planning wall top = 50 mm. Lateral padding is retained.
                bin_boxes = [
                    ("red_bin_bottom",      (-0.455, -0.230, 0.006), (0.176, 0.156, 0.012)),
                    ("red_bin_wall_xneg",   (-0.531, -0.230, 0.025), (0.020, 0.156, 0.050)),
                    ("red_bin_wall_xpos",   (-0.379, -0.230, 0.025), (0.020, 0.156, 0.050)),
                    ("red_bin_wall_yneg",   (-0.455, -0.296, 0.025), (0.160, 0.020, 0.050)),
                    ("red_bin_wall_ypos",   (-0.455, -0.164, 0.025), (0.160, 0.020, 0.050)),
                    ("blue_bin_bottom",     (-0.455,  0.230, 0.006), (0.176, 0.156, 0.012)),
                    ("blue_bin_wall_xneg",  (-0.531, 0.230, 0.025), (0.020, 0.156, 0.050)),
                    ("blue_bin_wall_xpos",  (-0.379, 0.230, 0.025), (0.020, 0.156, 0.050)),
                    ("blue_bin_wall_yneg",  (-0.455, 0.164, 0.025), (0.160, 0.020, 0.050)),
                    ("blue_bin_wall_ypos",  (-0.455, 0.296, 0.025), (0.160, 0.020, 0.050)),
                ]

                for object_id, center, size in bin_boxes:
                    static_objects.append(
                        _make_box_collision(object_id, center, size)
                    )

                scene.world.collision_objects = static_objects

                ros_scene_pub.publish(scene)
                ros_scene_count.value += 1

            ros_ready = True
            ros_bridge_ready.value = True

            print(
                "V69 ROS2 bridge READY: "
                "/arm_controller/controller_state -> V31 (primary), "
                "/control/joint_states -> V31 (fallback), "
                "V31 -> /feedback/joint_states",
                flush=True,
            )

        except Exception as ros_exc:
            ros_bridge_ready.value = False
            ros_ready = False
            print(
                "V53 ROS2 bridge unavailable. "
                "Start this script after sourcing ROS2 + agx_arm_ws. "
                f"Reason: {ros_exc}",
                flush=True,
            )

        gripper_actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper"
        )
        if gripper_actuator_id < 0:
            raise RuntimeError("Official gripper position actuator not found")

        gripper_force_actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_force"
        )
        if gripper_force_actuator_id < 0:
            raise RuntimeError("V16 gripper force actuator not found")

        print("\n========== V10 VERIFIED ARM MAPPING ==========", flush=True)
        print("Slider | Joint  | qpos_addr | dof_addr", flush=True)
        for i, name in enumerate(joint_names):
            print(
                f"{i+1:>6} | {name:<6} | "
                f"{qpos_addr[i]:>9} | {dof_addr[i]:>8}",
                flush=True,
            )
        print("==============================================\n", flush=True)

        # --------------------------------------------------------------
        # V14 SAFETY GUARD SETUP
        # --------------------------------------------------------------
        protected_geom_ids = set()
        for geom_name in (
            "play_table",
            "v5_grid_floor",
            "red_bin_bottom",
            "red_bin_wall_xneg",
            "red_bin_wall_xpos",
            "red_bin_wall_yneg",
            "red_bin_wall_ypos",
            "blue_bin_bottom",
            "blue_bin_wall_xneg",
            "blue_bin_wall_xpos",
            "blue_bin_wall_yneg",
            "blue_bin_wall_ypos",
        ):
            gid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            if gid >= 0:
                protected_geom_ids.add(int(gid))

        robot_body_ids = set()
        robot_body_names = (
            ["base_link"]
            + [f"link{i}" for i in range(1, 8)]
            + ["gripper_link1", "gripper_link2"]
        )
        for body_name in robot_body_names:
            bid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            if bid >= 0:
                robot_body_ids.add(int(bid))

        def is_robot_geom(geom_id):
            if geom_id < 0 or geom_id >= model.ngeom:
                return False
            return int(model.geom_bodyid[geom_id]) in robot_body_ids

        def protected_penetration_depth():
            """
            Return the most negative robot-vs-protected-surface contact distance.
            0.0 means no penetration of a protected surface.
            """
            worst = 0.0
            for ci in range(int(data.ncon)):
                c = data.contact[ci]
                g1 = int(c.geom1)
                g2 = int(c.geom2)
                robot_vs_surface = (
                    (g1 in protected_geom_ids and is_robot_geom(g2))
                    or
                    (g2 in protected_geom_ids and is_robot_geom(g1))
                )
                if robot_vs_surface:
                    worst = min(worst, float(c.dist))
            return worst

        object_geom_ids = set()
        for gi in range(model.ngeom):
            gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gi)
            if gname and gname.startswith("obj_"):
                object_geom_ids.add(int(gi))

        gripper_body_ids = set()
        for bname in ("gripper_link1", "gripper_link2"):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
            if bid >= 0:
                gripper_body_ids.add(int(bid))

        def is_gripper_geom(geom_id):
            if geom_id < 0 or geom_id >= model.ngeom:
                return False
            return int(model.geom_bodyid[geom_id]) in gripper_body_ids

        def gripper_object_contact_info():
            has_contact = False
            worst = 0.0
            for ci in range(int(data.ncon)):
                c = data.contact[ci]
                g1 = int(c.geom1)
                g2 = int(c.geom2)
                pair = (
                    (is_gripper_geom(g1) and g2 in object_geom_ids)
                    or (is_gripper_geom(g2) and g1 in object_geom_ids)
                )
                if pair:
                    if float(c.dist) <= 0.0005:
                        has_contact = True
                    worst = min(worst, float(c.dist))
            return has_contact, worst

        def gripper_object_penetration_depth():
            return gripper_object_contact_info()[1]

        # --------------------------------------------------------------
        # V22 CONTACT-FORCE CLOSED-LOOP SETUP
        #
        # MuJoCo does NOT provide a separate static-friction coefficient.
        # Keep the V19 pad friction fixed at mu=10 and close the loop on
        # the REAL normal contact force measured by mj_contactForce().
        # --------------------------------------------------------------
        link7_bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "link7"
        )
        if link7_bid < 0:
            raise RuntimeError("link7 body not found")

        finger1_bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "gripper_link1"
        )
        finger2_bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "gripper_link2"
        )
        if finger1_bid < 0 or finger2_bid < 0:
            raise RuntimeError("gripper finger bodies not found")

        pad_geom_ids = set()
        for gi in range(model.ngeom):
            gname = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, gi
            )
            if gname and (
                gname.startswith("finger1_rubber_pad")
                or gname.startswith("finger2_rubber_pad")
                or gname.startswith("finger1_stud_")
                or gname.startswith("finger2_stud_")
            ):
                pad_geom_ids.add(int(gi))

        def contacted_object_body():
            """Return object body with the largest number of pad contacts."""
            counts = {}
            for ci in range(int(data.ncon)):
                c = data.contact[ci]
                g1 = int(c.geom1)
                g2 = int(c.geom2)

                obj_gid = -1
                if g1 in pad_geom_ids and g2 in object_geom_ids:
                    obj_gid = g2
                elif g2 in pad_geom_ids and g1 in object_geom_ids:
                    obj_gid = g1

                if obj_gid >= 0:
                    bid = int(model.geom_bodyid[obj_gid])
                    counts[bid] = counts.get(bid, 0) + 1

            if not counts:
                return -1
            return max(counts, key=counts.get)

        # --------------------------------------------------------------
        # V26 RUNTIME CONNECT HOLD SETUP
        # --------------------------------------------------------------
        hold_object_names = [
                "obj_red_cube",
                "obj_blue_cube",
                "obj_red_ball",
                "obj_blue_ball",
            ]

        hold_eq_by_body = {}
        hold_name_by_body = {}

        for obj_name in hold_object_names:
            bid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, obj_name
            )
            eqid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_EQUALITY,
                f"hold_{obj_name}"
            )

            if bid >= 0 and eqid >= 0:
                hold_eq_by_body[int(bid)] = int(eqid)
                hold_name_by_body[int(bid)] = obj_name
                data.eq_active[eqid] = 0

        def deactivate_all_holds():
            for eqid in hold_eq_by_body.values():
                data.eq_active[eqid] = 0

        def activate_connect_hold(obj_bid):
            """
            Activate a CONNECT equality at the object's CURRENT pose.

            For body-semantic connect equalities:
              eq_data[0:3] = anchor in body1 (link7) local frame
              eq_data[3:6] = anchor in body2 (object) local frame

            We choose body2 anchor = object origin/COM and compute the
            matching point in link7 local coordinates. Therefore position
            error is ~zero at activation: no snap-to-old-pose impulse.
            """
            eqid = hold_eq_by_body.get(int(obj_bid), -1)
            if eqid < 0:
                return False

            # Always ensure only one object is held.
            deactivate_all_holds()

            p_link = np.array(data.xpos[link7_bid], dtype=float)
            p_obj = np.array(data.xpos[obj_bid], dtype=float)

            # MuJoCo xmat is local->world rotation, row-major 3x3.
            R_link = np.array(
                data.xmat[link7_bid], dtype=float
            ).reshape(3, 3)

            anchor_link_local = R_link.T @ (p_obj - p_link)

            # CONNECT eq_data uses 6 anchor values.
            model.eq_data[eqid, 0:3] = anchor_link_local
            model.eq_data[eqid, 3:6] = 0.0

            # Activate. Constraint will be rebuilt by the next step1.
            data.eq_active[eqid] = 1
            return True

        def gripper_contact_sides():
            """
            Contact-presence and worst penetration based ONLY on the
            visible rubber pad/stud geoms, not arbitrary gripper meshes.
            """
            left_touch = False
            right_touch = False
            worst = 0.0

            for ci in range(int(data.ncon)):
                c = data.contact[ci]
                g1 = int(c.geom1)
                g2 = int(c.geom2)

                pad_gid = -1
                if g1 in pad_geom_ids and g2 in object_geom_ids:
                    pad_gid = g1
                elif g2 in pad_geom_ids and g1 in object_geom_ids:
                    pad_gid = g2
                else:
                    continue

                bid = int(model.geom_bodyid[pad_gid])
                if float(c.dist) <= 0.0005:
                    if bid == finger1_bid:
                        left_touch = True
                    elif bid == finger2_bid:
                        right_touch = True

                worst = min(worst, float(c.dist))

            return left_touch, right_touch, worst

        def measure_pad_contact_forces():
            """
            Sum contact forces on each visible fingertip pad.

            mj_contactForce() returns a 6-vector in the contact frame:
              [normal, tangent1, tangent2, torque1, torque2, torque3]

            Returns:
              left_normal_N
              right_normal_N
              total_tangential_N
            """
            left_n = 0.0
            right_n = 0.0
            tangential = 0.0
            cf = np.zeros(6, dtype=float)

            for ci in range(int(data.ncon)):
                c = data.contact[ci]
                g1 = int(c.geom1)
                g2 = int(c.geom2)

                pad_gid = -1
                if g1 in pad_geom_ids and g2 in object_geom_ids:
                    pad_gid = g1
                elif g2 in pad_geom_ids and g1 in object_geom_ids:
                    pad_gid = g2
                else:
                    continue

                cf[:] = 0.0
                mujoco.mj_contactForce(model, data, ci, cf)

                fn = abs(float(cf[0]))
                ft = float(np.hypot(cf[1], cf[2]))
                bid = int(model.geom_bodyid[pad_gid])

                if bid == finger1_bid:
                    left_n += fn
                elif bid == finger2_bid:
                    right_n += fn

                tangential += ft

            return left_n, right_n, tangential

        try:
            model.opt.iterations = max(int(model.opt.iterations), 80)
        except Exception:
            pass
        try:
            model.opt.ls_iterations = max(int(model.opt.ls_iterations), 20)
        except Exception:
            pass

        # V38 default gravity ON.
        model.opt.gravity[:] = (0.0, 0.0, -9.81)

        mujoco.mj_forward(model, data)

        q0 = np.array(
            [data.qpos[a] for a in qpos_addr],
            dtype=float,
        )

        q_ref = q0.copy()
        q_ref_prev = q0.copy()

        # Last known safe ARM pose. Updated only when the robot is not
        # penetrating the table/riser/floor and is not moving excessively.
        last_safe_q = q0.copy()
        last_safe_gripper_q = 0.0
        safety_cooldown_until = 0.0

        # V31 adjustable two-stage clamp:
        #
        # BASE_APPROACH_QVEL is the original stable V19/V30 speed (1x).
        # Default GUI command is 10x during FREE SPACE.
        #
        # As soon as EITHER fingertip touches an object, the speed is
        # automatically reduced to CONTACT_BRAKE_MULT (1.5x) so the
        # last closing motion remains gentle and does not launch objects.
        #
        # Once BOTH fingertips contact, the original V22 force closed
        # loop takes over exactly as before.
        BASE_APPROACH_QVEL = 0.0030   # m/s master coordinate = original 1x
        CONTACT_BRAKE_MULT = 1.5

        APPROACH_VEL_K = 25.0        # N/(m/s)
        # V39: more drive in free-space so the requested 15x approach
        # speed is actually reached.  First contact still automatically
        # brakes to CONTACT_BRAKE_MULT before force control takes over.
        APPROACH_FORCE_LIMIT = 0.60  # N
        CONTACT_FORCE_RAMP_SEC = 2.0

        clamp_mode_prev = False
        contact_latched = False
        contact_started_at = 0.0
        clamp_applied_force = 0.0

        # --------------------------------------------------------------
        # V22 contact-force loop state.
        #
        # GUI clamp-force value is now interpreted as:
        #     TARGET NORMAL FORCE PER FINGER [N]
        #
        # The actuator generalized force is allowed to differ from this
        # number.  The PI loop adjusts motor force until BOTH fingertips
        # actually measure the requested normal force.
        # --------------------------------------------------------------
        MAX_EXTRA_TARGET_N = 2.0

        # Closed-loop tuning deliberately conservative.
        FORCE_KP = 0.70
        FORCE_KI = 0.80
        MOTOR_FEEDFORWARD_GAIN = 1.20
        MOTOR_MAX_N = 30.0
        MOTOR_RISE_N_PER_S = 4.0
        MOTOR_FALL_N_PER_S = 10.0
        FORCE_INTEGRAL_LIMIT = 12.0

        anti_slip_boost = 0.0
        anti_slip_obj_bid = -1
        anti_slip_prev_rel_z = None
        anti_slip_vz_lp = 0.0
        anti_slip_persist = 0.0

        measured_left_n = 0.0
        measured_right_n = 0.0
        measured_tangent_n = 0.0

        force_integral = 0.0
        motor_clamp_cmd = 0.0

        # V30 AUTO CREEP HOLD state.
        #
        # Automatic HOLD:
        #   clamp mode
        #   + gripper/object contact exists
        #   + downward relative slip <= -0.05 mm/s
        #   + condition persists for >= 0.10 s
        #
        # HOLD OFF:
        #   any explicit release/opening action
        #
        # Manual HOLD ON / HOLD OFF buttons remain available for debug.
        AUTO_HOLD_SLIP_MM_S = -0.05
        AUTO_HOLD_PERSIST_SEC = 0.10

        hold_ready_time = 0.0
        hold_obj_bid = -1
        last_contact_obj_bid = -1
        hold_active = False
        auto_hold_timer = 0.0

        # status:
        # 0 = ready / monitoring
        # 1 = holding
        # 2 = manual HOLD ON requested but no object contact/candidate found
        hold_active_shared.value = False
        hold_name_shared.value = -1
        hold_ready_shared.value = 0.0
        hold_now_request.value = False
        hold_off_request.value = False
        hold_status_code_shared.value = 0

        anti_slip_boost_shared.value = 0.0
        anti_slip_slip_mm_s_shared.value = 0.0
        anti_slip_mu_shared.value = 10.0
        contact_left_n_shared.value = 0.0
        contact_right_n_shared.value = 0.0
        contact_tangent_n_shared.value = 0.0
        contact_motor_cmd_shared.value = 0.0

        gripper_q0 = float(data.qpos[gripper_qpos_addr])
        last_safe_gripper_q = gripper_q0
        gripper_target_mm.value = max(0.0, min(100.0, 2.0 * gripper_q0 * 1000.0))
        gripper_actual_mm.value = gripper_target_mm.value

        def hard_restore_arm(q_restore, gripper_q=None, reason_code=1, count_trip=True):
            """
            Simulation-only safety recovery:
              - teleport arm back to a known safe joint pose
              - zero arm/gripper velocities and accelerations
              - synchronize GUI targets and internal references
              - clear applied forces before continuing
            """
            nonlocal q_ref, q_ref_prev, next_step_wall, safety_cooldown_until

            q_restore = np.asarray(q_restore, dtype=float).copy()

            for i, qadr in enumerate(qpos_addr):
                lo, hi = JOINT_LIMITS_RAD[i]
                q_restore[i] = float(np.clip(q_restore[i], lo, hi))
                data.qpos[qadr] = q_restore[i]

            for dadr in dof_addr:
                data.qvel[dadr] = 0.0
                data.qacc[dadr] = 0.0

            if gripper_q is None:
                gripper_q = float(data.qpos[gripper_qpos_addr])

            gripper_q = float(np.clip(gripper_q, 0.0, 0.05))
            data.qpos[gripper_qpos_addr] = gripper_q
            data.qpos[gripper2_qpos_addr] = -gripper_q
            data.qvel[gripper_dof_addr] = 0.0
            data.qvel[gripper2_dof_addr] = 0.0
            data.qacc[gripper_dof_addr] = 0.0
            data.qacc[gripper2_dof_addr] = 0.0

            data.qfrc_applied[:] = 0.0
            data.xfrc_applied[:] = 0.0
            if model.nu > 0:
                data.ctrl[:] = 0.0

            q_ref[:] = q_restore
            q_ref_prev[:] = q_restore

            for i in range(7):
                target_shared[i] = float(q_restore[i])
                actual_shared[i] = float(q_restore[i])

            grip_mm = 2.0 * gripper_q * 1000.0
            gripper_target_mm.value = grip_mm
            gripper_actual_mm.value = grip_mm

            # Safety recovery always returns gripper to position mode.
            gripper_force_mode.value = False

            anti_slip_boost_shared.value = 0.0
            anti_slip_slip_mm_s_shared.value = 0.0
            anti_slip_mu_shared.value = 10.0
            contact_left_n_shared.value = 0.0
            contact_right_n_shared.value = 0.0
            contact_tangent_n_shared.value = 0.0
            contact_motor_cmd_shared.value = 0.0

            deactivate_all_holds()
            hold_ready_time = 0.0
            hold_obj_bid = -1
            hold_active = False
            hold_active_shared.value = False
            hold_name_shared.value = -1
            hold_ready_shared.value = 0.0
            hold_now_request.value = False
            hold_off_request.value = False
            hold_status_code_shared.value = 0
            auto_hold_timer = 0.0

            mujoco.mj_forward(model, data)

            safety_state.value = int(reason_code)
            if bool(count_trip):
                safety_trip_count.value += 1
            safety_cooldown_until = time.perf_counter() + (1.0 if not count_trip else 0.25)
            next_step_wall = time.perf_counter()

        def soft_freeze_arm(reason_code=2, count_trip=False):
            """Stop the arm at its *current* physical pose without teleporting.

            Critical V58 behavior: preserve gripper force/position mode, gripper
            opening, held-object equality state, and object pose.  This replaces
            the old automatic rollback that could jerk toward an old checkpoint
            and fling a grasped object.
            """
            nonlocal q_ref, q_ref_prev, next_step_wall, safety_cooldown_until

            q_now = np.array(
                [float(data.qpos[a]) for a in qpos_addr], dtype=float
            )
            for dadr in dof_addr:
                data.qvel[dadr] = 0.0
                data.qacc[dadr] = 0.0
                data.qfrc_applied[dadr] = 0.0

            q_ref[:] = q_now
            q_ref_prev[:] = q_now
            for i in range(7):
                target_shared[i] = float(q_now[i])
                actual_shared[i] = float(q_now[i])

            # Do NOT touch gripper_qpos, gripper target, force mode, anti-slip,
            # or object HOLD state. The object must remain held during replanning.
            mujoco.mj_forward(model, data)
            safety_state.value = int(reason_code)
            if bool(count_trip):
                safety_trip_count.value += 1
            safety_cooldown_until = time.perf_counter() + (0.30 if count_trip else 0.10)
            next_step_wall = time.perf_counter()

        for i in range(7):
            target_shared[i] = float(q0[i])
            actual_shared[i] = float(q0[i])

        sim_ready.value = True
        sim_error_code.value = 0

        viewer_period = 1.0 / 60.0
        last_viewer_sync = 0.0
        next_step_wall = time.perf_counter()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and not stop_event.is_set():

                if bool(gravity_enabled.value):
                    model.opt.gravity[:] = (0.0, 0.0, -9.81)
                else:
                    model.opt.gravity[:] = (0.0, 0.0, 0.0)

                # First half of MuJoCo step.
                mujoco.mj_step1(model, data)

                # ------------------------------------------------------
                # MANUAL SAFETY REQUESTS
                # ------------------------------------------------------
                if bool(reset_zero_request.value):
                    reset_zero_request.value = False
                    ros_motion_fault_latched = False

                    zero_pose = np.zeros(7, dtype=float)
                    # Open gripper to 80 mm during hard reset so a trapped
                    # object is released rather than pinched during teleport.
                    hard_restore_arm(
                        zero_pose,
                        gripper_q=0.040,
                        reason_code=3,
                        count_trip=False,
                    )
                    last_safe_q[:] = zero_pose
                    last_safe_gripper_q = 0.040
                    continue

                if bool(recover_request.value):
                    # EXPLICIT user/planner recovery only. Automatic safety
                    # handlers never set this flag in V69.
                    recover_request.value = False
                    ros_motion_fault_latched = False
                    soft_freeze_arm(
                        reason_code=2,
                        count_trip=False,
                    )
                    continue

                # ------------------------------------------------------
                # AUTOMATIC ANTI-PENETRATION GUARD
                #
                # Soft MuJoCo contacts normally allow tiny negative
                # distances. We only trip on > 2.0 mm penetration.
                # ------------------------------------------------------
                penetration = protected_penetration_depth()
                if (
                    penetration < -0.002
                    and time.perf_counter() >= safety_cooldown_until
                ):
                    print(
                        f"SAFETY GUARD: protected-surface penetration "
                        f"{-penetration*1000.0:.2f} mm -> FREEZE-IN-PLACE + "
                        "MOVEIT COMMAND LATCH",
                        flush=True,
                    )
                    ros_motion_fault_latched = True
                    soft_freeze_arm(
                        reason_code=1,
                        count_trip=True,
                    )
                    continue

                q = np.array(
                    [data.qpos[a] for a in qpos_addr],
                    dtype=float,
                )
                qd = np.array(
                    [data.qvel[a] for a in dof_addr],
                    dtype=float,
                )

                gripper_q = float(data.qpos[gripper_qpos_addr])
                gripper_qd = float(data.qvel[gripper_dof_addr])

                # ------------------------------------------------------
                # V39 ROS2 bridge heartbeat.
                # ------------------------------------------------------
                if ros_ready and ros_node is not None:
                    now_ros = time.perf_counter()

                    # Process MoveIt commands with low latency.
                    if now_ros - ros_last_spin_wall >= 0.002:
                        try:
                            rclpy.spin_once(
                                ros_node,
                                timeout_sec=0.0,
                            )
                        except Exception as exc:
                            print(
                                f"ROS spin error: {exc}",
                                flush=True,
                            )
                        ros_last_spin_wall = now_ros

                    # Publish actual MuJoCo state at 100 Hz.
                    if now_ros - ros_last_feedback_wall >= 0.010:
                        try:
                            msg = ros_joint_state_type()
                            msg.header.stamp = (
                                ros_node.get_clock().now().to_msg()
                            )
                            # Match the official AgileX MoveIt model:
                            # arm joints joint1..joint7 + active "gripper".
                            # "gripper" is FULL opening width in meters.
                            msg.name = joint_names + ["gripper"]
                            msg.position = (
                                [float(v) for v in q]
                                + [
                                    float(
                                        np.clip(
                                            2.0 * gripper_q,
                                            0.0,
                                            0.10,
                                        )
                                    )
                                ]
                            )
                            msg.velocity = (
                                [float(v) for v in qd]
                                + [
                                    float(2.0 * gripper_qd)
                                ]
                            )

                            ros_feedback_pub.publish(msg)
                            ros_feedback_count.value += 1
                        except Exception as exc:
                            print(
                                f"ROS feedback publish error: {exc}",
                                flush=True,
                            )

                        ros_last_feedback_wall = now_ros

                    # Publish MuJoCo object poses as TF frames at 20 Hz.
                    if (
                        not hasattr(run_sim, "_v39_last_tf_wall")
                        or now_ros
                        - getattr(run_sim, "_v39_last_tf_wall")
                        >= 0.050
                    ):
                        try:
                            tf_msgs = []

                            for (
                                obj_name,
                                obj_bid,
                            ) in object_tf_bids.items():
                                if obj_bid < 0:
                                    continue

                                tfm = TransformStamped()
                                tfm.header.stamp = (
                                    ros_node.get_clock()
                                    .now()
                                    .to_msg()
                                )
                                tfm.header.frame_id = "base_link"
                                tfm.child_frame_id = (
                                    "sim_" + obj_name
                                )

                                p = data.xpos[obj_bid]
                                qwxyz = data.xquat[obj_bid]

                                tfm.transform.translation.x = float(
                                    p[0]
                                )
                                tfm.transform.translation.y = float(
                                    p[1]
                                )
                                tfm.transform.translation.z = float(
                                    p[2]
                                )

                                tfm.transform.rotation.w = float(
                                    qwxyz[0]
                                )
                                tfm.transform.rotation.x = float(
                                    qwxyz[1]
                                )
                                tfm.transform.rotation.y = float(
                                    qwxyz[2]
                                )
                                tfm.transform.rotation.z = float(
                                    qwxyz[3]
                                )

                                tf_msgs.append(tfm)

                            if tf_msgs:
                                object_tf_broadcaster.sendTransform(
                                    tf_msgs
                                )
                        except Exception as exc:
                            print(
                                f"Object TF publish error: {exc}",
                                flush=True,
                            )

                        setattr(
                            run_sim,
                            "_v39_last_tf_wall",
                            now_ros,
                        )

                    # Re-publish PlanningScene periodically so MoveIt
                    # started after MuJoCo still receives table + sorting bins.
                    if now_ros - ros_last_scene_wall >= 2.0:
                        try:
                            _publish_mujoco_scene()
                        except Exception as exc:
                            print(
                                f"PlanningScene publish error: {exc}",
                                flush=True,
                            )
                        ros_last_scene_wall = now_ros

                    if ros_cmd_seen:
                        ros_last_cmd_age.value = float(
                            max(
                                0.0,
                                now_ros - ros_last_command_wall,
                            )
                        )
                    else:
                        ros_last_cmd_age.value = -1.0

                target = shared_array(target_shared)

                # Per-joint reference speed.
                speed_pct = np.clip(
                    shared_array(speed_shared),
                    1.0,
                    100.0,
                )
                max_ref_vel = (
                    FULL_SPEED_REF_VEL * speed_pct / 100.0
                )

                max_step = max_ref_vel * dt
                delta = np.clip(
                    target - q_ref,
                    -max_step,
                    max_step,
                )
                q_ref += delta

                qd_ref = (q_ref - q_ref_prev) / dt
                q_ref_prev[:] = q_ref

                wn = np.clip(
                    shared_array(wn_shared),
                    0.05,
                    50.0,
                )
                zeta = np.clip(
                    shared_array(zeta_shared),
                    0.05,
                    5.0,
                )
                max_qacc = np.clip(
                    shared_array(qacc_shared),
                    0.05,
                    200.0,
                )

                # Same acceleration-space second-order controller
                # used by the stable version.
                qacc_cmd = (
                    (wn ** 2) * (q_ref - q)
                    + (2.0 * zeta * wn) * (qd_ref - qd)
                )
                qacc_cmd = np.clip(
                    qacc_cmd,
                    -max_qacc,
                    max_qacc,
                )

                # --------------------------------------------------------
                # V10 ARM CONTROL = EXACT STABLE V5/V6 APPROACH
                #
                # IMPORTANT:
                # Do not change the verified arm mapping/control path here.
                # J1..J7 are addressed ONLY through qpos_addr / dof_addr
                # obtained from the exact MuJoCo joint names.
                # --------------------------------------------------------

                # Inverse dynamics for the ARM ONLY, same as stable V5/V6.
                data.qacc[:] = 0.0
                for i, dadr in enumerate(dof_addr):
                    data.qacc[dadr] = qacc_cmd[i]

                mujoco.mj_inverse(model, data)
                required_force = data.qfrc_inverse.copy()

                # V13: DO NOT add qfrc_constraint here.
                # Keep the exact V10 stable arm force path.

                # Clear controls / arbitrary forces first.
                if model.nu > 0:
                    data.ctrl[:] = 0.0

                data.qfrc_applied[:] = 0.0
                data.xfrc_applied[:] = 0.0

                # Apply ONLY the seven verified arm generalized forces.
                for i, dadr in enumerate(dof_addr):
                    data.qfrc_applied[dadr] = required_force[dadr]

                # --------------------------------------------------------
                # V16 GRIPPER CONTROL
                #
                # POSITION MODE:
                #   slider -> position actuator
                #
                # FORCE MODE:
                #   "Clamp" button -> direct motor force.
                #   q1 increases while opening, so closing force is negative.
                #
                # The position actuator is commanded to CURRENT q during
                # force mode. That removes position spring error while keeping
                # its velocity damping term active for stable squeezing.
                # --------------------------------------------------------
                grip_target_q = float(np.clip(
                    float(gripper_target_mm.value) / 2000.0,
                    0.0,
                    0.05,
                ))

                force_mode_now = bool(gripper_force_mode.value)

                if force_mode_now:
                    user_target_per_finger = float(np.clip(
                        abs(float(gripper_force_N.value)),
                        0.5,
                        25.0,
                    ))

                    if not clamp_mode_prev:
                        contact_latched = False
                        contact_started_at = 0.0
                        clamp_applied_force = 0.0

                        deactivate_all_holds()
                        hold_ready_time = 0.0
                        hold_obj_bid = -1
                        hold_active = False
                        hold_active_shared.value = False
                        hold_name_shared.value = -1
                        hold_ready_shared.value = 0.0
                        hold_now_request.value = False
                        hold_off_request.value = False
                        hold_status_code_shared.value = 0
                        last_contact_obj_bid = -1
                        auto_hold_timer = 0.0

                        anti_slip_boost = 0.0
                        anti_slip_obj_bid = -1
                        anti_slip_prev_rel_z = None
                        anti_slip_vz_lp = 0.0
                        anti_slip_persist = 0.0

                        force_integral = 0.0
                        motor_clamp_cmd = 0.0

                    left_touch, right_touch, contact_dist = (
                        gripper_contact_sides()
                    )
                    both_touch = bool(left_touch and right_touch)

                    if both_touch and not contact_latched:
                        contact_latched = True
                        contact_started_at = time.perf_counter()
                        force_integral = 0.0

                        anti_slip_obj_bid = contacted_object_body()
                        anti_slip_prev_rel_z = None
                        anti_slip_vz_lp = 0.0
                        anti_slip_persist = 0.0

                    # If one side loses contact, go back to gentle approach.
                    if (
                        contact_latched
                        and not both_touch
                        and not hold_active
                    ):
                        contact_latched = False
                        force_integral = 0.0

                    # Neutralize position spring; retain built-in damping.
                    data.ctrl[gripper_actuator_id] = float(
                        np.clip(gripper_q, 0.0, 0.05)
                    )

                    if not contact_latched:
                        # ------------------------------------------------
                        # V31 STAGE A: FAST FREE-SPACE APPROACH
                        #
                        # User-selected multiplier controls only the
                        # free-space closing speed.
                        #
                        # The instant either fingertip contacts an object,
                        # automatically brake to 1.5x the original V30
                        # speed.  This preserves the fast one-click clamp
                        # without carrying 10x impact speed into the object.
                        # ------------------------------------------------
                        requested_mult = float(np.clip(
                            float(clamp_speed_mult_shared.value),
                            1.0,
                            20.0,
                        ))

                        any_touch = bool(left_touch or right_touch)

                        if any_touch:
                            active_mult = min(
                                requested_mult,
                                CONTACT_BRAKE_MULT,
                            )
                        else:
                            active_mult = requested_mult

                        active_approach_qvel = (
                            BASE_APPROACH_QVEL * active_mult
                        )
                        desired_qd = -active_approach_qvel

                        motor_force = APPROACH_VEL_K * (
                            desired_qd - gripper_qd
                        )
                        motor_force = float(np.clip(
                            motor_force,
                            -APPROACH_FORCE_LIMIT,
                            APPROACH_FORCE_LIMIT,
                        ))

                        if gripper_qd < -active_approach_qvel:
                            motor_force = max(0.0, motor_force)

                        motor_clamp_cmd = max(0.0, -motor_force)
                        clamp_applied_force = motor_clamp_cmd
                        data.ctrl[gripper_force_actuator_id] = motor_force

                        anti_slip_boost = 0.0
                        anti_slip_slip_mm_s_shared.value = 0.0
                        force_integral = 0.0

                    else:
                        # ------------------------------------------------
                        # MILD ANTI-SLIP:
                        # detect very slow downward creep relative to link7.
                        # Unlike V21, this does NOT alter friction.
                        # It only raises the NORMAL-FORCE TARGET slowly.
                        # ------------------------------------------------
                        current_obj_bid = contacted_object_body()
                        if current_obj_bid >= 0:
                            if current_obj_bid != anti_slip_obj_bid:
                                anti_slip_obj_bid = current_obj_bid
                                anti_slip_prev_rel_z = None
                                anti_slip_vz_lp = 0.0
                                anti_slip_persist = 0.0
                                anti_slip_boost = 0.0

                            rel_z = float(
                                data.xpos[anti_slip_obj_bid, 2]
                                - data.xpos[link7_bid, 2]
                            )

                            if anti_slip_prev_rel_z is not None:
                                raw_vz = (
                                    rel_z - anti_slip_prev_rel_z
                                ) / dt

                                # ~80 ms low-pass filter.
                                alpha = dt / (0.080 + dt)
                                anti_slip_vz_lp += alpha * (
                                    raw_vz - anti_slip_vz_lp
                                )

                            anti_slip_prev_rel_z = rel_z
                        else:
                            anti_slip_vz_lp *= 0.98

                        slip_down_mps = max(0.0, -anti_slip_vz_lp)
                        anti_slip_slip_mm_s_shared.value = (
                            1000.0 * anti_slip_vz_lp
                        )

                        contact_age = max(
                            0.0,
                            time.perf_counter() - contact_started_at,
                        )

                        # Detect much slower creep than V21:
                        # 0.15 mm/s instead of 1.5 mm/s.
                        if contact_age > 1.0:
                            if slip_down_mps > 0.00015:
                                anti_slip_persist += dt
                            else:
                                anti_slip_persist = max(
                                    0.0,
                                    anti_slip_persist - 2.0 * dt,
                                )

                            if anti_slip_persist > 0.25:
                                anti_slip_boost = min(
                                    MAX_EXTRA_TARGET_N,
                                    anti_slip_boost + 0.25 * dt,
                                )
                            elif slip_down_mps < 0.00005:
                                anti_slip_boost = max(
                                    0.0,
                                    anti_slip_boost - 0.10 * dt,
                                )

                        anti_slip_boost_shared.value = anti_slip_boost
                        anti_slip_mu_shared.value = 10.0

                        # ------------------------------------------------
                        # V22 TARGET CONTACT FORCE PER FINGER.
                        # Ramp target after bilateral contact to prevent
                        # impact, then close the loop on the weaker side.
                        # ------------------------------------------------
                        elapsed_contact = max(
                            0.0,
                            time.perf_counter() - contact_started_at,
                        )
                        target_ramp = min(
                            1.0,
                            elapsed_contact / CONTACT_FORCE_RAMP_SEC,
                        )

                        effective_target = (
                            user_target_per_finger + anti_slip_boost
                        ) * target_ramp

                        # Control the weaker fingertip so both sides must
                        # actually reach the requested normal force.
                        actual_control_force = min(
                            measured_left_n,
                            measured_right_n,
                        )

                        force_error = (
                            effective_target - actual_control_force
                        )

                        # Conditional integration / anti-windup.
                        if abs(force_error) < 8.0:
                            force_integral += force_error * dt
                            force_integral = float(np.clip(
                                force_integral,
                                -FORCE_INTEGRAL_LIMIT,
                                FORCE_INTEGRAL_LIMIT,
                            ))

                        desired_motor_cmd = (
                            MOTOR_FEEDFORWARD_GAIN * effective_target
                            + FORCE_KP * force_error
                            + FORCE_KI * force_integral
                        )
                        desired_motor_cmd = float(np.clip(
                            desired_motor_cmd,
                            0.0,
                            MOTOR_MAX_N,
                        ))

                        # Very gentle actuator-force slew.
                        if desired_motor_cmd > motor_clamp_cmd:
                            motor_clamp_cmd = min(
                                desired_motor_cmd,
                                motor_clamp_cmd
                                + MOTOR_RISE_N_PER_S * dt,
                            )
                        else:
                            motor_clamp_cmd = max(
                                desired_motor_cmd,
                                motor_clamp_cmd
                                - MOTOR_FALL_N_PER_S * dt,
                            )

                        closing_damping = -8.0 * gripper_qd
                        motor_force = (
                            -motor_clamp_cmd + closing_damping
                        )

                        penetration = max(
                            0.0,
                            -float(contact_dist),
                        )

                        # Preserve V19 penetration protection.
                        if penetration > 0.00025:
                            scale = 1.0 - (
                                (penetration - 0.00025) / 0.00030
                            )
                            scale = float(np.clip(scale, 0.0, 1.0))
                            motor_force = (
                                -motor_clamp_cmd * scale
                                + closing_damping
                            )

                        if penetration > 0.00055:
                            motor_force = max(
                                0.0,
                                closing_damping,
                            )

                        if penetration > 0.00080:
                            motor_force = 0.20

                        clamp_applied_force = motor_clamp_cmd
                        data.ctrl[gripper_force_actuator_id] = float(
                            np.clip(motor_force, -50.0, 50.0)
                        )

                else:
                    contact_latched = False
                    clamp_applied_force = 0.0
                    data.ctrl[gripper_force_actuator_id] = 0.0
                    data.ctrl[gripper_actuator_id] = grip_target_q

                    anti_slip_boost = 0.0
                    anti_slip_obj_bid = -1
                    anti_slip_prev_rel_z = None
                    anti_slip_vz_lp = 0.0
                    anti_slip_persist = 0.0

                    force_integral = 0.0
                    motor_clamp_cmd = 0.0

                    deactivate_all_holds()
                    hold_ready_time = 0.0
                    hold_obj_bid = -1
                    hold_active = False
                    hold_active_shared.value = False
                    hold_name_shared.value = -1
                    hold_ready_shared.value = 0.0
                    hold_now_request.value = False
                    hold_off_request.value = False
                    hold_status_code_shared.value = 0
                    last_contact_obj_bid = -1
                    auto_hold_timer = 0.0

                    anti_slip_boost_shared.value = 0.0
                    anti_slip_slip_mm_s_shared.value = 0.0
                    anti_slip_mu_shared.value = 10.0
                    contact_motor_cmd_shared.value = 0.0

                clamp_mode_prev = force_mode_now

                # ------------------------------------------------------
                # V30 AUTO CREEP HOLD
                #
                # Automatic trigger:
                #   force clamp mode
                #   + current/recent gripper-object contact
                #   + downward slip <= -0.05 mm/s
                #   + persists >= 0.10 s
                #
                # Release/opening disables HOLD immediately.
                # Manual HOLD ON / HOLD OFF remain available.
                # ------------------------------------------------------
                candidate_bid = contacted_object_body()

                if candidate_bid >= 0:
                    last_contact_obj_bid = int(candidate_bid)

                # ------------------------------------------------------
                # HOLD OFF: explicit user request.
                # ------------------------------------------------------
                if bool(hold_off_request.value):
                    hold_off_request.value = False
                    deactivate_all_holds()
                    hold_ready_time = 0.0
                    hold_obj_bid = -1
                    hold_active = False
                    auto_hold_timer = 0.0
                    hold_active_shared.value = False
                    hold_name_shared.value = -1
                    hold_ready_shared.value = 0.0
                    hold_status_code_shared.value = 0

                # ------------------------------------------------------
                # HOLD OFF: any opening / release state.
                #
                # force_mode_now=False means gripper has left CLAMP force
                # mode and is back in position/opening mode.
                # ------------------------------------------------------
                if (not force_mode_now) and hold_active:
                    deactivate_all_holds()
                    hold_ready_time = 0.0
                    hold_obj_bid = -1
                    hold_active = False
                    auto_hold_timer = 0.0
                    hold_active_shared.value = False
                    hold_name_shared.value = -1
                    hold_ready_shared.value = 0.0
                    hold_status_code_shared.value = 0

                # ------------------------------------------------------
                # Manual HOLD ON override.
                # ------------------------------------------------------
                if bool(hold_now_request.value):
                    hold_now_request.value = False

                    manual_bid = (
                        int(candidate_bid)
                        if candidate_bid >= 0
                        else int(last_contact_obj_bid)
                    )

                    if manual_bid >= 0:
                        hold_obj_bid = manual_bid

                        if activate_connect_hold(hold_obj_bid):
                            hold_active = True
                            auto_hold_timer = AUTO_HOLD_PERSIST_SEC
                            hold_active_shared.value = True
                            hold_name_shared.value = int(hold_obj_bid)
                            hold_ready_shared.value = AUTO_HOLD_PERSIST_SEC
                            hold_status_code_shared.value = 1
                        else:
                            hold_status_code_shared.value = 2
                    else:
                        hold_status_code_shared.value = 2

                # ------------------------------------------------------
                # Automatic HOLD ON from downward creep.
                # ------------------------------------------------------
                if (
                    bool(hold_enabled_shared.value)
                    and force_mode_now
                    and not hold_active
                ):
                    auto_bid = (
                        int(candidate_bid)
                        if candidate_bid >= 0
                        else int(last_contact_obj_bid)
                    )

                    contact_present = (auto_bid >= 0)

                    # anti_slip_vz_lp is object Z relative to link7 [m/s].
                    slip_mm_s = 1000.0 * float(anti_slip_vz_lp)

                    creep_condition = (
                        contact_present
                        and slip_mm_s <= AUTO_HOLD_SLIP_MM_S
                    )

                    if creep_condition:
                        auto_hold_timer = min(
                            AUTO_HOLD_PERSIST_SEC,
                            auto_hold_timer + dt,
                        )
                    else:
                        # Fast reset is okay here: HOLD should only trigger
                        # after a genuinely persistent downward creep.
                        auto_hold_timer = max(
                            0.0,
                            auto_hold_timer - 2.0 * dt,
                        )

                    hold_ready_shared.value = auto_hold_timer

                    if (
                        auto_hold_timer >= AUTO_HOLD_PERSIST_SEC
                        and auto_bid >= 0
                    ):
                        hold_obj_bid = int(auto_bid)

                        if activate_connect_hold(hold_obj_bid):
                            hold_active = True
                            hold_active_shared.value = True
                            hold_name_shared.value = int(hold_obj_bid)
                            hold_status_code_shared.value = 1

                # Checkbox OFF forces HOLD release and disables auto trigger.
                if not bool(hold_enabled_shared.value):
                    auto_hold_timer = 0.0
                    hold_ready_shared.value = 0.0

                    if hold_active:
                        deactivate_all_holds()
                        hold_obj_bid = -1
                        hold_active = False
                        hold_active_shared.value = False
                        hold_name_shared.value = -1
                        hold_status_code_shared.value = 0

                mujoco.mj_step2(model, data)

                # ------------------------------------------------------
                # V22 REAL CONTACT-FORCE MEASUREMENT
                # Use the solved contact forces from this physics step.
                # These values drive the NEXT 1-ms control iteration.
                # ------------------------------------------------------
                (
                    measured_left_n,
                    measured_right_n,
                    measured_tangent_n,
                ) = measure_pad_contact_forces()

                contact_left_n_shared.value = measured_left_n
                contact_right_n_shared.value = measured_right_n
                contact_tangent_n_shared.value = measured_tangent_n
                contact_motor_cmd_shared.value = motor_clamp_cmd

                # ------------------------------------------------------
                # POST-STEP RUNAWAY GUARD
                # ------------------------------------------------------
                arm_q_now = np.array(
                    [data.qpos[qadr] for qadr in qpos_addr],
                    dtype=float,
                )
                arm_qd_now = np.array(
                    [data.qvel[dadr] for dadr in dof_addr],
                    dtype=float,
                )

                finite_ok = (
                    np.all(np.isfinite(arm_q_now))
                    and np.all(np.isfinite(arm_qd_now))
                )
                speed_ok = np.all(np.abs(arm_qd_now) < 6.0)
                runaway_joint_idx = int(np.argmax(np.abs(arm_qd_now)))
                runaway_joint_speed = float(abs(arm_qd_now[runaway_joint_idx]))

                range_ok = True
                for i in range(7):
                    lo, hi = JOINT_LIMITS_RAD[i]
                    if (
                        arm_q_now[i] < lo - 0.05
                        or arm_q_now[i] > hi + 0.05
                    ):
                        range_ok = False
                        break

                if not (finite_ok and speed_ok and range_ok):
                    ros_motion_fault_latched = True
                    if finite_ok and range_ok:
                        # Excessive velocity but still a numerically valid pose:
                        # freeze in place and preserve the grasp.
                        print(
                            "SAFETY GUARD: high arm speed detected | "
                            f"max=joint{runaway_joint_idx+1}:{runaway_joint_speed:.3f} rad/s | "
                            f"qvel={np.round(arm_qd_now, 3)} -> FREEZE-IN-PLACE + "
                            "MOVEIT COMMAND LATCH",
                            flush=True,
                        )
                        soft_freeze_arm(reason_code=4, count_trip=True)
                    else:
                        # NaN/out-of-range state is a true numerical emergency;
                        # only here do we retain the old hard checkpoint restore.
                        print(
                            "SAFETY GUARD: invalid arm state -> emergency hard restore",
                            flush=True,
                        )
                        hard_restore_arm(
                            last_safe_q,
                            gripper_q=last_safe_gripper_q,
                            reason_code=4,
                        )
                    continue

                # Update safe checkpoint only when there is no meaningful
                # protected-surface penetration and arm speed is moderate.
                penetration_after = protected_penetration_depth()
                if (
                    penetration_after > -0.0005
                    and np.all(np.abs(arm_qd_now) < 2.5)
                    and time.perf_counter() >= safety_cooldown_until
                ):
                    last_safe_q[:] = arm_q_now
                    last_safe_gripper_q = float(
                        np.clip(data.qpos[gripper_qpos_addr], 0.0, 0.05)
                    )
                    if int(safety_state.value) != 0:
                        safety_state.value = 0

                for i, qadr in enumerate(qpos_addr):
                    actual_shared[i] = float(data.qpos[qadr])

                gripper_actual_mm.value = float(
                    np.clip(2.0 * data.qpos[gripper_qpos_addr] * 1000.0, 0.0, 100.0)
                )

                now = time.perf_counter()
                if now - last_viewer_sync >= viewer_period:
                    viewer.sync()
                    last_viewer_sync = now

                next_step_wall += dt
                sleep_time = next_step_wall - time.perf_counter()

                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_step_wall = time.perf_counter()

    except Exception:
        sim_error_code.value = 1
        sim_ready.value = False

        print("\n========== NERO SIMULATION ERROR ==========", flush=True)
        traceback.print_exc()
        print("===========================================\n", flush=True)

        # Keep traceback visible in terminal for diagnosis.
        time.sleep(0.5)


def run_gui(
    sim_process,
    target_shared,
    actual_shared,
    speed_shared,
    wn_shared,
    zeta_shared,
    qacc_shared,
    gravity_enabled,
    gripper_target_mm,
    gripper_actual_mm,
    gripper_force_mode,
    gripper_force_N,
    clamp_speed_mult_shared,
    anti_slip_boost_shared,
    anti_slip_slip_mm_s_shared,
    anti_slip_mu_shared,
    contact_left_n_shared,
    contact_right_n_shared,
    contact_tangent_n_shared,
    contact_motor_cmd_shared,
    hold_enabled_shared,
    hold_active_shared,
    hold_name_shared,
    hold_ready_shared,
    hold_now_request,
    hold_off_request,
    hold_status_code_shared,
    reset_zero_request,
    recover_request,
    safety_trip_count,
    safety_state,
    sim_ready,
    sim_error_code,
    ros_bridge_ready,
    ros_cmd_count,
    ros_feedback_count,
    ros_scene_count,
    ros_last_cmd_age,
    ros_last_cmd_joint_count,
    ros_controller_state_rx_shared,
    ros_fallback_joint_state_rx_shared,
    ros_last_command_source_shared,
    ros_arm_command_gate_shared,
    stop_event,
):
    # Import GUI only in main process.
    import tkinter as tk
    from tkinter import ttk, messagebox

    root = tk.Tk()
    root.title("NERO V2-6 Skill-Contract Agent + V69 Motion Stack")
    root.geometry("1360x930")
    root.minsize(1120, 680)

    # V56: all tabs share one persistent log panel at the bottom.
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    root.rowconfigure(1, weight=0)

    notebook = ttk.Notebook(root)
    notebook.grid(row=0, column=0, sticky="nsew")

    global_log_frame = ttk.LabelFrame(root, text="Unified Runtime Log")
    global_log_frame.grid(
        row=1, column=0, sticky="nsew", padx=8, pady=(2, 8)
    )
    global_log_frame.columnconfigure(0, weight=1)
    global_log_frame.rowconfigure(0, weight=1)

    global_log = tk.Text(
        global_log_frame, height=11, wrap="word", undo=False
    )
    global_log.grid(row=0, column=0, sticky="nsew")
    global_log_scroll = ttk.Scrollbar(
        global_log_frame, orient="vertical", command=global_log.yview
    )
    global_log_scroll.grid(row=0, column=1, sticky="ns")
    global_log.configure(yscrollcommand=global_log_scroll.set)
    global_log.insert(
        "end",
        "NERO V2-6 unified runtime log. V69 motion + BioIK/TRAC + Skill Contract / Agent messages appear here.\n\n",
    )

    def append_gui_log(text):
        global_log.insert("end", str(text).rstrip() + "\n")
        global_log.see("end")

    basic_tab = ttk.Frame(notebook)
    skill_tab = ttk.Frame(notebook)
    bio_tab = ttk.Frame(notebook)
    trac_tab = ttk.Frame(notebook)
    llm_tab = ttk.Frame(notebook)

    notebook.add(basic_tab, text="Basic Controls")
    notebook.add(skill_tab, text="Skill Test")
    notebook.add(bio_tab, text="BioIK Primary")
    notebook.add(trac_tab, text="TRAC-IK Reference")
    notebook.add(llm_tab, text="NERO V2-6 | Agent")

    skill_core = None
    skill_core_error = ""
    try:
        from NERO_V69_ReachableZone_BioIK_Planner_Core import (
            NeroMoveItCore, OBJECTS, BINS, OBJECT_ZONE,
            GRASP_Z_OFFSET, DEFAULT_GRASP_Z_OFFSET, BIN_RELEASE_CLEARANCE,
        )

        def set_ros_arm_gate(enabled):
            ros_arm_command_gate_shared.value = bool(enabled)

        def get_safety_trip_count():
            return int(safety_trip_count.value)

        def request_soft_recover():
            # V69 compatibility callback. Automatic execution no longer calls
            # this on safety trips; only explicit recovery actions may unlatch.
            recover_request.value = True

        skill_core = NeroMoveItCore(
            set_arm_command_gate=set_ros_arm_gate,
            get_safety_trip_count=get_safety_trip_count,
            request_soft_recover=request_soft_recover,
        )
        skill_core.start()
    except Exception as exc:
        OBJECTS = [
            "red_cube", "blue_cube", "red_ball", "blue_ball",
        ]
        skill_core_error = str(exc)

    def take_manual_arm_control(reason="manual arm control"):
        ros_arm_command_gate_shared.value = False

        if skill_core is not None:
            try:
                skill_core.force_recover(f"MANUAL ARM — {reason}")
            except Exception:
                try:
                    skill_core.cancel_active_motion()
                except Exception:
                    pass

    def take_manual_gripper_control(reason="manual gripper control"):
        if skill_core is not None:
            try:
                skill_core.emit(
                    f"[MODE] GRIPPER MANUAL — {reason}; "
                    "arm grasp branch preserved"
                )
            except Exception:
                pass

    # Make scrolling possible on smaller displays.
    outer = ttk.Frame(basic_tab)
    outer.pack(fill="both", expand=True)

    canvas = tk.Canvas(outer, highlightthickness=0)
    scrollbar = ttk.Scrollbar(
        outer,
        orient="vertical",
        command=canvas.yview,
    )
    content = ttk.Frame(canvas)

    content.bind(
        "<Configure>",
        lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")
        ),
    )

    canvas.create_window(
        (0, 0),
        window=content,
        anchor="nw",
    )
    canvas.configure(yscrollcommand=scrollbar.set)

    canvas.pack(
        side="left",
        fill="both",
        expand=True,
    )
    scrollbar.pack(
        side="right",
        fill="y",
    )

    title = ttk.Label(
        content,
        text="NERO V2-6 Skill-Contract Agent + V69 Motion Stack",
        font=("Arial", 18, "bold"),
    )
    title.pack(pady=(10, 4))

    status_var = tk.StringVar(value="Starting simulation...")
    status = ttk.Label(
        content,
        textvariable=status_var,
        font=("Arial", 11, "bold"),
    )
    status.pack(pady=(0, 8))

    # --------------------------------------------------------
    # V44 MoveIt2 bridge status
    # --------------------------------------------------------
    moveit_box = ttk.LabelFrame(
        content,
        text="MoveIt2 / ROS2 Bridge",
    )
    moveit_box.pack(
        fill="x",
        padx=12,
        pady=(2, 8),
    )

    moveit_status_var = tk.StringVar(
        value="ROS2 bridge: starting..."
    )
    ttk.Label(
        moveit_box,
        textvariable=moveit_status_var,
        font=("Arial", 10, "bold"),
    ).pack(
        anchor="w",
        padx=8,
        pady=(5, 2),
    )

    ttk.Label(
        moveit_box,
        text=(
            "MoveIt2 owns IK/path planning/collision avoidance. "
            "MuJoCo/V31 executes /arm_controller/controller_state (primary) or /control/joint_states (fallback), and publishes "
            "/feedback/joint_states. PlanningScene includes table + sorting bins."
        ),
    ).pack(
        anchor="w",
        padx=8,
        pady=(0, 5),
    )

    top = ttk.Frame(content)
    top.pack(fill="x", padx=12, pady=4)

    gravity_var = tk.BooleanVar(value=True)

    def gravity_callback():
        gravity_enabled.value = bool(gravity_var.get())

    gravity_check = ttk.Checkbutton(
        top,
        text="Gravity ON",
        variable=gravity_var,
        command=gravity_callback,
    )
    gravity_check.pack(side="left", padx=(0, 20))

    ttk.Label(
        top,
        text="Speed: 20% = original V2 speed | 100% = 5× V2 reference speed",
    ).pack(side="left")

    slider_vars = []
    target_labels = []
    actual_labels = []
    speed_vars = []
    wn_vars = []
    zeta_vars = []
    qacc_vars = []
    gain_labels = []

    joints_frame = ttk.Frame(content)
    joints_frame.pack(fill="both", expand=True, padx=12, pady=8)

    def update_gain_label(i):
        try:
            wn = float(wn_vars[i].get())
            zeta = float(zeta_vars[i].get())
            kp = wn * wn
            kd = 2.0 * zeta * wn
            gain_labels[i].configure(
                text=f"Kp≈{kp:.2f}  Kd≈{kd:.2f}"
            )
        except Exception:
            pass
    def slider_command(i, value):
        try:
            take_manual_arm_control(f"Joint {i+1} slider")
            deg = float(value)
            target_shared[i] = math.radians(deg)
            target_labels[i].configure(
                text=f"Target {deg:.1f}°"
            )
        except Exception:
            pass

    for i in range(7):
        lo, hi = JOINT_LIMITS_RAD[i]
        lo_deg = math.degrees(lo)
        hi_deg = math.degrees(hi)

        box = ttk.LabelFrame(
            joints_frame,
            text=f"Slider {i+1} → Joint {i+1}",
        )
        box.pack(fill="x", pady=5)

        box.columnconfigure(0, weight=1)

        s_var = tk.DoubleVar(value=0.0)
        slider = tk.Scale(
            box,
            from_=lo_deg,
            to=hi_deg,
            orient="horizontal",
            resolution=0.1,
            variable=s_var,
            command=lambda value, idx=i: slider_command(idx, value),
            length=520,
        )
        slider.grid(
            row=0,
            column=0,
            rowspan=2,
            sticky="ew",
            padx=(8, 10),
            pady=5,
        )

        t_label = ttk.Label(box, text="Target 0.0°", width=15)
        t_label.grid(row=0, column=1, padx=4)

        a_label = ttk.Label(box, text="Actual 0.0°", width=15)
        a_label.grid(row=1, column=1, padx=4)

        ttk.Label(box, text="Speed %").grid(row=0, column=2)
        speed_var = tk.StringVar(value="20")
        speed_spin = ttk.Spinbox(
            box,
            from_=1,
            to=100,
            increment=1,
            width=7,
            textvariable=speed_var,
        )
        speed_spin.grid(row=1, column=2, padx=4)

        ttk.Label(box, text="Wn").grid(row=0, column=3)
        wn_var = tk.StringVar(value="4.00")
        wn_spin = ttk.Spinbox(
            box,
            from_=0.05,
            to=50.0,
            increment=0.25,
            width=7,
            textvariable=wn_var,
        )
        wn_spin.grid(row=1, column=3, padx=4)

        ttk.Label(box, text="Zeta").grid(row=0, column=4)
        zeta_var = tk.StringVar(value="1.25")
        zeta_spin = ttk.Spinbox(
            box,
            from_=0.05,
            to=5.0,
            increment=0.05,
            width=7,
            textvariable=zeta_var,
        )
        zeta_spin.grid(row=1, column=4, padx=4)

        ttk.Label(box, text="Accel").grid(row=0, column=5)
        qacc_var = tk.StringVar(
            value=f"{DEFAULT_MAX_QACC[i]:.2f}"
        )
        qacc_spin = ttk.Spinbox(
            box,
            from_=0.05,
            to=200.0,
            increment=0.5,
            width=8,
            textvariable=qacc_var,
        )
        qacc_spin.grid(row=1, column=5, padx=4)

        gain_label = ttk.Label(
            box,
            text="Kp≈16.00  Kd≈10.00",
            width=23,
        )
        gain_label.grid(
            row=1,
            column=6,
            padx=(8, 8),
        )

        ttk.Label(
            box,
            text=f"Range {lo_deg:.1f}° … {hi_deg:.1f}°",
        ).grid(
            row=0,
            column=6,
            padx=(8, 8),
        )

        slider_vars.append(s_var)
        target_labels.append(t_label)
        actual_labels.append(a_label)
        speed_vars.append(speed_var)
        wn_vars.append(wn_var)
        zeta_vars.append(zeta_var)
        qacc_vars.append(qacc_var)
        gain_labels.append(gain_label)

    # --------------------------------------------------------
    # Original AgileX two-finger gripper
    # --------------------------------------------------------
    gripper_box = ttk.LabelFrame(
        content,
        text="Original AgileX Gripper",
    )
    gripper_box.pack(fill="x", padx=12, pady=(4, 8))

    gripper_box.columnconfigure(1, weight=1)

    ttk.Label(
        gripper_box,
        text="Opening",
        width=12,
    ).grid(row=0, column=0, padx=8, pady=8)

    gripper_var = tk.DoubleVar(value=0.0)

    def gripper_slider_command(value):
        try:
            take_manual_gripper_control("gripper slider")
            mm = max(0.0, min(100.0, float(value)))
            # Moving the manual slider always returns to position mode.
            gripper_force_mode.value = False
            gripper_target_mm.value = mm
            gripper_target_label.configure(text=f"Target {mm:.0f} mm")
            gripper_mode_label.configure(text="Mode: POSITION")
        except Exception:
            pass

    gripper_slider = tk.Scale(
        gripper_box,
        from_=0,
        to=100,
        orient="horizontal",
        resolution=1,
        variable=gripper_var,
        command=gripper_slider_command,
        length=650,
    )
    gripper_slider.grid(
        row=0,
        column=1,
        sticky="ew",
        padx=8,
        pady=8,
    )

    gripper_target_label = ttk.Label(
        gripper_box,
        text="Target 0 mm",
        width=16,
    )
    gripper_target_label.grid(row=0, column=2, padx=5)

    gripper_actual_label = ttk.Label(
        gripper_box,
        text="Actual 0 mm",
        width=16,
    )
    gripper_actual_label.grid(row=0, column=3, padx=5)

    ttk.Label(
        gripper_box,
        text="0 mm = closed    100 mm = fully open",
    ).grid(
        row=1,
        column=1,
        sticky="w",
        padx=8,
        pady=(0, 8),
    )

    # --------------------------------------------------------
    # V16 force-control panel
    # --------------------------------------------------------
    force_frame = ttk.Frame(gripper_box)
    force_frame.grid(
        row=2,
        column=0,
        columnspan=4,
        sticky="w",
        padx=8,
        pady=(4, 10),
    )

    ttk.Label(
        force_frame,
        text="Target normal force / finger (N):",
    ).pack(side="left", padx=(0, 6))

    gripper_force_var = tk.StringVar(value="5.0")
    gripper_force_spin = ttk.Spinbox(
        force_frame,
        from_=0.5,
        to=50.0,
        increment=0.5,
        width=8,
        textvariable=gripper_force_var,
    )
    gripper_force_spin.pack(side="left", padx=(0, 12))

    ttk.Label(
        force_frame,
        text="Clamp speed ×:",
    ).pack(side="left", padx=(0, 5))

    clamp_speed_var = tk.StringVar(value="15.0")
    clamp_speed_spin = ttk.Spinbox(
        force_frame,
        from_=1.0,
        to=20.0,
        increment=0.5,
        width=6,
        textvariable=clamp_speed_var,
    )
    clamp_speed_spin.pack(side="left", padx=(0, 12))

    gripper_mode_label = ttk.Label(
        force_frame,
        text="Mode: POSITION",
        width=38,
    )
    gripper_mode_label.pack(side="left", padx=(0, 12))

    def one_click_clamp():
        take_manual_gripper_control("Gripper FAST CLAMP")
        try:
            force_n = float(gripper_force_var.get())
        except Exception:
            force_n = 5.0
            gripper_force_var.set("5.0")

        force_n = max(0.5, min(50.0, force_n))
        gripper_force_N.value = force_n

        try:
            speed_mult = float(clamp_speed_var.get())
        except Exception:
            speed_mult = 15.0
            clamp_speed_var.set("15.0")

        speed_mult = max(1.0, min(20.0, speed_mult))
        clamp_speed_mult_shared.value = speed_mult

        gripper_force_mode.value = True

        gripper_mode_label.configure(
            text=(
                f"Mode: FORCE LOOP → {force_n:.1f} N/finger "
                f"| clamp {speed_mult:.1f}×"
            )
        )
        gripper_target_label.configure(
            text=f"Target {force_n:.1f} N/finger"
        )

    def one_click_release():
        take_manual_gripper_control("Gripper OPEN")
        # Release uses position mode and commands full opening.
        gripper_force_mode.value = False
        gripper_target_mm.value = 100.0
        gripper_var.set(100.0)

        gripper_mode_label.configure(text="Mode: RELEASE")
        gripper_target_label.configure(text="Target 100 mm")

    ttk.Button(
        force_frame,
        text="一键夹紧 / CLAMP",
        command=one_click_clamp,
    ).pack(side="left", padx=5)

    ttk.Button(
        force_frame,
        text="一键松开 / RELEASE",
        command=one_click_release,
    ).pack(side="left", padx=5)

    hold_var = tk.BooleanVar(value=True)

    def hold_toggle():
        hold_enabled_shared.value = bool(hold_var.get())

    ttk.Checkbutton(
        force_frame,
        text="Enable Auto HOLD",
        variable=hold_var,
        command=hold_toggle,
    ).pack(side="left", padx=(14, 5))

    def hold_now():
        hold_now_request.value = True

    ttk.Button(
        force_frame,
        text="HOLD ON",
        command=hold_now,
    ).pack(side="left", padx=5)

    def hold_off():
        hold_off_request.value = True

    ttk.Button(
        force_frame,
        text="HOLD OFF",
        command=hold_off,
    ).pack(side="left", padx=5)

    hold_enabled_shared.value = True

    ttk.Label(
        gripper_box,
        text=(
            "V31 clamp speed: selected multiplier applies in free space; "
            "first fingertip contact automatically brakes to 1.5× before "
            "the V22 force loop takes over."
        ),
    ).grid(
        row=3,
        column=0,
        columnspan=4,
        sticky="w",
        padx=8,
        pady=(0, 3),
    )

    ttk.Label(
        gripper_box,
        text="V22 closed loop: GUI target = measured normal contact force PER "
             "FINGER. Friction stays fixed at μ=10; no NoSlip/Newton tricks.",
    ).grid(
        row=4,
        column=0,
        columnspan=4,
        sticky="w",
        padx=8,
        pady=(0, 4),
    )

    contact_force_status_label = ttk.Label(
        gripper_box,
        text=(
            "Contact: L 0.00 N | R 0.00 N | Tangential 0.00 N | "
            "Motor 0.00 N"
        ),
    )
    contact_force_status_label.grid(
        row=5,
        column=0,
        columnspan=4,
        sticky="w",
        padx=8,
        pady=(0, 3),
    )

    anti_slip_status_label = ttk.Label(
        gripper_box,
        text="Slip: +0.00 mm/s | target boost +0.00 N | pad μ fixed 10.00",
    )
    anti_slip_status_label.grid(
        row=6,
        column=0,
        columnspan=4,
        sticky="w",
        padx=8,
        pady=(0, 8),
    )

    hold_status_label = ttk.Label(
        gripper_box,
        text="Auto HOLD: MONITORING creep | manual HOLD ON available",
    )
    hold_status_label.grid(
        row=7,
        column=0,
        columnspan=4,
        sticky="w",
        padx=8,
        pady=(0, 8),
    )

    bottom = ttk.Frame(content)
    bottom.pack(fill="x", padx=12, pady=(5, 15))

    def zero_all():
        take_manual_arm_control("ZERO ALL")
        for i in range(7):
            slider_vars[i].set(0.0)
            target_shared[i] = 0.0
            target_labels[i].configure(text="Target 0.0°")
        # V2-6 episode semantics: ZERO ALL is a fresh-task boundary when
        # no object is being held.  In V2-5.1 only the physical joint targets
        # were zeroed, leaving HOVER_REACHED / retry memory stale.
        if not bool(hold_active_shared.value):
            _v24_reset_skill_state("ZERO ALL", clear_adaptation=True)
        else:
            append_gui_log(
                "[V2-6 STATE RESET SKIPPED] ZERO ALL while hold_active=true; "
                "logical grasp state preserved"
            )

    ttk.Button(
        bottom,
        text="Zero All",
        command=zero_all,
    ).pack(side="left", padx=(0, 8))

    def emergency_recover():
        take_manual_arm_control("STOP + RECOVER")
        recover_request.value = True

    def hard_reset_zero():
        take_manual_arm_control("HARD RESET 0")
        gripper_force_mode.value = False
        reset_zero_request.value = True
        # HARD RESET is an explicit full episode reset.
        _v24_reset_skill_state("HARD RESET 0", clear_adaptation=True)
        # GUI is updated immediately; sim process then performs the
        # hard state reset and publishes the final actual values.
        for i in range(7):
            slider_vars[i].set(0.0)
            target_labels[i].configure(text="Target 0.0°")
        gripper_var.set(80.0)
        gripper_target_label.configure(text="Target 80 mm")

    ttk.Button(
        bottom,
        text="STOP + RECOVER",
        command=emergency_recover,
    ).pack(side="left", padx=8)

    ttk.Button(
        bottom,
        text="HARD RESET 0",
        command=hard_reset_zero,
    ).pack(side="left", padx=8)

    ttk.Button(
        bottom,
        text="Quit",
        command=root.destroy,
    ).pack(side="right")

    # ========================================================
    # V43 UNIFIED SKILL TEST TAB
    # ========================================================
    skill_outer = ttk.Frame(skill_tab)
    skill_outer.pack(fill="both", expand=True, padx=12, pady=12)

    skill_title = ttk.Label(
        skill_outer,
        text="MoveIt2 Skill Test",
        font=("Arial", 18, "bold"),
    )
    skill_title.pack(anchor="w", pady=(0, 5))

    skill_ready_var = tk.StringVar(
        value=(
            "Skill core starting..."
            if skill_core is not None
            else f"Skill core ERROR: {skill_core_error}"
        )
    )
    ttk.Label(
        skill_outer,
        textvariable=skill_ready_var,
        font=("Arial", 10, "bold"),
    ).pack(anchor="w", pady=(0, 3))

    control_mode_var = tk.StringVar(value="Arm input: MANUAL/IDLE")
    ttk.Label(
        skill_outer,
        textvariable=control_mode_var,
        font=("Arial", 10, "bold"),
    ).pack(anchor="w", pady=(0, 8))

    top_skill_row = ttk.Frame(skill_outer)
    top_skill_row.pack(fill="x", pady=(0, 8))

    ttk.Label(top_skill_row, text="Object:").pack(side="left")
    skill_object_var = tk.StringVar(value="red_cube")
    ttk.Combobox(
        top_skill_row,
        textvariable=skill_object_var,
        values=OBJECTS,
        state="readonly",
        width=22,
    ).pack(side="left", padx=(5, 12))

    MOTION_METHODS_REQUIRING_ARM_CONTROLLER = {
        "move_above", "approach", "lift", "move_to",
        "bio_move_above", "bio_approach", "bio_lift", "bio_move_above_bin",
        "trac_move_above", "trac_approach", "trac_lift", "trac_move_above_bin",
    }

    def submit_skill(method_name, *args):
        if skill_core is None:
            append_gui_log(f"[UI ERROR] Skill core unavailable: {skill_core_error}")
            return

        # V69: never plan a motion that cannot be executed.
        if method_name in MOTION_METHODS_REQUIRING_ARM_CONTROLLER:
            try:
                arm_ready_now = bool(skill_core.arm_controller_client.server_is_ready())
            except Exception:
                arm_ready_now = False
            if not arm_ready_now:
                append_gui_log(
                    "[UI BLOCKED] Root arm_controller is not ACTIVE: "
                    "/arm_controller/follow_joint_trajectory unavailable. "
                    "Motion was NOT planned or executed. Check launcher/controller status."
                )
                return

        append_gui_log(f"[UI] {method_name}{args}")
        try:
            method = getattr(skill_core, method_name)
            accepted = skill_core.submit(method, *args)
            if not accepted:
                append_gui_log("[UI] Skill request rejected because core is BUSY")
        except Exception as exc:
            append_gui_log(f"[UI ERROR] Could not submit {method_name}: {exc}")

    ttk.Button(
        top_skill_row,
        text="Observe Scene",
        command=lambda: submit_skill("observe_scene"),
    ).pack(side="left", padx=3)

    ttk.Button(
        top_skill_row,
        text="Move Above",
        command=lambda: submit_skill(
            "move_above", skill_object_var.get(), 0.12
        ),
    ).pack(side="left", padx=3)

    ttk.Button(
        top_skill_row,
        text="Approach",
        command=lambda: submit_skill(
            "approach", skill_object_var.get(), None
        ),
    ).pack(side="left", padx=3)

    ttk.Button(
        top_skill_row,
        text="Lift 100 mm",
        command=lambda: submit_skill("lift", 0.10),
    ).pack(side="left", padx=3)


    bin_skill_row = ttk.Frame(skill_outer)
    bin_skill_row.pack(fill="x", pady=(0, 8))
    ttk.Label(bin_skill_row, text="Release bin:").pack(side="left")
    skill_bin_var = tk.StringVar(value="red_bin")
    ttk.Combobox(
        bin_skill_row,
        textvariable=skill_bin_var,
        values=("red_bin", "blue_bin"),
        state="readonly",
        width=14,
    ).pack(side="left", padx=(5, 10))
    ttk.Button(
        bin_skill_row,
        text="Move Above Bin",
        command=lambda: submit_skill(
            "move_above_bin", skill_bin_var.get()
        ),
    ).pack(side="left", padx=3)
    ttk.Label(
        bin_skill_row,
        text="(positions gripper for release; does NOT open gripper)",
    ).pack(side="left", padx=(8, 0))

    # Requested local utility buttons beside Skill Test.
    util_box = ttk.LabelFrame(skill_outer, text="Local Robot Utilities")
    util_box.pack(fill="x", pady=(0, 8))

    ttk.Button(
        util_box,
        text="ZERO ALL",
        command=zero_all,
    ).pack(side="left", padx=6, pady=7)

    ttk.Button(
        util_box,
        text="Gripper OPEN",
        command=one_click_release,
    ).pack(side="left", padx=6, pady=7)

    ttk.Button(
        util_box,
        text="Gripper FAST CLAMP",
        command=one_click_clamp,
    ).pack(side="left", padx=6, pady=7)

    def unified_stop_recover():
        emergency_recover()

    def unified_hard_reset():
        hard_reset_zero()

    ttk.Button(
        util_box,
        text="STOP + RECOVER",
        command=unified_stop_recover,
    ).pack(side="left", padx=6, pady=7)

    ttk.Button(
        util_box,
        text="HARD RESET 0",
        command=unified_hard_reset,
    ).pack(side="left", padx=6, pady=7)

    xyz_box = ttk.LabelFrame(skill_outer, text="Move To World XYZ")
    xyz_box.pack(fill="x", pady=(0, 8))

    skill_x_var = tk.StringVar(value="-0.45")
    skill_y_var = tk.StringVar(value="0.00")
    skill_z_var = tk.StringVar(value="0.25")

    for lab, var in (("X", skill_x_var), ("Y", skill_y_var), ("Z", skill_z_var)):
        ttk.Label(xyz_box, text=lab).pack(side="left", padx=(8, 2), pady=8)
        ttk.Entry(xyz_box, textvariable=var, width=10).pack(side="left")

    def skill_move_to():
        try:
            xyz = (
                float(skill_x_var.get()),
                float(skill_y_var.get()),
                float(skill_z_var.get()),
            )
        except Exception:
            if skill_core is not None:
                skill_core.emit("Invalid XYZ")
            return
        submit_skill("move_to", *xyz)

    ttk.Button(
        xyz_box,
        text="MOVE TO",
        command=skill_move_to,
    ).pack(side="left", padx=12)

    # ========================================================
    # NERO V2-6 SKILL-CONTRACT TASK AGENT TAB
    # High-level agent only.  V69 remains the authoritative motion/safety layer.
    # ========================================================
    llm_outer = ttk.Frame(llm_tab)
    llm_outer.pack(fill="both", expand=True, padx=12, pady=12)

    ttk.Label(
        llm_outer,
        text="NERO V2-6 | Gripper-Aware Skill Contract Agent",
        font=("Arial", 18, "bold"),
    ).pack(anchor="w", pady=(0, 5))

    ttk.Label(
        llm_outer,
        text=(
            "The LLM is the high-level task planner and never commands joints directly. "
            "V2-6 adds continuous gripper-aperture awareness and a lightweight manipulation prior: "
            "observe_scene reports actual opening plus approximate object grasp width/recommended range; "
            "the Agent must prepare aperture before approaching an object. V69 motion/safety remains frozen."
        ),
        wraplength=1260,
        justify="left",
    ).pack(anchor="w", pady=(0, 8))

    llm_status_var = tk.StringVar(value="LLM agent: initializing...")
    ttk.Label(
        llm_outer,
        textvariable=llm_status_var,
        font=("Arial", 10, "bold"),
    ).pack(anchor="w", pady=(0, 8))

    llm_agent = None
    llm_agent_error = ""
    try:
        from NERO_V2_6_Agent import NeroLLMAgent, DEFAULT_MODEL, TOOLS as LLM_TOOLS
    except Exception as exc:
        DEFAULT_MODEL = os.environ.get("NERO_LLM_MODEL", "gpt-5.6-luna")
        LLM_TOOLS = []
        llm_agent_error = str(exc)
        append_gui_log(f"[LLM INIT ERROR] {exc}")

    # ----- Exact MuJoCo state exposed to LLM -----
    def llm_classify_object_location(xyz):
        x, y, z = [float(v) for v in xyz]
        # Physical V69 bin inner opening is ~144 x 124 mm.  Use a modest
        # center-margin so an object on/above a wall is not falsely verified.
        for bin_name, (bx, by, _rim_z) in BINS.items():
            if abs(x - float(bx)) <= 0.067 and abs(y - float(by)) <= 0.057 and z <= 0.10:
                return bin_name
        if (
            float(OBJECT_ZONE["x_min"]) <= x <= float(OBJECT_ZONE["x_max"])
            and float(OBJECT_ZONE["y_min"]) <= y <= float(OBJECT_ZONE["y_max"])
            and z <= 0.10
        ):
            return "object_zone"
        return "table_or_transit"

    # --------------------------------------------------------
    # V2-6 continuous gripper/object geometry + manipulation prior
    # --------------------------------------------------------
    # Current V69 scene geometry: cubes are 45 mm; balls are 52 mm diameter.
    # V2-6 intentionally keeps this first prior simple and explicit so later
    # experiments can replace/tune it with learned experience.
    V24_OBJECT_GEOMETRY = {
        "red_cube":  {"shape": "cube",   "approx_size_mm": [45.0, 45.0, 45.0], "effective_width_mm": 45.0},
        "blue_cube": {"shape": "cube",   "approx_size_mm": [45.0, 45.0, 45.0], "effective_width_mm": 45.0},
        "red_ball":  {"shape": "sphere", "approx_size_mm": [52.0, 52.0, 52.0], "effective_width_mm": 52.0},
        "blue_ball": {"shape": "sphere", "approx_size_mm": [52.0, 52.0, 52.0], "effective_width_mm": 52.0},
    }

    # V2-6 anti-poke adaptation memory.  This is intentionally session-local:
    # if a pre-grasp approach visibly disturbs an object, the manipulation prior
    # widens for that object on the next observation/retry instead of blindly
    # repeating the same aperture.
    V241_APERTURE_RETRY_BONUS_MM = {name: 0.0 for name in V24_OBJECT_GEOMETRY}
    # V2-6: one non-safety planning adaptation per object.  This is separate
    # from the physical safety-latch retry budget.
    V251_PLANNING_RETRY_USED = {name: 0 for name in V24_OBJECT_GEOMETRY}
    V251_GRASP_Z_RETRY_BONUS_M = {name: 0.0 for name in V24_OBJECT_GEOMETRY}

    # V2-6 reflective safety-retry memory.  A pre-grasp safety trip is not
    # automatically ignored or automatically unlatched.  Instead it becomes a
    # structured failure context for the Agent.  The Agent may explicitly
    # choose ONE recovery/retry per object.  This preserves V69's latched-safety
    # behavior while allowing reasoned recovery instead of immediate task abort.
    V242_SAFETY_RETRY_USED = {name: 0 for name in V24_OBJECT_GEOMETRY}
    V242_LAST_SAFETY_CONTEXT = {
        "active": False,
        "tool": None,
        "object_name": None,
        "trip_count": 0,
        "safety_state_code": 0,
        "likely_cause": None,
        "evidence": [],
        "suggested_strategy": None,
        "suggested_retry_opening_mm": None,
        "retry_allowed": False,
    }

    def _v242_set_safety_context(**kwargs):
        V242_LAST_SAFETY_CONTEXT.update({
            "active": False,
            "tool": None,
            "object_name": None,
            "trip_count": int(safety_trip_count.value),
            "safety_state_code": int(safety_state.value),
            "likely_cause": None,
            "evidence": [],
            "suggested_strategy": None,
            "suggested_retry_opening_mm": None,
            "retry_allowed": False,
        })
        V242_LAST_SAFETY_CONTEXT.update(kwargs)
        return dict(V242_LAST_SAFETY_CONTEXT)

    def _v242_clear_safety_context():
        return _v242_set_safety_context(active=False)

    # ------------------------------------------------------------------
    # Manipulation prior
    # ------------------------------------------------------------------
    # These margins encode simple, inspectable engineering experience: the
    # gripper should approach with meaningful lateral clearance rather than an
    # aperture that is merely equal to the nominal object width.  The prior is
    # intentionally expressed in millimeters so future experiments can replace
    # the constants with measured success statistics or a learned policy.
    #
    # IMPORTANT: this is NOT collision checking.  MoveIt still performs its own
    # geometric feasibility test; the prior only proposes a more robust starting
    # aperture for that check.
    def _v24_gripper_prior(object_name, actual_opening_mm=None):
        geom = dict(V24_OBJECT_GEOMETRY.get(str(object_name), {}))
        width = float(geom.get("effective_width_mm", 50.0))
        shape = str(geom.get("shape", "unknown"))

        # V2-6 shape-aware embodied manipulation prior.
        #
        # The V2-6 run showed that a 52 mm ball with a nominal 60 mm opening
        # still moved ~13 mm during the straight approach.  The joint opening
        # is therefore not equivalent to useful free clearance at the rubber
        # pad/stud collision envelope.  Spheres get a deliberately larger
        # pre-grasp aperture than box-like objects.
        #
        # These are empirical priors, not physical laws.  They are logged and
        # can later be replaced by a learned success model.
        if shape == "sphere":
            base_min_margin = 16.0      # 52 mm ball -> >=68 mm opening
            base_target_margin = 20.0   # preferred 72 mm
            base_max_margin = 28.0      # avoid needless full-open unless useful
            envelope_allowance = 8.0   # approximate pad/stud collision envelope
            policy = "sphere anti-poke prior: +16/+20/+28 mm total aperture margin"
        else:
            # V2-6: 53 mm on a 45 mm cube left only ~2 mm useful free
            # clearance per side after the pad envelope.  Use a more human-like
            # manipulation prior: clearly wider than the object, but not full-open.
            base_min_margin = 13.0      # 45 mm cube -> >=58 mm opening
            base_target_margin = 15.0   # preferred 60 mm
            base_max_margin = 21.0      # robust range ends at 66 mm
            envelope_allowance = 4.0
            policy = "box anti-poke prior: +13/+15/+21 mm total aperture margin"

        retry_bonus = float(V241_APERTURE_RETRY_BONUS_MM.get(str(object_name), 0.0))
        rec_min = min(100.0, width + base_min_margin + retry_bonus)
        rec_target = min(100.0, width + base_target_margin + retry_bonus)
        rec_max = min(100.0, width + base_max_margin + retry_bonus)
        result = {
            "shape": shape,
            "approx_size_mm": geom.get("approx_size_mm", [width, width, width]),
            "effective_width_mm": round(width, 2),
            "recommended_opening_range_mm": [round(rec_min, 2), round(rec_max, 2)],
            "recommended_target_opening_mm": round(rec_target, 2),
            "recommended_total_clearance_mm": round(rec_target - width, 2),
            "estimated_pad_envelope_allowance_total_mm": round(envelope_allowance, 2),
            "estimated_free_clearance_at_target_mm": round(max(0.0, rec_target - width - envelope_allowance), 2),
            "adaptive_retry_bonus_mm": round(retry_bonus, 2),
            "prior_basis": policy + "; +8 mm after detected poke; +15 mm after a pre-grasp safety-stop retry",
        }
        if actual_opening_mm is not None:
            opening = float(actual_opening_mm)
            result["current_opening_mm"] = round(opening, 2)
            result["geometric_clearance_total_mm"] = round(opening - width, 2)
            result["estimated_free_clearance_total_mm"] = round(opening - width - envelope_allowance, 2)
            result["estimated_free_clearance_per_side_mm"] = round((opening - width - envelope_allowance) / 2.0, 2)
            result["distance_to_recommended_target_mm"] = round(opening - rec_target, 2)
        return result

    def llm_scene_snapshot():
        if skill_core is None:
            raise RuntimeError(f"V69 skill core unavailable: {skill_core_error}")
        tcp_xyz, tcp_q = skill_core.current_tcp_pose()
        objects = {}
        for name in OBJECTS:
            try:
                xyz, quat = skill_core.object_pose(name)
                opening_now = float(gripper_actual_mm.value)
                objects[name] = {
                    "xyz_m": [round(float(v), 5) for v in xyz],
                    "quat_xyzw": [round(float(v), 5) for v in quat],
                    "location": llm_classify_object_location(xyz),
                    "grasp_geometry": _v24_gripper_prior(name, opening_now),
                }
            except Exception as exc:
                objects[name] = {"error": str(exc), "location": "unknown"}
        return {
            "tcp": {
                "xyz_m": [round(float(v), 5) for v in tcp_xyz],
                "quat_xyzw": [round(float(v), 5) for v in tcp_q],
            },
            "objects": objects,
            "bins": {
                name: {"center_xyz_m": [round(float(v), 5) for v in vals]}
                for name, vals in BINS.items()
            },
            "object_zone": {
                "x_min": float(OBJECT_ZONE["x_min"]),
                "x_max": float(OBJECT_ZONE["x_max"]),
                "y_min": float(OBJECT_ZONE["y_min"]),
                "y_max": float(OBJECT_ZONE["y_max"]),
            },
            "gripper": {
                "actual_opening_mm": round(float(gripper_actual_mm.value), 2),
                "position_target_opening_mm": round(float(gripper_target_mm.value), 2),
                "force_mode": bool(gripper_force_mode.value),
                "target_force_per_finger_N": round(float(gripper_force_N.value), 3),
                "contact_left_N": round(float(contact_left_n_shared.value), 3),
                "contact_right_N": round(float(contact_right_n_shared.value), 3),
                "hold_enabled": bool(hold_enabled_shared.value),
                "hold_active": bool(hold_active_shared.value),
            },
            "safety": {
                "trip_count": int(safety_trip_count.value),
                "state_code": int(safety_state.value),
            },
            "safety_recovery": {
                "last_context": dict(V242_LAST_SAFETY_CONTEXT),
                "retry_used_by_object": dict(V242_SAFETY_RETRY_USED),
                "planning_retry_used_by_object": dict(V251_PLANNING_RETRY_USED),
                "grasp_z_retry_bonus_mm_by_object": {k: round(float(v) * 1000.0, 1) for k, v in V251_GRASP_Z_RETRY_BONUS_M.items()},
                "policy": "one explicit safety recovery + one non-safety planning adaptation per object",
            },
        }

    def llm_require_arm_controller():
        try:
            ready = bool(skill_core.arm_controller_client.server_is_ready())
        except Exception:
            ready = False
        if not ready:
            raise RuntimeError("/arm_controller/follow_joint_trajectory unavailable")

    # ========================================================
    # NERO V2-6 SKILL CONTRACT + REFLECTIVE SAFETY RECOVERY
    # ========================================================
    # V69 remains byte-for-byte frozen. V2-6 wraps each V69 skill in a task-
    # semantic contract. The Agent sees the same semantics in its tool schema
    # BEFORE choosing an action. After execution, deterministic MuJoCo state
    # verifies the contract and returns a normalized result.

    V24_STAGE_IDLE = "IDLE"
    V24_STAGE_OBSERVED = "OBSERVED"
    V24_STAGE_HOVER = "HOVER_REACHED"
    V24_STAGE_GRASP_POSE = "GRASP_POSE_REACHED"
    V24_STAGE_GRASPED = "GRASP_ESTABLISHED"
    V24_STAGE_LIFTED = "LIFTED"
    V24_STAGE_BIN_HOVER = "BIN_HOVER_REACHED"
    V24_STAGE_RELEASED = "RELEASED"
    V24_STAGE_VERIFIED = "PLACEMENT_VERIFIED"
    V24_STAGE_SAFETY = "SAFETY_STOPPED"

    v24_state_lock = threading.RLock()
    v24_skill_state = {
        "stage": V24_STAGE_IDLE,
        "active_object": None,
        "target_bin": None,
        "last_tool": None,
        "last_ok": None,
    }

    def _v24_state_snapshot():
        with v24_state_lock:
            return dict(v24_skill_state)

    def _v24_set_state(stage=None, active_object="__KEEP__", target_bin="__KEEP__", last_tool=None, last_ok=None):
        with v24_state_lock:
            if stage is not None:
                v24_skill_state["stage"] = str(stage)
            if active_object != "__KEEP__":
                v24_skill_state["active_object"] = active_object
            if target_bin != "__KEEP__":
                v24_skill_state["target_bin"] = target_bin
            if last_tool is not None:
                v24_skill_state["last_tool"] = str(last_tool)
            if last_ok is not None:
                v24_skill_state["last_ok"] = bool(last_ok)
            return dict(v24_skill_state)

    def _v24_reset_skill_state(reason="manual reset", clear_adaptation=True):
        """Create a clean task episode boundary.

        Physical ZERO/HARD RESET and planner state must agree.  Without this
        reset, a robot that has physically returned home could still be labelled
        ``HOVER_REACHED`` from a previous task and the LLM might incorrectly call
        ``approach`` before a fresh ``move_above``.  Retry counters are therefore
        task-local and reset together with the semantic state.

        Reset the logical episode state so physical pose and planner state cannot diverge.

        V2-5.1 reset only the safety retry budget, which allowed stale
        HOVER_REACHED state and an already-consumed planning retry to leak into
        the next RUN TASK after ZERO ALL.  V2-6 treats these as task-local.
        """
        with v24_state_lock:
            v24_skill_state.update({
                "stage": V24_STAGE_IDLE,
                "active_object": None,
                "target_bin": None,
                "last_tool": None,
                "last_ok": None,
            })

        for _obj in V242_SAFETY_RETRY_USED:
            V242_SAFETY_RETRY_USED[_obj] = 0
        for _obj in V251_PLANNING_RETRY_USED:
            V251_PLANNING_RETRY_USED[_obj] = 0
        for _obj in V251_GRASP_Z_RETRY_BONUS_M:
            V251_GRASP_Z_RETRY_BONUS_M[_obj] = 0.0
        if clear_adaptation:
            # Retry aperture widening is task-local.  A future learned prior
            # should live in a separate persistent memory, not in failure state.
            for _obj in V241_APERTURE_RETRY_BONUS_MM:
                V241_APERTURE_RETRY_BONUS_MM[_obj] = 0.0

        _v242_clear_safety_context()
        skill_core.emit(
            f"[V2-6 EPISODE RESET] {reason} -> IDLE | "
            "safety_retry=0 planning_retry=0 grasp_z_bonus=0 "
            + ("aperture_bonus=0" if clear_adaptation else "aperture_bonus=preserved")
        )
        return _v24_state_snapshot()

    def _v24_norm3(v):
        return math.sqrt(sum(float(x) * float(x) for x in v))

    def _v24_sub3(a, b):
        return [float(a[i]) - float(b[i]) for i in range(3)]

    def _v24_xyz_distance(a, b):
        return _v24_norm3(_v24_sub3(a, b))

    def _v24_xy_distance(a, b):
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))

    def _v24_contract(name, args):
        state = _v24_state_snapshot()
        obj = str(args.get("object_name", state.get("active_object") or "<active_object>"))
        bin_name = str(args.get("bin_name", state.get("target_bin") or "<target_bin>"))
        opening = float(gripper_actual_mm.value)
        prior = _v24_gripper_prior(obj, opening) if obj in V24_OBJECT_GEOMETRY else {}
        rr = prior.get("recommended_opening_range_mm", ["?", "?"])
        target = prior.get("recommended_target_opening_mm", "?")
        contracts = {
            "observe_scene": {
                "purpose": "Read exact MuJoCo scene, continuous gripper opening, object geometry/manipulation priors, and V2-6 state without moving anything.",
                "preconditions": ["none"],
                "expected_effects": ["fresh scene/state snapshot"],
                "expected_non_effects": ["robot does not move", "gripper does not move", "objects do not move"],
                "success_criteria": ["scene snapshot returned"],
            },
            "recover_from_safety": {
                "purpose": "Explicitly clear a latched PRE-GRASP safety stop, retreat vertically away from the object, and prepare exactly one reasoned retry.",
                "preconditions": [
                    "state=SAFETY_STOPPED",
                    "no object is held",
                    "a structured safety context exists",
                    "this object has not already consumed its one safety retry",
                ],
                "expected_effects": [
                    "V69 safety latch is explicitly cleared",
                    "TCP retreats upward about 90 mm before any aperture narrowing",
                    "selected recovery strategy updates the manipulation prior when appropriate",
                ],
                "expected_non_effects": [
                    "no grasp is created",
                    "the failed approach is NOT blindly resumed from the contact pose",
                ],
                "success_criteria": [
                    "safety latch explicitly recovered",
                    "vertical retreat is achieved without a new safety trip",
                    "one-retry budget is consumed and a fresh re-observation is requested",
                ],
            },
            "set_gripper_opening": {
                "purpose": f"Set pre-grasp gripper aperture to {float(args.get('target_opening_mm', opening)):.1f} mm in position mode.",
                "preconditions": ["hold_active=false", "not transporting a grasped object", "normally called after observe_scene"],
                "expected_effects": ["actual gripper opening converges to requested millimeter target"],
                "expected_non_effects": ["arm TCP does not move", "objects do not move", "no grasp is created"],
                "success_criteria": ["actual opening within 3 mm of target", "hold_active=false", "no new safety trip"],
            },
            "move_above": {
                "purpose": f"Move TCP to a top-down hover about 120 mm above {obj}.",
                "preconditions": [
                    "no safety trip",
                    "not currently transporting a held object",
                    f"continuous gripper opening should be within robust range {rr} mm for {obj}; preferred target={target} mm",
                ],
                "expected_effects": ["TCP moves to object XY and hover Z"],
                "expected_non_effects": [f"{obj} should remain stationary", "gripper stays unchanged", "no grasp is expected"],
                "success_criteria": ["TCP within 12 mm of hover target", "no new safety trip", "no large unexpected object displacement"],
            },
            "approach": {
                "purpose": f"Descend from hover to the grasp pose for {obj}.",
                "preconditions": [
                    "state=HOVER_REACHED", f"active_object={obj}",
                    f"actual gripper opening remains within robust range {rr} mm for {obj}",
                ],
                "expected_effects": ["TCP descends to object grasp pose without touching/pushing the object before clamp"],
                "expected_non_effects": [f"{obj} should remain approximately stationary during pre-grasp descent", "grasp/hold is not required yet"],
                "success_criteria": ["TCP within 10 mm of grasp target", "object displacement during approach <= 4 mm", "no new safety trip"],
            },
            "clamp_gripper": {
                "purpose": f"Close gripper and establish provisional contact/hold on {obj}.",
                "preconditions": ["state=GRASP_POSE_REACHED", "active object is known"],
                "expected_effects": ["gripper opening decreases", "two-finger contact or hold becomes established"],
                "expected_non_effects": [f"{obj} does not need to translate yet"],
                "success_criteria": ["two-finger contact OR hold_active"],
            },
            "lift": {
                "purpose": f"Lift {obj} vertically with the gripper and physically validate the grasp.",
                "preconditions": ["state=GRASP_ESTABLISHED", "active object is known"],
                "expected_effects": ["TCP rises", f"{obj} MUST rise with TCP", "object-to-TCP relation remains approximately stable"],
                "expected_non_effects": ["object must NOT stay on table"],
                "success_criteria": ["TCP reaches lift target", "object rises sufficiently", "follow drift within tolerance", "no new safety trip"],
            },
            "move_above_bin": {
                "purpose": f"Transport held {obj} to a safe release hover above {bin_name}.",
                "preconditions": ["state=LIFTED", "active object is known"],
                "expected_effects": ["TCP and object move together", f"object XY approaches {bin_name} center"],
                "expected_non_effects": ["grasp must not be lost during transport"],
                "success_criteria": ["TCP within 35 mm of bin-hover target", "object XY near bin center", "object-to-TCP drift within tolerance", "no new safety trip"],
            },
            "open_gripper": {
                "purpose": f"RELEASE held {obj} above {bin_name}; this is not the pre-grasp aperture tool.",
                "preconditions": ["state=BIN_HOVER_REACHED", "active object and target bin are known"],
                "expected_effects": ["gripper opens", "hold becomes false", "object detaches and may fall/settle"],
                "expected_non_effects": ["this skill does NOT prove final placement"],
                "success_criteria": ["opening >= 80 mm", "hold_active=false"],
            },
            "verify_object_in_bin": {
                "purpose": f"Verify final placement of {obj} in {bin_name} from MuJoCo coordinates.",
                "preconditions": ["normally state=RELEASED"],
                "expected_effects": ["read-only final placement verdict"],
                "expected_non_effects": ["robot/object motion not required"],
                "success_criteria": [f"actual_location == {bin_name}"],
            },
        }
        return contracts.get(name, {"purpose": name})

    def _v24_log_contract(name, args, contract, state_before):
        skill_core.emit(f"[SKILL CONTRACT] {name} | state_before={state_before.get('stage')} | args={json.dumps(args, ensure_ascii=False)}")
        skill_core.emit("[SKILL CONTRACT PURPOSE] " + str(contract.get("purpose", "")))
        skill_core.emit("[SKILL CONTRACT PRE] " + "; ".join(contract.get("preconditions", [])))
        skill_core.emit("[SKILL CONTRACT EXPECTED EFFECT] " + "; ".join(contract.get("expected_effects", [])))
        skill_core.emit("[SKILL CONTRACT EXPECTED NON-EFFECT] " + "; ".join(contract.get("expected_non_effects", [])))
        skill_core.emit("[SKILL CONTRACT SUCCESS] " + "; ".join(contract.get("success_criteria", [])))

    def _v24_next_actions(stage, ok=True):
        if not ok:
            return ["observe_scene", "replan_or_stop_based_on_failure_reason"]
        return {
            V24_STAGE_IDLE: ["observe_scene"],
            V24_STAGE_OBSERVED: ["set_gripper_opening", "move_above"],
            V24_STAGE_HOVER: ["approach"],
            V24_STAGE_GRASP_POSE: ["clamp_gripper"],
            V24_STAGE_GRASPED: ["lift"],
            V24_STAGE_LIFTED: ["move_above_bin"],
            V24_STAGE_BIN_HOVER: ["open_gripper"],
            V24_STAGE_RELEASED: ["verify_object_in_bin"],
            V24_STAGE_VERIFIED: ["observe_scene", "move_above_next_object"],
            V24_STAGE_SAFETY: ["observe_scene", "recover_from_safety"],
        }.get(stage, ["observe_scene"])

    def _v24_precondition_failure(name, args, reason, contract, state_before, physical_verification=None, recommended_next_actions=None):
        skill_core.emit(f"[SKILL PRECONDITION FAILED] {name} | {reason}")
        return {
            "ok": False,
            "tool": name,
            "reason": "PRECONDITION_FAILED",
            "detail": str(reason),
            "state_before": state_before,
            "state_after": _v24_state_snapshot(),
            "contract": contract,
            "physical_verification": dict(physical_verification or {}),
            "warnings": [],
            "recommended_next_actions": list(recommended_next_actions or ["observe_scene", "follow_state_machine"]),
        }

    def _v24_result(name, ok, contract, state_before, verification, warnings=None, extra=None, next_stage=None, active_object="__KEEP__", target_bin="__KEEP__", recommended_next_actions=None):
        warnings = list(warnings or [])
        if ok and next_stage is not None:
            state_after = _v24_set_state(next_stage, active_object=active_object, target_bin=target_bin, last_tool=name, last_ok=True)
        else:
            state_after = _v24_set_state(last_tool=name, last_ok=bool(ok))
        result = {
            "ok": bool(ok),
            "tool": name,
            "state_before": state_before,
            "state_after": state_after,
            "contract": contract,
            "physical_verification": verification,
            "warnings": warnings,
            "recommended_next_actions": list(recommended_next_actions) if recommended_next_actions is not None else _v24_next_actions(state_after.get("stage"), bool(ok)),
        }
        if extra:
            result.update(extra)
        skill_core.emit("[PHYSICAL VERIFY] " + name + " | " + json.dumps(verification, ensure_ascii=False, separators=(",", ":")))
        if warnings:
            skill_core.emit("[SKILL WARNINGS] " + name + " | " + " | ".join(str(x) for x in warnings))
        skill_core.emit(
            f"[SKILL RESULT] {name} | {'SUCCESS' if ok else 'FAILED'} | "
            f"{state_before.get('stage')} -> {state_after.get('stage')} | next={','.join(result['recommended_next_actions'])}"
        )
        return result

    def _v24_aperture_check(object_name):
        opening = float(gripper_actual_mm.value)
        prior = _v24_gripper_prior(object_name, opening)
        rec_min, rec_max = [float(v) for v in prior["recommended_opening_range_mm"]]
        rec_target = float(prior["recommended_target_opening_mm"])
        width = float(prior["effective_width_mm"])
        robust = rec_min <= opening <= rec_max
        if opening < rec_min:
            status = "TOO_NARROW"
        elif opening > rec_max:
            status = "UNNECESSARILY_WIDE"
        else:
            status = "ROBUST_RANGE"
        return {
            "object_name": str(object_name),
            "actual_opening_mm": round(opening, 2),
            "effective_width_mm": round(width, 2),
            "geometric_clearance_total_mm": round(opening - width, 2),
            "estimated_free_clearance_total_mm": prior.get("estimated_free_clearance_total_mm"),
            "estimated_free_clearance_per_side_mm": prior.get("estimated_free_clearance_per_side_mm"),
            "recommended_opening_range_mm": [round(rec_min, 2), round(rec_max, 2)],
            "recommended_target_opening_mm": round(rec_target, 2),
            "adaptive_retry_bonus_mm": prior.get("adaptive_retry_bonus_mm", 0.0),
            "aperture_status": status,
            "robust_aperture_ok": bool(robust),
        }

    # Keep V69 safety/joint checks, but interpret V69's legacy final-joint
    # mismatch at the TASK semantic layer using the actual MuJoCo TCP goal.
    def _v24_cartesian_postcheck(label, target_xyz, tol_m, safety_before, caught_exc=None):
        actual_xyz, actual_q = skill_core.current_tcp_pose()
        dist = _v24_xyz_distance(actual_xyz, target_xyz)
        safety_after = int(safety_trip_count.value)
        tracking_only = caught_exc is not None and "PHYSICAL TRACKING MISMATCH" in str(caught_exc)
        verification = {
            "target_xyz_m": [round(float(v), 5) for v in target_xyz],
            "actual_xyz_m": [round(float(v), 5) for v in actual_xyz],
            "tcp_error_mm": round(dist * 1000.0, 2),
            "tolerance_mm": round(float(tol_m) * 1000.0, 2),
            "safety_trip_before": int(safety_before),
            "safety_trip_after": int(safety_after),
        }
        if safety_after != int(safety_before):
            verification.update({
                "cartesian_ok": False,
                "reason": "SAFETY_TRIP_CHANGED",
                "safety_state_code": int(safety_state.value),
            })
            if caught_exc is not None:
                verification["motion_exception"] = str(caught_exc)
            return verification, actual_q, False, []
        if dist <= float(tol_m):
            warnings = []
            if tracking_only:
                warnings.append("Legacy V69 final-joint verifier reported PHYSICAL TRACKING MISMATCH, but the deterministic MuJoCo TCP postcondition is satisfied.")
                verification["legacy_joint_mismatch_overridden_by_task_postcondition"] = True
                skill_core.emit(
                    f"[V2-6 CARTESIAN ACCEPT] {label} | V69 joint mismatch is diagnostic only here; "
                    f"TCP error={dist*1000.0:.1f}mm <= {tol_m*1000.0:.1f}mm | safety unchanged"
                )
            verification.update({"cartesian_ok": True})
            return verification, actual_q, True, warnings
        verification.update({"cartesian_ok": False, "reason": "CARTESIAN_POSTCONDITION_FAILED"})
        return verification, actual_q, False, []

    def _v24_run_motion(label, fn, target_xyz, tol_m, preserve_grasp_quat=False):
        safety0 = int(safety_trip_count.value)
        caught = None
        raw_ok = False
        try:
            raw_ok = bool(fn())
        except Exception as exc:
            caught = exc
            _msg = str(exc)
            # V2-6 normalizes a physical safety stop into structured task
            # feedback so the Agent can reason about it.  The V69 latch remains
            # held; only recover_from_safety may explicitly clear it.
            if (
                "PHYSICAL TRACKING MISMATCH" not in _msg
                and "PHYSICAL_SAFETY_STOP" not in _msg
                and "CARTESIAN APPROACH BLOCKED" not in _msg
            ):
                raise
        verification, actual_q, cart_ok, warnings = _v24_cartesian_postcheck(label, target_xyz, tol_m, safety0, caught_exc=caught)

        # V2-6: controller completion can lag the first postcondition sample.
        # The log showed move_above reported ~35 mm error, then the very next
        # observation was already at the correct hover.  Recheck briefly before
        # turning a late settle into an Agent-visible planning failure.
        planning_blocked = caught is not None and "CARTESIAN APPROACH BLOCKED" in str(caught)
        if (not cart_ok) and (not planning_blocked) and int(safety_trip_count.value) == safety0:
            settle_deadline = time.time() + 1.8
            best = float(verification.get("tcp_error_mm", 1e9))
            while time.time() < settle_deadline:
                time.sleep(0.12)
                xyz2, q2 = skill_core.current_tcp_pose()
                d2 = _v24_xyz_distance(xyz2, target_xyz)
                best = min(best, d2 * 1000.0)
                if d2 <= float(tol_m):
                    actual_q = q2
                    cart_ok = True
                    verification.update({
                        "actual_xyz_m": [round(float(v), 5) for v in xyz2],
                        "tcp_error_mm": round(d2 * 1000.0, 2),
                        "cartesian_ok": True,
                        "late_settle_recheck": True,
                        "late_settle_best_error_mm": round(best, 2),
                    })
                    verification.pop("reason", None)
                    skill_core.emit(
                        f"[V2-6 LATE SETTLE ACCEPT] {label} | TCP reached tolerance on recheck: "
                        f"{d2*1000.0:.1f}mm <= {tol_m*1000.0:.1f}mm"
                    )
                    break

        if cart_ok and preserve_grasp_quat:
            skill_core.active_grasp_quat = [float(v) for v in actual_q]
            skill_core.emit(f"[V2-6 BRANCH PRESERVE] {label} | active_grasp_quat <- actual TCP quaternion")
        verification["legacy_skill_return"] = bool(raw_ok)
        if caught is not None:
            verification["legacy_joint_verify_error"] = str(caught)
        return verification, cart_ok, warnings

    def llm_tool_runner(name, args):
        """Synchronous V2-6 Gripper-Aware Skill Contract bridge used by Agent and manual debugger."""
        if skill_core is None:
            raise RuntimeError(f"V69 skill core unavailable: {skill_core_error}")
        name = str(name)
        args = dict(args or {})
        state_before = _v24_state_snapshot()
        contract = _v24_contract(name, args)
        skill_core.emit(f"[V2-6 TOOL START] {name} | args={json.dumps(args, ensure_ascii=False)}")
        _v24_log_contract(name, args, contract, state_before)
        t0 = time.perf_counter()

        if name == "observe_scene":
            if state_before.get("stage") == V24_STAGE_IDLE:
                _v24_set_state(V24_STAGE_OBSERVED, last_tool=name, last_ok=True)
            scene = llm_scene_snapshot()
            scene["v2_skill_state"] = _v24_state_snapshot()
            skill_core.emit("[V2-6 SCENE] " + json.dumps(scene, ensure_ascii=False, separators=(",", ":")))
            opening_now = float(gripper_actual_mm.value)
            for obj_name in OBJECTS:
                pr = _v24_gripper_prior(obj_name, opening_now)
                skill_core.emit(
                    f"[GRIPPER PRIOR] {obj_name} | opening={opening_now:.1f}mm | "
                    f"effective_width={pr['effective_width_mm']:.1f}mm | "
                    f"recommended={pr['recommended_opening_range_mm']}mm | "
                    f"target={pr['recommended_target_opening_mm']:.1f}mm"
                )
            return _v24_result(
                name, True, contract, state_before,
                {"observation_only": True, "physical_motion_expected": False},
                extra={"scene": scene, "elapsed_s": round(time.perf_counter()-t0, 3)},
            )

        if name == "recover_from_safety":
            obj = str(args.get("object_name") or state_before.get("active_object") or "")
            strategy = str(args.get("strategy", "retreat_and_retry_once"))
            diagnosis = str(args.get("diagnosis", "")).strip()
            ctx = dict(V242_LAST_SAFETY_CONTEXT)

            if state_before.get("stage") != V24_STAGE_SAFETY:
                return _v24_precondition_failure(
                    name, args, f"requires SAFETY_STOPPED; current={state_before}",
                    contract, state_before, recommended_next_actions=["observe_scene"],
                )
            if not ctx.get("active") or ctx.get("object_name") != obj:
                return _v24_precondition_failure(
                    name, args, f"no matching active safety context for {obj}: {ctx}",
                    contract, state_before, recommended_next_actions=["observe_scene"],
                )
            if bool(hold_active_shared.value):
                return _v24_precondition_failure(
                    name, args, "hold_active=true; automatic pre-grasp retry is not allowed while holding an object",
                    contract, state_before, recommended_next_actions=["stop_and_preserve_grasp"],
                )
            used = int(V242_SAFETY_RETRY_USED.get(obj, 0))
            if used >= 1:
                return _v24_precondition_failure(
                    name, args, f"one safety retry already consumed for {obj}; do not loop",
                    contract, state_before,
                    physical_verification={"retry_used": used, "safety_context": ctx},
                    recommended_next_actions=["stop_and_explain_failure"],
                )

            # Consumes the one retry budget as soon as explicit recovery begins.
            V242_SAFETY_RETRY_USED[obj] = used + 1
            prior_before = _v24_gripper_prior(obj, float(gripper_actual_mm.value))
            old_bonus = float(V241_APERTURE_RETRY_BONUS_MM.get(obj, 0.0))
            new_bonus = old_bonus
            if strategy == "widen_aperture_and_retry_once":
                new_bonus = min(30.0, old_bonus + 15.0)
                V241_APERTURE_RETRY_BONUS_MM[obj] = new_bonus
            prior_after = _v24_gripper_prior(obj, float(gripper_actual_mm.value))

            skill_core.emit(
                f"[SAFETY REFLECTION] object={obj} | diagnosis={diagnosis or ctx.get('likely_cause')} | "
                f"strategy={strategy} | retry=1/1 | aperture_bonus={old_bonus:.1f}->{new_bonus:.1f}mm"
            )

            # Explicit planner recovery: this is the programmatic equivalent of
            # STOP + RECOVER, not an automatic bypass of V69 safety.
            trip_before = int(safety_trip_count.value)
            recover_request.value = True
            deadline = time.time() + 1.5
            while time.time() < deadline:
                # The MuJoCo loop converts recovery to reason_code=2 and clears
                # its internal ros_motion_fault_latched flag.
                if int(safety_state.value) == 2:
                    break
                time.sleep(0.03)
            time.sleep(0.12)

            retreat_start, _ = skill_core.current_tcp_pose()
            retreat_target = [float(retreat_start[0]), float(retreat_start[1]), float(retreat_start[2]) + 0.090]
            retreat_ok = False
            retreat_rise_mm = 0.0
            retreat_error = None
            retreat_motion_verification = {}
            try:
                llm_require_arm_controller()
                skill_core.emit("[SAFETY RECOVERY] explicit unlatch complete; vertical retreat +90 mm with current aperture preserved")
                retreat_motion_verification, retreat_cart_ok, retreat_warnings = _v24_run_motion(
                    f"safety_recovery_retreat({obj})",
                    lambda: skill_core.bio_lift(0.090),
                    retreat_target, 0.015, True,
                )
                retreat_end, _ = skill_core.current_tcp_pose()
                retreat_rise_mm = (float(retreat_end[2]) - float(retreat_start[2])) * 1000.0
                retreat_ok = bool(
                    retreat_cart_ok
                    and retreat_rise_mm > 70.0
                    and int(safety_trip_count.value) == trip_before
                )
                for _w in retreat_warnings:
                    skill_core.emit(f"[SAFETY RECOVERY WARN] {_w}")
            except Exception as exc:
                retreat_error = str(exc)

            verification = {
                "safety_context": ctx,
                "agent_diagnosis": diagnosis,
                "strategy": strategy,
                "retry_used": 1,
                "retry_limit": 1,
                "aperture_bonus_before_mm": round(old_bonus, 2),
                "aperture_bonus_after_mm": round(new_bonus, 2),
                "recommended_retry_opening_mm": prior_after.get("recommended_target_opening_mm"),
                "retreat_rise_mm": round(retreat_rise_mm, 2),
                "retreat_ok": bool(retreat_ok),
                "retreat_motion_verification": retreat_motion_verification,
                "new_safety_trip_during_recovery": int(safety_trip_count.value) != trip_before,
            }
            if retreat_error:
                verification["retreat_error"] = retreat_error

            if not retreat_ok:
                _v24_set_state(
                    V24_STAGE_SAFETY, active_object=obj, target_bin=None,
                    last_tool=name, last_ok=False,
                )
                return _v24_result(
                    name, False, contract, state_before, verification,
                    warnings=["Safety recovery retreat did not complete cleanly; do not attempt another automatic retry."],
                    extra={"object_name": obj, "elapsed_s": round(time.perf_counter()-t0, 3)},
                    recommended_next_actions=["stop_and_explain_failure"],
                )

            _v242_clear_safety_context()
            _v24_set_state(
                V24_STAGE_OBSERVED, active_object=None, target_bin=None,
                last_tool=name, last_ok=True,
            )
            skill_core.emit(
                f"[V2-6 RECOVERY OK] {obj} | retreat={retreat_rise_mm:.1f}mm | fresh re-observation required | "
                f"next recommended aperture={prior_after.get('recommended_target_opening_mm')}mm | retry budget now 1/1"
            )
            return _v24_result(
                name, True, contract, state_before, verification,
                warnings=[],
                extra={"object_name": obj, "elapsed_s": round(time.perf_counter()-t0, 3)},
                next_stage=V24_STAGE_OBSERVED, active_object=None, target_bin=None,
                recommended_next_actions=["set_gripper_opening", "observe_scene", "move_above", "approach"],
            )

        if name == "set_gripper_opening":
            stage = state_before.get("stage")
            if stage not in (V24_STAGE_OBSERVED, V24_STAGE_HOVER, V24_STAGE_VERIFIED, V24_STAGE_RELEASED):
                return _v24_precondition_failure(
                    name, args,
                    f"pre-grasp aperture adjustment not allowed from state={stage}; observe scene first",
                    contract, state_before,
                    recommended_next_actions=["observe_scene"],
                )
            if bool(hold_active_shared.value):
                return _v24_precondition_failure(
                    name, args, "hold_active=true; do not use pre-grasp aperture adjustment while holding an object",
                    contract, state_before,
                    recommended_next_actions=["resolve_or_release_held_object"],
                )
            target_mm = max(5.0, min(100.0, float(args.get("target_opening_mm", 60.0))))
            opening_before = float(gripper_actual_mm.value)
            tcp_before, _ = skill_core.current_tcp_pose()
            object_before = {}
            for obj_name in OBJECTS:
                try:
                    object_before[obj_name] = skill_core.object_pose(obj_name)[0]
                except Exception:
                    pass
            safety_before = int(safety_trip_count.value)
            skill_core.emit(
                f"[V2-6 GRIPPER PREP] position mode | actual={opening_before:.1f}mm -> target={target_mm:.1f}mm"
            )
            hold_off_request.value = True
            gripper_force_mode.value = False
            gripper_target_mm.value = target_mm
            deadline = time.time() + 4.0
            last_report = 0.0
            while time.time() < deadline:
                actual = float(gripper_actual_mm.value)
                now = time.time()
                if now - last_report >= 0.50:
                    skill_core.emit(
                        f"[V2-6 GRIPPER PREP WAIT] target={target_mm:.1f}mm | actual={actual:.1f}mm | "
                        f"error={actual-target_mm:+.1f}mm"
                    )
                    last_report = now
                if abs(actual - target_mm) <= 2.0:
                    break
                time.sleep(0.05)
            time.sleep(0.08)
            opening_after = float(gripper_actual_mm.value)
            tcp_after, _ = skill_core.current_tcp_pose()
            tcp_motion = _v24_xyz_distance(tcp_after, tcp_before)
            max_obj_motion = 0.0
            max_obj_name = None
            for obj_name, before_xyz in object_before.items():
                try:
                    after_xyz, _ = skill_core.object_pose(obj_name)
                    d = _v24_xyz_distance(after_xyz, before_xyz)
                    if d > max_obj_motion:
                        max_obj_motion = d
                        max_obj_name = obj_name
                except Exception:
                    pass
            safety_after = int(safety_trip_count.value)
            error_mm = abs(opening_after - target_mm)
            ok = (
                error_mm <= 3.0
                and not bool(hold_active_shared.value)
                and safety_after == safety_before
                and tcp_motion <= 0.003
                and max_obj_motion <= 0.005
            )
            verification = {
                "target_opening_mm": round(target_mm, 2),
                "opening_before_mm": round(opening_before, 2),
                "opening_after_mm": round(opening_after, 2),
                "opening_error_mm": round(error_mm, 2),
                "hold_active": bool(hold_active_shared.value),
                "tcp_motion_mm": round(tcp_motion * 1000.0, 2),
                "max_object_motion_mm": round(max_obj_motion * 1000.0, 2),
                "max_object_motion_object": max_obj_name,
                "safety_trip_before": safety_before,
                "safety_trip_after": safety_after,
                "physical_motion_expected": "gripper fingers only",
            }
            if not ok:
                verification["reason"] = "GRIPPER_APERTURE_TARGET_NOT_VERIFIED"
            return _v24_result(
                name, ok, contract, state_before, verification, [],
                {"elapsed_s": round(time.perf_counter()-t0, 3)},
                next_stage=None,
                recommended_next_actions=["observe_scene"] if ok else ["observe_scene", "retry_or_stop_based_on_failure_reason"],
            )

        if name == "move_above":
            stage = state_before.get("stage")
            if stage not in (V24_STAGE_OBSERVED, V24_STAGE_VERIFIED, V24_STAGE_RELEASED):
                return _v24_precondition_failure(name, args, f"move_above not allowed from state={stage}", contract, state_before)
            if bool(hold_active_shared.value):
                return _v24_precondition_failure(name, args, "hold_active=true; release/resolve current object before selecting another", contract, state_before)
            obj = str(args["object_name"])
            aperture = _v24_aperture_check(obj)
            if not aperture["robust_aperture_ok"]:
                reason = (
                    f"gripper aperture {aperture['actual_opening_mm']:.1f}mm is {aperture['aperture_status']} for {obj}; "
                    f"effective width={aperture['effective_width_mm']:.1f}mm, "
                    f"recommended range={aperture['recommended_opening_range_mm']}mm, "
                    f"recommended target={aperture['recommended_target_opening_mm']:.1f}mm"
                )
                return _v24_precondition_failure(
                    name, args, reason, contract, state_before,
                    physical_verification={"gripper_aperture": aperture},
                    recommended_next_actions=["set_gripper_opening", "observe_scene", "move_above"],
                )
            llm_require_arm_controller()
            obj_before, _ = skill_core.object_pose(obj)
            target = [float(obj_before[0]), float(obj_before[1]), float(obj_before[2]) + 0.12]
            verification, cart_ok, warnings = _v24_run_motion(
                f"move_above({obj})", lambda: skill_core.bio_move_above(obj, 0.12), target, 0.012, True
            )
            obj_after, _ = skill_core.object_pose(obj)
            object_move = _v24_xyz_distance(obj_after, obj_before)
            verification.update({
                "object_xyz_before_m": [round(float(v),5) for v in obj_before],
                "object_xyz_after_m": [round(float(v),5) for v in obj_after],
                "object_motion_mm": round(object_move*1000.0, 2),
                "object_motion_expected": False,
            })
            if object_move > 0.005:
                warnings.append(f"Object moved {object_move*1000.0:.1f} mm during hover motion; object motion is not expected at this stage.")
            ok = bool(cart_ok and object_move <= 0.020)
            if cart_ok and not ok:
                verification["reason"] = "UNEXPECTED_OBJECT_MOTION_DURING_HOVER"
            return _v24_result(name, ok, contract, state_before, verification, warnings,
                               {"object_name": obj, "elapsed_s": round(time.perf_counter()-t0,3)},
                               V24_STAGE_HOVER if ok else None, obj if ok else "__KEEP__", None if ok else "__KEEP__")

        if name == "approach":
            obj = str(args["object_name"])
            if state_before.get("stage") != V24_STAGE_HOVER or state_before.get("active_object") != obj:
                return _v24_precondition_failure(name, args, f"requires HOVER_REACHED for same object; current={state_before}", contract, state_before)
            aperture = _v24_aperture_check(obj)
            if not aperture["robust_aperture_ok"]:
                reason = (
                    f"approach blocked: current aperture {aperture['actual_opening_mm']:.1f}mm is "
                    f"{aperture['aperture_status']} for {obj}; recommended target="
                    f"{aperture['recommended_target_opening_mm']:.1f}mm"
                )
                return _v24_precondition_failure(
                    name, args, reason, contract, state_before,
                    physical_verification={"gripper_aperture": aperture},
                    recommended_next_actions=["set_gripper_opening", "observe_scene", "approach"],
                )
            llm_require_arm_controller()
            obj_before, _ = skill_core.object_pose(obj)
            opening_before = float(gripper_actual_mm.value)
            base_zoff = float(GRASP_Z_OFFSET.get(obj, DEFAULT_GRASP_Z_OFFSET))
            z_bonus = float(V251_GRASP_Z_RETRY_BONUS_M.get(obj, 0.0))
            zoff = base_zoff + z_bonus
            target = [float(obj_before[0]), float(obj_before[1]), float(obj_before[2]) + zoff]
            if z_bonus > 0.0:
                skill_core.emit(
                    f"[V2-6 ADAPTED APPROACH] {obj} | base_z_offset={base_zoff*1000.0:.1f}mm "
                    f"+ retry_bonus={z_bonus*1000.0:.1f}mm"
                )
            verification, cart_ok, warnings = _v24_run_motion(
                f"approach({obj})", lambda: skill_core.approach(obj, zoff), target, 0.010, True
            )
            obj_after, _ = skill_core.object_pose(obj)
            opening_after = float(gripper_actual_mm.value)
            object_move = _v24_xyz_distance(obj_after, obj_before)
            object_move_mm = object_move * 1000.0
            opening_change_mm = opening_after - opening_before
            verification.update({
                "object_motion_mm": round(object_move_mm, 2),
                "object_motion_required": False,
                "gripper_opening_before_approach_mm": round(opening_before, 2),
                "gripper_opening_after_approach_mm": round(opening_after, 2),
                "gripper_opening_change_during_approach_mm": round(opening_change_mm, 2),
                "anti_poke_object_motion_limit_mm": 4.0,
            })

            # Planning failure recovery (pre-contact)
            # -------------------------------------------
            # A Cartesian collision-check refusal happens BEFORE execution.
            # It is therefore not a V69 safety trip and needs no unlatch.
            # Treat it as a recoverable planning failure once: widen the aperture
            # and raise the grasp target slightly, then re-observe and retry.
            motion_error = str(verification.get("legacy_joint_verify_error", ""))
            if "CARTESIAN APPROACH BLOCKED" in motion_error:
                m = re.search(r"only\s+([0-9.]+)%\s+collision-free", motion_error)
                fraction_pct = float(m.group(1)) if m else None
                used_plan = int(V251_PLANNING_RETRY_USED.get(obj, 0))
                retry_allowed = bool(used_plan < 1 and not bool(hold_active_shared.value))
                verification.update({
                    "reason": "PREGRASP_PATH_BLOCKED_REQUIRES_ADAPTATION",
                    "collision_free_fraction_pct": fraction_pct,
                    "planning_retry_allowed": retry_allowed,
                    "planning_retry_used": used_plan,
                    "planning_retry_limit": 1,
                })
                if retry_allowed:
                    V251_PLANNING_RETRY_USED[obj] = used_plan + 1
                    old_ap_bonus = float(V241_APERTURE_RETRY_BONUS_MM.get(obj, 0.0))
                    # A modest +6 mm opening change plus +8 mm grasp-height change
                    # is materially different without jumping to a full-open hand.
                    V241_APERTURE_RETRY_BONUS_MM[obj] = min(30.0, old_ap_bonus + 6.0)
                    V251_GRASP_Z_RETRY_BONUS_M[obj] = max(float(V251_GRASP_Z_RETRY_BONUS_M.get(obj, 0.0)), 0.008)
                    adapted = _v24_gripper_prior(obj, opening_after)
                    verification.update({
                        "diagnosis": "Cartesian collision checker predicts insufficient pre-grasp clearance; no physical contact occurred",
                        "retry_recommended_opening_mm": float(adapted["recommended_target_opening_mm"]),
                        "retry_grasp_z_bonus_mm": round(V251_GRASP_Z_RETRY_BONUS_M[obj] * 1000.0, 1),
                    })
                    warnings.append(
                        f"PRE-GRASP PATH BLOCKED at {fraction_pct if fraction_pct is not None else '?'}% collision-free. "
                        f"No physical collision occurred. Adapt once: opening -> {adapted['recommended_target_opening_mm']:.1f} mm; "
                        f"grasp target +{V251_GRASP_Z_RETRY_BONUS_M[obj]*1000.0:.0f} mm higher."
                    )
                    skill_core.emit(
                        f"[V2-6 PLANNING FAILURE DIAGNOSIS] approach({obj}) | "
                        f"collision_free={fraction_pct}% | no physical contact | "
                        f"opening_target={adapted['recommended_target_opening_mm']:.1f}mm | "
                        f"grasp_z_bonus={V251_GRASP_Z_RETRY_BONUS_M[obj]*1000.0:.0f}mm | retry=1/1"
                    )
                    return _v24_result(
                        name, False, contract, state_before, verification, warnings,
                        {"object_name": obj, "elapsed_s": round(time.perf_counter()-t0,3)},
                        None,
                        recommended_next_actions=["set_gripper_opening", "observe_scene", "approach"],
                    )
                warnings.append("Second Cartesian planning block for this object; no further planning retry allowed.")
                skill_core.emit(f"[V2-6 PLANNING FINAL ABORT] approach({obj}) | retry budget exhausted")
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"object_name": obj, "elapsed_s": round(time.perf_counter()-t0,3)},
                    None, recommended_next_actions=["stop_this_object_and_explain_failure"],
                )

            # Physical safety recovery (post-execution evidence)
            # ----------------------------------------------------
            # A latched safety stop during the pre-grasp descent is
            # converted into a structured failure for the Agent to reflect on.
            # We do NOT auto-unlatch here.  The Agent must observe, explain a
            # diagnosis, choose recover_from_safety, adjust strategy, and gets
            # at most one retry for this object.
            safety_stop = bool(
                verification.get("reason") == "SAFETY_TRIP_CHANGED"
                or "PHYSICAL_SAFETY_STOP" in str(verification.get("legacy_joint_verify_error", ""))
                or "PHYSICAL_SAFETY_STOP" in str(verification.get("motion_exception", ""))
            )
            if safety_stop:
                used = int(V242_SAFETY_RETRY_USED.get(obj, 0))
                current_prior = _v24_gripper_prior(obj, opening_after)
                proposed_bonus = min(30.0, float(V241_APERTURE_RETRY_BONUS_MM.get(obj, 0.0)) + 15.0)
                # Compute the proposed target without mutating the prior yet.
                old_bonus = float(V241_APERTURE_RETRY_BONUS_MM.get(obj, 0.0))
                V241_APERTURE_RETRY_BONUS_MM[obj] = proposed_bonus
                proposed_prior = _v24_gripper_prior(obj, opening_after)
                V241_APERTURE_RETRY_BONUS_MM[obj] = old_bonus

                evidence = [
                    f"safety trip occurred during PRE-GRASP approach",
                    f"opening before={opening_before:.1f}mm, after={opening_after:.1f}mm, change={opening_change_mm:+.1f}mm",
                    f"object motion before clamp={object_move_mm:.1f}mm",
                    f"effective object width={current_prior.get('effective_width_mm')}mm",
                ]
                if abs(opening_change_mm) >= 8.0:
                    likely = "OPENING_OR_COLLISION_ENVELOPE_TOO_TIGHT / possible lateral approach offset"
                    confidence = "high"
                elif object_move_mm > 2.0:
                    likely = "LATERAL_ALIGNMENT_OR_CLEARANCE_ERROR / premature object contact"
                    confidence = "moderate"
                else:
                    likely = "UNCERTAIN_PREGRASP_CONTACT_OR_TRAJECTORY_ERROR; fresh perception and replanning required"
                    confidence = "moderate"
                retry_allowed = bool(used < 1 and not bool(hold_active_shared.value))
                ctx = _v242_set_safety_context(
                    active=True, tool="approach", object_name=obj,
                    trip_count=int(safety_trip_count.value),
                    safety_state_code=int(safety_state.value),
                    likely_cause=likely, confidence=confidence, evidence=evidence,
                    suggested_strategy="widen_aperture_and_retry_once" if retry_allowed else "stop",
                    suggested_retry_opening_mm=float(proposed_prior.get("recommended_target_opening_mm", opening_after)),
                    retry_allowed=retry_allowed, retry_used=used, retry_limit=1,
                )
                verification["reason"] = "PREGRASP_SAFETY_STOP_REQUIRES_REFLECTION"
                verification["safety_reflection_context"] = ctx
                verification["proposed_retry_opening_mm"] = ctx.get("suggested_retry_opening_mm")
                warnings.append(
                    f"PRE-GRASP SAFETY STOP: likely cause={likely} ({confidence}). "
                    f"Do not blindly repeat. One reasoned recovery/retry is {'available' if retry_allowed else 'not available'}."
                )
                skill_core.emit(
                    f"[V2-6 FAILURE DIAGNOSIS REQUIRED] approach({obj}) | likely={likely} | "
                    f"opening={opening_before:.1f}->{opening_after:.1f}mm | object_motion={object_move_mm:.1f}mm | "
                    f"retry_allowed={retry_allowed} | proposed_opening={ctx.get('suggested_retry_opening_mm')}mm"
                )
                _v24_set_state(
                    V24_STAGE_SAFETY, active_object=obj, target_bin=None,
                    last_tool=name, last_ok=False,
                )
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"object_name": obj, "elapsed_s": round(time.perf_counter()-t0,3)},
                    None,
                    recommended_next_actions=["observe_scene", "recover_from_safety"] if retry_allowed else ["stop_and_explain_failure"],
                )

            # V2-5: pre-grasp descent should be non-contact.  A visible object
            # displacement before clamp is not "centering"; it is evidence that
            # the aperture/collision envelope was too tight or the approach was
            # laterally imperfect.  Treat it as a failed precondition and adapt.
            poke_detected = bool(object_move_mm > 4.0)
            if poke_detected:
                old_bonus = float(V241_APERTURE_RETRY_BONUS_MM.get(obj, 0.0))
                new_bonus = min(24.0, old_bonus + 8.0)
                V241_APERTURE_RETRY_BONUS_MM[obj] = new_bonus
                new_prior = _v24_gripper_prior(obj, opening_after)
                retry_target = float(new_prior["recommended_target_opening_mm"])
                verification.update({
                    "reason": "PREGRASP_CONTACT_POKE_DETECTED",
                    "adaptive_retry_bonus_before_mm": round(old_bonus, 2),
                    "adaptive_retry_bonus_after_mm": round(new_bonus, 2),
                    "retry_recommended_opening_mm": round(retry_target, 2),
                })
                warnings.append(
                    f"ANTI-POKE: {obj} moved {object_move_mm:.1f} mm during pre-grasp descent (>4 mm). "
                    f"Treating this as premature contact, not success. Next recommended aperture={retry_target:.1f} mm."
                )
                skill_core.emit(
                    f"[MANIPULATION PRIOR ADAPT] {obj} | pregrasp_motion={object_move_mm:.1f}mm | "
                    f"retry_bonus={old_bonus:.1f}->{new_bonus:.1f}mm | next_target={retry_target:.1f}mm"
                )

                # Recover vertically before asking the Agent to retry.  Widening
                # while still down around the object is less safe than first
                # separating the fingers from the contact region.
                retreat_ok = False
                retreat_rise_mm = 0.0
                try:
                    retreat_start, _ = skill_core.current_tcp_pose()
                    skill_core.emit("[ANTI-POKE RECOVERY] vertical retreat +90 mm before aperture retry")
                    skill_core.bio_lift(0.090)
                    retreat_end, _ = skill_core.current_tcp_pose()
                    retreat_rise_mm = (float(retreat_end[2]) - float(retreat_start[2])) * 1000.0
                    retreat_ok = retreat_rise_mm > 70.0
                except Exception as exc:
                    warnings.append(f"Anti-poke vertical retreat raised: {exc}")
                verification["anti_poke_retreat_ok"] = bool(retreat_ok)
                verification["anti_poke_retreat_rise_mm"] = round(retreat_rise_mm, 2)
                _v24_set_state(
                    V24_STAGE_OBSERVED,
                    active_object=None,
                    target_bin=None,
                    last_tool=name,
                    last_ok=False,
                )
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"object_name": obj, "elapsed_s": round(time.perf_counter()-t0,3)},
                    None,
                    recommended_next_actions=["set_gripper_opening", "observe_scene", "move_above", "approach"],
                )

            ok = bool(cart_ok and object_move_mm <= 4.0)
            if not cart_ok:
                verification.setdefault("reason", "CARTESIAN_POSTCONDITION_FAILED")
            return _v24_result(name, ok, contract, state_before, verification, warnings,
                               {"object_name": obj, "elapsed_s": round(time.perf_counter()-t0,3)},
                               V24_STAGE_GRASP_POSE if ok else None)

        if name == "clamp_gripper":
            if state_before.get("stage") != V24_STAGE_GRASP_POSE or not state_before.get("active_object"):
                return _v24_precondition_failure(name, args, f"requires GRASP_POSE_REACHED with active object; current={state_before}", contract, state_before)
            obj = str(state_before["active_object"])
            obj_before, _ = skill_core.object_pose(obj)
            opening_before = float(gripper_actual_mm.value)
            force_n = max(0.5, min(15.0, float(args.get("force_n", 5.0))))
            skill_core.emit(f"[V2-6 GRIPPER] FAST CLAMP -> {force_n:.1f} N/finger")
            gripper_force_N.value = force_n
            clamp_speed_mult_shared.value = 15.0
            hold_enabled_shared.value = True
            gripper_force_mode.value = True
            deadline = time.time() + 3.2
            last_report = 0.0
            while time.time() < deadline:
                left = float(contact_left_n_shared.value); right = float(contact_right_n_shared.value); holding = bool(hold_active_shared.value)
                now = time.time()
                if now-last_report >= 0.55:
                    skill_core.emit(f"[V2-6 GRIPPER WAIT] L={left:.2f}N R={right:.2f}N opening={float(gripper_actual_mm.value):.1f}mm hold={holding}")
                    last_report = now
                if holding or (left >= 0.30 and right >= 0.30): break
                time.sleep(0.05)
            left = float(contact_left_n_shared.value); right = float(contact_right_n_shared.value); holding = bool(hold_active_shared.value)
            opening_after = float(gripper_actual_mm.value)
            obj_after, _ = skill_core.object_pose(obj)
            object_move = _v24_xyz_distance(obj_after, obj_before)
            contacted = left >= 0.30 and right >= 0.30
            ok = bool(contacted or holding)
            warnings = []
            if object_move > 0.015:
                warnings.append(f"Object shifted {object_move*1000.0:.1f} mm while clamping; some centering motion can be normal.")
            verification = {
                "active_object": obj, "contacted_two_finger": contacted, "hold_active": holding,
                "left_N": round(left,3), "right_N": round(right,3),
                "opening_before_mm": round(opening_before,2), "opening_after_mm": round(opening_after,2),
                "object_motion_mm": round(object_move*1000.0,2), "object_motion_required": False,
            }
            if not ok: verification["reason"] = "NO_RELIABLE_GRASP_CONTACT"
            return _v24_result(name, ok, contract, state_before, verification, warnings,
                               {"elapsed_s": round(time.perf_counter()-t0,3)},
                               V24_STAGE_GRASPED if ok else None)

        if name == "lift":
            if state_before.get("stage") != V24_STAGE_GRASPED or not state_before.get("active_object"):
                return _v24_precondition_failure(name, args, f"requires GRASP_ESTABLISHED; current={state_before}", contract, state_before)
            llm_require_arm_controller()
            obj = str(state_before["active_object"])
            dz = max(0.04, min(0.15, float(args.get("height_m", 0.10))))
            tcp_before, _ = skill_core.current_tcp_pose(); obj_before, _ = skill_core.object_pose(obj)
            target = [float(tcp_before[0]), float(tcp_before[1]), float(tcp_before[2]) + dz]
            verification, cart_ok, warnings = _v24_run_motion(
                f"lift({dz:.3f}m)", lambda: skill_core.bio_lift(dz), target, 0.012, True
            )
            time.sleep(0.10)
            tcp_after, _ = skill_core.current_tcp_pose(); obj_after, _ = skill_core.object_pose(obj)
            tcp_dz = float(tcp_after[2])-float(tcp_before[2]); obj_dz = float(obj_after[2])-float(obj_before[2])
            rel_before = _v24_sub3(obj_before, tcp_before); rel_after = _v24_sub3(obj_after, tcp_after)
            follow_drift = _v24_xyz_distance(rel_after, rel_before)
            required_rise = max(0.020, min(0.060, 0.50*dz))
            object_follow_ok = obj_dz >= required_rise and follow_drift <= 0.040
            verification.update({
                "active_object": obj, "requested_lift_mm": round(dz*1000.0,1),
                "actual_tcp_rise_mm": round(tcp_dz*1000.0,2), "object_rise_mm": round(obj_dz*1000.0,2),
                "required_object_rise_mm": round(required_rise*1000.0,2), "object_tcp_follow_drift_mm": round(follow_drift*1000.0,2),
                "max_follow_drift_mm": 40.0, "hold_active": bool(hold_active_shared.value), "object_motion_required": True,
            })
            if not bool(hold_active_shared.value): warnings.append("hold_active=false after lift; object-follow verification is therefore especially important.")
            ok = bool(cart_ok and object_follow_ok)
            if cart_ok and not object_follow_ok: verification["reason"] = "OBJECT_DID_NOT_FOLLOW_LIFT"
            return _v24_result(name, ok, contract, state_before, verification, warnings,
                               {"height_m": dz, "elapsed_s": round(time.perf_counter()-t0,3)},
                               V24_STAGE_LIFTED if ok else None)

        if name == "move_above_bin":
            if state_before.get("stage") != V24_STAGE_LIFTED or not state_before.get("active_object"):
                return _v24_precondition_failure(name, args, f"requires LIFTED with active object; current={state_before}", contract, state_before)
            llm_require_arm_controller()
            obj = str(state_before["active_object"]); bin_name = str(args["bin_name"])
            tcp_before, _ = skill_core.current_tcp_pose(); obj_before, _ = skill_core.object_pose(obj)
            rel_before = _v24_sub3(obj_before, tcp_before)
            bx, by, rim_z = [float(v) for v in BINS[bin_name]]
            target = [bx, by, rim_z + float(BIN_RELEASE_CLEARANCE)]
            verification, cart_ok, warnings = _v24_run_motion(
                f"move_above_bin({bin_name})", lambda: skill_core.bio_move_above_bin(bin_name), target, 0.035, True
            )
            time.sleep(0.10)
            tcp_after, _ = skill_core.current_tcp_pose(); obj_after, _ = skill_core.object_pose(obj)
            rel_after = _v24_sub3(obj_after, tcp_after)
            follow_drift = _v24_xyz_distance(rel_after, rel_before)
            object_bin_xy = _v24_xy_distance(obj_after, [bx,by,0.0])
            transport_ok = follow_drift <= 0.050 and object_bin_xy <= 0.060
            verification.update({
                "active_object": obj, "target_bin": bin_name, "object_xy_error_to_bin_mm": round(object_bin_xy*1000.0,2),
                "max_object_xy_error_mm": 60.0, "object_tcp_follow_drift_mm": round(follow_drift*1000.0,2),
                "max_follow_drift_mm": 50.0, "hold_active": bool(hold_active_shared.value), "object_motion_required": True,
            })
            if not bool(hold_active_shared.value): warnings.append("hold_active=false during/after bin transport; check object-follow metrics.")
            ok = bool(cart_ok and transport_ok)
            if cart_ok and not transport_ok: verification["reason"] = "OBJECT_DID_NOT_REMAIN_WITH_GRIPPER_TO_BIN"
            return _v24_result(name, ok, contract, state_before, verification, warnings,
                               {"bin_name": bin_name, "elapsed_s": round(time.perf_counter()-t0,3)},
                               V24_STAGE_BIN_HOVER if ok else None, "__KEEP__", bin_name if ok else "__KEEP__")

        if name == "open_gripper":
            if state_before.get("stage") != V24_STAGE_BIN_HOVER or not state_before.get("active_object") or not state_before.get("target_bin"):
                return _v24_precondition_failure(name, args, f"requires BIN_HOVER_REACHED with active object/bin; current={state_before}", contract, state_before)
            obj = str(state_before["active_object"]); bin_name = str(state_before["target_bin"])
            obj_before, _ = skill_core.object_pose(obj)
            skill_core.emit("[V2-6 GRIPPER] RELEASE -> detach HOLD + 100 mm opening")
            hold_off_request.value = True; gripper_force_mode.value = False; gripper_target_mm.value = 100.0
            deadline = time.time()+2.5
            while time.time()<deadline:
                if float(gripper_actual_mm.value)>=80.0 and not bool(hold_active_shared.value): break
                time.sleep(0.05)
            time.sleep(0.70)
            obj_after, _ = skill_core.object_pose(obj)
            opening = float(gripper_actual_mm.value); holding = bool(hold_active_shared.value)
            object_move = _v24_xyz_distance(obj_after, obj_before)
            ok = opening >= 80.0 and not holding
            verification = {
                "active_object": obj, "target_bin": bin_name, "opening_mm": round(opening,2),
                "hold_active": holding, "object_motion_after_release_mm": round(object_move*1000.0,2),
                "placement_verified": False,
            }
            if not ok: verification["reason"] = "RELEASE_NOT_CONFIRMED"
            return _v24_result(name, ok, contract, state_before, verification, [],
                               {"elapsed_s": round(time.perf_counter()-t0,3)},
                               V24_STAGE_RELEASED if ok else None)

        if name == "verify_object_in_bin":
            obj = str(args["object_name"]); bin_name = str(args["bin_name"])
            xyz, _ = skill_core.object_pose(obj); location = llm_classify_object_location(xyz)
            same_active = state_before.get("active_object") in (None, obj)
            same_bin = state_before.get("target_bin") in (None, bin_name)
            ok = bool(location == bin_name and not bool(hold_active_shared.value))
            warnings = []
            if not same_active: warnings.append(f"Verification object {obj} differs from state active_object={state_before.get('active_object')}")
            if not same_bin: warnings.append(f"Verification bin {bin_name} differs from state target_bin={state_before.get('target_bin')}")
            verification = {
                "object_name": obj, "requested_bin": bin_name, "actual_location": location,
                "xyz_m": [round(float(v),5) for v in xyz], "hold_active": bool(hold_active_shared.value),
                "placement_verified": bool(ok),
            }
            if not ok: verification["reason"] = "OBJECT_NOT_CONFIRMED_IN_TARGET_BIN"
            return _v24_result(name, ok, contract, state_before, verification, warnings,
                               {"object_name": obj, "requested_bin": bin_name, "elapsed_s": round(time.perf_counter()-t0,3)},
                               V24_STAGE_VERIFIED if ok else None, obj if ok else "__KEEP__", bin_name if ok else "__KEEP__")

        raise RuntimeError(f"Unknown V2-6 Agent tool: {name}")

    if not llm_agent_error:
        try:
            llm_agent = NeroLLMAgent(
                emit=(skill_core.emit if skill_core is not None else append_gui_log),
                tool_runner=llm_tool_runner,
            )
            append_gui_log("[LLM INIT] NERO V2-6 Skill-Contract agent module loaded")
        except Exception as exc:
            llm_agent_error = str(exc)
            append_gui_log(f"[LLM INIT ERROR] {exc}")

    # ----- Configuration -----
    llm_cfg = ttk.LabelFrame(llm_outer, text="NERO V2-6 LLM planner / reasoning / API")
    llm_cfg.pack(fill="x", pady=(0, 8))

    # Public release: do not hard-code the planner to a particular branded
    # model family.  The field is editable; optional comma-separated presets can
    # be supplied with NERO_LLM_MODEL_PRESETS, and "Refresh API Models" can query
    # the IDs visible to the current API key.
    _preset_env = os.environ.get("NERO_LLM_MODEL_PRESETS", "")
    curated_models = [x.strip() for x in _preset_env.split(",") if x.strip()]
    if DEFAULT_MODEL not in curated_models:
        curated_models.insert(0, DEFAULT_MODEL)
    llm_model_var = tk.StringVar(value=os.environ.get("NERO_LLM_MODEL", os.environ.get("NERO_ASTRA_MODEL", DEFAULT_MODEL)))
    llm_effort_var = tk.StringVar(value="low")
    llm_max_steps_var = tk.StringVar(value="48")
    llm_dry_run_var = tk.BooleanVar(value=False)
    llm_api_key_var = tk.StringVar(value="")

    ttk.Label(llm_cfg, text="LLM model (editable)").grid(row=0, column=0, padx=(8, 3), pady=7, sticky="w")
    llm_model_combo = ttk.Combobox(
        llm_cfg, textvariable=llm_model_var, values=curated_models,
        state="normal", width=24,
    )
    llm_model_combo.grid(row=0, column=1, padx=(0, 6), pady=7, sticky="w")
    ttk.Button(
        llm_cfg, text="Refresh API Models",
        command=lambda: llm_refresh_models_cmd(),
    ).grid(row=0, column=2, padx=(0, 10), pady=7, sticky="w")

    ttk.Label(llm_cfg, text="Reasoning").grid(row=0, column=3, padx=(3, 3), pady=7)
    ttk.Combobox(
        llm_cfg, textvariable=llm_effort_var,
        values=("none", "low", "medium", "high", "xhigh", "max"),
        state="readonly", width=9,
    ).grid(row=0, column=4, padx=(0, 10), pady=7)

    ttk.Label(llm_cfg, text="Max tool steps").grid(row=0, column=5, padx=(3, 3), pady=7)
    ttk.Spinbox(
        llm_cfg, textvariable=llm_max_steps_var,
        from_=1, to=60, increment=1, width=6,
    ).grid(row=0, column=6, padx=(0, 10), pady=7)
    ttk.Checkbutton(
        llm_cfg, text="Dry run", variable=llm_dry_run_var,
    ).grid(row=0, column=7, padx=8, pady=7, sticky="w")

    ttk.Label(llm_cfg, text="API key (optional; blank = OPENAI_API_KEY)").grid(
        row=1, column=0, columnspan=2, padx=(8, 3), pady=(0, 7), sticky="w"
    )
    llm_key_entry = ttk.Entry(llm_cfg, textvariable=llm_api_key_var, show="*", width=44)
    llm_key_entry.grid(row=1, column=2, columnspan=3, padx=(0, 10), pady=(0, 7), sticky="w")
    ttk.Label(
        llm_cfg,
        text="Refresh lists every model ID visible to your API key. Unknown/specialized models may not support Responses function calling.",
        wraplength=620, justify="left",
    ).grid(row=1, column=5, columnspan=3, padx=(0, 8), pady=(0, 7), sticky="w")

    # ----- Task prompt -----
    llm_task_box = ttk.LabelFrame(llm_outer, text="Natural-language task")
    llm_task_box.pack(fill="x", pady=(0, 8))
    llm_task_var = tk.StringVar(value="Put the red ball into the red bin.")
    llm_task_entry = ttk.Entry(llm_task_box, textvariable=llm_task_var)
    llm_task_entry.pack(fill="x", padx=8, pady=(8, 5))

    llm_examples = ttk.Frame(llm_task_box)
    llm_examples.pack(fill="x", padx=8, pady=(0, 8))
    ttk.Label(llm_examples, text="Examples:").pack(side="left")
    for label, prompt in (
        ("Red ball → red bin", "Put the red ball into the red bin."),
        ("Blue ball → blue bin", "Put the blue ball into the blue bin."),
        ("Sort both balls", "Put both balls into their matching-color bins."),
        ("Sort all 4 objects", "Sort all four objects into their matching-color bins."),
    ):
        ttk.Button(
            llm_examples,
            text=label,
            command=lambda p=prompt: llm_task_var.set(p),
        ).pack(side="left", padx=4)

    llm_status_state = {"text": "LLM agent: IDLE"}
    llm_ui_worker_lock = threading.Lock()

    def llm_capture_config():
        try:
            steps = int(llm_max_steps_var.get())
        except Exception:
            steps = 48
            llm_max_steps_var.set("48")
        return {
            "model": llm_model_var.get().strip() or "gpt-6.1-sol",
            "api_key": llm_api_key_var.get().strip(),
            "reasoning_effort": llm_effort_var.get().strip() or "medium",
            "max_tool_steps": max(1, min(60, steps)),
            "dry_run": bool(llm_dry_run_var.get()),
            "task": llm_task_var.get().strip(),
        }

    def llm_background(label, fn, lock_motion=False):
        if llm_agent is None:
            append_gui_log(f"[LLM UI ERROR] Agent unavailable: {llm_agent_error}")
            return
        if not llm_ui_worker_lock.acquire(blocking=False):
            append_gui_log("[LLM UI] Another LLM/V2 debug job is already BUSY")
            return
        if llm_agent.running:
            llm_ui_worker_lock.release()
            append_gui_log("[LLM UI] Agent is already BUSY")
            return
        if lock_motion and skill_core is not None and skill_core.busy:
            llm_ui_worker_lock.release()
            append_gui_log("[LLM UI] V69 skill core is BUSY; wait or STOP + RECOVER")
            return

        llm_status_state["text"] = f"LLM agent: {label}..."

        def worker():
            if lock_motion and skill_core is not None:
                skill_core._set_busy(True)
            try:
                fn()
                llm_status_state["text"] = f"LLM agent: {label} complete"
            except Exception as exc:
                if skill_core is not None:
                    skill_core.emit(f"[LLM UI FAILED] {label}: {exc}")
                else:
                    append_gui_log(f"[LLM UI FAILED] {label}: {exc}")
                llm_status_state["text"] = f"LLM agent: FAILED — {exc}"
            finally:
                if lock_motion and skill_core is not None:
                    skill_core._set_busy(False)
                llm_ui_worker_lock.release()

        threading.Thread(target=worker, daemon=True, name=f"nero_v2_4_2_{label}").start()

    def llm_refresh_models_cmd():
        cfg = llm_capture_config()
        def run():
            ids = llm_agent.list_available_models(api_key=cfg["api_key"])
            # Keep exact API-visible list, but place common general-purpose models first.
            ordered = []
            for mid in curated_models:
                if mid in ids and mid not in ordered:
                    ordered.append(mid)
            for mid in ids:
                if mid not in ordered:
                    ordered.append(mid)
            def apply_models():
                llm_model_combo.configure(values=ordered)
                append_gui_log(
                    f"[MODEL DISCOVERY] GUI updated with {len(ordered)} API-visible model IDs; "
                    f"current={llm_model_var.get()}"
                )
            root.after(0, apply_models)
        llm_background("refresh API models", run, lock_motion=False)

    def llm_preflight_estimate(cfg, show_dialog=True):
        scene = llm_scene_snapshot()
        try:
            scene["v2_skill_state"] = _v24_state_snapshot()
        except Exception:
            pass
        estimate = llm_agent.estimate_task_usage(
            cfg["task"], scene, model=cfg["model"],
            reasoning_effort=cfg["reasoning_effort"],
            max_tool_steps=cfg["max_tool_steps"],
        )
        skill_core.emit("[PRE-FLIGHT ESTIMATE] " + json.dumps(estimate, ensure_ascii=False, separators=(",", ":")))
        likely = estimate.get("likely_cost_usd")
        upper = estimate.get("conservative_cost_usd")
        if estimate.get("pricing_known"):
            price_line = f"Estimated cost: about ${likely:.4f} likely; <= ${upper:.4f} conservative uncached estimate"
        else:
            price_line = "Estimated cost: unavailable for this model ID (pricing not in local map)"
        message = (
            f"Model: {estimate['model']}\n"
            f"Reasoning: {estimate['reasoning_effort']}\n"
            f"Estimated objects: {estimate['objects_estimated']}\n"
            f"Estimated tool steps: {estimate['tool_steps_estimated']}\n"
            f"Estimated API requests: {estimate['api_requests_estimated']}\n"
            f"Estimated input tokens: {estimate['input_tokens_estimated']:,}\n"
            f"Estimated output tokens: {estimate['output_tokens_estimated']:,}\n"
            f"{price_line}\n\n"
            "This is a local heuristic, not a billing quote. Retries and longer reasoning can increase usage."
        )
        if show_dialog:
            return estimate, bool(messagebox.askyesno("NERO V2-6 API Pre-flight", message + "\n\nExecute this task?"))
        return estimate, message

    def llm_estimate_cmd():
        cfg = llm_capture_config()
        if not cfg["task"]:
            append_gui_log("[PRE-FLIGHT ESTIMATE] Task is empty")
            return
        try:
            _estimate, message = llm_preflight_estimate(cfg, show_dialog=False)
            messagebox.showinfo("NERO V2-6 API Estimate", message)
        except Exception as exc:
            append_gui_log(f"[PRE-FLIGHT ESTIMATE FAILED] {exc}")
            messagebox.showerror("NERO V2-6", f"Could not prepare task estimate:\n{exc}")

    def llm_test_api_cmd():
        cfg = llm_capture_config()
        llm_background(
            "API test",
            lambda: llm_agent.test_connection(
                model=cfg["model"], api_key=cfg["api_key"], reasoning_effort="low"
            ),
            lock_motion=False,
        )

    def llm_observe_cmd():
        def run():
            result = llm_tool_runner("observe_scene", {})
            skill_core.emit("[LLM MANUAL OBSERVE RESULT] " + json.dumps(result, ensure_ascii=False))
        llm_background("observe scene", run, lock_motion=False)

    def llm_plan_only_cmd():
        cfg = llm_capture_config()
        def run():
            scene = llm_scene_snapshot()
            try:
                scene["v2_skill_state"] = _v24_state_snapshot()
            except Exception:
                pass
            llm_agent.plan_only(
                cfg["task"], scene,
                model=cfg["model"], api_key=cfg["api_key"],
                reasoning_effort=cfg["reasoning_effort"],
            )
        llm_background("plan only", run, lock_motion=False)

    def llm_run_task_cmd():
        cfg = llm_capture_config()
        if not cfg["task"]:
            append_gui_log("[LLM UI] Task is empty")
            return
        try:
            estimate, confirmed = llm_preflight_estimate(cfg, show_dialog=True)
        except Exception as exc:
            append_gui_log(f"[PRE-FLIGHT ESTIMATE FAILED] {exc}")
            messagebox.showerror("NERO V2-6", f"Could not prepare task estimate:\n{exc}")
            return
        if not confirmed:
            append_gui_log(
                f"[TASK CANCELLED BY USER] model={cfg['model']} | task={cfg['task']!r} | "
                f"preflight_estimate={json.dumps(estimate, ensure_ascii=False, separators=(',', ':'))}"
            )
            return
        # V2-6: every new user-confirmed task starts a fresh logical episode
        # when no object is physically held.  This prevents ZERO ALL or a prior
        # failed task from leaking HOVER_REACHED / planning retry memory into the
        # next command.
        if not bool(hold_active_shared.value):
            _v24_reset_skill_state("new RUN TASK", clear_adaptation=True)
        else:
            append_gui_log(
                "[V2-6 NEW TASK] hold_active=true; preserving current grasp/task "
                "state rather than erasing an object that is physically held"
            )
        append_gui_log(
            f"[TASK CONFIRMED BY USER] model={cfg['model']} | task={cfg['task']!r} | "
            "fresh_episode=true | safety_retry_budget=1/object | planning_retry_budget=1/object"
        )
        llm_background(
            "RUN TASK",
            lambda: llm_agent.run_task(
                cfg["task"],
                model=cfg["model"],
                api_key=cfg["api_key"],
                reasoning_effort=cfg["reasoning_effort"],
                max_tool_steps=cfg["max_tool_steps"],
                dry_run=cfg["dry_run"],
            ),
            lock_motion=not cfg["dry_run"],
        )

    def llm_stop_cmd():
        if llm_agent is not None:
            llm_agent.request_stop()
        if skill_core is not None:
            try:
                skill_core.force_recover("NERO V2-6 AGENT STOP")
            except Exception as exc:
                append_gui_log(f"[LLM STOP WARN] {exc}")
        recover_request.value = True
        try:
            _v24_reset_skill_state("STOP + ARM RECOVER")
        except Exception as exc:
            append_gui_log(f"[V2-6 STATE RESET WARN] {exc}")
        llm_status_state["text"] = "Agent: STOP requested / arm recovery requested / skill state reset"

    def llm_reset_context_cmd():
        if llm_agent is None:
            return
        try:
            llm_agent.reset_context()
            llm_status_state["text"] = "LLM agent: context reset"
        except Exception as exc:
            append_gui_log(f"[LLM CONTEXT ERROR] {exc}")

    llm_control = ttk.LabelFrame(llm_outer, text="LLM agent controls")
    llm_control.pack(fill="x", pady=(0, 8))
    ttk.Button(llm_control, text="Test LLM API", command=llm_test_api_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(llm_control, text="Observe Scene", command=llm_observe_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(llm_control, text="Plan Only", command=llm_plan_only_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(llm_control, text="Estimate Cost", command=llm_estimate_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(llm_control, text="RUN TASK", command=llm_run_task_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(llm_control, text="STOP AGENT + ARM RECOVER", command=llm_stop_cmd).pack(side="left", padx=12, pady=8)
    ttk.Button(llm_control, text="Reset Agent Context", command=llm_reset_context_cmd).pack(side="left", padx=5, pady=8)

    # ----- Manual V2 tool debugger: executes exactly the same callbacks LLM sees -----
    llm_debug = ttk.LabelFrame(llm_outer, text="V2-6 tool debugger — Skill Contracts + Failure-Aware Recovery")
    llm_debug.pack(fill="x", pady=(0, 8))
    llm_debug_object_var = tk.StringVar(value="red_ball")
    llm_debug_bin_var = tk.StringVar(value="red_bin")
    ttk.Label(llm_debug, text="Object").pack(side="left", padx=(8, 3), pady=8)
    ttk.Combobox(
        llm_debug, textvariable=llm_debug_object_var,
        values=OBJECTS, state="readonly", width=13,
    ).pack(side="left", padx=(0, 8), pady=8)
    ttk.Label(llm_debug, text="Bin").pack(side="left", padx=(3, 3), pady=8)
    ttk.Combobox(
        llm_debug, textvariable=llm_debug_bin_var,
        values=("red_bin", "blue_bin"), state="readonly", width=11,
    ).pack(side="left", padx=(0, 8), pady=8)

    def llm_manual_tool(name, args_fn, motion=True):
        def cmd():
            args = args_fn()
            def run():
                result = llm_tool_runner(name, args)
                skill_core.emit(
                    f"[V2 MANUAL TOOL RESULT] {name} | "
                    + json.dumps(result, ensure_ascii=False)
                )
            llm_background(f"manual {name}", run, lock_motion=motion)
        return cmd

    ttk.Button(
        llm_debug, text="Prep Aperture",
        command=llm_manual_tool(
            "set_gripper_opening",
            lambda: {"target_opening_mm": _v24_gripper_prior(llm_debug_object_var.get())["recommended_target_opening_mm"]},
        ),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Move Above",
        command=llm_manual_tool("move_above", lambda: {"object_name": llm_debug_object_var.get()}),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Approach",
        command=llm_manual_tool("approach", lambda: {"object_name": llm_debug_object_var.get()}),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Safety Recover 1x",
        command=llm_manual_tool(
            "recover_from_safety",
            lambda: {
                "object_name": llm_debug_object_var.get(),
                "strategy": "widen_aperture_and_retry_once",
                "diagnosis": "Manual debug: pre-grasp contact/safety stop; widen aperture and retry once",
            },
        ),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Clamp 5N",
        command=llm_manual_tool("clamp_gripper", lambda: {"force_n": 5.0}),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Lift 100mm",
        command=llm_manual_tool("lift", lambda: {"height_m": 0.10}),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Move Above Bin",
        command=llm_manual_tool("move_above_bin", lambda: {"bin_name": llm_debug_bin_var.get()}),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Release",
        command=llm_manual_tool("open_gripper", lambda: {}),
    ).pack(side="left", padx=3, pady=8)
    ttk.Button(
        llm_debug, text="Verify",
        command=llm_manual_tool(
            "verify_object_in_bin",
            lambda: {"object_name": llm_debug_object_var.get(), "bin_name": llm_debug_bin_var.get()},
            motion=False,
        ),
    ).pack(side="left", padx=3, pady=8)

    llm_util = ttk.LabelFrame(llm_outer, text="V2-6 Skill Contract / Safety Reflection diagnostics")
    llm_util.pack(fill="x", pady=(0, 8))

    def llm_show_tools_cmd():
        if not LLM_TOOLS:
            append_gui_log("[LLM TOOLS] Tool schema unavailable")
            return
        summary = [f"{t.get('name')}: {t.get('description')}" for t in LLM_TOOLS]
        append_gui_log("[LLM TOOL SCHEMA]\n  - " + "\n  - ".join(summary))

    ttk.Button(llm_util, text="Show Skill Contracts", command=llm_show_tools_cmd).pack(side="left", padx=5, pady=7)

    def llm_show_skill_state_cmd():
        append_gui_log("[V2-6 SKILL STATE] " + json.dumps(_v24_state_snapshot(), ensure_ascii=False))

    def llm_reset_skill_state_cmd():
        if llm_agent is not None and llm_agent.running:
            append_gui_log("[V2-6 STATE RESET BLOCKED] Agent is BUSY")
            return
        _v24_reset_skill_state("GUI manual reset")

    ttk.Button(llm_util, text="Show Skill State", command=llm_show_skill_state_cmd).pack(side="left", padx=5, pady=7)
    ttk.Button(llm_util, text="Reset Skill State", command=llm_reset_skill_state_cmd).pack(side="left", padx=5, pady=7)
    ttk.Button(llm_util, text="ZERO ALL", command=zero_all).pack(side="left", padx=5, pady=7)
    ttk.Button(llm_util, text="STOP + RECOVER", command=emergency_recover).pack(side="left", padx=5, pady=7)
    ttk.Button(llm_util, text="HARD RESET 0", command=hard_reset_zero).pack(side="left", padx=5, pady=7)

    def poll_llm_status():
        try:
            if llm_agent is None:
                llm_status_var.set(f"LLM agent ERROR: {llm_agent_error}")
            else:
                key_ok, key_text = llm_agent.api_key_status(llm_api_key_var.get())
                running_text = "BUSY" if llm_agent.running else "IDLE"
                extra = llm_status_state.get("text", "")
                llm_status_var.set(
                    f"Agent: {running_text} | model={llm_model_var.get().strip() or 'gpt-6.1-sol'} | "
                    f"API={'READY' if key_ok else 'NOT CONFIGURED'} | skill={_v24_state_snapshot().get('stage')} | {extra}"
                )
        except Exception as exc:
            llm_status_var.set(f"LLM status error: {exc}")
        finally:
            try:
                root.after(300, poll_llm_status)
            except Exception:
                pass

    root.after(300, poll_llm_status)

    # ========================================================
    # V69 BIOIK PRIMARY TAB
    # ========================================================
    bio_outer = ttk.Frame(bio_tab)
    bio_outer.pack(fill="both", expand=True, padx=12, pady=12)

    ttk.Label(
        bio_outer,
        text="BioIK Primary Planner",
        font=("Arial", 18, "bold"),
    ).pack(anchor="w", pady=(0, 5))

    ttk.Label(
        bio_outer,
        text=(
            "Primary solver: BioIK local optimizer (gd_c) with seed-near / joint-limit "
            "regularization. Move Above Bin uses BioIK incremental continuation first and "
            "the independent TRAC-IK service only as a bounded fallback."
        ),
        wraplength=1250,
        justify="left",
    ).pack(anchor="w", pady=(0, 5))

    bio_status_var = tk.StringVar(value="BioIK status: waiting...")
    ttk.Label(
        bio_outer, textvariable=bio_status_var, font=("Arial", 10, "bold")
    ).pack(anchor="w", pady=(0, 8))

    bio_params = ttk.LabelFrame(bio_outer, text="BioIK target / seed / request")
    bio_params.pack(fill="x", pady=(0, 8))

    bio_object_var = tk.StringVar(value="red_cube")
    bio_height_mm_var = tk.StringVar(value="120")
    bio_yaw_deg_var = tk.StringVar(value="0")
    bio_timeout_var = tk.StringVar(value="0.05")
    bio_runs_var = tk.StringVar(value="20")
    bio_seed_var = tk.StringVar(value="current")
    bio_collision_var = tk.BooleanVar(value=True)

    ttk.Label(bio_params, text="Object").grid(row=0, column=0, padx=(8,2), pady=8)
    ttk.Combobox(
        bio_params, textvariable=bio_object_var, values=OBJECTS,
        state="readonly", width=19,
    ).grid(row=0, column=1, padx=(0,10), pady=8)
    ttk.Label(bio_params, text="Above mm").grid(row=0, column=2, padx=(3,2), pady=8)
    ttk.Entry(bio_params, textvariable=bio_height_mm_var, width=8).grid(row=0, column=3, padx=(0,10), pady=8)
    ttk.Label(bio_params, text="Yaw deg").grid(row=0, column=4, padx=(3,2), pady=8)
    ttk.Entry(bio_params, textvariable=bio_yaw_deg_var, width=8).grid(row=0, column=5, padx=(0,10), pady=8)
    ttk.Label(bio_params, text="Seed").grid(row=0, column=6, padx=(3,2), pady=8)
    ttk.Combobox(
        bio_params, textvariable=bio_seed_var,
        values=("current", "zero", "bent"), state="readonly", width=10,
    ).grid(row=0, column=7, padx=(0,10), pady=8)
    ttk.Label(bio_params, text="Request timeout s").grid(row=0, column=8, padx=(3,2), pady=8)
    ttk.Entry(bio_params, textvariable=bio_timeout_var, width=8).grid(row=0, column=9, padx=(0,10), pady=8)
    ttk.Label(bio_params, text="Benchmark runs").grid(row=1, column=0, padx=(8,2), pady=(0,8))
    ttk.Entry(bio_params, textvariable=bio_runs_var, width=8).grid(row=1, column=1, padx=(0,10), pady=(0,8), sticky="w")
    ttk.Checkbutton(
        bio_params, text="avoid_collisions", variable=bio_collision_var,
    ).grid(row=1, column=2, columnspan=2, padx=(3,10), pady=(0,8), sticky="w")
    ttk.Label(
        bio_params,
        text="BioIK YAML: gd_c + minimal displacement + avoid joint limits + center joints",
    ).grid(row=1, column=4, columnspan=6, padx=(3,8), pady=(0,8), sticky="w")

    def bio_values():
        try:
            h = float(bio_height_mm_var.get()) / 1000.0
            yaw = float(bio_yaw_deg_var.get())
            timeout_s = float(bio_timeout_var.get())
            runs = int(bio_runs_var.get())
        except Exception as exc:
            if skill_core is not None:
                skill_core.emit(f"[BIO] invalid parameter: {exc}")
            return None
        if h <= 0.0 or timeout_s <= 0.0 or runs <= 0:
            if skill_core is not None:
                skill_core.emit("[BIO] height, timeout and runs must be > 0")
            return None
        return h, yaw, timeout_s, runs

    bio_diag = ttk.LabelFrame(bio_outer, text="BioIK diagnostics")
    bio_diag.pack(fill="x", pady=(0,8))

    ttk.Button(
        bio_diag, text="Observe Scene",
        command=lambda: submit_skill("observe_scene"),
    ).pack(side="left", padx=5, pady=8)

    def bio_self_test_cmd():
        vals = bio_values()
        if vals is None: return
        submit_skill(
            "bio_self_test", bio_seed_var.get(), bool(bio_collision_var.get()), vals[2]
        )

    def bio_probe_cmd():
        vals = bio_values()
        if vals is None: return
        submit_skill(
            "bio_object_probe", bio_object_var.get(), vals[0], vals[1],
            bio_seed_var.get(), bool(bio_collision_var.get()), vals[2]
        )

    def bio_bench_cmd():
        vals = bio_values()
        if vals is None: return
        submit_skill(
            "bio_benchmark", bio_object_var.get(), vals[0], vals[1],
            bio_seed_var.get(), bool(bio_collision_var.get()), vals[2], vals[3]
        )

    ttk.Button(bio_diag, text="Self-Test Current TCP", command=bio_self_test_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(bio_diag, text="Probe Object IK", command=bio_probe_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(bio_diag, text="Benchmark IK", command=bio_bench_cmd).pack(side="left", padx=5, pady=8)

    bio_skill = ttk.LabelFrame(bio_outer, text="BioIK manipulation")
    bio_skill.pack(fill="x", pady=(0,8))

    def bio_move_above_cmd():
        vals = bio_values()
        if vals is None: return
        submit_skill("bio_move_above", bio_object_var.get(), vals[0])

    ttk.Button(bio_skill, text="Move Above (BIO)", command=bio_move_above_cmd).pack(side="left", padx=5, pady=8)
    ttk.Button(
        bio_skill, text="Approach (BIO)",
        command=lambda: submit_skill("bio_approach", bio_object_var.get()),
    ).pack(side="left", padx=5, pady=8)
    ttk.Button(
        bio_skill, text="Gripper FAST CLAMP", command=one_click_clamp,
    ).pack(side="left", padx=5, pady=8)
    ttk.Button(
        bio_skill, text="Lift 100 mm", command=lambda: submit_skill("bio_lift", 0.10),
    ).pack(side="left", padx=5, pady=8)

    bio_bin_row = ttk.Frame(bio_outer)
    bio_bin_row.pack(fill="x", pady=(0,8))
    ttk.Label(bio_bin_row, text="Release bin:").pack(side="left")
    bio_bin_var = tk.StringVar(value="red_bin")
    ttk.Combobox(
        bio_bin_row, textvariable=bio_bin_var,
        values=("red_bin", "blue_bin"), state="readonly", width=14,
    ).pack(side="left", padx=(5,10))
    ttk.Button(
        bio_bin_row, text="Move Above Bin (BIO; stop at safe frontier if blocked)",
        command=lambda: submit_skill("bio_move_above_bin", bio_bin_var.get()),
    ).pack(side="left", padx=5, pady=5)
    ttk.Button(
        bio_bin_row, text="Gripper OPEN", command=one_click_release,
    ).pack(side="left", padx=5, pady=5)

    bio_util = ttk.LabelFrame(bio_outer, text="BioIK tab utilities")
    bio_util.pack(fill="x", pady=(0,8))
    ttk.Button(bio_util, text="ZERO ALL", command=zero_all).pack(side="left", padx=6, pady=7)
    ttk.Button(bio_util, text="STOP + RECOVER", command=emergency_recover).pack(side="left", padx=6, pady=7)
    ttk.Button(bio_util, text="HARD RESET 0", command=hard_reset_zero).pack(side="left", padx=6, pady=7)

    # ========================================================
    # V53 TRAC-IK LAB TAB
    # ========================================================
    trac_outer = ttk.Frame(trac_tab)
    trac_outer.pack(fill="both", expand=True, padx=12, pady=12)

    ttk.Label(
        trac_outer,
        text="TRAC-IK Solver Lab",
        font=("Arial", 18, "bold"),
    ).pack(anchor="w", pady=(0, 5))

    ttk.Label(
        trac_outer,
        text=(
            "TRAC-IK runs in an independent /trac_ref MoveIt instance. "
            "This keeps the primary BioIK stack untouched while providing a true "
            "solver baseline and bounded fallback."
        ),
        wraplength=1250,
        justify="left",
    ).pack(anchor="w", pady=(0, 5))

    trac_solver_name = os.environ.get("NERO_TRAC_IK_SOLVER", "TRAC-IK")
    trac_solve_type = os.environ.get("NERO_TRAC_IK_SOLVE_TYPE", "unknown")
    trac_config_timeout = os.environ.get("NERO_TRAC_IK_TIMEOUT", "unknown")

    trac_status_var = tk.StringVar(
        value=(
            f"Configured solver: {trac_solver_name} | "
            f"solve_type={trac_solve_type} | "
            f"config timeout={trac_config_timeout}s"
        )
    )
    ttk.Label(
        trac_outer,
        textvariable=trac_status_var,
        font=("Arial", 10, "bold"),
    ).pack(anchor="w", pady=(0, 8))

    trac_params = ttk.LabelFrame(
        trac_outer,
        text="IK test target / seed / request",
    )
    trac_params.pack(fill="x", pady=(0, 8))

    trac_object_var = tk.StringVar(value="red_cube")
    trac_height_mm_var = tk.StringVar(value="120")
    trac_yaw_deg_var = tk.StringVar(value="0")
    trac_timeout_var = tk.StringVar(value="0.10")
    trac_runs_var = tk.StringVar(value="20")
    trac_seed_var = tk.StringVar(value="current")
    trac_collision_var = tk.BooleanVar(value=True)

    ttk.Label(trac_params, text="Object").grid(
        row=0, column=0, padx=(8, 2), pady=8, sticky="w"
    )
    ttk.Combobox(
        trac_params,
        textvariable=trac_object_var,
        values=OBJECTS,
        state="readonly",
        width=19,
    ).grid(row=0, column=1, padx=(0, 10), pady=8)

    ttk.Label(trac_params, text="Above mm").grid(
        row=0, column=2, padx=(3, 2), pady=8
    )
    ttk.Entry(
        trac_params,
        textvariable=trac_height_mm_var,
        width=8,
    ).grid(row=0, column=3, padx=(0, 10), pady=8)

    ttk.Label(trac_params, text="Yaw deg").grid(
        row=0, column=4, padx=(3, 2), pady=8
    )
    ttk.Entry(
        trac_params,
        textvariable=trac_yaw_deg_var,
        width=8,
    ).grid(row=0, column=5, padx=(0, 10), pady=8)

    ttk.Label(trac_params, text="Seed").grid(
        row=0, column=6, padx=(3, 2), pady=8
    )
    ttk.Combobox(
        trac_params,
        textvariable=trac_seed_var,
        values=("current", "zero", "bent"),
        state="readonly",
        width=10,
    ).grid(row=0, column=7, padx=(0, 10), pady=8)

    ttk.Label(trac_params, text="Request timeout s").grid(
        row=0, column=8, padx=(3, 2), pady=8
    )
    ttk.Entry(
        trac_params,
        textvariable=trac_timeout_var,
        width=8,
    ).grid(row=0, column=9, padx=(0, 10), pady=8)

    ttk.Label(trac_params, text="Benchmark runs").grid(
        row=1, column=0, padx=(8, 2), pady=(0, 8), sticky="w"
    )
    ttk.Entry(
        trac_params,
        textvariable=trac_runs_var,
        width=8,
    ).grid(row=1, column=1, padx=(0, 10), pady=(0, 8), sticky="w")

    ttk.Checkbutton(
        trac_params,
        text="avoid_collisions",
        variable=trac_collision_var,
    ).grid(
        row=1, column=2, columnspan=2,
        padx=(3, 10), pady=(0, 8), sticky="w"
    )

    ttk.Label(
        trac_params,
        text=(
            "Seeds: current = live J1..J7; zero = all zero; "
            "bent = [0, 0.4, 0, 0.2, 0, 0, 0]"
        ),
    ).grid(
        row=1, column=4, columnspan=6,
        padx=(3, 8), pady=(0, 8), sticky="w"
    )

    def trac_values():
        try:
            h = float(trac_height_mm_var.get()) / 1000.0
            yaw = float(trac_yaw_deg_var.get())
            timeout_s = float(trac_timeout_var.get())
            runs = int(trac_runs_var.get())
        except Exception as exc:
            if skill_core is not None:
                skill_core.emit(f"[TRAC] invalid parameter: {exc}")
            return None

        if h <= 0.0 or timeout_s <= 0.0 or runs <= 0:
            if skill_core is not None:
                skill_core.emit(
                    "[TRAC] height, timeout and runs must be > 0"
                )
            return None

        return h, yaw, timeout_s, runs

    trac_actions = ttk.LabelFrame(
        trac_outer,
        text="TRAC-IK diagnostics",
    )
    trac_actions.pack(fill="x", pady=(0, 8))

    def trac_self_test_cmd():
        vals = trac_values()
        if vals is None:
            return
        _, _, timeout_s, _ = vals
        submit_skill(
            "trac_self_test",
            trac_seed_var.get(),
            bool(trac_collision_var.get()),
            timeout_s,
        )

    def trac_probe_cmd():
        vals = trac_values()
        if vals is None:
            return
        h, yaw, timeout_s, _ = vals
        submit_skill(
            "trac_object_probe",
            trac_object_var.get(),
            h,
            yaw,
            trac_seed_var.get(),
            bool(trac_collision_var.get()),
            timeout_s,
        )

    def trac_bench_cmd():
        vals = trac_values()
        if vals is None:
            return
        h, yaw, timeout_s, runs = vals
        submit_skill(
            "trac_benchmark",
            trac_object_var.get(),
            h,
            yaw,
            trac_seed_var.get(),
            bool(trac_collision_var.get()),
            timeout_s,
            runs,
        )

    ttk.Button(
        trac_actions,
        text="Observe Scene",
        command=lambda: submit_skill("observe_scene"),
    ).pack(side="left", padx=5, pady=8)

    ttk.Button(
        trac_actions,
        text="Self-Test Current TCP",
        command=trac_self_test_cmd,
    ).pack(side="left", padx=5, pady=8)

    ttk.Button(
        trac_actions,
        text="Probe Object IK",
        command=trac_probe_cmd,
    ).pack(side="left", padx=5, pady=8)

    ttk.Button(
        trac_actions,
        text="Benchmark IK",
        command=trac_bench_cmd,
    ).pack(side="left", padx=5, pady=8)

    def trac_move_above_cmd():
        vals = trac_values()
        if vals is None:
            return
        h = vals[0]
        submit_skill(
            "trac_move_above",
            trac_object_var.get(),
            h,
        )

    ttk.Button(
        trac_actions,
        text="Move Above (TRAC)",
        command=trac_move_above_cmd,
    ).pack(side="left", padx=5, pady=8)

    ttk.Button(
        trac_actions,
        text="Approach (TRAC)",
        command=lambda: submit_skill(
            "trac_approach",
            trac_object_var.get(),
        ),
    ).pack(side="left", padx=5, pady=8)

    ttk.Button(
        trac_actions,
        text="Lift 100 mm",
        command=lambda: submit_skill(
            "trac_lift", 0.10
        ),
    ).pack(side="left", padx=5, pady=8)


    trac_bin_row = ttk.Frame(trac_outer)
    trac_bin_row.pack(fill="x", pady=(0, 8))
    ttk.Label(trac_bin_row, text="Release bin:").pack(side="left")
    trac_bin_var = tk.StringVar(value="red_bin")
    ttk.Combobox(
        trac_bin_row,
        textvariable=trac_bin_var,
        values=("red_bin", "blue_bin"),
        state="readonly",
        width=14,
    ).pack(side="left", padx=(5, 10))
    ttk.Button(
        trac_bin_row,
        text="Move Above Bin (stop at safe frontier)",
        command=lambda: submit_skill(
            "trac_move_above_bin", trac_bin_var.get()
        ),
    ).pack(side="left", padx=5, pady=5)
    ttk.Label(
        trac_bin_row,
        text="If blocked, V69 moves only to the last validated state for inspection.",
    ).pack(side="left", padx=(8, 0))

    trac_util = ttk.LabelFrame(
        trac_outer,
        text="Existing robot utilities",
    )
    trac_util.pack(fill="x", pady=(0, 8))

    ttk.Button(
        trac_util,
        text="Gripper OPEN",
        command=one_click_release,
    ).pack(side="left", padx=6, pady=7)

    ttk.Button(
        trac_util,
        text="Gripper FAST CLAMP",
        command=one_click_clamp,
    ).pack(side="left", padx=6, pady=7)

    ttk.Button(
        trac_util,
        text="ZERO ALL",
        command=zero_all,
    ).pack(side="left", padx=6, pady=7)

    ttk.Button(
        trac_util,
        text="STOP + RECOVER",
        command=emergency_recover,
    ).pack(side="left", padx=6, pady=7)

    ttk.Button(
        trac_util,
        text="HARD RESET 0",
        command=hard_reset_zero,
    ).pack(side="left", padx=6, pady=7)

    ttk.Label(
        trac_outer,
        text=(
            "Best first diagnosis: Self-Test Current TCP with collision OFF, "
            "then ON. Next Probe Object IK. Only after probes succeed, run Move Above."
        ),
        wraplength=1250,
        justify="left",
    ).pack(anchor="w", pady=(0, 5))

    ttk.Label(
        trac_outer,
        text="All TRAC / skill / recovery logs are shown in the unified log panel below the tabs.",
        font=("Arial", 10, "bold"),
    ).pack(anchor="w", pady=(4, 8))

    ttk.Label(
        skill_outer,
        text=(
            "Execution path: MoveIt planning → official arm_controller "
            "FollowJointTrajectory → arm_controller/controller_state → V31 MuJoCo controller"
        ),
    ).pack(anchor="w", pady=(0, 5))

    ttk.Label(
        skill_outer,
        text="All runtime logs are shown in the unified log panel below the tabs.",
        font=("Arial", 10, "bold"),
    ).pack(anchor="w", pady=(4, 8))

    # Launcher writes every startup/process/interface check to this file.
    # The GUI tails it so the user never has to guess whether ROS2,
    # MoveIt, controllers, services or RViz actually started.
    startup_status_file = Path(
        os.environ.get(
            "NERO_STATUS_FILE",
            str(
                Path.home()
                / "mujoco_ws"
                / "nero"
                / "logs"
                / "v62_startup_status.log"
            ),
        )
    )
    startup_status_state = {
        "offset": 0,
        "missing_reported": False,
    }

    def tail_startup_status():
        try:
            if not startup_status_file.exists():
                if not startup_status_state["missing_reported"]:
                    append_gui_log(
                        "[WAIT] Launcher status file not created yet: "
                        + str(startup_status_file)
                    )
                    startup_status_state["missing_reported"] = True
                return

            with startup_status_file.open(
                "r",
                encoding="utf-8",
                errors="replace",
            ) as f:
                f.seek(startup_status_state["offset"])
                chunk = f.read()
                startup_status_state["offset"] = f.tell()

            if chunk:
                for line in chunk.splitlines():
                    if line.strip():
                        append_gui_log(line)

        except Exception as exc:
            append_gui_log(
                "[WARN] Could not read launcher status log: "
                + str(exc)
            )

    skill_poll_state = {
        "last_safety_trip": int(safety_trip_count.value),
        "sim_ready_logged": False,
        "ros_bridge_logged": False,
        "move_ready_logged": False,
        "arm_ready_logged": False,
        "arm_was_ready": False,
        "cart_ready_logged": False,
        "all_skill_interfaces_logged": False,
    }

    initial_synced = {"done": False}

    def drain_runtime_log_watchdog():
        """Independent log pump; survives failures in the main GUI telemetry poll."""
        try:
            if skill_core is not None:
                for line in skill_core.drain_logs(limit=500):
                    append_gui_log(line)
        except Exception as exc:
            append_gui_log(f"[LOG WATCHDOG ERROR] {exc}")
        finally:
            try:
                root.after(80, drain_runtime_log_watchdog)
            except Exception:
                pass

    def poll():
        # First mirror launcher/process/interface startup checks into GUI.
        tail_startup_status()

        # Log local MuJoCo/bridge readiness once.
        if bool(sim_ready.value) and not skill_poll_state["sim_ready_logged"]:
            append_gui_log(
                "[OK] MuJoCo simulator process READY "
                f"(PID {sim_process.pid})"
            )
            skill_poll_state["sim_ready_logged"] = True

        if (
            bool(ros_bridge_ready.value)
            and not skill_poll_state["ros_bridge_logged"]
        ):
            append_gui_log(
                "[OK] MuJoCo ↔ ROS2 bridge READY | "
                "/feedback/joint_states publisher active | "
                "/arm_controller/controller_state primary bridge active; /control/joint_states fallback active"
            )
            skill_poll_state["ros_bridge_logged"] = True

        control_mode_var.set(
            "Arm input: MOVEIT EXECUTING"
            if bool(ros_arm_command_gate_shared.value)
            else "Arm input: MANUAL/IDLE"
        )

        # Unified MoveIt skill status/log.
        if skill_core is not None:
            move_ready, arm_ready, cart_ready = skill_core.readiness()

            # V69 runtime watchdog: if the action server disappears after being
            # ready, surface it immediately and block new motion requests.
            if skill_poll_state["arm_was_ready"] and not arm_ready:
                append_gui_log(
                    "[CRITICAL] Root arm_controller action server disappeared: "
                    "/arm_controller/follow_joint_trajectory. Motion buttons are blocked "
                    "until the controller is restored/restarted."
                )
                skill_poll_state["arm_ready_logged"] = False
                skill_poll_state["all_skill_interfaces_logged"] = False
            skill_poll_state["arm_was_ready"] = bool(arm_ready)

            if move_ready and not skill_poll_state["move_ready_logged"]:
                append_gui_log(
                    "[OK] MoveIt action READY: /move_action"
                )
                skill_poll_state["move_ready_logged"] = True

            if arm_ready and not skill_poll_state["arm_ready_logged"]:
                append_gui_log(
                    "[OK] Arm controller action READY: "
                    "/arm_controller/follow_joint_trajectory"
                )
                skill_poll_state["arm_ready_logged"] = True

            if cart_ready and not skill_poll_state["cart_ready_logged"]:
                append_gui_log(
                    "[OK] Cartesian service READY: "
                    "/compute_cartesian_path"
                )
                skill_poll_state["cart_ready_logged"] = True

            if (
                move_ready
                and arm_ready
                and cart_ready
                and not skill_poll_state["all_skill_interfaces_logged"]
            ):
                append_gui_log(
                    "[READY] V69 skill interfaces are fully connected"
                )
                skill_poll_state[
                    "all_skill_interfaces_logged"
                ] = True
            skill_ready_var.set(
                "MoveIt %s | arm_controller %s | Cartesian %s | %s"
                % (
                    "READY" if move_ready else "WAIT",
                    "READY" if arm_ready else "WAIT",
                    "READY" if cart_ready else "WAIT",
                    "BUSY" if skill_core.busy else "IDLE",
                )
            )

            bio_ik_ready, bio_solver, bio_mode, bio_cfg_timeout, bio_weights = (
                skill_core.bio_readiness()
            )
            bio_status_var.set(
                "Configured solver: %s | mode=%s | config timeout=%ss | %s | /compute_ik %s | %s"
                % (
                    bio_solver, bio_mode, bio_cfg_timeout, bio_weights,
                    "READY" if bio_ik_ready else "WAIT",
                    "BUSY" if skill_core.busy else "IDLE",
                )
            )

            trac_ik_ready, trac_solver, trac_type, trac_cfg_timeout = (
                skill_core.trac_readiness()
            )
            trac_status_var.set(
                "Configured solver: %s | solve_type=%s | "
                "config timeout=%ss | /trac_ref/compute_ik %s | %s"
                % (
                    trac_solver,
                    trac_type,
                    trac_cfg_timeout,
                    "READY" if trac_ik_ready else "WAIT",
                    "BUSY" if skill_core.busy else "IDLE",
                )
            )

            # V69: automatic safety events are NON-DESTRUCTIVE AND LATCHED.
            # Cancel the active trajectory and gate commands OFF, but DO NOT
            # clear the MuJoCo safety latch automatically.
            trip_now = int(safety_trip_count.value)
            if trip_now != skill_poll_state["last_safety_trip"]:
                skill_poll_state["last_safety_trip"] = trip_now
                try:
                    skill_core.cancel_active_motion()
                except Exception:
                    pass
                # V69: DO NOT set recover_request here.  An automatic safety
                # event must remain latched until the user explicitly presses
                # STOP + RECOVER (or HARD RESET 0).  This prevents stale
                # FollowJointTrajectory commands from re-entering immediately.
                ros_arm_command_gate_shared.value = False
                state_code_now = int(safety_state.value)
                state_text_now = {
                    1: "PROTECTED-SURFACE PENETRATION",
                    4: "RUNAWAY",
                }.get(state_code_now, f"STATE {state_code_now}")
                skill_core.emit(
                    f"[SAFETY STOP #{trip_now}] {state_text_now}; "
                    "arm frozen in place; gripper/grasp preserved; SAFETY LATCH HELD — press STOP + RECOVER"
                )

        # V42 MoveIt2 bridge telemetry.
        if bool(ros_bridge_ready.value):
            age = float(ros_last_cmd_age.value)
            age_text = (
                "no command yet"
                if age < 0.0
                else f"last cmd {age:.2f}s ago"
            )
            moveit_status_var.set(
                "ROS2 bridge: READY | "
                f"commands={int(ros_cmd_count.value)} "
                f"({int(ros_last_cmd_joint_count.value)} joints last) | "
                f"ctrl_state_rx={int(ros_controller_state_rx_shared.value)} | "
                f"fallback_rx={int(ros_fallback_joint_state_rx_shared.value)} | "
                f"src={ {0: 'none', 1: '/arm_controller/controller_state', 2: '/control/joint_states'}.get(int(ros_last_command_source_shared.value), 'unknown') } | "
                f"feedback={int(ros_feedback_count.value)} | "
                f"scene pubs={int(ros_scene_count.value)} | "
                f"{age_text}"
            )
        else:
            moveit_status_var.set(
                "ROS2 bridge: NOT READY — source /opt/ros/humble/setup.bash "
                "and agx_arm_ws/install/setup.bash before starting V56"
            )

        # Update controller settings from spinboxes.
        for i in range(7):
            try:
                speed = float(speed_vars[i].get())
                speed_shared[i] = max(1.0, min(100.0, speed))
            except Exception:
                pass

            try:
                wn = float(wn_vars[i].get())
                wn_shared[i] = max(0.05, min(50.0, wn))
            except Exception:
                pass

            try:
                zeta = float(zeta_vars[i].get())
                zeta_shared[i] = max(0.05, min(5.0, zeta))
            except Exception:
                pass

            try:
                qa = float(qacc_vars[i].get())
                qacc_shared[i] = max(0.05, min(200.0, qa))
            except Exception:
                pass

            update_gain_label(i)

            deg = math.degrees(float(actual_shared[i]))
            actual_labels[i].configure(
                text=f"Actual {deg:.1f}°"
            )

        gripper_actual_label.configure(
            text=f"Actual {float(gripper_actual_mm.value):.0f} mm"
        )

        # Keep force command synchronized with the GUI entry.
        try:
            force_n = float(gripper_force_var.get())
            gripper_force_N.value = max(0.5, min(50.0, force_n))
        except Exception:
            pass

        # Keep clamp-speed multiplier synchronized with the GUI entry.
        try:
            speed_mult = float(clamp_speed_var.get())
            clamp_speed_mult_shared.value = max(
                1.0,
                min(20.0, speed_mult),
            )
        except Exception:
            pass

        if bool(gripper_force_mode.value):
            gripper_mode_label.configure(
                text=(
                    f"Mode: FORCE LOOP → "
                    f"{float(gripper_force_N.value):.1f} N/finger "
                    f"| clamp "
                    f"{float(clamp_speed_mult_shared.value):.1f}×"
                )
            )

        contact_force_status_label.configure(
            text=(
                f"Contact: L {float(contact_left_n_shared.value):.2f} N"
                f" | R {float(contact_right_n_shared.value):.2f} N"
                f" | Tangential "
                f"{float(contact_tangent_n_shared.value):.2f} N"
                f" | Motor {float(contact_motor_cmd_shared.value):.2f} N"
            )
        )

        anti_slip_status_label.configure(
            text=(
                f"Slip: "
                f"{float(anti_slip_slip_mm_s_shared.value):+.2f} mm/s"
                f" | target boost "
                f"+{float(anti_slip_boost_shared.value):.2f} N"
                f" | pad μ fixed 10.00"
            )
        )

        code = int(hold_status_code_shared.value)

        if not bool(hold_enabled_shared.value):
            hold_text = "DISABLED"
        elif bool(hold_active_shared.value):
            hold_text = "HOLDING (3-DOF connect)"
        elif code == 2:
            hold_text = "NO CONTACT OBJECT - clamp first, then HOLD ON"
        else:
            hold_text = (
                f"MONITORING "
                f"{float(hold_ready_shared.value):.2f}/0.10s "
                f"(trigger <= -0.05 mm/s)"
            )

        hold_status_label.configure(
            text=f"Auto HOLD: {hold_text}"
        )

        # Once sim publishes initial pose, mirror GUI sliders once.
        if bool(sim_ready.value) and not initial_synced["done"]:
            for i in range(7):
                deg = math.degrees(float(actual_shared[i]))
                slider_vars[i].set(round(deg, 1))
                target_shared[i] = math.radians(round(deg, 1))
                target_labels[i].configure(
                    text=f"Target {deg:.1f}°"
                )
            grip_mm = float(gripper_actual_mm.value)
            gripper_var.set(grip_mm)
            gripper_target_mm.value = grip_mm
            gripper_target_label.configure(text=f"Target {grip_mm:.0f} mm")
            initial_synced["done"] = True

        # Crucially: GUI stays alive if simulation dies.
        if bool(sim_ready.value) and sim_process.is_alive():
            grav = "ON" if bool(gravity_enabled.value) else "OFF"
            state_code = int(safety_state.value)
            state_text = {
                0: "NORMAL",
                1: "AUTO ROLLBACK",
                2: "MANUAL RECOVER",
                3: "HARD RESET",
                4: "RUNAWAY ROLLBACK",
            }.get(state_code, f"STATE {state_code}")

            status_var.set(
                f"SIM RUNNING | Gravity {grav} | "
                f"Safety {state_text} | Trips {int(safety_trip_count.value)}"
            )
        elif not sim_process.is_alive():
            if int(sim_error_code.value) != 0:
                status_var.set(
                    "SIM EXITED WITH ERROR — GUI kept open; check terminal traceback"
                )
            else:
                status_var.set(
                    "SIMULATION WINDOW CLOSED — GUI kept open"
                )
        else:
            status_var.set("Starting MuJoCo simulation...")

        root.after(50, poll)

    def on_close():
        if skill_core is not None:
            try:
                skill_core.shutdown()
            except Exception:
                pass
        stop_event.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    root.after(50, poll)
    root.after(80, drain_runtime_log_watchdog)

    try:
        root.mainloop()
    finally:
        if skill_core is not None:
            try:
                skill_core.shutdown()
            except Exception:
                pass
        stop_event.set()


def main():
    target_shared = mp.Array("d", [0.0] * 7)
    actual_shared = mp.Array("d", [0.0] * 7)

    speed_shared = mp.Array(
        "d",
        DEFAULT_SPEED_PERCENT.tolist(),
    )
    wn_shared = mp.Array(
        "d",
        DEFAULT_WN.tolist(),
    )
    zeta_shared = mp.Array(
        "d",
        DEFAULT_ZETA.tolist(),
    )
    qacc_shared = mp.Array(
        "d",
        DEFAULT_MAX_QACC.tolist(),
    )

    gravity_enabled = mp.Value("b", True)
    gripper_target_mm = mp.Value("d", 0.0)
    gripper_actual_mm = mp.Value("d", 0.0)

    # V16 gripper force-control state.
    gripper_force_mode = mp.Value("b", False)
    gripper_force_N = mp.Value("d", 5.0)

    # V31 adjustable clamp approach speed.
    # 1x = original V30 speed; V42 default free-space speed = 15x.
    clamp_speed_mult_shared = mp.Value("d", 15.0)

    # V22 anti-slip + real contact-force telemetry.
    anti_slip_boost_shared = mp.Value("d", 0.0)
    anti_slip_slip_mm_s_shared = mp.Value("d", 0.0)
    anti_slip_mu_shared = mp.Value("d", 10.0)

    contact_left_n_shared = mp.Value("d", 0.0)
    contact_right_n_shared = mp.Value("d", 0.0)
    contact_tangent_n_shared = mp.Value("d", 0.0)
    contact_motor_cmd_shared = mp.Value("d", 0.0)

    # V30 auto-creep runtime equality HOLD.
    hold_enabled_shared = mp.Value("b", True)
    hold_active_shared = mp.Value("b", False)
    hold_name_shared = mp.Value("i", -1)
    hold_ready_shared = mp.Value("d", 0.0)  # retained for compatibility
    hold_now_request = mp.Value("b", False)
    hold_off_request = mp.Value("b", False)
    hold_status_code_shared = mp.Value("i", 0)

    # V14 safety control/status.
    reset_zero_request = mp.Value("b", False)
    recover_request = mp.Value("b", False)
    safety_trip_count = mp.Value("i", 0)
    safety_state = mp.Value("i", 0)

    sim_ready = mp.Value("b", False)
    sim_error_code = mp.Value("i", 0)

    # V43 ROS2 / MoveIt2 bridge telemetry.
    ros_bridge_ready = mp.Value("b", False)
    ros_cmd_count = mp.Value("i", 0)
    ros_feedback_count = mp.Value("i", 0)
    ros_scene_count = mp.Value("i", 0)
    ros_last_cmd_age = mp.Value("d", -1.0)
    ros_last_cmd_joint_count = mp.Value("i", 0)
    # V56: controller-state bridge telemetry must be shared with the Tk parent.
    ros_controller_state_rx_shared = mp.Value("i", 0)
    ros_fallback_joint_state_rx_shared = mp.Value("i", 0)
    # 0=none, 1=/arm_controller/controller_state, 2=/control/joint_states
    ros_last_command_source_shared = mp.Value("i", 0)

    # V45: OFF except during a skill trajectory execution.
    ros_arm_command_gate_shared = mp.Value("b", False)

    stop_event = mp.Event()

    sim_process = mp.Process(
        target=run_sim,
        args=(
            target_shared,
            actual_shared,
            speed_shared,
            wn_shared,
            zeta_shared,
            qacc_shared,
            gravity_enabled,
            gripper_target_mm,
            gripper_actual_mm,
            gripper_force_mode,
            gripper_force_N,
            clamp_speed_mult_shared,
            anti_slip_boost_shared,
            anti_slip_slip_mm_s_shared,
            anti_slip_mu_shared,
            contact_left_n_shared,
            contact_right_n_shared,
            contact_tangent_n_shared,
            contact_motor_cmd_shared,
            hold_enabled_shared,
            hold_active_shared,
            hold_name_shared,
            hold_ready_shared,
            hold_now_request,
            hold_off_request,
            hold_status_code_shared,
            reset_zero_request,
            recover_request,
            safety_trip_count,
            safety_state,
            sim_ready,
            sim_error_code,
            ros_bridge_ready,
            ros_cmd_count,
            ros_feedback_count,
            ros_scene_count,
            ros_last_cmd_age,
            ros_last_cmd_joint_count,
            ros_controller_state_rx_shared,
            ros_fallback_joint_state_rx_shared,
            ros_last_command_source_shared,
            ros_arm_command_gate_shared,
            stop_event,
        ),
        daemon=False,
        name="nero_mujoco_sim",
    )

    sim_process.start()

    try:
        run_gui(
            sim_process,
            target_shared,
            actual_shared,
            speed_shared,
            wn_shared,
            zeta_shared,
            qacc_shared,
            gravity_enabled,
            gripper_target_mm,
            gripper_actual_mm,
            gripper_force_mode,
            gripper_force_N,
            clamp_speed_mult_shared,
            anti_slip_boost_shared,
            anti_slip_slip_mm_s_shared,
            anti_slip_mu_shared,
            contact_left_n_shared,
            contact_right_n_shared,
            contact_tangent_n_shared,
            contact_motor_cmd_shared,
            hold_enabled_shared,
            hold_active_shared,
            hold_name_shared,
            hold_ready_shared,
            hold_now_request,
            hold_off_request,
            hold_status_code_shared,
            reset_zero_request,
            recover_request,
            safety_trip_count,
            safety_state,
            sim_ready,
            sim_error_code,
            ros_bridge_ready,
            ros_cmd_count,
            ros_feedback_count,
            ros_scene_count,
            ros_last_cmd_age,
            ros_last_cmd_joint_count,
            ros_controller_state_rx_shared,
            ros_fallback_joint_state_rx_shared,
            ros_last_command_source_shared,
            ros_arm_command_gate_shared,
            stop_event,
        )
    finally:
        stop_event.set()

        sim_process.join(timeout=2.0)

        if sim_process.is_alive():
            sim_process.terminate()
            sim_process.join(timeout=1.0)


if __name__ == "__main__":
    mp.freeze_support()

    # IMPORTANT:
    # spawn gives the MuJoCo viewer a fresh interpreter/process
    # and avoids inherited Qt/Tk/GLFW GUI state.
    mp.set_start_method("spawn", force=True)

    main()
