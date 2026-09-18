"""Build Pinocchio models from an arm description.

All kinematics and rigid-body dynamics run in Pinocchio (C++); this module only
translates our URDF dialect into a model Pinocchio accepts:

* The URDF is normalized before urdfdom sees it: our ``<drive>``,
  ``<drive_coupling>`` and ``<tool_tip>`` extensions are removed, every
  ``<limit>`` gets the ``effort``/``velocity`` attributes urdfdom insists on
  (we allow effort to be derived from the drive), and ``continuous`` joints are
  declared ``revolute``. Pinocchio would otherwise model a continuous joint
  with a (cos, sin) pair, and every joint here is addressed by a single angle.
  Standard ``<mimic>`` elements are kept and passed to Pinocchio.
* Each motor and gearbox is attached to its host link's parent joint with
  ``Model.appendBodyToJoint`` (plus a body frame of the same name), which is
  exactly "a rigid lump bolted to that link".
* Reflected drive inertia goes into ``Model.armature``, which Pinocchio's
  RNEA and CRBA include.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Optional

import numpy as np
import pinocchio as pin

ACTUATED_TYPES = ("revolute", "prismatic")


def normalized_urdf(urdf_xml: str) -> str:
    """URDF text that urdfdom (and so Pinocchio) will accept."""
    robot = ET.fromstring(urdf_xml)
    for coupling in robot.findall("drive_coupling"):
        robot.remove(coupling)  # our extension; Pinocchio models the joints only
    for link in robot.findall("link"):
        for tip in link.findall("tool_tip"):
            link.remove(tip)
    for joint in robot.findall("joint"):
        for drive in joint.findall("drive"):
            joint.remove(drive)
        typ = (joint.get("type") or "").lower()
        if typ == "continuous":
            joint.set("type", "revolute")
            for lim in joint.findall("limit"):
                joint.remove(lim)
            ET.SubElement(
                joint, "limit", {"lower": str(-np.pi), "upper": str(np.pi), "effort": "0", "velocity": "0"}
            )
            continue
        if typ not in ACTUATED_TYPES:
            continue
        lim = joint.find("limit")
        if lim is None:
            lim = ET.SubElement(joint, "limit")
        for attr in ("effort", "velocity"):
            if not (lim.get(attr) or "").strip():
                lim.set(attr, "0")
    return ET.tostring(robot, encoding="unicode")


def body_frame(model: pin.Model, link: str) -> pin.Frame:
    fid = model.getFrameId(link, pin.FrameType.BODY)
    if fid >= len(model.frames):
        raise ValueError(f"link {link!r} is not in the Pinocchio model")
    return model.frames[fid]


def attach_body(
    model: pin.Model,
    link: str,
    name: str,
    mass: float,
    T_link_body: np.ndarray,
    inertia: Optional[np.ndarray] = None,
    com: Optional[np.ndarray] = None,
) -> None:
    """Rigidly attach a body (``T_link_body`` in ``link``'s frame) to ``link``.

    ``inertia`` is about the body's centre of mass, which sits at ``com`` in the
    body frame (default: its origin), both expressed in the body frame.
    """
    frame = body_frame(model, link)
    placement = frame.placement * pin.SE3(np.asarray(T_link_body, dtype=float))
    lever = np.zeros(3) if com is None else np.asarray(com, dtype=float)
    I = np.zeros((3, 3)) if inertia is None else np.asarray(inertia, dtype=float)
    model.appendBodyToJoint(frame.parentJoint, pin.Inertia(float(mass), lever, I), placement)
    model.addBodyFrame(name, frame.parentJoint, placement, model.getFrameId(link, pin.FrameType.BODY))


def build_model(arm, *, with_drives: bool = True) -> pin.Model:
    """Pinocchio model of ``arm``: structure, plus drive lumps and armature.

    ``mimic=True`` lets urdfdom's ``<mimic>`` through, so a passive linkage
    (the rod levelling a palletizer's tool plate) costs no degree of freedom.
    Coupled drives get no armature entry: their reflected inertia is a matrix,
    not a diagonal, and ``arm_analyzer.dynamics`` adds it after the RNEA.
    """
    model = pin.buildModelFromXML(normalized_urdf(arm.urdf), True)
    if not with_drives:
        return model
    for lump in arm.lumps():
        attach_body(model, lump.link, lump.name, lump.mass, lump.T, lump.inertia)
    coupled = {n for c in arm.couplings for n in c.joints}
    armature = np.array(model.armature, dtype=float)
    for name, drive in arm.drives.items():
        if name in coupled:
            continue
        armature[model.joints[model.getJointId(name)].idx_v] = drive.reflected_inertia
    model.armature = armature
    return model


def joint_indices(model: pin.Model, names) -> tuple[np.ndarray, np.ndarray]:
    """``(idx_q, idx_v)`` arrays for the named single-DOF joints."""
    idx_q, idx_v = [], []
    for n in names:
        jid = model.getJointId(n)
        if jid >= model.njoints:
            raise ValueError(f"joint {n!r} is not in the Pinocchio model")
        j = model.joints[jid]
        if j.nq != 1 or j.nv != 1:
            raise ValueError(f"joint {n!r} is not a single-DOF joint")
        idx_q.append(j.idx_q)
        idx_v.append(j.idx_v)
    return np.array(idx_q, dtype=int), np.array(idx_v, dtype=int)


def frame_placements(model: pin.Model, q: np.ndarray) -> tuple[pin.Data, dict[str, np.ndarray]]:
    """World pose of every body frame at configuration ``q``."""
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    out = {
        f.name: data.oMf[i].homogeneous
        for i, f in enumerate(model.frames)
        if f.type == pin.FrameType.BODY
    }
    return data, out
