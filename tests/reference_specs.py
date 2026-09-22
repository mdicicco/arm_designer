"""Published specifications of the real robots the examples are modelled on.

The example arms exist to be optimised against, so it is worth knowing how far
they start from the machines they imitate. Every number here is from a
manufacturer datasheet or manual, with the source recorded next to it; nothing
is inferred from the models being checked.

Retrieved 2026-09-20.

Reach is quoted differently by different manufacturers -- to the tool flange,
to the mounting plate, or as a rated working envelope -- so ``reach_to`` says
which frame the published figure refers to and the check uses that frame.
"""

from __future__ import annotations

# Sources
UR5E_TECHSHEET = (
    "https://www.universal-robots.com/manuals/EN/TechSheets/"
    "UR5e_techsheet_pdf_online/UR5e_techsheet_en.pdf"
)
WAM_MANUAL = "https://web.barrett.com/support/WAM_Documentation/WAM_UserManual_AH-00.pdf"
KR6_DATASHEET = (
    "https://assets.robots.com/robots/KUKA/Small-Robots/KUKA_KR_6_R900_Sixx_Datasheet.pdf"
)
KR6_R900_2 = "https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/697/K_6_R900-2.pdf"


REFERENCE = {
    "ur_style_6dof.urdf": {
        "real": "Universal Robots UR5e",
        "source": UR5E_TECHSHEET,
        "mass_kg": 20.6,          # "Weight including cable"
        "payload_kg": 5.0,
        "reach_mm": 850.0,
        "reach_to": "flange",
        "dof": 6,
        # Working range and maximum speed, all six axes alike.
        "joint_range_deg": [(-360, 360)] * 6,
        "joint_speed_deg_s": [180.0] * 6,
        # Link lengths from the dimension drawing; these are the standard UR5e
        # DH parameters and the generator reproduces them exactly.
        "link_lengths_mm": {
            "d1": 162.5, "a2": 425.0, "a3": 392.2,
            "d4": 133.3, "d5": 99.7, "d6": 99.6,
        },
    },
    "wam_style_7dof.urdf": {
        "real": "Barrett WAM, 7-DOF",
        "source": WAM_MANUAL,
        "mass_kg": 27.4,
        "payload_kg": 3.0,
        "reach_mm": 910.0,        # "0.91 m to mounting plate"
        "reach_to": "tool_tip",
        "dof": 7,
        "joint_range_deg": [
            (-150, 150), (-113, 113), (-157, 157),
            (-50, 180), (-273, 71), (-90, 90), (-172, 172),
        ],
        "joint_speed_deg_s": None,   # not published per axis
        # Barrett's published DH lengths.
        "link_lengths_mm": {
            "base_to_shoulder": 346.0, "upper_arm": 550.0,
            "elbow_offset": 45.0, "forearm": 300.0, "wrist_to_plate": 60.9,
        },
        # Joint torque limits, 7-DOF (N*m).
        "joint_torque_nm": [77.3, 160.6, 95.6, 29.4, 11.6, 11.6, 2.7],
    },
    "kr_style_6dof.urdf": {
        "real": "KUKA KR 6 R900 sixx (KR AGILUS)",
        "source": KR6_DATASHEET,
        "mass_kg": 52.0,
        "payload_kg": 6.0,
        "reach_mm": 901.0,
        "reach_to": "flange",
        "dof": 6,
        # KUKA's zero pose is not the one this example uses, so only the width
        # of each range is comparable -- see the test.
        "joint_range_deg": [
            (-170, 170), (-190, 45), (-120, 156), (-185, 185), (-120, 120), (-350, 350),
        ],
        "joint_speed_deg_s": None,  # KR 6 R900-2: up to 360 deg/s on A1-A3
        "link_lengths_mm": None,    # not published; the example approximates
    },
}
