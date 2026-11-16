"""Preliminary catalogue screening and explainable robot/tool ranking."""
import csv
from math import isfinite
from pathlib import Path

CONFIGURATIONS = ("cartesian", "cylindrical", "polar", "articulated")
TOOLS = {
    "jaw": dict(name="Parallel-jaw gripper", mass=0.6, max_part=3, min_width=15, max_width=82,
                surfaces=["rigid","smooth","steel"], reason="Opposing jaws grip accessible, rigid sides."),
    "vacuum": dict(name="Vacuum cup", mass=0.35, max_part=1.2, min_width=32, max_width=250,
                   surfaces=["smooth","steel"], reason="A flat, sealed face allows suction."),
    "magnet": dict(name="Magnetic gripper", mass=0.8, max_part=4, min_width=35, max_width=250,
                   surfaces=["steel"], reason="A flat ferromagnetic face allows magnetic pickup."),
}
DEFAULT = dict(mass=0.5, width=64, surface="rigid", drive="any",
               reach=660, x=210, y=200, z=175, radial=210, repeatability=0.05, accuracy=None)
NUMERIC = ("payload","repeatability","reach","x","y","z","radial")


def catalogue():
    path = Path(__file__).resolve().parent / "public" / "robots.csv"
    with path.open(encoding="utf-8", newline="") as stream:
        for robot in csv.DictReader(stream):
            for key in NUMERIC:
                robot[key] = float(robot[key]) if robot[key] else None
            yield robot


def recommend(task):
    values = {}
    for key in ("mass","width","reach","x","y","z","radial","repeatability"):
        value = task.get(key)
        allow_zero = key in ("x","y","z","radial")
        if (type(value) not in (float,int) or abs(value) > 100000 or not isfinite(value) or
                value < 0 or (not allow_zero and value == 0)):
            raise ValueError(f"Enter a valid number for {key}, up to 100,000.")
        values[key] = value
    accuracy = task.get("accuracy")
    if accuracy is not None and (
            type(accuracy) not in (float,int) or not 0 < accuracy <= 100000):
        raise ValueError("Enter a positive absolute accuracy limit, or leave it blank.")
    values["accuracy"] = accuracy
    surface, drive = task.get("surface"), task.get("drive")
    if surface not in ("rigid","smooth","steel") or drive not in ("any","Electric","Hydraulic"):
        raise ValueError("Choose a supported surface and drive option.")
    values.update(surface=surface, drive=drive)
    tools = []
    for key, tool in TOOLS.items():
        reasons = []
        if surface not in tool["surfaces"]:
            reasons.append("Surface or material incompatible.")
        if not tool["min_width"] <= values["width"] <= tool["max_width"]:
            reasons.append(f"Requires {tool['min_width']}–{tool['max_width']} mm part width.")
        if values["mass"] > tool["max_part"]:
            reasons.append(f"Part exceeds {tool['max_part']} kg tool limit.")
        tools.append(dict(tool, id=key, reasons=reasons))
    preference = {
        "rigid": ["jaw","vacuum","magnet"], "smooth": ["vacuum","jaw","magnet"],
        "steel": ["magnet","vacuum","jaw"],
    }[surface]
    criteria = {"cartesian": ("x","y","z"), "cylindrical": ("radial","z"),
                "polar": ("reach",), "articulated": ("reach",)}
    descriptions = dict(x="X stroke",y="Y stroke",z="Z stroke",radial="Radial stroke",reach="Reach")
    rows = []
    for robot in catalogue():
        failed, unknown = [], []
        if drive != "any" and drive != robot["drive"]:
            failed.append(f"Requires {drive.lower()} drive.")
        for key in criteria[robot["id"]]:
            capacity = robot[key]
            if capacity is None:
                unknown.append(f"{descriptions[key]} unavailable.")
            elif capacity < values[key]:
                failed.append(f"{descriptions[key]} {capacity:g} mm is below {values[key]:g} mm.")
        precision = robot["repeatability"]
        if precision is None:
            unknown.append("Repeatability unavailable.")
        elif precision > values["repeatability"]:
            failed.append(f"Repeatability ±{precision:g} mm exceeds the limit.")
        if accuracy is not None:
            unknown.append("Absolute positioning accuracy unavailable.")
        viable_tools = [tool for tool in tools if not tool["reasons"] and
                        values["mass"]+tool["mass"] <= robot["payload"]]
        chosen = min(viable_tools, key=lambda tool: preference.index(tool["id"]), default=None)
        if not chosen:
            failed.append("No compatible tool fits the combined robot payload limit.")
        score = None
        if not failed and not unknown:
            payload_margin = 1-(values["mass"]+chosen["mass"])/robot["payload"]
            workspace_margin = min(1-values[k]/robot[k] for k in criteria[robot["id"]])
            precision_margin = 1-precision/values["repeatability"]
            score = round(100*(.35*payload_margin+.35*workspace_margin+.30*precision_margin),1)
        rows.append(dict(robot, status="Rejected" if failed else "Needs verification" if unknown else "Eligible",
                         reasons=failed+unknown, tool=chosen, score=score))
    rows.sort(key=lambda row: (row["score"] is None, -(row["score"] or 0)))
    winner = next((row for row in rows if row["score"] is not None), None)
    return dict(task=values, rows=rows, tools=tools, recommendation=winner)
