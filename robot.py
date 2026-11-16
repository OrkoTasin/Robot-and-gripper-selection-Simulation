"""Kinematics and overhead image processing. All world coordinates are in mm."""
from math import acos, atan2, cos, degrees, dist, hypot, isfinite, pi, sin

from selection import CONFIGURATIONS, TOOLS
from physics import DT, Block, Workcell, robot_colliders

HOME = [440, 0, 230]
TRAY = [650, 95]
JAW = 86
PARTS = [
    dict(name="Red block", x=440, y=-105, w=64, d=62, h=55, rgb=[239,105,100],mass=.5,surface="rigid"),
    dict(name="Blue block", x=490, y=45, w=54, d=66, h=45, rgb=[87,170,249],mass=.35,surface="smooth"),
    dict(name="Gold block", x=340, y=120, w=92, d=62, h=65, rgb=[241,185,83],mass=1.1,surface="steel"),
]

JOINT_LABELS = {
    "articulated": ["Base °","Shoulder °","Elbow °"],
    "cartesian": ["X mm","Y mm","Z mm"],
    "cylindrical": ["Base °","Radial mm","Z mm"],
    "polar": ["Azimuth °","Elevation °","Extension mm"],
}


def point(value):
    if (not isinstance(value, list) or len(value) != 3 or
            any(type(v) not in (int, float) or abs(v) > 10000 or not isfinite(v) for v in value)):
        raise ValueError("Enter three finite coordinates in millimetres (within ±10,000).")
    return value


def setup(data):
    model, tool = data.get("model","articulated"), data.get("tool_id","jaw")
    if model not in CONFIGURATIONS or not isinstance(tool,str) or tool not in TOOLS:
        raise ValueError("Choose a supported robot configuration and tool.")
    return model, tool


def pose(target, model="articulated"):
    x, y, z = point(target)
    radial, height = hypot(x, y), z + 65 - 235
    yaw = atan2(y, x)
    if model == "cartesian":
        if not (210 <= x <= 700 and abs(y) <= 220 and 3 <= z <= 365):
            raise ValueError("Target or path is outside the Cartesian demonstration workspace.")
        return dict(joints=[x,y,z],links=[[210,0,490],[x,0,490],[x,y,490],[x,y,z]],
                    tool=[x,y,z],error=0)
    if model == "cylindrical":
        if not (210 <= radial <= 705 and 3 <= z <= 420 and abs(yaw) <= pi/2):
            raise ValueError("Target or path is outside the cylindrical demonstration workspace.")
        return dict(joints=[degrees(yaw),radial,z],links=[[0,0,z+65],[x,y,z+65],[x,y,z]],
                    tool=[x,y,z],error=0)
    if model == "polar":
        extension, elevation = hypot(radial,height), atan2(height,radial)
        if not (210 <= extension <= 745 and 3 <= z <= 450 and
                abs(yaw) <= pi/2 and abs(elevation) <= pi/3):
            raise ValueError("Target or path is outside the polar demonstration workspace.")
        return dict(joints=[degrees(yaw),degrees(elevation),extension],
                    links=[[0,0,235],[x,y,z+65],[x,y,z]],tool=[x,y,z],error=0)
    l1, l2 = 350, 370
    elbow_cos = (radial**2 + height**2 - l1**2 - l2**2) / (2*l1*l2)
    if not (210 <= radial <= 705 and 3 <= z <= 470 and abs(yaw) <= pi/2 and abs(elbow_cos) <= 1):
        raise ValueError("Target or path is outside the robot workspace.")
    elbow = -acos(elbow_cos)
    shoulder = atan2(height, radial) - atan2(l2*sin(elbow), l1 + l2*cos(elbow))
    if not (-20 <= degrees(shoulder) <= 135 and -150 <= degrees(elbow) <= -5):
        raise ValueError("Target or path exceeds a joint limit.")
    elbow_r = l1*cos(shoulder)
    wrist_r = elbow_r + l2*cos(shoulder + elbow)
    elbow_z = 235 + l1*sin(shoulder)
    wrist_z = elbow_z + l2*sin(shoulder + elbow)
    tool = [wrist_r*cos(yaw), wrist_r*sin(yaw), wrist_z-65]
    return dict(joints=[degrees(a) for a in (yaw, shoulder, elbow)],
                links=[[0,0,235], [elbow_r*cos(yaw), elbow_r*sin(yaw), elbow_z],
                       [tool[0],tool[1],wrist_z], tool],
                tool=tool, error=dist(tool, target))


def positions(data):
    values = data["parts"]
    if not isinstance(values, list) or len(values) != len(PARTS):
        raise ValueError("Invalid part positions.")
    return [point(p) for p in values]


def held_part(data):
    held = data.get("held")
    if held is not None and (type(held) is not int or not 0 <= held < len(PARTS)):
        raise ValueError("Invalid held part.")
    return held


def orientations(data):
    values = data.get("orientations",[[0,1,0] for _ in PARTS])
    if not isinstance(values,list) or len(values)!=len(PARTS):
        raise ValueError("Invalid block orientations.")
    result = [point(item) for item in values]
    if any(abs(hypot(x,y)-1)>1e-6 for x,y,_ in result):
        raise ValueError("Block rotation needs a unit axis.")
    return result


def move(data):
    start, end = point(data["start"]), point(data["target"])
    parts, held = positions(data), held_part(data)
    model, tool_id = setup(data)
    steps = max(12,int(dist(start,end)/4)+1)
    frames = [pose([a+(b-a)*i/steps for a,b in zip(start,end)],model)
              for i in range(steps+1)]
    if held is not None and min(start[2],end[2]) < PARTS[held]["h"]-1:
        raise ValueError("The held part would hit the workbench.")
    physics = Workcell(PARTS,parts,orientations(data),held)
    opening = JAW if held is None else PARTS[held]["w"]
    previous = robot_colliders(frames[0],model,tool_id,opening)
    if held is not None:
        cargo = physics.blocks[held]
        cargo.position = [start[0],start[1],start[2]-cargo.half[2]]

    def simulate(frame, current, past):
        frame["hits"] = physics.step(current,past,frame["tool"])
        frame["parts"],frame["orientations"] = physics.snapshot()

    for frame in frames[1:]:
        current = robot_colliders(frame,model,tool_id,opening)
        simulate(frame,current,previous)
        previous = current
    if physics.contacts or any(abs(a[2])>.03 or p[2]>1
                               for a,p in zip(orientations(data),parts)):
        for _ in range(72):
            frame = dict(frames[-1])
            simulate(frame,previous,previous)
            frames.append(frame)
    return dict(frames=frames, contacts=sorted(physics.contacts),
                arm_contacts=sorted(physics.arm_contacts),
                link_contacts=sorted(physics.link_contacts),
                block_contacts=sorted(physics.block_contacts), frame_ms=1000*DT)


def grip(data):
    if held_part(data) is not None:
        raise ValueError("The gripper already holds a part.")
    x,y,z = point(data["tool"])
    _, tool_id = setup(data)
    tool = TOOLS[tool_id]
    angles = orientations(data)
    for index, (px,py,pz) in enumerate(positions(data)):
        part = PARTS[index]
        if hypot(x-px, y-py) <= 6 and abs(angles[index][2])>.05:
            raise ValueError(f"{part['name']} is tipped. Reset the cell for an upright pickup.")
        if hypot(x-px, y-py) <= 6 and abs(z-pz-part["h"]) <= 4:
            if not tool["min_width"] <= part["w"] <= tool["max_width"]:
                raise ValueError(f"{part['name']} is outside the {tool['min_width']}–{tool['max_width']} mm tool width range.")
            if part["mass"] > tool["max_part"] or part["surface"] not in tool["surfaces"]:
                raise ValueError(f"{tool['name']} is unsuitable for {part['name']}'s mass or surface.")
            return index
    raise ValueError("Align within 6 mm horizontally and 4 mm vertically of the part's top centre.")


def scene_blocks(data):
    return [Block(part,base,angle) for part,base,angle in
            zip(PARTS,positions(data),orientations(data))]


def overlaps_placement(x,y,index,scene):
    part = PARTS[index]
    for other, block in enumerate(scene):
        px,py,pz = block.position
        ex,ey,ez = block.extents()
        if (other != index and pz+ez > 0 and pz-ez < part["h"] and
                abs(x-px) < part["w"]/2+ex and abs(y-py) < part["d"]/2+ey):
            return True
    return False


def release(data):
    x,y,z = point(data["tool"])
    index = held_part(data)
    if index is None:
        raise ValueError("Grip a part before releasing.")
    part = PARTS[index]
    if abs(z-part["h"]) > 4:
        raise ValueError(f"Lower the part to Z = {part['h']} mm before releasing.")
    if not (180+part["w"]/2 <= x <= 750-part["w"]/2 and abs(y) <= 230-part["d"]/2):
        raise ValueError("Place the entire part on the workbench.")
    if overlaps_placement(x,y,index,scene_blocks(data)):
        raise ValueError("The part overlaps another object.")
    return [x,y,0]


def camera(data):
    """Rasterize the parts, segment the colour labels and back-project centroids."""
    width, height, scale, left, top = 246, 192, 2.5, 160, 240
    image = [[-1]*width for _ in range(height)]
    held = held_part(data)
    scene = scene_blocks(data)
    for index, block in enumerate(scene):
        x,y,z = block.position
        ex,ey,ez = block.extents()
        if index == held or z-ez < -1:
            continue
        for row in range(height):
            if abs(top-(row+0.5)*scale-y) <= ey:
                for col in range(width):
                    if abs(left+(col+0.5)*scale-x) <= ex:
                        image[row][col] = index
    counts = [[0,0,0] for _ in PARTS]
    runs = []
    for row, samples in enumerate(image):
        start = 0
        for col, index in enumerate(samples + [-1]):
            if index >= 0:
                counts[index][0] += 1
                counts[index][1] += col+0.5
                counts[index][2] += row+0.5
            if col and index != samples[col-1]:
                if samples[col-1] >= 0:
                    runs.append([start,row,col-start,samples[col-1]])
                start = col
    detections = []
    for i,(n,u,v) in enumerate(counts):
        if not n:
            continue
        block = scene[i]
        x,y,z = block.position
        ex,ey,ez = block.extents()
        clipped = x-ex < left or x+ex > left+scale*width or y-ey < top-scale*height or y+ey > top
        measured = [round(left+scale*u/n,2),round(top-scale*v/n,2),round(z+ez,2)]
        obscured = hypot(measured[0]-x,measured[1]-y)>5
        issue = ("Part is tipped." if abs(block.angle)>.05 else
                 "Part is outside the full camera view." if clipped else
                 "Part is partially obscured." if obscured else None)
        detections.append(dict(index=i,pixel=[u/n,v/n],target=measured,area=n,
            pickup_issue=issue,drop_occupied=overlaps_placement(*TRAY,i,scene),
            in_tray=(
                abs(x-TRAY[0])+ex<=60 and abs(y-TRAY[1])+ey<=52
                and abs(z-ez)<1
            )))
    return dict(width=width,height=height,runs=runs,detections=detections)
