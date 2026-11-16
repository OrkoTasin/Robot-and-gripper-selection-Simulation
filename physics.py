"""Small deterministic rigid-body approximation for the educational workcell.

Distances use millimetres and time uses seconds. Arm bodies are kinematic.
Loose blocks use gravity, damped rotation, impulses and bench/floor support.
"""
from math import cos, hypot, pi, sin, sqrt

DT = 1 / 60
FLOOR = -104
BENCH = (180, 750, -230, 230)


def add(a,b):
    return [x+y for x,y in zip(a,b)]


def sub(a,b):
    return [x-y for x,y in zip(a,b)]


def scale(a,k):
    return [x*k for x in a]


def dot(a,b):
    return sum(x*y for x,y in zip(a,b))


def length(v):
    return sqrt(dot(v,v))


def rotate(v, axis, angle):
    x,y,z = v
    ax,ay = axis
    c,s = cos(angle),sin(angle)
    parallel = (ax*x+ay*y)*(1-c)
    return [c*x+s*ay*z+parallel*ax, c*y-s*ax*z+parallel*ay,
            c*z+s*(ax*y-ay*x)]


class Block:
    def __init__(self, part, base, orientation):
        self.half = [part[key]/2 for key in ("w","d","h")]
        self.mass = part["mass"]
        self.axis = orientation[:2]
        self.angle = orientation[2]
        self.position = [base[0],base[1],base[2]+self.extents()[2]]
        self.velocity = [0,0,0]
        self.spin = 0

    def extents(self):
        axes = [rotate(v,self.axis,self.angle) for v in ([1,0,0],[0,1,0],[0,0,1])]
        return [sum(abs(axis[k])*h for axis,h in zip(axes,self.half)) for k in range(3)]

    def sphere(self, position, radius, motion):
        local = rotate(sub(position,self.position), self.axis, -self.angle)
        nearest = [max(-h,min(h,b)) for b,h in zip(local,self.half)]
        difference = sub(nearest,local)
        distance = length(difference)
        if distance >= radius:
            return None
        if distance > 1e-6:
            normal = rotate(scale(difference,1/distance),self.axis,self.angle)
            return radius-distance, normal
        # A sample inside the box pushes in its direction of travel.
        speed = length(motion)
        if speed < .01:
            return None
        normal = scale(motion,1/speed)
        return radius, normal

    def tumble(self, direction, speed, height):
        horizontal = hypot(*direction[:2])
        if horizontal < .15 or speed < 45:
            return
        dx,dy = direction[:2]
        axis = [0,1 if dx>=0 else -1] if abs(dx)>=abs(dy) else [-1 if dy>=0 else 1,0]
        if self.angle == 0 and abs(self.spin) < .05:
            self.axis = axis
        alignment = dot(self.axis,axis)
        self.spin += alignment * min(9, speed*.024) * max(.12,min(1.3,height/self.half[2]))
        self.spin = max(-10,min(10,self.spin))

    def integrate(self):
        self.velocity[2] -= 9810*DT
        self.position = add(self.position, scale(self.velocity,DT))
        self.angle += self.spin*DT
        self.spin *= .975
        self.support()

    def support(self):
        ex,ey,ez = self.extents()
        x,y,z = self.position
        bench = BENCH[0] <= x <= BENCH[1] and BENCH[2] <= y <= BENCH[3]
        surface = 0 if bench and z >= 0 else FLOOR
        if z-ez > surface:
            return
        self.position[2] = surface+ez
        self.velocity[2] = -self.velocity[2]*.20 if self.velocity[2] < -125 else max(0,self.velocity[2])
        self.velocity[:2] = [v*.87 for v in self.velocity[:2]]
        self.spin *= .88
        # Gravity brings the block down onto the nearest stable face.
        rest = round(self.angle/(pi/2))*(pi/2)
        self.spin += (rest-self.angle)*1.6
        if abs(rest-self.angle)<.018 and abs(self.spin)<.36:
            self.angle,self.spin = rest,0

    def snapshot(self):
        angle = (self.angle+pi)%(2*pi)-pi
        return [*self.position[:2],self.position[2]-self.extents()[2]], [*self.axis,angle]


def robot_colliders(frame, model, tool_id, opening):
    """Sample capsule chains at stable fractions across consecutive poses."""
    links = frame["links"]
    x,y,z = frame["tool"]
    if model == "cartesian":
        geometry = [([x,-250,490],[x,250,490],23,24),
                    (links[-2],[x,y,z+38],15,30)]
    elif model == "cylindrical":
        geometry = [(links[0],links[1],31,40),
                    (links[1],[x,y,z+38],15,5)]
    else:
        geometry = [(a,b,34 if i==0 else 26,40) for i,(a,b) in
                    enumerate(zip(links[:-2],links[1:-1]))]
        geometry.append((links[-2],[x,y,z+38],15,5))
    if model != "cartesian":
        for joint,radius in zip(links[:-1],(40,30,25)):
            geometry.append((joint,joint,radius,1))
    link_count = len(geometry)
    geometry.append(([x,y,z+31],[x,y,z+38],21,2))
    if tool_id == "jaw":
        geometry.append(([x-opening/2,y,z+18],[x+opening/2,y,z+18],9,12))
        for sign in (-1,1):
            px = x+sign*(opening/2+7)
            geometry.append(([px,y,z-12],[px,y,z+9],6,4))
    else:
        radius = 15 if tool_id == "vacuum" else 18
        geometry.append(([x,y,z+radius],[x,y,z+radius+3],radius,2))
    # A fixed sample count keeps point correspondence when a slide extends.
    return [(add(a,scale(sub(b,a),i/count)),r,"link" if index<link_count else "gripper")
            for index,(a,b,r,count) in enumerate(geometry)
            for i in range(count+1)]


def interact_robot(block, current, past):
    hits = []
    for (centre,radius,group),(previous,_,_) in zip(current,past):
        relative = sub(scale(sub(centre,previous),1/DT),block.velocity)
        if length(relative) < 1:
            continue
        hit = block.sphere(centre,radius,relative)
        if hit:
            depth,normal = hit
            speed = dot(relative,normal)
            if speed > 1:
                hits.append((speed+depth*10,depth,normal,speed,centre,group))
    if not hits:
        return None
    _,depth,normal,speed,centre,group = max(hits,key=lambda item:item[0])
    block.position = add(block.position,scale(normal,min(depth,10)*.72))
    block.velocity = add(block.velocity,scale(normal,min(speed,1200)*.93))
    block.tumble(normal,speed,centre[2]-block.position[2]+block.half[2]*.65)
    return group


def interact_blocks(a,b, anchored=False):
    delta = sub(b.position,a.position)
    overlap = [x+y-abs(d) for x,y,d in zip(a.extents(),b.extents(),delta)]
    if min(overlap) <= 0:
        return False
    axis = min(range(3),key=overlap.__getitem__)
    normal = [0,0,0]
    normal[axis] = 1 if delta[axis]>=0 else -1
    inv_a,inv_b = (0 if anchored else 1/a.mass),1/b.mass
    total = inv_a+inv_b
    correction = scale(normal,overlap[axis]*.76/total)
    a.position = sub(a.position,scale(correction,inv_a))
    b.position = add(b.position,scale(correction,inv_b))
    speed = dot(sub(a.velocity,b.velocity),normal)
    if speed > 6:
        impulse = scale(normal,1.23*speed/total)
        a.velocity = sub(a.velocity,scale(impulse,inv_a))
        b.velocity = add(b.velocity,scale(impulse,inv_b))
        if not anchored:
            a.tumble(scale(normal,-1),speed*.45,a.half[2]*.6)
        b.tumble(normal,speed,b.half[2]*.8)
    return speed > 6


class Workcell:
    def __init__(self, parts, positions, orientations, held):
        self.blocks = [Block(part,base,angle) for part,base,angle in zip(parts,positions,orientations)]
        self.held = held
        self.contacts = set()
        self.arm_contacts = set()
        self.link_contacts = set()
        self.block_contacts = set()

    def step(self,current,past,tool):
        hits = set()
        loose = [i for i in range(len(self.blocks)) if i!=self.held]
        for i in loose:
            block = self.blocks[i]
            block.integrate()
            group = interact_robot(block,current,past)
            if group:
                hits.add(i)
                self.arm_contacts.add(i)
                if group=="link":
                    self.link_contacts.add(i)
        if self.held is not None:
            cargo = self.blocks[self.held]
            previous = cargo.position
            cargo.position = [tool[0],tool[1],tool[2]-cargo.half[2]]
            cargo.velocity = scale(sub(cargo.position,previous),1/DT)
        for offset,i in enumerate(loose):
            a = self.blocks[i]
            for j in loose[offset+1:]:
                if interact_blocks(a,self.blocks[j]):
                    hits.update((i,j))
                    self.block_contacts.add((i,j))
            if self.held is not None and interact_blocks(self.blocks[self.held],a,anchored=True):
                hits.add(i)
                self.block_contacts.add(tuple(sorted((self.held,i))))
            a.support()
        self.contacts.update(hits)
        return sorted(hits)

    def snapshot(self):
        positions,orientations = zip(*(block.snapshot() for block in self.blocks))
        return list(positions),list(orientations)
