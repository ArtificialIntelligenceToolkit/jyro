from jyro.simulator.simulator import Segment, Line
from jyro.simulator.device import Speech

import math
from collections import defaultdict

PIOVER180 = math.pi / 180.0
PIOVER2   = math.pi / 2.0

class Robot():
    def __init__(self, name, x, y, a, boundingBox=None, color="red"):
        """
        a - angle in radians
        """
        if boundingBox is None:
            boundingBox = []
        self.name = name.replace(" ", "_")
        self.type = "robot"
        self.stepScalar = 1.0 # normally = 1.0
        self._xya = (x, y, a) # original, for reset
        self.reset()
        self.boundingBox = boundingBox # ((x1, x2), (y1, y2)) NOTE: Xs then Ys of bounding box
        self.boundingSeg = []
        if boundingBox != []:
            self.radius = max(max(map(abs, boundingBox[0])), max(map(abs, boundingBox[1]))) # meters
        else:
            self.radius = 0.0
        self.color = color
        self.colorParts = {"ir": "pink", "sonar": "lightgray", "bumper": "black", "trail": color}
        self.devices = []
        self.device = defaultdict(lambda: None)
        self.physics = None # will be set when added to simulator
        # -1: don't automatically turn display on when subscribing:
        self.display = {"body": 1, "boundingBox": 0, "gripper": -1, "camera": 0, "sonar": 0,
                        "light": -1, "lightBlocked": 0, "trail": -1, "ir": -1, "bumper": 1,
                        "speech": 1, "robots": 1, "devices": 1}
        self.shapes = []
        self.trail = []
        self.useTrail = False
        self.addDevice(Speech())

    def __getitem__(self, item):
        return self.device[item]

    def brain(self, robot):
        pass

    def reset(self):
        self._gx, self._gy, self._ga = self._xya
        self._px, self._py, self._pa = self._xya
        self.stall = 0
        self.energy = 10000.0
        self.x, self.y, self.a = (0.0, 0.0, 0.0) # localize
        self.maxEnergyCostPerStep = 1.0
        self.sayText = ""
        self.friction = 1.0
        self.vx, self.vy, self.va = (0.0, 0.0, 0.0) # meters / second, rads / second
        self.trail = []

    def _repr_svg_(self):
        from jyro.simulator import Physics, Canvas
        canvas = Canvas((240, 240))
        canvas.max_x = 1.0 # meters
        canvas.max_y = 1.0 # meters
        canvas.scale = min(canvas.width/canvas.max_x, canvas.height/canvas.max_y)
        xya = self._gx, self._gy, self._ga
        self._gx, self._gy, self._ga = (0.5, 0.5, 0)
        remove_sim = False
        if self.physics is None:
            self.physics = Physics() # for drawings
            self.updateDevices() # ASSUME: if old sim, it is up to date (self.shapes are there)
            remove_sim = True
        self.draw(canvas)
        if remove_sim:
            self.physics = None
        svg = canvas._repr_svg_()
        self._gx, self._gy, self._ga = xya
        return svg

    def drawRay(self, dtype, x1, y1, x2, y2, color):
        self.shapes.append(Line((x1, y1), (x2, y2), outline=color))

    def additionalSegments(self, propose, x, y, cos_a90, sin_a90, **dict):
        """
        propose is where it would go, if moved with device velocity, etc.
        """
        # dynamic segments
        retval = []
        for device in self.devices:
            segs = device.additionalSegments(propose, x, y, cos_a90, sin_a90, **dict)
            if segs:
                retval.extend(segs)
        return retval

    def addBoundingSeg(self, boundingSeg):
        if self.boundingSeg == []:
            self.boundingSeg = boundingSeg
        else:
            self.boundingSeg[0].extend(boundingSeg[0])
            self.boundingSeg[1].extend(boundingSeg[1])
        segradius = max(max(map(abs, boundingSeg[0])), max(map(abs, boundingSeg[1]))) # meters
        self.radius = max(self.radius, segradius)

    def localize(self, x = 0, y = 0, th = 0):
        self.x, self.y, self.a = (x, y, th)

    def say(self, text):
        self.sayText = text

    def updatePosition(self, x, y, a=None):
        # first, figure out how much we moved in the global coords:
        a90 = -self._ga
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        dx =  (x - self._gx) * cos_a90 - (y - self._gy) * sin_a90
        dy =  (x - self._gx) * sin_a90 + (y - self._gy) * cos_a90
        # then, move that much in the local coords:
        local90 = -self.a
        cos_local90 = math.cos(local90)
        sin_local90 = math.sin(local90)
        a90 = -self.a
        self.y += dx * cos_local90 - dy * sin_local90
        self.x += dx * sin_local90 + dy * cos_local90
        self._px, self._py = self._gx, self._gy
        self._gx, self._gy = x, y
        if a is not None:
            self.setAngle(a)
        self.updateTrail()

    def updateTrail(self):
        if self.useTrail:
            if len(self.trail) == 0 or self.trail[-1] != (self._gx, self._gy, self._ga):
                self.trail.append((self._gx, self._gy, self._ga))

    def setAngle(self, a):
        # if our angle changes, update localized position:
        diff = a - self._ga
        self.a += diff
        self.a = self.a % (2 * math.pi) # keep in the positive range
        self._ga = a % (2 * math.pi) # keep in the positive range
        self.updateTrail()

    def setPose(self, x, y, a=None):
        self._px, self._py = self._gx, self._gy
        self._gx = x
        self._gy = y
        if a is not None:
            self._ga = a % (2 * math.pi) # keep in the positive range
        self.updateDevices()
        self.updateTrail()

    def move(self, vx, va):
        self.vx = vx
        self.va = va

    def rotate(self, va):
        self.va = va

    def translate(self, vx):
        self.vx = vx

    def getPose(self):
        """ Returns global coordinates. """
        return (self._gx, self._gy, self._ga)

    def getIndex(self, dtype, i):
        index = 0
        for d in self.devices:
            if d.type == dtype:
                if i == index:
                    return d
                index += 1
        return None

    def updateDevices(self):
        # FIXME: updateDevices() has side effect:
        # puts robot.shapes in sim.canvas coords
        self.shapes[:] = []
        for device in self.devices:
            if device.active:
                device.update(self)

    def step(self, timeslice=100):
        """
        Move the robot self.velocity amount, if not blocked.
        """
        self.ovx, self.ovy, self.ova = self.vx, self.vy, self.va
        gvx = self.ovx * self.stepScalar
        gvy = self.ovy * self.stepScalar
        vx = gvx * math.sin(-self._ga) + gvy * math.cos(-self._ga)
        vy = gvx * math.cos(-self._ga) - gvy * math.sin(-self._ga)
        va = self.ova
        # proposed positions:
        p_x = self._gx + vx * (timeslice / 1000.0) # miliseconds
        p_y = self._gy + vy * (timeslice / 1000.0) # miliseconds
        p_a = self._ga + va * (timeslice / 1000.0) # miliseconds
        # for each of the robot's bounding box segments:
        a90 = p_a + PIOVER2
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        if vx != 0 or vy != 0 or va != 0:
            self.energy -= self.maxEnergyCostPerStep
        # let's check if that movement would be ok:
        segments = []
        if self.boundingBox != []:
            xys = map(lambda x, y: (p_x + x * cos_a90 - y * sin_a90,
                                    p_y + x * sin_a90 + y * cos_a90),
                      self.boundingBox[0], self.boundingBox[1])
            xys = list(xys)
            for i in range(len(xys)):
                bb = Segment( xys[i], xys[i - 1])
                segments.append(bb)
        if self.boundingSeg != []:
            xys = map(lambda x, y: (p_x + x * cos_a90 - y * sin_a90,
                                    p_y + x * sin_a90 + y * cos_a90),
                      self.boundingSeg[0], self.boundingSeg[1])
            xys = list(xys)
            for i in range(0, len(xys), 2):
                bb = Segment( xys[i], xys[i + 1])
                segments.append(bb)
        for s in self.additionalSegments(True, p_x, p_y, cos_a90, sin_a90):
            segments.append(s)
        for bb in segments:
            # check each segment of the robot's bounding segs for wall obstacles:
            for w in self.physics.world:
                if bb.intersects(w):
                    self.stall = 1
                    return False # collision
            # check each segment of the robot's bounding box for other robots:
            for r in self.physics.robots:
                if r.name == self.name:
                    continue # don't compare with your own!
                r_a90 = r._ga + PIOVER2
                cos_r_a90 = math.cos(r_a90)
                sin_r_a90 = math.sin(r_a90)
                r_segments = []
                if r.boundingBox != []:
                    r_xys = map(lambda x, y: (r._gx + x * cos_r_a90 - y * sin_r_a90,
                                              r._gy + x * sin_r_a90 + y * cos_r_a90),
                                r.boundingBox[0], r.boundingBox[1])
                    r_xys = list(r_xys)
                    for j in range(len(r_xys)):
                        r_seg = Segment(r_xys[j], r_xys[j - 1])
                        r_segments.append(r_seg)
                if r.boundingSeg != []:
                    r_xys = map(lambda x, y: (r._gx + x * cos_r_a90 - y * sin_r_a90,
                                              r._gy + x * sin_r_a90 + y * cos_r_a90),
                                r.boundingSeg[0], r.boundingSeg[1])
                    r_xys = list(r_xys)
                    for j in range(0, len(r_xys), 2):
                        r_seg = Segment(r_xys[j], r_xys[j + 1])
                        r_segments.append(r_seg)
                for s in r.additionalSegments(False, r._gx, r._gy, cos_r_a90, sin_r_a90):
                    r_segments.append(s)
                for r_seg in r_segments:
                    bbintersect = bb.intersects(r_seg)
                    if bbintersect:
                        self.stall = 1
                        return False # collision
        # ok! move the robot, if it wanted to move
        for device in self.devices:
            device.step()
        if self.friction != 1.0:
            self.ovx *= self.friction
            self.ovy *= self.friction
            if 0.0 < self.ovx < 0.1: self.ovx = 0.0
            if 0.0 < self.ovy < 0.1: self.ovy = 0.0
            if 0.0 > self.ovx > -0.1: self.ovx = 0.0
            if 0.0 > self.ovy > -0.1: self.ovy = 0.0
        self.stall = 0
        self.updatePosition(p_x, p_y, p_a)
        return True # able to move

    def draw(self, canvas):
        for shape in self.shapes:
            shape.draw(canvas)
        if self.display["robots"] == 1:
            self.drawRobot(canvas)
        if self.display["devices"] == 1:
            self.drawDevices(canvas)

    def drawDevices(self, canvas):
        for device in self.devices:
            device.draw(self, canvas)

    def addDevice(self, dev):
        dev.robot = self
        self.devices.append(dev)
        self.device[dev.type] = dev
        dev.update(self)
        return self # so can give nice display with new device visual

    def serialize(self, item='all'):
        """
        """
        d = {}
        d["pose"] = self.getPose()
        for dname in self.device.keys():
            if self.device[dname]:
                d[dname] = self.device[dname].serialize(item)
        return d

class Blimp(Robot):
    def __init__(self, *args, **kwargs):
        Robot.__init__(self, *args, **kwargs)
        self.radius = 0.44 # meters
        self.color = "purple"

    def drawRobot(self, canvas):
        a90 = self._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        radius = self.radius
        canvas.drawOval(self._gx - radius,
                        self._gy - radius,
                        self._gx + radius,
                        self._gy + radius,
                        fill=self.color, outline="blue")
        x = (self._gx + radius * cos_a90 - 0 * sin_a90)
        y = (self._gy + radius * sin_a90 + 0 * cos_a90)
        canvas.drawLine(self._gx, self._gy, x, y, outline="blue", width=3)

class Puck(Robot):
    def __init__(self, *args, **kwargs):
        Robot.__init__(self, *args, **kwargs)
        self.radius = 0.05
        self.friction = 0.90
        self.type = "puck"

    def drawRobot(self, canvas):
        """
        Draws the body of the robot. Not very efficient.
        """
        if self.display["body"] == 1:
            radius = self.radius
            x1, y1, x2, y2 = ((self._gx - radius),
                              (self._gy - radius),
                              (self._gx + radius),
                              (self._gy + radius))
            canvas.drawOval(x1, y1, x2, y2, fill=self.color, outline="black")
        if self.display["boundingBox"] == 1 and self.boundingBox != []:
            # Body Polygon, by x and y lists:
            a90 = self._ga + PIOVER2 # angle is 90 degrees off for graphics
            cos_a90 = math.cos(a90)
            sin_a90 = math.sin(a90)
            xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                   self._gy + x * sin_a90 + y * cos_a90),
                     self.boundingBox[0], self.boundingBox[1])
            xy = list(xy)
            canvas.drawPolygon(xy, fill="", outline="purple")

class Pioneer(Robot):
    def __init__(self, name, x, y, a, color="red"):
        Robot.__init__(self, name, x, y, a,
                       boundingBox=((.225, .225, -.225, -.225),
                                    (.175, -.175, -.175, .175)), color=color)
        self.radius = 0.4

    def drawRobot(self, canvas):
        """
        Draws the body of the robot. Not very efficient.
        """
        a90 = self._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        # Draw trail:
        if self.display["trail"] == 1:
            cx, cy = -1, -1
            for (x, y, a) in self.trail: # poses
                if (cx, cy) != (-1, -1):
                    canvas.drawLine(cx, cy, x, y, width=1, outline="purple")
                cx, cy = x, y
        # Body Polygon, by x and y lists:
        sx = [.225, .15, -.15, -.225, -.225, -.15, .15, .225]
        sy = [.08, .175, .175, .08, -.08, -.175, -.175, -.08]
        if self.display["body"] == 1:
            xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                   self._gy + x * sin_a90 + y * cos_a90),
                     sx, sy)
            xy = list(xy)
            if self.stall:
                canvas.drawPolygon(xy, fill="white", outline="black")
            else:
                canvas.drawPolygon(xy, fill=self.color, outline=self.color)
        if self.display["boundingBox"] == 1:
            if self.boundingBox != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingBox[0], self.boundingBox[1])
                xy = list(xy)
                canvas.drawPolygon(xy, fill="", outline="purple")
            if self.boundingSeg != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingSeg[0], self.boundingSeg[1])
                xy = list(xy)
                for i in range(0, len(xy), 2):
                    canvas.drawLine(xy[i][0], xy[i][1],
                                            xy[i + 1][0], xy[i + 1][1],
                                            outline="purple")
            additionalSegments = self.additionalSegments(False, self._gx, self._gy, cos_a90, sin_a90)
            if additionalSegments != []:
                for s in additionalSegments:
                    canvas.drawLine((s.start[0]),
                                    (s.start[1]),
                                    (s.end[0]),
                                    (s.end[1]),
                                    outline="purple")
        # Draw an arrowhead at (x, y), pointing in heading direction
        canvas.drawArrow(self._gx, self._gy, self._ga, 0.05)

class Myro(Robot):
    def __init__(self, *args, **kwargs):
        Robot.__init__(self, *args, **kwargs)
        self.radius = 0.25

    def drawRobot(self, canvas):
        """
        Draws the body of the robot. Not very efficient.
        """
        # Body Polygon, by x and y lists:
        sx = [ .20, .20,-.10,-.10]
        sy = [ .15,-.15,-.15, .15]
        a90 = self._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        if self.display["body"] == 1:
            xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                   self._gy + x * sin_a90 + y * cos_a90),
                     sx, sy)
            xy = list(xy)
            canvas.drawPolygon(xy, fill=self.color, outline="black")
            # --------------------------------------------------------------------------
            # Parts: wheel, wheel, battery
            bx = [[ .10, .10, -.10, -.10],
                  [ .10, .10, -.10, -.10],
                  [.05, .05, -.10, -.10],
                  [.16, .17, .18, .17],
                  [.16, .17, .18, .17]]
            by = [[ .18, .16, .16, .18],
                  [ -.18, -.16, -.16, -.18],
                  [.14, -.14, -.14, .14],
                  [.13, .135, .115, .11],
                  [-.13, -.135, -.115, -.11]]
            colors = ["black", "black", "gray", "yellow", "yellow"]
            for i in range(len(bx)):
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         bx[i], by[i])
                xy = list(xy)
                canvas.drawPolygon(xy, fill=colors[i])
            # --------------------------------------------------------------------------
        if self.display["boundingBox"] == 1:
            if self.boundingBox != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingBox[0], self.boundingBox[1])
                xy = list(xy)
                canvas.drawPolygon(xy, fill="", outline="purple")
            if self.boundingSeg != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingSeg[0], self.boundingSeg[1])
                xy = list(xy)
                for i in range(0, len(xy), 2):
                    canvas.drawLine(xy[i][0], xy[i][1],
                                            xy[i + 1][0], xy[i + 1][1],
                                            outline="purple")
            additionalSegments = self.additionalSegments(False, self._gx, self._gy, cos_a90, sin_a90)
            if additionalSegments != []:
                for s in additionalSegments:
                    canvas.drawLine(
                        (s.start[0]),
                        (s.start[1]),
                        (s.end[0]),
                        (s.end[1]),
                        outline="purple")
