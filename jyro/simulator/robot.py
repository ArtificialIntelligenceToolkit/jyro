from jyro.simulator.simulator import Segment, Line
from jyro.simulator.svgcanvas import SVGCanvas
from jyro.simulator.device import Device

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
        self.proposePosition = 0 # used to check for obstacles before moving
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
        self.simulator = None # will be set when added to simulator
        # -1: don't automatically turn display on when subscribing:
        self.display = {"body": 1, "boundingBox": 0, "gripper": -1, "camera": 0, "sonar": 0,
                        "light": -1, "lightBlocked": 0, "trail": -1, "ir": -1, "bumper": 1,
                        "speech": 1}
        self.shapes = []
        self.addDevice(Device("speech"))

    def brain(self):
        pass
            
    def reset(self):
        self._gx, self._gy, self._ga = self._xya
        self.stall = 0
        self.energy = 10000.0
        self.x, self.y, self.a = (0.0, 0.0, 0.0) # localize
        self.maxEnergyCostPerStep = 1.0
        self.sayText = ""
        self.friction = 1.0
        self.vx, self.vy, self.va = (0.0, 0.0, 0.0) # meters / second, rads / second

    def _repr_svg_(self):
        from jyro.simulator import Simulator
        canvas = SVGCanvas((240, 240))
        canvas.max_x = 1.0 # meters
        canvas.max_y = 1.0 # meters
        canvas.scale = min(canvas.width/canvas.max_x, canvas.height/canvas.max_y)
        xya = self._gx, self._gy, self._ga
        self._gx, self._gy, self._ga = (0.5, 0.5, 0)
        remove_sim = False
        if self.simulator is None:
            self.simulator = Simulator() # for drawings
            self.updateDevices() # ASSUME: if old sim, it is up to date (self.shapes are there)
            remove_sim = True
        self.draw(canvas)
        if remove_sim:
            self.simulator = None
        svg = canvas._repr_svg_()
        self._gx, self._gy, self._ga = xya
        return svg
        
    def drawRay(self, dtype, x1, y1, x2, y2, color):
        self.shapes.append(Line((x1, y1), (x2, y2), outline=color))
                
    def additionalSegments(self, x, y, cos_a90, sin_a90, **dict):
        # dynamic segments
        retval = []
        if self.device["gripper"]:
            g = self.device["gripper"]
            x1, x2, x3, x4 = g.pose[0], g.pose[0] + g.armLength, g.pose[0], g.pose[0] + g.armLength
            y1, y2, y3, y4 = g.armPosition, g.armPosition, -g.armPosition,  -g.armPosition
            if g.robot.proposePosition and g.velocity != 0.0:
                armPosition, velocity = g.moveWhere()
                y1, y2, y3, y4 = armPosition, armPosition, -armPosition,  -armPosition
            xys = map(lambda nx, ny: (x + nx * cos_a90 - ny * sin_a90,
                                      y + nx * sin_a90 + ny * cos_a90),
                      (x1, x2, x3, x4), (y1, y2, y3, y4))
            xys = list(xys)
            w = [Segment(xys[0], xys[1], type="gripper"),
                 Segment(xys[2], xys[3], type="gripper")]
            for s in w:
                for key in dict:
                    s.__dict__[key] = dict[key]
                retval.append(s)
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
        self._gx, self._gy = x, y
        if a is not None:
            self.setAngle(a)

    def setAngle(self, a):
        # if our angle changes, update localized position:
        diff = a - self._ga
        self.a += diff
        self.a = self.a % (2 * math.pi) # keep in the positive range
        self._ga = a % (2 * math.pi) # keep in the positive range

    def setPose(self, x, y, a=None):
        self._gx = x
        self._gy = y
        if a is not None:
            self._ga = a % (2 * math.pi) # keep in the positive range
        self.updateDevices()
            
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
        self.proposePosition = 1
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
        for s in self.additionalSegments(p_x, p_y, cos_a90, sin_a90):
            segments.append(s)
        for bb in segments:
            # check each segment of the robot's bounding segs for wall obstacles:
            for w in self.simulator.world:
                if bb.intersects(w):
                    self.proposePosition = 0
                    if self.device["gripper"] and self.device["gripper"].velocity != 0:
                        self.device["gripper"].state = "stop"
                        self.device["gripper"].velocity = 0
                    if self.ovx != 0 or self.ovy != 0 or self.ova != 0:
                        self.stall = 1
                    return False # collision
            # check each segment of the robot's bounding box for other robots:
            for r in self.simulator.robots:
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
                for s in r.additionalSegments(r._gx, r._gy, cos_r_a90, sin_r_a90):
                    r_segments.append(s)
                for r_seg in r_segments:
                    bbintersect = bb.intersects(r_seg)
                    if bbintersect:
                        self.proposePosition = 0
                        if self.device["gripper"] and self.device["gripper"].velocity != 0:
                            self.device["gripper"].state = "stop"
                            self.device["gripper"].velocity = 0
                        if self.ovx != 0 or self.ovy != 0 or self.ova != 0:
                            self.stall = 1
                        return False # collision
        self.proposePosition = 0
        # ok! move the robot, if it wanted to move
        if self.device["gripper"] and self.device["gripper"].velocity != 0:
            # handle moving paddles
            d = self.device["gripper"]
            d.armPosition, d.velocity = d.moveWhere()
            if d.armPosition == d.openPosition:
                ## Drop puck:
                if d.storage != [] and d.state == "deploy":
                    x = d.pose[0] + d.armLength/2
                    y = 0
                    rx, ry = (p_x + x * cos_a90 - y * sin_a90,
                              p_y + x * sin_a90 + y * cos_a90)
                    r = d.storage.pop()
                    r.setPose(rx, ry, 0.0)
                    d.state = "open"
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

    def addDevice(self, dev):
        self.devices.append(dev)
        self.device[dev.type] = dev
        dev.robot = self
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

    def draw(self, canvas):
        Robot.draw(self, canvas)
        a90 = self._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        canvas.drawOval(canvas.pos_x(self._gx - self.radius),
                        canvas.pos_y(self._gy - self.radius),
                        canvas.pos_x(self._gx + self.radius),
                        canvas.pos_y(self._gy + self.radius),
                        fill=self.color, outline="blue")
        x = canvas.pos_x(self._gx + self.radius * cos_a90 - 0 * sin_a90)
        y = canvas.pos_y(self._gy + self.radius * sin_a90 + 0 * cos_a90)
        canvas.drawLine(canvas.pos_x(self._gx), canvas.pos_y(self._gy), x, y, outline="blue", width=3)

class Puck(Robot):
    def __init__(self, *args, **kwargs):
        Robot.__init__(self, *args, **kwargs)
        self.radius = 0.05
        self.friction = 0.90
        self.type = "puck"

    def draw(self, canvas):
        """
        Draws the body of the robot. Not very efficient.
        """
        Robot.draw(self, canvas)
        if self.display["body"] == 1:
            x1, y1, x2, y2 = (canvas.pos_x(self._gx - self.radius),
                              canvas.pos_y(self._gy - self.radius),
                              canvas.pos_x(self._gx + self.radius),
                              canvas.pos_y(self._gy + self.radius))
            canvas.drawOval(x1, y1, x2, y2, fill=self.color, outline="black")
        if self.display["boundingBox"] == 1 and self.boundingBox != []:
            # Body Polygon, by x and y lists:
            a90 = self._ga + PIOVER2 # angle is 90 degrees off for graphics
            cos_a90 = math.cos(a90)
            sin_a90 = math.sin(a90)
            xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                   self._gy + x * sin_a90 + y * cos_a90),
                     self.boundingBox[0], self.boundingBox[1])
            xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
            canvas.drawPolygon(xy, fill="", outline="purple")

class Pioneer(Robot):
    def __init__(self, name, x, y, a, color="red"):
        Robot.__init__(self, name, x, y, a,
                       boundingBox=((.225, .225, -.225, -.225),
                                    (.175, -.175, -.175, .175)), color=color)
        self.radius = 0.4

    def draw(self, canvas):
        """
        Draws the body of the robot. Not very efficient.
        """
        Robot.draw(self, canvas)
        # Body Polygon, by x and y lists:
        sx = [.225, .15, -.15, -.225, -.225, -.15, .15, .225]
        sy = [.08, .175, .175, .08, -.08, -.175, -.175, -.08]
        a90 = self._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        if self.display["body"] == 1:
            xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                   self._gy + x * sin_a90 + y * cos_a90),
                     sx, sy)
            xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
            if self.stall:
                canvas.drawPolygon(xy, fill="white", outline="black")
            else:
                canvas.drawPolygon(xy, fill=self.color, outline=self.color)
            bx = [ .14, .06, .06, .14] # front camera
            by = [-.06, -.06, .06, .06]
            xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                   self._gy + x * sin_a90 + y * cos_a90),
                     bx, by)
            xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
            canvas.drawPolygon(xy, fill="black")
            if self.device["bulb"]:
                x = canvas.pos_x(self._gx + self.device["bulb"].x * cos_a90 - self.device["bulb"].y * sin_a90)
                y = canvas.pos_y(self._gy + self.device["bulb"].x * sin_a90 + self.device["bulb"].y * cos_a90)
                radius = .05
                canvas.drawOval(x - radius, y - radius, x + radius, y + radius,
                                        fill=self.color, outline=self.color)
            if self.device["light"]:
                for (bx, by, ba) in self.device["light"].geometry:
                    x = canvas.pos_x(self._gx + bx * cos_a90 - by * sin_a90)
                    y = canvas.pos_y(self._gy + bx * sin_a90 + by * cos_a90)
                    radius = .025 * canvas.scale
                    canvas.drawCircle(x, y, radius, fill="yellow", outline="orange")
            if self.device["gripper"]:
                # draw grippers:
                # base:
                xy = [(canvas.pos_x(self._gx + x * cos_a90 - y * sin_a90),
                       canvas.pos_y(self._gy + x * sin_a90 + y * cos_a90)) for (x,y) in
                      ((self.device["gripper"].pose[0], self.device["gripper"].openPosition),
                       (self.device["gripper"].pose[0], -self.device["gripper"].openPosition))]
                canvas.drawLine(xy[0][0], xy[0][1], xy[1][0], xy[1][1],
                                        outline="black")
                # left arm:
                xs = []
                ys = []
                xs.append(self.device["gripper"].pose[0]);     ys.append(self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(self.device["gripper"].armPosition - 0.01)
                xs.append(self.device["gripper"].pose[0]);     ys.append(self.device["gripper"].armPosition - 0.01)
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         xs, ys)
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                canvas.drawPolygon(xy, fill="black", outline="black")
                # right arm:
                xs = []
                ys = []
                xs.append(self.device["gripper"].pose[0]);     ys.append(-self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(-self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(-self.device["gripper"].armPosition - 0.01)
                xs.append(self.device["gripper"].pose[0]);     ys.append(-self.device["gripper"].armPosition - 0.01)
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         xs, ys)
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                canvas.drawPolygon(xy, fill="black", outline="black")
        if self.display["boundingBox"] == 1:
            if self.boundingBox != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingBox[0], self.boundingBox[1])
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                canvas.drawPolygon(xy, fill="", outline="purple")
            if self.boundingSeg != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingSeg[0], self.boundingSeg[1])
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                for i in range(0, len(xy), 2):
                    canvas.drawLine(xy[i][0], xy[i][1],
                                            xy[i + 1][0], xy[i + 1][1],
                                            outline="purple")
            additionalSegments = self.additionalSegments(self._gx, self._gy, cos_a90, sin_a90)
            if additionalSegments != []:
                for s in additionalSegments:
                    canvas.drawLine(canvas.pos_x(s.start[0]),
                                    canvas.pos_y(s.start[1]),
                                    canvas.pos_x(s.end[0]),
                                    canvas.pos_y(s.end[1]),
                                    outline="purple")
        if self.display["speech"] == 1:
            if self.sayText != "":
                # center of robot:
                x, y = canvas.pos_x(self._gx), canvas.pos_y(self._gy)
                canvas.drawText(x, y, self.sayText) # % self.name)

class Myro(Robot):
    def __init__(self, *args, **kwargs):
        Robot.__init__(self, *args, **kwargs)
        self.radius = 0.25

    def draw(self, canvas):
        """
        Draws the body of the robot. Not very efficient.
        """
        Robot.draw(self, canvas)
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
            xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
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
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                canvas.drawPolygon(xy, fill=colors[i])
            # --------------------------------------------------------------------------
            if self.device["bulb"]:
                x = canvas.pos_x(self._gx + self.device["bulb"].x * cos_a90 - self.device["bulb"].y * sin_a90)
                y = canvas.pos_y(self._gy + self.device["bulb"].x * sin_a90 + self.device["bulb"].y * cos_a90)
                radius = .04
                canvas.drawOval(x - radius, y - radius, x + radius, y + radius,
                                        fill=self.color, outline="black")
            if self.device["gripper"]:
                # draw grippers:
                # base:
                xy = [(self._gx + x * cos_a90 - y * sin_a90,
                       self._gy + x * sin_a90 + y * cos_a90) for (x,y) in
                      ((self.device["gripper"].pose[0], self.device["gripper"].openPosition),
                       (self.device["gripper"].pose[0], -self.device["gripper"].openPosition))]
                canvas.drawLine(xy[0][0], xy[0][1], xy[1][0], xy[1][1],
                                        outline="black")
                # left arm:
                xs = []
                ys = []
                xs.append(self.device["gripper"].pose[0]);     ys.append(self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(self.device["gripper"].armPosition - 0.01)
                xs.append(self.device["gripper"].pose[0]);     ys.append(self.device["gripper"].armPosition - 0.01)
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         xs, ys)
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                canvas.drawPolygon(xy, fill="black", outline="black")
                # right arm:
                xs = []
                ys = []
                xs.append(self.device["gripper"].pose[0]);     ys.append(-self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(-self.device["gripper"].armPosition + 0.01)
                xs.append(self.device["gripper"].pose[0] + self.device["gripper"].armLength); ys.append(-self.device["gripper"].armPosition - 0.01)
                xs.append(self.device["gripper"].pose[0]);     ys.append(-self.device["gripper"].armPosition - 0.01)
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         xs, ys)
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                canvas.drawPolygon(xy, fill="black", outline="black")
        if self.display["boundingBox"] == 1:
            if self.boundingBox != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingBox[0], self.boundingBox[1])
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                canvas.drawPolygon(xy, fill="", outline="purple")
            if self.boundingSeg != []:
                xy = map(lambda x, y: (self._gx + x * cos_a90 - y * sin_a90,
                                       self._gy + x * sin_a90 + y * cos_a90),
                         self.boundingSeg[0], self.boundingSeg[1])
                xy = [(canvas.pos_x(x), canvas.pos_y(y)) for (x, y) in list(xy)]
                for i in range(0, len(xy), 2):
                    canvas.drawLine(xy[i][0], xy[i][1],
                                            xy[i + 1][0], xy[i + 1][1],
                                            outline="purple")
            additionalSegments = self.additionalSegments(self._gx, self._gy, cos_a90, sin_a90)
            if additionalSegments != []:
                for s in additionalSegments:
                    canvas.drawLine(
                        canvas.pos_x(s.start[0]),
                        canvas.pos_y(s.start[1]),
                        canvas.pos_x(s.end[0]),
                        canvas.pos_y(s.end[1]),
                        outline="purple")


