"""
A Pure Python 2D Robot Simulator

(c) 2017 Calysto Developers. Licensed under the GNU GPL.

"""
import time
import math
import random
import sys

from jyro.simulator.color import colorMap, colorCode

PIOVER180 = math.pi / 180.0
PIOVER2   = math.pi / 2.0
RESOLUTION = 7 # decimal places of accuracy in making comparisons, rounding
MAXRAYLENGTH = 1000.0 # some large measurement in meters

## Support functions

def sgn(v):
    """
    Return the sign of v.
    """
    if v >= 0:
        return +1
    else:
        return -1

def normRad(x):
    """
    Compute angle in range radians(-180) to radians(180)
    """
    while (x > math.pi):
        x -= 2 * math.pi
    while (x < -math.pi):
        x += 2 * math.pi
    return x

class Segment():
    """
    Represent a line segment.
    """
    def __init__(self, start, end, id=None, partOf=None):
        self.start = [round(v, RESOLUTION) for v in start]
        self.end = [round(v, RESOLUTION) for v in end]
        self.id = id
        self.partOf = partOf
        self.vertical = self.start[0] == self.end[0]
        if not self.vertical:
            self.slope = round((self.end[1] - self.start[1])/
                               (self.end[0] - self.start[0]), RESOLUTION)
            self.yintercept = round(self.start[1] -
                                    self.start[0] * self.slope, RESOLUTION)

    def length(self):
        return math.sqrt((self.start[0] - self.end[0])**2 +
                         (self.start[1] - self.end[1])**2)

    def angle(self):
        return math.atan2(self.end[1] - self.start[1],
                          self.end[0] - self.start[0])

    def parallel(self, other):
        if self.vertical:
            return other.vertical
        elif other.vertical:
            return 0
        else:
            return self.slope == other.slope

    # return the point at which two segments would intersect if they extended
    # far enough
    def intersection(self, other):
        if self.parallel(other):
            # the segments may intersect eventually, but we don't care
            return None
        elif self.vertical:
            return other.intersection(self)
        elif other.vertical:
            return (other.start[0],
                    self.yintercept + other.start[0] * self.slope)
        else:
            # m1x + b1 = m2x + b2; so
            # (m1 - m2)x + b1 - b2 == 0
            # (m1 - m2)x = b2 - b1
            # x = (b2 - b1)/(m1 - m2)
            # figure intersect:
            # putting a round() around both of these next 2 computations caused problems:
            x = ((other.yintercept - self.yintercept) / (self.slope - other.slope))
            return (x, self.yintercept + x * self.slope)

    def in_bbox(self, point):
        return (((self.end[0]   <= round(point[0], RESOLUTION) <= self.start[0]) or
                 (self.start[0] <= round(point[0], RESOLUTION) <= self.end[0])) and
                ((self.end[1]   <= round(point[1], RESOLUTION) <= self.start[1]) or
                 (self.start[1] <= round(point[1], RESOLUTION) <= self.end[1])))

    def on_line(self, point):
        # is a point collinear with this line
        if self.vertical:
            return round(point[0], RESOLUTION) == self.start[0]
        else:
            return (round(point[0] * self.slope + self.yintercept, RESOLUTION) ==
                    round(point[1]), RESOLUTION)

    def intersects(self, other):
        if self.parallel(other):
            # they can "intersect" if they are collinear and overlap
            if not (self.in_bbox(other.start) or self.in_bbox(other.end)):
                return None
            elif self.vertical:
                if self.start[0] == other.start[0]:
                    return self.intersection(other)
                else:
                    return None
            else:
                if self.yintercept == other.yintercept:
                    return self.intersection(other)
                else:
                    return None
        else:
            i = self.intersection(other)
            if self.in_bbox(i) and other.in_bbox(i):
                return i
            else:
                return None

class Simulator():
    def __init__(self, size, offsets, scale):
        self.width, self.height = size
        self.offset_x, self.offset_y = offsets
        self.scale = scale
        self.robots = []
        self.robotsByName = {}
        self.needToMove = [] # list of robots that need to move (see step)
        self.maxTrailSize = 10 # 5 * 60 * 10 # 5 minutes (one timeslice = 1/10 sec)
        self.trailStart = 0
        self.world = []
        self.time = 0.0
        self.timeslice = 100 # in milliseconds
        self.lightAboveWalls = 0
        self.properties = ["stall", "x", "y", "th", "thr", "energy"]
        # connections to pyrobot:
        self.ports = []
        self.assoc = {}
        self.done = 0
        self.stepCount = 0
        self.running = 0
        self.lights = []
        self.trail = []
        self.shapes = []

    def update_idletasks(self):
        pass

    def mainloop(self):
        self.running = 1
        while not self.done:
            self.step()
            time.sleep(self.timeslice/1000.0) # to run in real time
        self.running = 0

    def __getitem__(self, name):
        if name in self.robotsByName:
            return self.robotsByName[name]
        else:
            return None

    def remove(self, thing):
        pass

    def update(self):
        pass

    def addWall(self, x1, y1, x2, y2, color="black"):
        seg = Segment((x1, y1), (x2, y2), len(self.world) + 1, "wall")
        seg.color = color
        seg.type = "wall"
        self.world.append(seg)

    def addShape(self, name, *args, **nargs):
        # addShape("box", x, y, x, y, color)
        # addShape("polygon", points, fill = "black", outline = "purple")
        # addshape("line", (x1, y1), (x2, y2), fill = "purple", width?)
        # addshape("oval", (x1, y1), (x2, y2), fill = "purple", outline="yellow")
        if len(nargs) == 0:
            temp = list(args)
            temp.insert(0, name)
            self.shapes.append(temp)
        else:
            self.shapes.append( (name, args, nargs) )

    def addBox(self, ulx, uly, lrx, lry, color="white", wallcolor="black"):
        self.addWall( ulx, uly, ulx, lry, wallcolor)
        self.addWall( ulx, uly, lrx, uly, wallcolor)
        self.addWall( ulx, lry, lrx, lry, wallcolor)
        self.addWall( lrx, uly, lrx, lry, wallcolor)

    def addLight(self, x, y, brightness, color="yellow"):
        self.lights.append(Light(x, y, brightness, color))

    def refillLights(self, brightness):
        """
        Set all the lights to the given brightness.
        """
        for light in self.lights:
            if light.type != "fixed":
                continue
            light.brightness = brightness

    def resetPaths(self):
        pass

    def resetPath(self, pos):
        pass

    def reset(self):
        for r in self.robots:
            r._gx, r._gy, r._ga = r._xya
            r.energy = 10000.0
        for l in self.lights:
            l.x, l.y, l.brightness = l._xyb

    def draw(self):
        self.canvas.clear()
        for shape in self.shapes:
            if shape[0] == "box":
                name, ulx, uly, lrx, lry, fill = shape
                outline = "black"
                if self.canvas.display["wireframe"]:
                    if fill != "white":
                        outline = fill
                    else:
                        outline = "black"
                    fill = ""
                self.canvas.drawRectangle(self.scale_x(ulx), self.scale_y(uly),
                                     self.scale_x(lrx), self.scale_y(lry),
                                     tag="line", fill=fill, outline=outline)
            elif shape[0] == "polygon":
                name, points, nargs = shape
                xys = [(self.scale_x(x), self.scale_y(y)) for (x, y) in points]
                self.canvas.drawPolygon(xys, tag="line", **nargs)
            elif shape[0] == "line":
                name, ((x1, y1), (x2, y2)), nargs = shape
                x1, y1, x2, y2 = self.scale_x(x1), self.scale_y(y1), self.scale_x(x2), self.scale_y(y2)
                self.canvas.drawLine(x1, y1, x2, y2, tag="line", **nargs)
            elif shape[0] == "oval":
                name, ((x1, y1), (x2, y2)), nargs = shape
                x1, y1, x2, y2 = self.scale_x(x1), self.scale_y(y1), self.scale_x(x2), self.scale_y(y2)
                self.canvas.drawOval(x1, y1, x2, y2, tag="line", **nargs)
        if not self.canvas.display["wireframe"]:
            for segment in self.world:
                (x1, y1), (x2, y2) = segment.start, segment.end
                id = self.drawLine(x1, y1, x2, y2, fill="black", tag="line")
                segment.id = id
        for light in self.lights:
            if light.type != "fixed": continue 
            x, y, brightness, color = light.x, light.y, light.brightness, light.color
            self.drawOval((x - brightness), (y - brightness),
                          (x + brightness), (y + brightness),
                          tag="line", fill=color, outline="orange")
        i = 0
        for path in self.trail:
            if self.robots[i].subscribed and self.robots[i].display["trail"] == 1:
                if path[self.trailStart] != None:
                    lastX, lastY, lastA = path[self.trailStart]
                    #lastX, lastY = self.scale_x(lastX), self.scale_y(lastY)
                    color = self.robots[i].colorParts["trail"]
                    for p in range(self.trailStart, self.trailStart + self.maxTrailSize):
                        xya = path[p % self.maxTrailSize]
                        if xya == None: break
                        x, y = xya[0], xya[1]
                        self.drawLine(lastX, lastY, x, y, fill=color, tag="trail")
                        lastX, lastY = x, y
            i += 1
        for robot in self.robots:
            robot._last_pose = (-1, -1, -1)

    def resetLights(self, brightness, width, height):
        """
        Randomly relocate all lights in the environment within the
        bounding box defined by the given width and height.  Make sure
        that they are not too close to the edge of the bounding box.
        """
        for light in self.lights:
            if light.type != "fixed":
                continue
            light.x = 0.5 + random.random() * (width-1)
            light.y = 0.5 + random.random() * (height-1)
            light.brightness = brightness
            light.rgb = colorMap[light.color]

    def resetLightsToValues(self, brightnessList, width, height):
        """
        Randomly relocate all lights in the environment within the
        bounding box defined by the given width and height.  Make sure
        that they are not too close to the edge of the bounding box.
        """
        for i in range(len(self.lights)):
            light = self.lights[i]
            if light.type != "fixed":
                continue
            light.x = 0.5 + random.random() * (width-1)
            light.y = 0.5 + random.random() * (height-1)
            light.brightness = brightnessList[i]
            light.rgb = colorMap[light.color]

    def resetLightPositions(self, coords):
        """
        Relocate lights in the environment to the given list of
        coordinates.
        """
        for i in range(len(self.lights)):
            light = self.lights[i]
            if light.type != "fixed":
                continue
            light.x = coords[i][0]
            light.y = coords[i][1]

    def addRobot(self, port, r):
        self.robots.append(r)
        self.robotsByName[r.name] = r
        self.trail.append([None] * self.maxTrailSize)
        r.simulator = self
        r._xya = r._gx, r._gy, r._ga # save original position for later reset
        r._port = port
        if port != None:
            self.assoc[port] = r
            self.ports.append(port)

    def scale_x(self, x):
        return self.offset_x + (x * self.scale)

    def scale_y(self, y):
        return self.offset_y - (y * self.scale)

    def addTrail(self, pos, index, robot):
        self.trail[pos][index] = robot._gx, robot._gy, robot._ga

    def step(self):
        """
        Advance the world by timeslice milliseconds.
        """
        # might want to randomize this order so the same ones
        # don't always move first:
        self.needToMove = []
        self.time += (self.timeslice / 1000.0)
        i = 0
        for r in self.robots:
            r.ovx, r.ovy, r.ova = r.vx, r.vy, r.va
            resetVelocities = 0
            if r.stall:
                resetVelocities = 1
                ovx, r.ovx = r.ovx, r.ovx/5.0
                ovy, r.ovy = r.ovy, r.ovy/5.0
                ova, r.ova = r.ova, r.ova/5.0
            r.step(self.timeslice)
            if r.type != "puck" and resetVelocities:
                r.vx = ovx
                r.vy = ovy
                r.va = ova
            self.addTrail(i, self.stepCount % self.maxTrailSize, r)
            i += 1
        for r in self.needToMove:
            r.step(self.timeslice, movePucks = 0)
        if self.stepCount > self.maxTrailSize:
            self.trailStart = ((self.stepCount + 1) % self.maxTrailSize)
        self.stepCount += 1

    def castRay(self, robot, x1, y1, a, maxRange = MAXRAYLENGTH,
                ignoreRobot = "self",
                rayType = "range"):
        # ignoreRobot: all, self, other;
        hits = []
        x2, y2 = math.sin(a) * maxRange + x1, math.cos(a) * maxRange + y1
        seg = Segment((x1, y1), (x2, y2))
        # go down list of walls, and see if it hit anything:
        # check if it is not a light ray, or if it is, and not above walls:
        if (rayType != "light") or (rayType == "light" and not self.lightAboveWalls):
            for w in self.world:
                retval = w.intersects(seg)
                if retval:
                    dist = Segment(retval, (x1, y1)).length()
                    if dist <= maxRange:
                        hits.append( (dist, retval, w) ) # distance, hit, obj
        # go down list of robots, and see if you hit one:
        if ignoreRobot != "all":
            for r in self.robots:
                # don't hit your own bounding box if ignoreRobot == "self":
                if r.name == robot.name and ignoreRobot == "self":
                    continue
                # don't hit other's bounding box if ignoreRobot == "other":
                if r.name != robot.name and ignoreRobot == "other":
                    continue
                a90 = r._ga + PIOVER2
                cos_a90 = math.cos(a90)
                sin_a90 = math.sin(a90)
                segments = []
                if r.boundingBox != []:
                    xys = map(lambda x, y: (r._gx + x * cos_a90 - y * sin_a90,
                                            r._gy + x * sin_a90 + y * cos_a90),
                              r.boundingBox[0], r.boundingBox[1])
                    # for each of the bounding box segments:
                    xys = list(xys)

                    for i in range(len(xys)):
                        w = Segment( xys[i], xys[i - 1]) # using the previous one completes the polygon
                        w.color = r.color
                        w.type = r.type
                        w.robot = r
                        segments.append(w)
                if r.boundingSeg != []:
                    # bounding segments
                    xys = map(lambda x, y: (r._gx + x * cos_a90 - y * sin_a90,
                                            r._gy + x * sin_a90 + y * cos_a90),
                              r.boundingSeg[0], r.boundingSeg[1])
                    # for each of the bounding segments:
                    xys = list(xys)

                    for i in range(0, len(xys), 2):
                        w = Segment( xys[i], xys[i + 1]) # assume that they come in pairs
                        w.color = r.color
                        w.type = r.type
                        w.robot = r
                        segments.append(w)
                for s in r.additionalSegments(r._gx, r._gy, cos_a90, sin_a90,
                                              color=r.color, type=r.type, robot=r):
                    segments.append(s)
                for w in segments:
                    retval = w.intersects(seg)
                    if retval:
                        dist = Segment(retval, (x1, y1)).length()
                        if dist <= maxRange:
                            hits.append( (dist, retval, w) ) # distance,hit,obj
        if len(hits) == 0:
            return (None, None, None)
        else:
            return min(hits)

    def process(self, request, sockname):
        """
        Process does all of the work.
        request  - a string message
        sockname - (IPNUMBER (str), SOCKETNUM (int)) from client
        """
        retval = 'error'
        if request == 'reset':
            self.reset()
            retval = "ok"
        elif request.count('connectionNum'):
            connectionNum, port = request.split(":")
            retval = self.ports.index( int(port) )
        elif request == 'end' or request == 'exit':
            retval = "ok"
            self.done = 1
        elif request == 'quit':
            retval = "ok"
            self.done = 1
        elif request == "disconnect":
            retval = "ok"
        elif request == 'properties':
            retval = self.properties
        elif request == 'builtinDevices':
            retval = self.assoc[sockname[1]].builtinDevices
        elif request == 'forward':
            self.assoc[sockname[1]].move(0.3, 0.0)
            retval = "ok"
        elif request == 'left':
            self.assoc[sockname[1]].move(0.0, 0.3)
            retval = "ok"
        elif request == 'right':
            self.assoc[sockname[1]].move(0.0, -0.3)
            retval = "ok"
        elif request == 'back':
            self.assoc[sockname[1]].move(-0.3, 0.0)
            retval = "ok"
        elif request == 'name':
            retval = self.assoc[sockname[1]].name
        elif request == 'x':
            retval = self.assoc[sockname[1]].x
        elif request == 'energy':
            retval = self.assoc[sockname[1]].energy
        elif request == 'y':
            retval = self.assoc[sockname[1]].y
        elif request == 'stall':
            retval = self.assoc[sockname[1]].stall
        elif request == 'radius':
            retval = self.assoc[sockname[1]].radius
        elif request == 'thr':
            retval = self.assoc[sockname[1]].a
        elif request == 'th':
            retval = self.assoc[sockname[1]].a / PIOVER180
        elif len(request) > 1 and request[0] == '!': # eval
            try:
                retval = str(eval(request[1:]))
            except Exception(msg):
                try:
                    exec(request[1:])
                    retval = "ok"
                except:
                    retval = "error: %s" % msg
        else:
            # assume a package
            message = request.split("_")
            if message[0] == "m": # "m_t_r" move:translate:rotate
                t, r = 0, 0
                try:
                    t, r = float(message[1]), float(message[2])
                except:
                    pass
                retval = self.assoc[sockname[1]].move(t, r)
            elif message[0] == "l": # "l_string" say text
                el, strng = None, None
                try:
                    el, strng = message
                except:
                    pass
                strng = strng.replace("~-", "_")
                self.assoc[sockname[1]].say(strng)
                retval = "ok"
            elif message[0] == "a": # "a_name_x_y_th" simulation placement
                simulation, name, x, y, thr = None, None, None, None, None
                try:
                    simulation, name, x, y, thr = message
                    x = float(x)
                    y = float(y)
                    thr = float(thr)
                except:
                    pass
                if name in self.robotsByName:
                    r = self.robotsByName[name]
                    r.setPose(x, y, thr, 1)#handofgod
                    r.localize(0, 0, 0)
                    return "ok"
                elif name.isdigit():
                    pos = int(name)
                    r = self.robots[pos]
                    r.setPose(x, y, thr, 1)#handofgod
                    r.localize(0, 0, 0)
                    return "ok"
                return "error: no such robot position '%s'" % name
            elif message[0] == "b": # "b_x_y_th" localize
                localization, x, y, thr = None, None, None, None
                try:
                    localization, x, y, thr = message
                    x = float(x)
                    y = float(y)
                    thr = float(thr)
                except:
                    pass
                retval = self.assoc[sockname[1]].localize(x, y, thr)
            elif message[0] == "c": # "c_name" getpose
                simulation, name = None, None
                try:
                    simulation, name = message
                except:
                    pass
                if name in self.robotsByName:
                    r = self.robotsByName[name]
                    retval = (r._gx, r._gy, r._ga)
                elif name.isdigit():
                    pos = int(name)
                    r = self.robots[pos]
                    retval = (r._gx, r._gy, r._ga)
            elif message[0] == "f": # "f_i_v" rgb[i][v]
                index, pos = 0, 0
                try:
                    index, pos = int(message[1]), int(message[2])
                except:
                    pass
                device = self.assoc[sockname[1]].getIndex("light", index)
                if device:
                    retval = device.rgb[pos]
            elif message[0] == "h": # "h_v" bulb:value
                val = None
                try:
                    val = float(message[1])
                except:
                    pass
                self.assoc[sockname[1]].bulb.brightness = val
                retval = "ok"
            elif message[0] == "i": # "i_name_index_property_val"
                try:
                    code, dtype, index, property, val = message
                    index = int(index)
                    device = self.assoc[sockname[1]].getIndex(dtype, index)
                    oldval = device.__dict__[property]
                    if type(oldval) == str:
                        device.__dict__[property] = val
                    elif type(oldval) == int:
                        device.__dict__[property] = int(val)
                    elif type(oldval) == float:
                        device.__dict__[property] = float(val)
                    retval = "ok"
                except:
                    pass
            elif message[0] == "j": # "j_index_p_t_z" ptz[index].setPose(p, t, z)
                code, index, p, t, z = [None] * 5
                try:
                    code, index, p, t, z = message
                    index = int(index)
                except: pass
                device = self.assoc[sockname[1]].getIndex("ptz", index)
                if device:
                    if p == "None":
                        p = None
                    else:
                        p = float(p)
                    if t == "None":
                        t = None
                    else:
                        t = float(t)
                    if z == "None":
                        z = None
                    else:
                        z = float(z)
                    retval = device.setPose(p, t, z)
            elif message[0] == "k": # "k_index" ptz[index].getPose()
                try:
                    code, index = message
                    index = int(index)
                except:
                    pass
                device = self.assoc[sockname[1]].getIndex("ptz", index)
                if device:
                    retval = device.getPose()
            elif message[0] == "t": # "t_v" translate:value
                val = 0
                try:
                    val = float(message[1])
                except:
                    pass
                retval = self.assoc[sockname[1]].translate(val)
            elif message[0] == "v": # "v_v" global step scalar:value
                val = 0
                try:
                    val = float(message[1])
                except:
                    pass
                self.assoc[sockname[1]].stepScalar = val
                retval = "ok"
            elif message[0] == "o": # "o_v" rotate:value
                val = 0
                try:
                    val = float(message[1])
                except:
                    pass
                retval = self.assoc[sockname[1]].rotate(val)
            elif message[0] == "d": # "d_sonar" display:keyword
                val = 0
                try:
                    val = message[1]
                except:
                    pass
                retval = self.assoc[sockname[1]].display[val] = 1
            elif message[0] == "e": # "e_amt" eat:keyword
                val = 0
                if message[1] == "all":
                    val = -1.0 # code for "eat all light; returns 1.0"
                else:
                    try:
                        val = float(message[1])
                    except:
                        pass
                retval = self.assoc[sockname[1]].eat(val)
            elif message[0] == "x": # "x_expression" expression
                try:
                    retval = eval(message[1])
                except:
                    pass
            elif message[0] == "z": # "z_gripper_0_command" command
                dtype, index, command = None, None, None
                try:
                    dtype = message[1]
                    index = int(message[2])
                    command = message[3]
                except:
                    pass
                device = self.assoc[sockname[1]].getIndex(dtype, index)
                if device:
                    retval = device.__class__.__dict__[command](device)
            elif message[0] == "g": # "g_sonar_0" geometry_sensor_id
                index = 0
                for d in self.assoc[sockname[1]].devices:
                    if d.type == message[1]:
                        if int(message[2]) == index:
                            if message[1] in ["sonar", "light", "directional", "bulb", "ir", "bumper"]:
                                retval = d.geometry, d.arc, d.maxRange
                            elif message[1] == "camera":
                                retval = d.width, d.height
                        index += 1
            elif message[0] == "r": # "r_sonar_0" groups_sensor_id
                index = 0
                for d in self.assoc[sockname[1]].devices:
                    if d.type == message[1]:
                        if int(message[2]) == index:
                            if message[1] in ["sonar", "light", "directional", "ir", "bumper"]:
                                retval = d.groups
                        index += 1
            elif message[0] == "s": # "s_sonar_0" subscribe
                if message[1] in self.assoc[sockname[1]].display and self.assoc[sockname[1]].display[message[1]] != -1:
                    self.assoc[sockname[1]].display[message[1]] = 1
                self.properties.append("%s_%s" % (message[1], message[2]))
                self.assoc[sockname[1]].subscribed = 1
                retval = "ok"
            elif message[0] in ["sonar", "light", "directional",
                                "camera", "gripper", "ir", "bumper"]: # sonar_0, light_0...
                index = 0
                for d in self.assoc[sockname[1]].devices:
                    if d.type == message[0]:
                        try:
                            i = int(message[1])
                        except:
                            i = -1
                        if i == index:
                            retval = d.scan
                        index += 1
        return retval

    def setCanvas(self, canvas):
        self.canvas = canvas

    def addBox(self, ulx, uly, lrx, lry, color="white", wallcolor="black"):
        Simulator.addBox(self, ulx, uly, lrx, lry, color, wallcolor)
        self.shapes.append( ("box", ulx, uly, lrx, lry, color) )
        self.redraw()

    def addWall(self, x1, y1, x2, y2, color="black"):
        seg = Segment((x1, y1), (x2, y2), partOf="wall")
        seg.color = color
        seg.type = "wall"
        id = self.drawLine(x1, y1, x2, y2, fill=color, tag="line")
        seg.id = id
        self.world.append( seg )

if __name__ == "__main__":
    from jyro.simulator.canvas import Canvas
    from jyro.simulator.robot import Pioneer
    from jyro.simulator.device import PioneerFrontSonars
    
    sim = Simulator((443,466), (22,420), 40.357554)
    canvas = Canvas()
    robot = Pioneer("Pioneer", 4.99, 1.32, 6.28)
    robot.addDevice(PioneerFrontSonars())
    sim.setCanvas(canvas)
    sim.addRobot(60000, robot)

