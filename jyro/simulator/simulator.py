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
RESOLUTION = 7        # decimal places in making comparisons, rounding
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

class Light():
    def __init__(self, x, y, brightness, color):
        self.x = x
        self.y = y
        self.brightness = brightness
        self.color = color
        self._xyb = self.x, self.y, self.brightness
        self.type = "fixed"
        self.rgb = colorMap[self.color]

class Segment():
    """
    Represent a line segment.
    """
    def __init__(self, start, end, id=None, type=None):
        self.start = [round(v, RESOLUTION) for v in start]
        self.end = [round(v, RESOLUTION) for v in end]
        self.id = id
        self.type = type
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
    def __init__(self):
        """
        All distances in meters, angles in radians.
        .world - list of Segments, used for determining collisions, views
        .shapes - list of things to draw
        .lights - list of light sources
        """
        self.robots = []
        self.robotsByName = {}
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

    def update(self):
        pass

    def addBox(self, ulx, uly, lrx, lry, wallcolor="white"):
        ulx, lrx = min(ulx, lrx), max(ulx, lrx)
        uly, lry = max(uly, lry), min(uly, lry)
        self.addWall( ulx, uly, ulx, lry, wallcolor)
        self.addWall( ulx, uly, lrx, uly, wallcolor)
        self.addWall( ulx, lry, lrx, lry, wallcolor)
        self.addWall( lrx, uly, lrx, lry, wallcolor)
        self.addShape(Box((ulx, uly), (lrx, lry), fill=wallcolor))

    def addWall(self, x1, y1, x2, y2, color="black"):
        seg = Segment((x1, y1), (x2, y2), len(self.world) + 1, "wall")
        seg.color = color
        seg.type = "wall"
        self.world.append(seg)

    def addShape(self, shape):
        self.shapes.append(shape)

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

    def draw(self, canvas, scale=None):
        canvas.max_x = max([segment.start[0] for segment in self.world] +
                           [segment.end[0] for segment in self.world])
        canvas.max_y = max([segment.start[1] for segment in self.world] +
                           [segment.end[1] for segment in self.world])
        if scale is None:
            if canvas.height is None:
                canvas.scale = canvas.width/canvas.max_x
            elif canvas.width is None:
                canvas.scale =  canvas.height/canvas.max_y
            else:
                canvas.scale = min(canvas.width/canvas.max_x, canvas.height/canvas.max_y)
        else:
            canvas.scale = scale

        if canvas.height is None:
            canvas.height = canvas.max_y * canvas.scale
        if canvas.width is None:
            canvas.width = canvas.max_x * canvas.scale
            
        canvas.clear()
        for shape in self.shapes:
            shape.draw(canvas)
        for light in self.lights:
            if light.type != "fixed":
                continue 
            x, y, brightness, color = (canvas.pos_x(light.x),
                                       canvas.pos_y(light.y),
                                       light.brightness * canvas.scale,
                                       light.color)
            canvas.drawCircle(x, y, brightness, fill=color, outline="orange")
        for robot in self.robots:
            robot.draw(canvas)

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

    def addRobot(self, r, port=None):
        self.robots.append(r)
        self.robotsByName[r.name] = r
        r.simulator = self
        r.updateDevices()
        r._port = port
        if port != None:
            self.assoc[port] = r
            self.ports.append(port)

    def step(self):
        """
        Advance the world by timeslice milliseconds.
        """
        self.time += (self.timeslice / 1000.0)
        for r in self.robots:
            collision = r.step(self.timeslice)
        for r in self.robots:
            r.updateDevices()
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
                ## Direct light:
                retval = w.intersects(seg)
                if retval:
                    dist = Segment(retval, (x1, y1)).length()
                    if dist <= maxRange:
                        hits.append( (dist, retval, w) ) # distance, hit, obj
                ## Ambient light:
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
            return min(hits, key=lambda items: items[0])

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
                    r.setPose(x, y, thr) # handofgod
                    r.localize(0, 0, 0)
                    return "ok"
                elif name.isdigit():
                    pos = int(name)
                    r = self.robots[pos]
                    r.setPose(x, y, thr) # handofgod
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
                self.assoc[sockname[1]].device["bulb"].brightness = val
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

class Shape():
    def __init__(self):
        pass
    
class Box(Shape):
    def __init__(self, ul, lr, outline="black", fill="white"):
        Shape.__init__(self)
        self.x1, self.y1 = ul
        self.x2, self.y2 = lr
        self.outline = outline
        self.fill = fill

    def draw(self, canvas):
        outline, fill = self.outline, self.fill
        if canvas.display["wireframe"]:
            if fill != "white":
                outline = self.fill
            else:
                outline = "black"
            fill = ""
        canvas.drawRectangle(
            canvas.pos_x(self.x1),
            canvas.pos_y(self.y1),
            canvas.pos_x(self.x2),
            canvas.pos_y(self.y2),
            fill=fill, outline=outline)

class Polygon(Shape):
    def __init__(self, points, outline="black", fill="white"):
        Shape.__init__(self)
        self.points = points
        self.outline = outline
        self.fill = fill

    def draw(self, canvas):
        outline, fill = self.outline, self.fill
        if canvas.display["wireframe"]:
            if fill != "white":
                outline = fill
            else:
                outline = "black"
            fill = ""
        xys = [(canvas.pos_x(x),
                canvas.pos_y(y)) for (x, y) in self.points]
        canvas.drawPolygon(xys, fill=fill, outline=outline)

class Line(Shape):
    def __init__(self, p1, p2, outline="black", fill="white"):
        Shape.__init__(self)
        self.p1 = p1
        self.p2 = p2
        self.outline = outline
        self.fill = fill

    def draw(self, canvas):
        x1, y1, x2, y2 = (canvas.pos_x(self.p1[0]),
                          canvas.pos_y(self.p1[1]),
                          canvas.pos_x(self.p2[0]),
                          canvas.pos_y(self.p2[1]))
        canvas.drawLine(x1, y1, x2, y2, outline=self.outline)

class Oval(Shape):
    def __init__(self, p1, p2, outline="black", fill="white"):
        Shape.__init__(self)
        self.p1 = p1
        self.p2 = p2
        self.outline = outline
        self.fill = fill

    def draw(self, canvas):
        outline, fill = self.outline, self.fill
        if canvas.display["wireframe"]:
            if fill != "white":
                outline = fill
            else:
                outline = "black"
            fill = ""
        x1, y1, x2, y2 = (canvas.pos_x(self.p1[0]),
                          canvas.pos_y(self.p1[1]),
                          canvas.pos_x(self.p2[0]),
                          canvas.pos_y(self.p2[1]))
        canvas.drawOval(x1, y1, x2, y2, fill=fill, outline=outline)

def main():
    from jyro.simulator import Pioneer, Simulator, PioneerFrontSonars, Gripper
    
    sim = Simulator()
    # (443,466), (22,420), 40.357554)
    sim.addBox(0, 0, 10, 10, wallcolor="white") # meters
    sim.addBox(1, 1, 2, 2, wallcolor="purple")
    sim.addBox(7, 7, 8, 8, wallcolor="purple")
    robot = Pioneer("Pioneer", 5.00, 5.00, math.pi / 2) # meters, radians
    robot.addDevice(PioneerFrontSonars(maxRange=4.0))
    robot.addDevice(Gripper())
    sim.addRobot(robot)
    return sim
    
if __name__ == "__main__":
    from jyro.simulator import Canvas
    from jyro.simulator.svgcanvas import SVGCanvas
    sim = main()
    print("pose:", sim["Pioneer"].getPose())
    sim["Pioneer"].move(1, 1)
    canvas = SVGCanvas((400, 400))
    for i in range(500):
        sim.step()
        for r in sim.robots:
            r.updateDevices()
        print("pose:", sim["Pioneer"].getPose())
        sim.draw(canvas)
        canvas.save("canvas%d.svg" % i)
