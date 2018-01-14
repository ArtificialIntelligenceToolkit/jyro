"""
A Pure Python 2D Robot Simulator

(c) 2017 Calysto Developers. Licensed under the GNU GPL.

"""
import time
import math
import random
import base64
import html
import sys
import io
import os
import threading

from jyro.simulator.color import colorMap, colorCode
from jyro.simulator.canvas import Canvas

from calysto.graphics import Color
import PIL.Image
import ipywidgets
from IPython.display import display
from ipywidgets import (HTML, Button, VBox, HBox, IntSlider, Select, Text,
                        Layout, Label, FloatSlider, Checkbox, IntText,
                        Box, Accordion, FloatText, Output, Widget, register,
                        widget_serialization, DOMWidget)
from traitlets import Bool, Dict, Int, Float, Unicode, List, Instance

PIOVER180 = math.pi / 180.0
PIOVER2   = math.pi / 2.0
RESOLUTION = 7        # decimal places in making comparisons, rounding
MAXRAYLENGTH = 1000.0 # some large measurement in meters

## Support functions

def sgn(v):
    """
    Return the sign of v.

    >>> sgn(13)
    1
    >>> sgn(-2)
    -1
    """
    if v >= 0:
        return +1
    else:
        return -1

def normRad(x):
    """
    Compute angle in range radians(-180) to radians(180)

    >>> "%.2f" % normRad(90)
    '2.04'
    >>> "%.2f" % normRad(-90)
    '-2.04'
    """
    while (x > math.pi):
        x -= 2 * math.pi
    while (x < -math.pi):
        x += 2 * math.pi
    return x

class Light():
    """
    A light source.

    Args:
        x (float): x coordinate
        y (float): y coordinate
        brightness (float): illumination, typically 1.0
        color (Color): an instance of the Color class that defines an RGB value

    Example:
        >>> lgt = Light(3, 2, 1.0, Color(100,100,100))
        >>> lgt.reset()

    """
    def __init__(self, x, y, brightness, color):
        self._xyb = (x, y, brightness)
        self.reset()
        if isinstance(color, str):
            self.color = colorMap[color]
        else:
            self.color = color
        if isinstance(self.color, str):
            self.rgb = colorMap[self.color]
        else:
            self.rgb = self.color

    def reset(self):
        self.x, self.y, self.brightness = self._xyb

    def draw(self, canvas):
        radius = 0.25 # meters
        x, y, brightness, color = ((self.x),
                                   (self.y),
                                   radius,
                                   self.color)
        canvas.drawCircle(x, y, brightness, fill=color, outline=color)

class Segment():
    """
    Represent a line segment.

    >>> segment = Segment((0,0), (0,10), 42, "wall")
    >>> segment.length()
    10.0
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

class Physics():
    """
    >>> physics = Physics()

    """
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

    def addBox(self, ulx, uly, lrx, lry, fill="purple", wallcolor=None):
        if wallcolor is None:
            wallcolor = fill
        ulx, lrx = min(ulx, lrx), max(ulx, lrx)
        uly, lry = max(uly, lry), min(uly, lry)
        self.addWall( ulx, uly, ulx, lry, wallcolor)
        self.addWall( ulx, uly, lrx, uly, wallcolor)
        self.addWall( ulx, lry, lrx, lry, wallcolor)
        self.addWall( lrx, uly, lrx, lry, wallcolor)
        self.addShape(Box((ulx, uly), (lrx, lry), fill=fill, outline=wallcolor))

    def addWall(self, x1, y1, x2, y2, color="black"):
        seg = Segment((x1, y1), (x2, y2), len(self.world) + 1, "wall")
        seg.color = color
        seg.type = "wall"
        self.world.append(seg)

    def addShape(self, shape):
        self.shapes.append(shape)

    def addLight(self, x, y, brightness, color="yellow"):
        self.lights.append(Light(x, y, brightness, color))

    def resetPaths(self):
        pass

    def resetPath(self, pos):
        pass

    def reset(self):
        for r in self.robots:
            r.reset()
            r.updateDevices()
        for l in self.lights:
            l.reset()

    def draw(self, canvas, scale=None, ignore=[], trace=False):
        """
        ignore can be a list of any of ["shapes", "robots", "lights"]
        """
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
        if "shapes" not in ignore:
            for shape in self.shapes:
                shape.draw(canvas)
        if "lights" not in ignore:
            for light in self.lights:
                light.draw(canvas)
        ## Anything in the overlay gets added here:
        if "robots" not in ignore:
            for robot in self.robots:
                if trace:
                    if (robot._px, robot._py) != (robot._gx, robot._py):
                        canvas._overlay.scale = canvas.scale
                        canvas._overlay.max_y = canvas.max_y
                        canvas._overlay.drawLine(robot._px, robot._py,
                                                 robot._gx, robot._gy,
                                                 width=1, outline="purple")
        canvas._canvas.shapes.extend(canvas._overlay._canvas.shapes)
        if "robots" not in ignore:
            for robot in self.robots:
                robot.draw(canvas)
        return canvas

    def addRobot(self, r, port=None):
        self.robots.append(r)
        self.robotsByName[r.name] = r
        r.physics = self
        r.updateDevices()
        r._port = port
        if port != None:
            self.assoc[port] = r
            self.ports.append(port)

    def step(self, run_brain=True):
        """
        Advance the world by timeslice milliseconds.
        """
        self.time += (self.timeslice / 1000.0)
        for r in self.robots:
            # if not running in a thread, call brain now:
            if run_brain:
                r.brain(r) # brain is a function that takes self
            collision = r.step(self.timeslice)
        for r in self.robots:
            r.updateDevices()
        self.stepCount += 1

    def castRay(self, robot, x1, y1, a, maxRange = MAXRAYLENGTH,
                ignoreRobot=["self"], rayType="range"):
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
        if "all" not in ignoreRobot:
            for r in self.robots:
                # don't hit your own bounding box if ignoreRobot == "self":
                if r.name == robot.name and "self" in ignoreRobot:
                    continue
                # don't hit other's bounding box if ignoreRobot == "other":
                if r.name != robot.name and "other" in ignoreRobot:
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
                for s in r.additionalSegments(False, r._gx, r._gy, cos_a90, sin_a90,
                                              color=r.color, type=r.type):
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

class Shape():
    def __init__(self):
        pass

class Box(Shape):
    """
    >>> box = Box((0, 10), (5, 0))
    >>> box.max_min()
    ((5, 10), (0, 0))
    """
    def __init__(self, ul, lr, outline="black", fill="white"):
        Shape.__init__(self)
        self.x1, self.y1 = ul
        self.x2, self.y2 = lr
        if isinstance(outline, str):
            self.outline = colorMap[outline]
        else:
            self.outline = outline
        if isinstance(fill, str):
            self.fill = colorMap[fill]
        else:
            self.fill = fill

    def max_min(self):
        return ((max(self.x1, self.x2), max(self.y1, self.y2)),
                (min(self.x1, self.x2), min(self.y1, self.y2)))

    def draw(self, canvas):
        outline, fill = self.outline, self.fill
        if canvas.display["wireframe"]:
            if fill != "white":
                outline = self.fill
            else:
                outline = "black"
            fill = ""
        canvas.drawRectangle(
            (self.x1),
            (self.y1),
            (self.x2),
            (self.y2),
            fill=fill, outline=outline)

class Polygon(Shape):
    def __init__(self, points, outline="black", fill="white"):
        Shape.__init__(self)
        self.points = points
        if isinstance(outline, str):
            self.outline = colorMap[outline]
        else:
            self.outline = outline
        if isinstance(fill, str):
            self.fill = colorMap[fill]
        else:
            self.fill = fill

    def max_min(self):
        return ((max([xy[0] for xy in self.points]),
                 max([xy[1] for xy in self.points])),
                (min([xy[0] for xy in self.points]),
                 min([xy[1] for xy in self.points])))

    def draw(self, canvas):
        outline, fill = self.outline, self.fill
        if canvas.display["wireframe"]:
            if fill != "white":
                outline = fill
            else:
                outline = "black"
            fill = ""
        xys = self.points
        canvas.drawPolygon(xys, fill=fill, outline=outline)

class Line(Shape):
    def __init__(self, p1, p2, outline="black", fill="white"):
        Shape.__init__(self)
        self.p1 = p1
        self.p2 = p2
        if isinstance(outline, str):
            self.outline = colorMap[outline]
        else:
            self.outline = outline
        if isinstance(fill, str):
            self.fill = colorMap[fill]
        else:
            self.fill = fill

    def max_min(self):
        return ((max(self.p1[0], self.p2[0]), max(self.p1[1], self.p2[1])),
                (min(self.p1[0], self.p2[0]), min(self.p1[1], self.p2[1])))

    def draw(self, canvas):
        x1, y1, x2, y2 = ((self.p1[0]),
                          (self.p1[1]),
                          (self.p2[0]),
                          (self.p2[1]))
        canvas.drawLine(x1, y1, x2, y2, outline=self.outline)

class Oval(Shape):
    def __init__(self, p1, p2, outline="black", fill="white"):
        Shape.__init__(self)
        self.p1 = p1
        self.p2 = p2
        if isinstance(outline, str):
            self.outline = colorMap[outline]
        else:
            self.outline = outline
        if isinstance(fill, str):
            self.fill = colorMap[fill]
        else:
            self.fill = fill

    def max_min(self):
        return ((max(self.p1[0], self.p2[0]), max(self.p1[1], self.p2[1])),
                (min(self.p1[0], self.p2[0]), min(self.p1[1], self.p2[1])))

    def draw(self, canvas):
        outline, fill = self.outline, self.fill
        if canvas.display["wireframe"]:
            if fill != "white":
                outline = fill
            else:
                outline = "black"
            fill = ""
        x1, y1, x2, y2 = ((self.p1[0]),
                          (self.p1[1]),
                          (self.p2[0]),
                          (self.p2[1]))
        canvas.drawOval(x1, y1, x2, y2, fill=fill, outline=outline)

class Simulator():
    """
    >>> from jyro.simulator import (Pioneer, Simulator, Camera,
    ...                             PioneerFrontSonars, Gripper,
    ...                             PioneerFrontLightSensors)
    >>> def worldf(sim):
    ...     sim.addBox(0, 0, 10, 10, fill="white", wallcolor="grey") # meters
    ...     sim.addBox(1, 1, 2, 2, "purple")
    ...     sim.addBox(7, 7, 8, 8, "purple")
    ...     ## brightness of 1 is radius 1 meter
    ...     sim.addLight(7, 7, 4.25, color=Color(255, 255, 0, 64))

    >>> robot = Pioneer("Pioneer", 5.00, 5.00, math.pi / 2) # meters, radians
    >>> robot.addDevice(PioneerFrontSonars(maxRange=4.0)) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.addDevice(Gripper()) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.addDevice(PioneerFrontLightSensors(maxRange=1.0)) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.addDevice(Camera()) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.brain = lambda self: self.move(1, 1)

    >>> sim = Simulator(robot, worldf)
    >>> for i in range(10):
    ...     sim.step()
    ...     sim.canvas.save("canvas%d.svg" % i)

    """
    def __init__(self, robot=None, worldf=None, size=None, gamepad=False,
                 trace=False):
        if robot is None:
            self.robot = None
            self.robots = []
        elif isinstance(robot, list):
            self.robot = robot[0]
            self.robots = robot
        else:
            self.robot = robot
            self.robots = [robot]
        self.worldf = worldf
        self.size = size if size else (400, 400)
        self.trace = trace
        self.canvas = self.makeCanvas()
        self.widgets = {}
        self.reset()
        self.physics.draw(self.canvas, trace=self.trace)
        self.widget = self.create_widgets(gamepad)
        if self.widget:
            display(self.widget)
        self.update_gui()

    def makeCanvas(self):
        """
        >>> def worldf(physics):
        ...     physics.addBox(0, 5, 10, 0)
        >>> sim = Simulator(worldf=worldf)
        >>> sim.makeCanvas()                  # doctest: +ELLIPSIS
        <jyro.simulator.canvas.Canvas object at ...>
        """
        return Canvas(self.size)

    def reset(self):
        self.canvas.reset()
        self.physics = Physics()
        if self.worldf is not None:
            self.worldf(self.physics)
        for robot in self.robots:
            robot.reset()
            self.physics.addRobot(robot)
        if self.widgets:
            self.update_gui()

    def create_widgets(self, gamepad=False):
        pass

    def render(self, canvas=None):
        if canvas is None:
            canvas = self.canvas
        self.physics.draw(canvas, trace=self.trace)
        return canvas

    def save(self, filename):
        self.canvas.save(filename)

    def update(self):
        self.physics.draw(self.canvas, trace=self.trace)

    def update_gui(self, data=None, set_angle=True):
        self.physics.draw(self.canvas, trace=self.trace)
        if data:
            print("%.2f seconds" % self.physics.time)
            for robot in self.robots:
                print("    %s: %s" % (robot.name, robot.getPose()))

    def step(self, step_seconds=None, run_brain=False):
        ## Update Simulator:
        if step_seconds == 0:
            self.reset()
            self.physics.time = 0.0
        elif step_seconds is None:
            self.physics.step(run_brain)
        else:
            for step in range(int(step_seconds/.1)):
                self.physics.step(run_brain)
        self.update_gui()

    def get_image(self):
        if self.robot and self.robot.device["camera"]:
            image = self.robot.device["camera"].getImage()
        else:
            image = PIL.Image.new("RGB", (120, 60))
        return image

    def get_image_uri(self):
        return self.image_to_uri(self.get_image())

    def image_to_uri(self, img_src):
        # Convert to binary data:
        b = io.BytesIO()
        try:
            img_src.save(b, format='gif')
        except:
            return ""
        data = b.getvalue()
        data = base64.b64encode(data)
        if not isinstance(data, str):
            data = data.decode("latin1")
        return "data:image/gif;base64,%s" % html.escape(data)

    def movie(self, poses, function, movie_name=None, start=0, stop=None, step=1,
              loop=0, optimize=True, duration=100, embed=False, mp4=True):
        """
        Make a movie from a list of poses and a function.

        function(simulator, index) and returns a displayable.

        >>> from jyro.simulator import (Pioneer, Simulator, Camera,
        ...                             PioneerFrontSonars, Gripper,
        ...                             PioneerFrontLightSensors)
        >>> def worldf(sim):
        ...     sim.addBox(0, 0, 10, 10, fill="white", wallcolor="grey") # meters
        ...     sim.addBox(1, 1, 2, 2, "purple")
        ...     sim.addBox(7, 7, 8, 8, "purple")
        ...     ## brightness of 1 is radius 1 meter
        ...     sim.addLight(7, 7, 4.25, color=Color(255, 255, 0, 64))

        >>> robot = Pioneer("Pioneer", 5.00, 5.00, math.pi / 2) # meters, radians
        >>> robot.addDevice(PioneerFrontSonars(maxRange=4.0)) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.addDevice(Gripper()) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.addDevice(PioneerFrontLightSensors(maxRange=1.0)) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.addDevice(Camera()) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.brain = lambda self: self.move(1, 1)
        >>> sim = Simulator(robot, worldf)

        >>> from IPython.display import SVG
        >>> def function(simulator, index):
        ...     cam_image = simulator.get_image()
        ...     return simulator.canvas.render("pil")
        >>> sim.movie([(0,0,0), (0,1,0)], function, movie_name="/tmp/movie.gif", mp4=False)
        <IPython.core.display.Image object>
        """
        from IPython.display import Image
        if stop is None:
            stop = len(poses)
        frames = []
        indices = []
        for index in range(start, stop, step):
            self.robots[0].setPose(*poses[index])
            self.update_gui()
            frames.append(function(self, index))
            indices.append(index)
        if stop - 1 not in indices:
            self.robots[0].setPose(*poses[index])
            self.update_gui()
            frames.append(function(self, stop - 1))
        if movie_name is None:
            movie_name = "movie.gif"
        if frames:
            frames[0].save(movie_name, save_all=True, append_images=frames[1:],
                           optimize=optimize, loop=loop, duration=duration)
            if mp4 is False:
                return Image(url=movie_name, embed=embed)
            else:
                return gif2mp4(movie_name)

    def playback(self, poses, function, play_rate=0.0):
        """
        Playback a list of poses.

        function(simulator) and returns displayable(s).

        >>> from jyro.simulator import (Pioneer, Simulator, Camera,
        ...                             PioneerFrontSonars, Gripper,
        ...                             PioneerFrontLightSensors)
        >>> def worldf(sim):
        ...     sim.addBox(0, 0, 10, 10, fill="white", wallcolor="grey") # meters
        ...     sim.addBox(1, 1, 2, 2, "purple")
        ...     sim.addBox(7, 7, 8, 8, "purple")
        ...     ## brightness of 1 is radius 1 meter
        ...     sim.addLight(7, 7, 4.25, color=Color(255, 255, 0, 64))

        >>> robot = Pioneer("Pioneer", 5.00, 5.00, math.pi / 2) # meters, radians
        >>> robot.addDevice(PioneerFrontSonars(maxRange=4.0)) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.addDevice(Gripper()) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.addDevice(PioneerFrontLightSensors(maxRange=1.0)) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.addDevice(Camera()) # doctest: +ELLIPSIS
        <jyro.simulator.robot.Pioneer object at ...>
        >>> robot.brain = lambda self: self.move(1, 1)
        >>> sim = Simulator(robot, worldf)

        >>> from IPython.display import SVG
        >>> def function(simulator, index):
        ...     cam_image = simulator.get_image()
        ...     return (simulator.canvas.render("pil"),
        ...             cam_image.resize((cam_image.size[0] * 4,
        ...                               cam_image.size[1] * 4)))
        >>> sv = sim.playback([(0,0,0), (0,1,0)], function)
        """
        def position_robot_at(index):
            self.robots[0].setPose(*poses[index])
            self.update_gui()
            return function(self, index)

        sv = SequenceViewer("Jyro Playback", position_robot_at, len(poses),
                            play_rate=play_rate)
        return sv

    def display(self):
        """
        Provide a representation of the simulation.
        """
        return self

class VSimulator(Simulator):
    """
    >>> from jyro.simulator import (Pioneer, Simulator, Camera,
    ...                             PioneerFrontSonars, Gripper,
    ...                             PioneerFrontLightSensors)
    >>> def worldf(sim):
    ...     sim.addBox(0, 0, 10, 10, fill="white", wallcolor="grey") # meters
    ...     sim.addBox(1, 1, 2, 2, "purple")
    ...     sim.addBox(7, 7, 8, 8, "purple")
    ...     ## brightness of 1 is radius 1 meter
    ...     sim.addLight(7, 7, 4.25, color=Color(255, 255, 0, 64))

    >>> robot = Pioneer("Pioneer", 5.00, 5.00, math.pi / 2) # meters, radians
    >>> robot.addDevice(PioneerFrontSonars(maxRange=4.0)) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.addDevice(Gripper()) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.addDevice(PioneerFrontLightSensors(maxRange=1.0)) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.addDevice(Camera()) # doctest: +ELLIPSIS
    <jyro.simulator.robot.Pioneer object at ...>
    >>> robot.brain = lambda self: self.move(1, 1)

    >>> sim = VSimulator(robot, worldf)   # doctest: +ELLIPSIS
    VBox(...)
    >>> for i in range(10):
    ...     sim.step()
    ...     sim.canvas.save("canvas%d.svg" % i)

    """
    def create_widgets(self, gamepad=False):
        step = ipywidgets.Button(icon="fa-step-forward")
        clear = ipywidgets.Button(description="Clear Output")
        play = ipywidgets.Play(max=1000000, show_repeat=False, layout={"width": "100%"})
        refresh_button = Button(icon="refresh", layout=Layout(width="25%"))
        time = ipywidgets.Text(description="Time:", value="0.0 seconds")
        html_canvas = ipywidgets.HTML(value=self.canvas._repr_svg_())
        output = ipywidgets.Output()
        camera_image = ipywidgets.HTML(value="""<img style="image-rendering: pixelated;" src="%s" width="240"/>""" % self.get_image_uri())
        update = ipywidgets.Checkbox(description="Update GUI", value=True)
        trace = ipywidgets.Checkbox(description="Trace Path", value=self.trace)
        y = ipywidgets.FloatSlider(readout=False, orientation="vertical")
        x = ipywidgets.FloatSlider(readout=False, orientation="horizontal")
        html_canvas_with_sliders =  ipywidgets.VBox(
            [ipywidgets.HBox([y, html_canvas]),
             x], layout={"height": "%spx" % (self.size[1] + 60)})
        pan = ipywidgets.FloatSlider(readout=False, orientation="horizontal")
        camera_image_with_sliders = ipywidgets.VBox([camera_image, pan])
        row1 = [html_canvas_with_sliders, camera_image_with_sliders]
        if gamepad:
            gamepad = ipywidgets.Controller()
            def init_gamepad(data):
                if len(data["owner"].buttons) != 0:
                    data["owner"].buttons[0].observe(lambda data: self.robot.device["gripper"].open())
                    data["owner"].buttons[1].observe(lambda data: self.robot.device["gripper"].close())
                    data["owner"].axes[1].observe(lambda data: self.robot.move(-data["new"], 0), 'value')
                    data["owner"].axes[0].observe(lambda data: self.robot.move(0, -data["new"]), 'value')
                    data["owner"].unobserve(init_gamepad)
            gamepad.observe(init_gamepad)
            row1.append(gamepad)
        horz = ipywidgets.HBox(row1)
        title = ipywidgets.VBox([ipywidgets.HBox([update, trace, time]), horz])
        controls = ipywidgets.HBox([step, play, clear, refresh_button])
        vbox = ipywidgets.VBox([title, controls, output])
        play.observe(self.step, 'value')
        refresh_button.on_click(lambda widget: self.reset())
        step.on_click(lambda data: self.step({"new": -1})) # signal to step once
        clear.on_click(lambda data: self.widgets["output"].clear_output())
        update.observe(self.update_gui, 'value')
        trace.observe(self.set_trace, 'value')
        x.observe(self.set_x, 'value')
        y.observe(self.set_y, 'value')
        pan.observe(self.set_a, 'value')
        y.layout = ipywidgets.Layout(height="%spx" % (self.size[1]), padding="0px 0px 0px 0px", width="15px")
        x.layout = ipywidgets.Layout(height="15px", padding="0px 0px 0px 10px", width="%spx" % (self.size[0] + 20))
        # Camera pan scroll:
        pan.layout = ipywidgets.Layout(height="15px", padding="0px 0px 0px 0px", width="248px")
        self.widgets.update({
            "step": step,
            "play": play,
            "refresh": refresh_button,
            "time": time,
            "html_canvas": html_canvas,
            "output": output,
            "camera_image": camera_image,
            "update": update,
            "pan": pan,
            "x": x,
            "y": y,
            "gamepad": gamepad,
        })
        return vbox

    def set_x(self, data):
        x, y, a = self.robot.getPose()
        new_x = data["new"]/100 * self.canvas.max_x # 0 to 100
        self.robot.setPose(new_x, y, a)
        self.update_gui()

    def set_trace(self, data):
        self.trace = data["new"]

    def set_y(self, data):
        x, y, a = self.robot.getPose()
        new_y = data["new"]/100 * self.canvas.max_y # 0 to 100
        self.robot.setPose(x, new_y, a)
        self.update_gui()

    def set_a(self, data):
        x, y, a = self.robot.getPose()
        new_a = (100 - data["new"])/100 * math.pi * 2
        self.robot.setPose(x, y, new_a)
        self.update_gui(set_angle=False)

    def render(self, canvas=None):
        if canvas is not None:
            self.physics.draw(canvas, trace=self.trace)
            return canvas
        else:
            self.update_gui()
            return self.canvas

    def update_gui(self, data=None, set_angle=True):
        self.physics.draw(self.canvas, trace=self.trace)
        self.widgets["html_canvas"].value = self.canvas._repr_svg_()
        if self.robot and self.robot.device["camera"]:
            self.widgets["camera_image"].value = """<img style="image-rendering: pixelated;" src="%s" width="240"/>""" % self.get_image_uri()
        x, y, a = self.robot.getPose()
        self.widgets["x"].value = (x/self.canvas.max_x) * 100
        self.widgets["y"].value = (y/self.canvas.max_y) * 100
        self.widgets["time"].value = "%.2f seconds" % self.physics.time
        if set_angle:
            self.widgets["pan"].value = 100 - ((a % (math.pi * 2))/(math.pi * 2)) * 100

    def step(self, data={"new": 1}, run_brain=True):
        if data["new"] == 0:
            self.reset()
            self.physics.time = 0.0
        elif not self.widgets["update"].value:
            step_size = 50 # each is 0.1 seconds
            for step in range(step_size):
                self.physics.step()
        else:
            with self.widgets["output"]:
                self.physics.step()
        ## Update GUI:
        if self.widgets["update"].value or data["new"] in [0, -1]: # -1: step; 0: reset
            self.update_gui()
        else:
            self.widgets["time"].value = "%.2f seconds" % self.physics.time

    def display(self):
        """
        Provide a representation of the simulation.
        """
        return self.widget

class _Player(threading.Thread):
    """
    Background thread for running a player.
    """
    def __init__(self, controller, time_wait=0.5):
        self.controller = controller
        threading.Thread.__init__(self)
        self.time_wait = time_wait
        self.can_run = threading.Event()
        self.can_run.clear()  ## paused
        self.daemon =True ## allows program to exit without waiting for join

    def run(self):
        while True:
            self.can_run.wait()
            self.controller.goto("next")
            time.sleep(self.time_wait)

    def pause(self):
        self.can_run.clear()

    def resume(self):
        self.can_run.set()

class SequenceViewer(VBox):
    def __init__(self, title, function, length, play_rate=0.5):
        self.player = _Player(self, play_rate)
        self.player.start()
        self.title = title
        self.function = function
        self.length = length
        self.output = Output()
        self.position_text = IntText(value=0, layout=Layout(width="100%"))
        self.total_text = Label(value="of %s" % self.length, layout=Layout(width="100px"))
        controls = self.make_controls()
        super().__init__([controls, self.output])

    def goto(self, position):
        #### Position it:
        if position == "begin":
            self.control_slider.value = 0
        elif position == "end":
            self.control_slider.value = self.length - 1
        elif position == "prev":
            if self.control_slider.value - 1 < 0:
                self.control_slider.value = self.length - 1 # wrap around
            else:
                self.control_slider.value = max(self.control_slider.value - 1, 0)
        elif position == "next":
            if self.control_slider.value + 1 > self.length - 1:
                self.control_slider.value = 0 # wrap around
            else:
                self.control_slider.value = min(self.control_slider.value + 1, self.length - 1)
        self.position_text.value = self.control_slider.value

    def toggle_play(self, button):
        ## toggle
        if self.button_play.description == "Play":
            self.button_play.description = "Stop"
            self.button_play.icon = "pause"
            self.player.resume()
        else:
            self.button_play.description = "Play"
            self.button_play.icon = "play"
            self.player.pause()

    def make_controls(self):
        button_begin = Button(icon="fast-backward", layout=Layout(width='100%'))
        button_prev = Button(icon="backward", layout=Layout(width='100%'))
        button_next = Button(icon="forward", layout=Layout(width='100%'))
        button_end = Button(icon="fast-forward", layout=Layout(width='100%'))
        self.button_play = Button(icon="play", description="Play", layout=Layout(width="100%"))
        self.control_buttons = HBox([
            button_begin,
            button_prev,
            self.position_text,
            button_next,
            button_end,
            self.button_play,
        ], layout=Layout(width='100%', height="50px"))
        self.control_slider = IntSlider(description=self.title,
                                        continuous_update=False,
                                        min=0,
                                        max=max(self.length - 1, 0),
                                        value=0,
                                        style={"description_width": 'initial'},
                                        layout=Layout(width='100%'))
        ## Hook them up:
        button_begin.on_click(lambda button: self.goto("begin"))
        button_end.on_click(lambda button: self.goto("end"))
        button_next.on_click(lambda button: self.goto("next"))
        button_prev.on_click(lambda button: self.goto("prev"))
        self.button_play.on_click(self.toggle_play)
        self.control_slider.observe(self.update_slider_control, names='value')
        controls = VBox([HBox([self.control_slider, self.total_text], layout=Layout(height="40px")),
                         self.control_buttons], layout=Layout(width='100%'))
        controls.on_displayed(lambda widget: self.initialize())
        return controls

    def initialize(self):
        results = self.function(self.control_slider.value)
        try:
            results = list(results)
        except:
            results = [results]
        self.displayers = [display(x, display_id=True) for x in results]

    def update_slider_control(self, change):
        if change["name"] == "value":
            self.position_text.value = self.control_slider.value
            self.output.clear_output(wait=True)
            results = self.function(self.control_slider.value)
            try:
                results = list(results)
            except:
                results = [results]
            for i in range(len(self.displayers)):
                self.displayers[i].update(results[i])

## Utilities

def gif2mp4(filename):
    """
    Convert an animated gif into a mp4, to show with controls.
    """
    from IPython.display import HTML
    if filename.endswith(".gif"):
        filename = filename[:-4]
    if os.path.exists(filename + ".mp4"):
        os.remove(filename + ".mp4")
    retval = os.system("""ffmpeg -i {0}.gif -movflags faststart -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" {0}.mp4""".format(filename))
    if retval == 0:
        return HTML("""<video src='{0}.mp4' controls></video>""".format(filename))
    else:
        print("error running ffmpeg; see console log message or use mp4=False")
