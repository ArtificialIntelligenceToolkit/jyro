from jyro.simulator.simulator import Segment
from jyro.simulator.color import colorMap, colorCode, colorNames

import math
import numpy as np
import PIL.Image
import PIL.ImageDraw

PIOVER180 = math.pi / 180.0
PIOVER2   = math.pi / 2.0

class Device():
    def __init__(self, type):
        self.type = type
        self.active = True

    def __len__(self):
        return 1

    def update(self, robot):
        pass

    def draw(self, robot, canvas):
        pass

    def serialize(self, item='all'):
        """
        item = 'all' or 'data'
        """
        d = {}
        if item == 'all':
            d["type"] = self.type
        d["active"] = self.active
        return d

    def additionalSegments(self, propose, x, y, cos_a90, sin_a90, **dict):
        """
        Add dynamic, extra bounding box segments to robot.
        """
        pass

    def step(self):
        pass

class Speech(Device):
    def __init__(self):
        Device.__init__(self, "speech")
        self.sayText = ""

    def draw(self, robot, canvas):
        if self.sayText != "":
            # center of robot:
            canvas.drawText(x, y, self.sayText) # % self.name)

    def serialize(self, item='all'):
        """
        item = 'all' or 'data'
        """
        d = Device.serialize(self, item)
        if item == 'all':
            d["sayText"] = self.sayText
        return d

class RangeSensor(Device):
    def __init__(self, name, geometry, arc, maxRange, noise=0.0):
        self.type = name
        self.active = 1
        # geometry = (x, y, a) origin in meters and radians
        self.geometry = geometry
        self.arc = arc
        self.maxRange = maxRange
        self.noise = noise
        self.groups = {}
        self.scan = [0] * len(geometry) # for data

    def getData(self):
        return self.scan[:] # copy

    def serialize(self, item='all'):
        """
        item = 'all' or 'data'
        """
        d = Device.serialize(self, item)
        if item == 'all':
            d["geometry"] = self.geometry
            d["arc"] = self.arc
            d["maxRange"] = self.maxRange
            d["noise"] = self.noise
            d["groups"] = self.groups
        d["scan"] = self.scan[:] # copy
        d["active"] = self.active
        return d

    def __len__(self):
        return len(self.geometry)

    def update(self, robot):
        # measure and draw the new device data:
        # do some computations and save for speed
        a90 = robot._ga + PIOVER2
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        i = 0
        for x, y, a in self.geometry:
            ga = (robot._ga + a)
            gx = robot._gx + (x * cos_a90 - y * sin_a90)
            gy = robot._gy + (x * sin_a90 + y * cos_a90)
            if robot.physics is None:
                self.scan[i] = 0
                continue
            dist, hit, obj = robot.physics.castRay(robot, gx, gy, -ga, self.maxRange, rayType="range")
            if hit:
                if robot.display["devices"] == 1:
                    robot.drawRay(self.type, gx, gy, hit[0], hit[1], "lightblue")
            else:
                hx, hy = math.sin(-ga) * self.maxRange, math.cos(-ga) * self.maxRange
                dist = self.maxRange
                if robot.display["devices"] == 1:
                    robot.drawRay(self.type, gx, gy, gx + hx, gy + hy, robot.colorParts[self.type])
            if self.type == "bumper":
                if dist < self.maxRange:
                    self.scan[i] = 1
                else:
                    self.scan[i] = 0
            else:
                self.scan[i] = dist
            i += 1

class LightSensor(Device):
    def __init__(self, geometry, maxRange, noise=0.0):
        """
        geometry - list of [x, y, a] for each sensor
        maxRange - range of light, in meters
        """
        self.type = "light"
        self.active = 1
        self.geometry = geometry
        self.arc = None
        self.maxRange = maxRange
        self.noise = noise
        self.groups = {}
        self.scan = [0] * len(geometry) # for data
        self.rgb = [[0,0,0] for g in geometry]
        self.ignore = ["self"] # can contain "self", "other", or "all"
        self.lightMode = "linear" # or "direct", "ambient"

    def getData(self):
        return self.scan[:] # copy

    def getPose(self, index):
        a90 = self.robot._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        (bx, by, ba) = self.geometry[index]
        gx = self.robot._gx + bx * cos_a90 - by * sin_a90
        gy = self.robot._gy + bx * sin_a90 + by * cos_a90
        return (gx, gy)

    def draw(self, robot, canvas):
        a90 = robot._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        for (bx, by, ba) in self.geometry:
            x = robot._gx + bx * cos_a90 - by * sin_a90
            y = robot._gy + bx * sin_a90 + by * cos_a90
            radius = .025
            canvas.drawCircle(x, y, radius, fill="yellow", outline="orange")

    def serialize(self, item='all'):
        """
        item = 'all' or 'data'
        """
        d = Device.serialize(self, item)
        if item == 'all':
            d["geometry"] = self.geometry
            d["arc"] = self.arc
            d["maxRange"] = self.maxRange
            d["noise"] = self.noise
            d["groups"] = self.groups
        d["scan"] = self.scan[:] # copy
        d["rgb"] = self.rgb[:] # copy
        d["active"] = self.active
        return d

    def update(self, robot):
        min_dist_meters = 0.0 if self.lightMode == "linear" else 0.2
        # for each light sensor:
        for i in range(len(self.geometry)):
            gx, gy = self.getPose(i)
            sum = 0.0
            rgb = [0, 0, 0]
            if robot.physics is None:
                self.scan[i] = 0
                continue
            for light in robot.physics.lights: # for each light source:
                x, y, brightness, light_rgb = light.x, light.y, light.brightness, light.rgb
                seg = Segment((x,y), (gx, gy))
                seg_length = seg.length()
                a = -seg.angle() + PIOVER2
                dist, hit, obj = robot.physics.castRay(robot, x, y, a, seg_length - .1,
                                                       ignoreRobot=self.ignore, rayType="light")
                # scaled over distance, but not zero:
                dist_to_light = min(max(seg_length, min_dist_meters), self.maxRange) / self.maxRange
                min_scaled_d = min_dist_meters/self.maxRange
                if self.lightMode == "ambient":
                    maxValueAmbient = 1.0 / min_scaled_d
                    intensity = (1.0 / dist_to_light) / maxValueAmbient
                elif self.lightMode == "direct":
                    maxValueIntensity = 1.0 / (min_scaled_d ** 2)
                    intensity = (1.0 / (dist_to_light ** 2)) / maxValueIntensity
                elif self.lightMode == "linear":
                    intensity = 1.0 - dist_to_light
                if hit:
                    intensity /= 2.0 # cut in half if in shadow
                sum += intensity * brightness
                if not hit: # no hit means it has a clear shot:
                    if robot.display["devices"] == 1:
                        robot.drawRay("light", x, y, gx, gy, "orange")
                else:
                    if robot.display["devices"] == 1:
                        robot.drawRay("lightBlocked", x, y, hit[0], hit[1], "purple")
            self.scan[i] = sum
            for c in [0, 1, 2]:
                self.rgb[i][c] = min(int(rgb[c]), 255)

class Gripper(Device):
    def __init__(self):
        self.type = "gripper"
        self.active = 1
        self.scan = []
        self.objs = []
        self.armLength  = 0.200 # length of the paddles
        self.velocity   = 0.0   # moving?
        self.openPosition  = 0.12
        self.closePosition = 0.0
        self.pose = (0.225, 0, 0) # position of gripper on robot
        self.state = "open"
        self.armPosition   = self.openPosition
        self.breakBeam = []
        self.storage = []

    def step(self):
        self.armPosition, self.velocity = self.moveWhere()

    def additionalSegments(self, propose, x, y, cos_a90, sin_a90, **dict):
        x1, x2, x3, x4 = self.pose[0], self.pose[0] + self.armLength, self.pose[0], self.pose[0] + self.armLength
        y1, y2, y3, y4 = self.armPosition, self.armPosition, -self.armPosition,  -self.armPosition
        if propose and self.velocity != 0.0:
            armPosition, velocity = self.moveWhere()
            y1, y2, y3, y4 = armPosition, armPosition, -armPosition,  -armPosition
        xys = map(lambda nx, ny: (x + nx * cos_a90 - ny * sin_a90,
                                  y + nx * sin_a90 + ny * cos_a90),
                  (x1, x2, x3, x4), (y1, y2, y3, y4))
        xys = list(xys)
        segs = [Segment(xys[0], xys[1], type="gripper"),
                Segment(xys[2], xys[3], type="gripper")]
        # add colors, robots, etc:
        for s in segs:
            for key in dict:
                s.__dict__[key] = dict[key]
        return segs

    def draw(self, robot, canvas):
        # draw grippers:
        # base:
        a90 = robot._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        xy = [((robot._gx + x * cos_a90 - y * sin_a90),
               (robot._gy + x * sin_a90 + y * cos_a90)) for (x,y) in
              ((self.pose[0], self.openPosition),
               (self.pose[0], -self.openPosition))]
        canvas.drawLine(xy[0][0], xy[0][1], xy[1][0], xy[1][1],
                        outline="black")
        # left arm:
        xs = []
        ys = []
        xs.append(self.pose[0])
        ys.append(self.armPosition + 0.01)
        xs.append(self.pose[0] + self.armLength)
        ys.append(self.armPosition + 0.01)
        xs.append(self.pose[0] + self.armLength)
        ys.append(self.armPosition - 0.01)
        xs.append(self.pose[0])
        ys.append(self.armPosition - 0.01)
        xy = map(lambda x, y: (robot._gx + x * cos_a90 - y * sin_a90,
                               robot._gy + x * sin_a90 + y * cos_a90),
                 xs, ys)
        xy = list(xy)
        canvas.drawPolygon(xy, fill="black", outline="black")
        # right arm:
        xs = []
        ys = []
        xs.append(self.pose[0])
        ys.append(-self.armPosition + 0.01)
        xs.append(self.pose[0] + self.armLength)
        ys.append(-self.armPosition + 0.01)
        xs.append(self.pose[0] + self.armLength)
        ys.append(-self.armPosition - 0.01)
        xs.append(self.pose[0])
        ys.append(-self.armPosition - 0.01)
        xy = map(lambda x, y: (robot._gx + x * cos_a90 - y * sin_a90,
                               robot._gy + x * sin_a90 + y * cos_a90),
                 xs, ys)
        xy = list(xy)
        canvas.drawPolygon(xy, fill="black", outline="black")

    def serialize(self, item='all'):
        """
        item = 'all' or 'data'
        """
        d = Device.serialize(self, item)
        if item == 'all':
            d["armLength"] = self.armLength
            d["openPosition"] = self.openPosition
            d["closePosition"] = self.closePosition
        d["scan"] = self.scan[:] # copy
        d["active"] = self.active
        d["velocity"] = self.velocity
        d["pose"] = self.pose
        d["breakBeam"] = self.breakBeam
        d["storage"] = self.storage
        d["state"] = self.state
        d["armPosition"] = self.armPosition
        return d

    def close(self):
        self.state = "close"
        self.velocity = -0.01

    def deploy(self):
        self.state = "deploy"
        self.velocity = 0.01

    def store(self):
        self.state = "store"
        self.velocity = -0.01
        for segment in self.objs:
            segment.robot.setPose(-1000.0, -1000.0, 0.0)
            if segment.robot not in self.storage:
                self.storage.append( segment.robot )

    def open(self):
        self.state = "open"
        self.velocity = 0.01

    def stop(self):
        self.state = "stop"
        self.velocity = 0.0

    def moveWhere(self):
        armPosition = self.armPosition
        velocity = self.velocity
        if velocity > 0: # opening +
            armPosition += velocity
            if armPosition >= self.openPosition:
                armPosition = self.openPosition
                velocity = 0.0
        elif velocity < 0: # closing -
            armPosition += velocity
            if armPosition <= self.closePosition:
                armPosition = self.closePosition
                velocity = 0.0
        return armPosition, velocity

    def isClosed(self):
        return self.velocity == 0 and self.armPosition == self.closePosition

    def isOpened(self):
        return self.velocity == 0 and self.armPosition == self.openPosition

    def isMoving(self):
        return self.velocity != 0

    def update(self, robot):
        a90 = robot._ga + PIOVER2
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        # cast a ray in two places, set scan = 1 if it is "broken"
        x = self.pose[0] + .07 # first beam distance from center of robot
        y = self.armPosition # distance between arms
        self.scan = [0] * (2 + 3) # two beams, 3 sensors (no lift)
        self.objs = []
        for i in range(2): # two beams
            gx = robot._gx + (x * cos_a90 - y * sin_a90)
            gy = robot._gy + (x * sin_a90 + y * cos_a90)
            ogx = robot._gx + (x * cos_a90 + y * sin_a90)
            ogy = robot._gy + (x * sin_a90 - y * cos_a90)
            if robot.physics is None:
                self.scan[i] = 0
                continue
            dist,hit,obj = robot.physics.castRay(robot, gx, gy, -robot._ga + PIOVER2, 2 * y,
                                                  rayType = "breakBeam")
            if hit:
                self.scan[i] = 1
                self.objs.append(obj) # for gripping
                if robot.display["gripper"] == 1: # breaker beams
                    robot.drawRay("gripper", gx, gy, ogx, ogy, "orange")
            elif robot.display["gripper"] == 1:
                robot.drawRay("gripper", gx, gy, ogx, ogy, "purple")
            x += .07  # distance between beams
        self.scan[2] = self.isClosed()
        self.scan[3] = self.isOpened()
        self.scan[4] = self.isMoving()

class Camera(Device):
    def __init__(self, width=60, height=40, field=120):
        """
        field is in degrees
        """
        self.type = "camera"
        self.active = 1
        self.depth = 3
        self.scan = []
        self.lights = []
        self.width = width
        self.height = height
        self.field = field
        self.startAngle = (self.field * PIOVER180)/2
        self.ground_color = colorMap["backgroundgreen"]
        self.sky_color = colorMap["lightblue"]
        self.colors_fade_with_distance = True

    def draw(self, robot, canvas):
        bx = [ .14, .06, .06, .14] # front camera
        by = [-.06, -.06, .06, .06]
        a90 = robot._ga + PIOVER2 # angle is 90 degrees off for graphics
        cos_a90 = math.cos(a90)
        sin_a90 = math.sin(a90)
        xy = map(lambda x, y: (robot._gx + x * cos_a90 - y * sin_a90,
                               robot._gy + x * sin_a90 + y * cos_a90),
                 bx, by)
        xy = list(xy)
        canvas.drawPolygon(xy, fill="black")

    def serialize(self, item='all'):
        """
        item = 'all' or 'data'
        """
        d = Device.serialize(self, item)
        if item == 'all':
            d["width"] = self.width
            d["height"] = self.height
            d["field"] = self.field
            d["startAngle"] = self.startAngle
            d["ground_color"] = self.ground_color
            d["sky_color"] = self.sky_color
        d["scan"] = self.scan[:] # copy
        d["active"] = self.active
        d["lights"] = self.lights[:] # copy
        return d

    def update(self, robot):
        x, y = robot._gx, robot._gy # camera location
        stepAngle = (self.field * PIOVER180) / float(self.width - 1)
        a = self.startAngle
        self.scan = []
        self.lights = []
        if robot.physics:
            for light in robot.physics.lights:
                seg = Segment((robot._gx, robot._gy), (light.x, light.y))
                raw_angle = (seg.angle() - PIOVER2) % (math.pi * 2)
                diff = ((raw_angle - robot._ga + math.pi * 5/2) % (math.pi * 2)) - PIOVER2
                self.lights.append((diff, seg.length()))
        for i in range(self.width):
            # FIX: move camera to self.pose; currently assumes robot center
            if robot.physics is None:
                self.scan.append((None, None, None, None))
                continue
            ga = (robot._ga + a)
            distance, hit, obj = robot.physics.castRay(robot, x, y, -ga,
                                                         ignoreRobot="self",
                                                         rayType="camera")
            if obj != None:
                if i in [0, self.width - 1]:
                    if robot.display["devices"] == 1:
                        robot.drawRay("camera", x, y, hit[0], hit[1], "purple")
                dist = (10 - min(distance, 10))/10.0 # 10 meter range
                if obj.type == "wall":
                    height = int(min(max((dist ** 2) * self.height/2.0, 1), self.height/2))
                else:
                    height = int(min(max((dist ** 2) * self.height/4.0, 1), self.height/4))
                self.scan.append((colorCode[obj.color], height, distance, dist))
            else:
                self.scan.append((None, None, None, None))
            a -= stepAngle

    def getData(self):
        """
        Return the data as a 3D matrix in (width, height, channel) order.
        """
        image = self.getImage()
        return np.array(image, "float32") / 255.0

    def getImage(self):
        ## get all shapes, rectangles, circles, etc
        shapes = []
        for w in range(self.width):
            (color, height, distance, dist) = self.scan[w]
            if color is not None:
                shapes.append(("line", distance, w, height, color, dist))
        for light in self.lights:
            diff, d = light # d in meters
            x = self.width/2 - diff/(self.startAngle) * self.width/2
            shapes.append(("light", d, diff, x))
        img = PIL.Image.new("RGB", (60, 40), "white")
        draw = PIL.ImageDraw.Draw(img)
        ## draw ground and sky:
        draw.rectangle([(0, 0), (self.width, self.height/2)], fill=tuple(self.sky_color))
        draw.rectangle([(0, self.height/2), (self.width, self.height)], fill=tuple(self.ground_color))
        ## sort on distance from camera
        for shape in sorted(shapes, key=lambda tup: tup[1], reverse=True):
            ## draw furtherest ones to closest ones
            if shape[0] == "line":
                stype, distance, w, height, ccode, dist = shape
                h = (self.height - height)/2
                y1 = h
                y2 = self.height - h
                if isinstance(ccode, str):
                    ccode = colorMap[ccode]
                elif isinstance(ccode, int):
                    ccode = colorMap[colorNames[ccode]]
                if self.colors_fade_with_distance:
                    color = tuple([int(v * dist) for v in tuple(ccode)])
                else:
                    color = tuple([int(v) for v in tuple(ccode)])
                draw.line([(w, y1), (w, y2)], fill=color, width=1)
            elif shape[0] == "light":
                stype, d, diff, x = shape
                MAXSIZE = 5
                SIZE = 10
                distance = (MAXSIZE - min(d, MAXSIZE))/MAXSIZE
                x1 = x - int(SIZE * distance)
                y1 = self.height/2 - int(SIZE * distance)
                x2 = x + int(SIZE * distance)
                y2 = self.height/2 + int(SIZE * distance)
                draw.ellipse([(x1, y1), (x2, y2)], fill=tuple(colorMap["yellow"]))
            else:
                raise Exception("Invalid shape")
        return img

    # def getImage(self):
    #     """
    #     Return a PIL.Image from the raw data.
    #     """
    #     data = self.loadData()
    #     img = PIL.Image.fromarray(data, mode="RGB") # saves as gif ok!
    #     if self.lights:
    #         draw = PIL.ImageDraw.Draw(img)
    #         for light in self.lights:
    #             diff, d = light # d in meters
    #             distance = (10 - min(d, 10))/10.0
    #             if abs(diff) < self.startAngle * 1.5: # make a little bigger to show edge of circle
    #                 x = self.width/2 - diff/(self.startAngle) * self.width/2
    #                 draw.ellipse((x - int(20 * distance),
    #                               20 - int(20 * distance),
    #                               x + int(20 * distance),
    #                               20 + int(20 * distance)), fill=tuple(colorMap["yellow"]))
    #     return img

    # def loadData(self):
    #     """
    #     Turns self.scan information into a vector of uint8.
    #     self.data is in np.array format for easy PIL image, (height, width, channel) order.
    #     """
    #     self.data = [128 for i in range(self.height * self.width * 3)]
    #     for w in range(self.width):
    #         (color, height, distance) = self.scan[w]
    #         if color is None or height is None or distance is None:
    #             continue
    #         for h in range(self.height):
    #             if h < (self.height - height)/2: # sky
    #                 ccode = self.sky_color
    #                 scale = 1.0
    #             elif h > self.height - (self.height - height)/2: # ground
    #                 ccode = self.ground_color
    #                 scale = 1.0
    #             else:
    #                 ccode = color
    #                 scale = distance
    #             if isinstance(ccode, str):
    #                 ccode = colorMap[ccode]
    #             elif isinstance(ccode, int):
    #                 ccode = colorMap[colorNames[ccode]]
    #             # else it should be a Color
    #             for d in range(self.depth):
    #                 self.data[(w + h * self.width) * self.depth + d] = ccode[d] * scale
    #     # also return it, for external use
    #     self.data = np.array(self.data, "uint8").reshape((self.height, self.width, self.depth))
    #     return self.data

class PioneerFrontSonars(RangeSensor):
    def __init__(self, maxRange=8.0, noise=0.0):
        RangeSensor.__init__(self,
            "sonar", geometry = (( 0.10, 0.175, 90 * PIOVER180),
                                 ( 0.17, 0.15, 65 * PIOVER180),
                                 ( 0.20, 0.11, 40 * PIOVER180),
                                 ( 0.225, 0.05, 15 * PIOVER180),
                                 ( 0.225,-0.05,-15 * PIOVER180),
                                 ( 0.20,-0.11,-40 * PIOVER180),
                                 ( 0.17,-0.15,-65 * PIOVER180),
                                 ( 0.10,-0.175,-90 * PIOVER180)),
                             arc = 5 * PIOVER180, maxRange=maxRange, noise=noise)
        self.groups = {'all': range(8),
                       'front': (3, 4),
                       'front-left' : (1,2,3),
                       'front-right' : (4, 5, 6),
                       'front-all' : (1,2, 3, 4, 5, 6),
                       'left' : (0,),
                       'right' : (7,),
                       'left-front' : (1,2),
                       'right-front' : (5,6, ),
                       'left-back' : [],
                       'right-back' : [],
                       'back-right' : [],
                       'back-left' : [],
                       'back' : [],
                       'back-all' : []}

class PioneerBackSonars(RangeSensor):
    def __init__(self, maxRange=8.0, noise=0.0):
        RangeSensor.__init__(self,
            "sonar", geometry = (( -0.10,-0.175,-90 * PIOVER180),
                                 ( -0.17,-0.15, (180 + 65) * PIOVER180),
                                 ( -0.20,-0.11, (180 + 40) * PIOVER180),
                                 ( -0.225,-0.05,(180 + 15) * PIOVER180),
                                 ( -0.225, 0.05,(180 - 15) * PIOVER180),
                                 ( -0.20, 0.11, (180 - 40) * PIOVER180),
                                 ( -0.17, 0.15, (180 - 65) * PIOVER180),
                                 ( -0.10, 0.175,(180 - 90) * PIOVER180)),
                             arc = 5 * PIOVER180, maxRange=maxRange, noise=noise)
        self.groups = {'all': range(8),
                       'front': [],
                       'front-left' : [],
                       'front-right' : [],
                       'front-all' : [],
                       'left' : (7, ),
                       'right' : (0, ),
                       'left-front' : [],
                       'right-front' : [],
                       'left-back' : (7, ),
                       'right-back' : (0, ),
                       'back-right' : (1, 2, 3),
                       'back-left' : (4, 5, 6),
                       'back' : (3, 4),
                       'back-all' : ( 1, 2, 3, 4, 5, 6)}

class Pioneer16Sonars(RangeSensor):
    def __init__(self, maxRange=8.0, noise=0.0):
        RangeSensor.__init__(self,
            "sonar", geometry = (( 0.10, 0.175, 90 * PIOVER180),
                                 ( 0.17, 0.15, 65 * PIOVER180),
                                 ( 0.20, 0.11, 40 * PIOVER180),
                                 ( 0.225, 0.05, 15 * PIOVER180),
                                 ( 0.225,-0.05,-15 * PIOVER180),
                                 ( 0.20,-0.11,-40 * PIOVER180),
                                 ( 0.17,-0.15,-65 * PIOVER180),
                                 ( 0.10,-0.175,-90 * PIOVER180),
                                 ( -0.10,-0.175,-90 * PIOVER180),
                                 ( -0.17,-0.15, (180 + 65) * PIOVER180),
                                 ( -0.20,-0.11, (180 + 40) * PIOVER180),
                                 ( -0.225,-0.05,(180 + 15) * PIOVER180),
                                 ( -0.225, 0.05,(180 - 15) * PIOVER180),
                                 ( -0.20, 0.11, (180 - 40) * PIOVER180),
                                 ( -0.17, 0.15, (180 - 65) * PIOVER180),
                                 ( -0.10, 0.175,(180 - 90) * PIOVER180)),
                             arc = 5 * PIOVER180, maxRange = maxRange, noise = noise)
        self.groups = {'all': range(16),
                       'front': (3, 4),
                       'front-left' : (1,2,3),
                       'front-right' : (4, 5, 6),
                       'front-all' : (1,2, 3, 4, 5, 6),
                       'left' : (0, 15),
                       'right' : (7, 8),
                       'left-front' : (0,),
                       'right-front' : (7, ),
                       'left-back' : (15, ),
                       'right-back' : (8, ),
                       'back-right' : (9, 10, 11),
                       'back-left' : (12, 13, 14),
                       'back' : (11, 12),
                       'back-all' : ( 9, 10, 11, 12, 13, 14)}

class Pioneer4Sonars(RangeSensor):
    def __init__(self, maxRange=8.0, noise=0.0):
        RangeSensor.__init__(self, "sonar",
             geometry = (( 0.225, 0.05, 15 * PIOVER180),
                         ( 0.225,-0.05,-15 * PIOVER180),
                         ( -0.225,-0.05,(180 + 15) * PIOVER180),
                         ( -0.225, 0.05,(180 - 15) * PIOVER180),
                         ), arc = 5 * PIOVER180, maxRange = maxRange, noise = noise)
        self.groups = {'all': range(4),
                       'front': (0, 1),
                       'front-left' : (0,),
                       'front-right' : (1,),
                       'front-all' : (0,1),
                       'left' : [],
                       'right' : [],
                       'left-front' : [],
                       'right-front' : [],
                       'left-back' : [],
                       'right-back' : [],
                       'back-right' : (2,),
                       'back-left' : (3,),
                       'back' : (2, 3),
                       'back-all' : ( 2, 3)}

class PioneerFrontLightSensors(LightSensor):
    def __init__(self, maxRange):
        # make sure outside of bb!
        LightSensor.__init__(self, ((.225,  .175, 0), (.225, -.175, 0)),
                             maxRange,
                             noise=0.0)
        self.groups = {"front-all": (0, 1),
                       "all": (0, 1),
                       "front": (0, 1),
                       "front-left": (0, ),
                       "front-right": (1, ),
                       'left' : (0,),
                       'right' : (1,),
                       'left-front' : (0,),
                       'right-front' : (1, ),
                       'left-back' : [],
                       'right-back' : [],
                       'back-right' : [],
                       'back-left' : [],
                       'back' : [],
                       'back-all' : []}

class Pioneer4FrontLightSensors(LightSensor):
    def __init__(self, maxRange):
        # make sure outside of bb!
        LightSensor.__init__(
            self, (
                (.225,  .175, 0),
                (.225,  .0875, 0),
                (.225, -.0875, 0),
                (.225, -.175, 0),
            ),
            maxRange,
            noise=0.0)
        self.groups = {"front-all": (0, 1, 2, 3),
                       "all": (0, 1, 2, 3),
                       "front": (1, 2),
                       "front-left": (0, ),
                       "front-right": (3, ),
                       'left' : (0, 1),
                       'right' : (2, 3),
                       'left-front' : (0,),
                       'right-front' : (3, ),
                       'left-back' : [],
                       'right-back' : [],
                       'back-right' : [],
                       'back-left' : [],
                       'back' : [],
                       'back-all' : []}

class MyroIR(RangeSensor):
    def __init__(self):
        RangeSensor.__init__(self,
                             "ir", geometry = (( 0.175, 0.13, 45 * PIOVER180),
                                               ( 0.175,-0.13,-45 * PIOVER180)),
                             arc = 5 * PIOVER180, maxRange = 0.5, noise = 0.0)
        self.groups = {'all': range(2),
                       'front': (0, 1),
                       'front-left' : (0, ),
                       'front-right' : (1, ),
                       'front-all' : (0, 1,),
                       'left' : (0,),
                       'right' : (1,),
                       'left-front' : (0, ),
                       'right-front' : (1, ),
                       'left-back' : [],
                       'right-back' : [],
                       'back-right' : [],
                       'back-left' : [],
                       'back' : [],
                       'back-all' : []}

class MyroBumper(RangeSensor):
    def __init__(self):
        RangeSensor.__init__(self,
                             "bumper", geometry = (( 0.20, 0.0, 80 * PIOVER180),
                                                   ( 0.20, 0.0,-80 * PIOVER180)),
                             arc = 5 * PIOVER180, maxRange = 0.20, noise = 0.0)
        self.groups = {'all': range(2),
                       'front': (0, 1),
                       'front-left' : (0, ),
                       'front-right' : (1, ),
                       'front-all' : (0, 1,),
                       'left' : (0,),
                       'right' : (1,),
                       'left-front' : (0, ),
                       'right-front' : (1, ),
                       'left-back' : [],
                       'right-back' : [],
                       'back-right' : [],
                       'back-left' : [],
                       'back' : [],
                       'back-all' : []}

class MyroLightSensors(LightSensor):
    def __init__(self):
        LightSensor.__init__(self, ((.18, .13, 0), (.18, -.13, 0)),
                             maxRange,
                             noise=0.0)
        self.groups = {"front-all": (0, 1),
                       "all": (0, 1),
                       "front": (0, 1),
                       "front-left": (0, ),
                       "front-right": (1, ),
                       'left' : (0,),
                       'right' : (1,),
                       'left-front' : (0,),
                       'right-front' : (1, ),
                       'left-back' : [],
                       'right-back' : [],
                       'back-right' : [],
                       'back-left' : [],
                       'back' : [],
                       'back-all' : []}
