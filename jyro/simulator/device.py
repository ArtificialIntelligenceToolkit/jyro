import math

PIOVER180 = math.pi / 180.0
PIOVER2   = math.pi / 2.0

class Device():
    def __init__(self, type):
        self.type = type
        self.active = True

class RangeSensor():
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

class Light():
    def __init__(self, x, y, brightness, color="yellow"):
        self.active = 1
        self.x = x
        self.y = y
        self.brightness = brightness
        self.color = color
        self.origColor = color
        self._xyb = x, y, brightness # original settings for reset
        self.rgb = colorMap[color]
        self.type = "fixed"

class BulbDevice(Light):
    """
    Bulb will have color of robot.
    """
    def __init__(self, x, y):
        Light.__init__(self, x, y, 1.0)
        self.type = "bulb"
        self.active = 1
        self.geometry = (0, 0, 0)

class LightSensor():
    def __init__(self, geometry, noise=0.0):
        self.type = "light"
        self.active = 1
        self.geometry = geometry
        self.arc = None
        self.maxRange = 1000.0
        self.noise = noise
        self.groups = {}
        self.scan = [0] * len(geometry) # for data
        self.rgb = [[0,0,0] for g in geometry]

class DirectionalLightSensor():
    def __init__(self, geometry, arc, noise=0.0):
        # geometry (x, y, a) where a is direction (in radians) relative to robot
        self.type = "directional"
        self.active = 1
        self.geometry = geometry
        self.arc = arc
        self.maxRange = 1000.0 # in mm
        self.noise = noise
        self.groups = {}
        self.scan = [0] * len(geometry) # for data

class Gripper():
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

    def close(self):
        self.state = "close"
        self.velocity = -0.01
        return "ok"

    def deploy(self):
        self.state = "deploy"
        self.velocity = 0.01
        return "ok"

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
        return "ok"

    def stop(self):
        self.state = "stop"
        self.velocity = 0.0
        return "ok"

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

class PTZ():
    def __init__(self, camera):
        self.type = "ptz"
        self.camera = camera
        self.active = 1

    def setPose(self, p=None, t=None, z=None):
        if p != None:
            self.camera.pan = p * PIOVER180
        if z != None:
            self.camera.zoom = z * PIOVER180
        self.camera.startAngle = self.camera.pan + self.camera.zoom/2
        self.camera.stopAngle = self.camera.pan - self.camera.zoom/2
        return "ok"

    def getPose(self):
        return self.camera.pan / PIOVER180, 0, self.camera.zoom / PIOVER180

class Camera():
    def __init__(self, width, height, pan, zoom, x, y, thr):
        self.type = "camera"
        self.active = 1
        self.scan = []
        self.width = width
        self.height = height
        self.pan = pan * PIOVER180
        self.tilt = 0
        self.zoom = zoom * PIOVER180
        self.startAngle = self.pan + self.zoom/2
        self.stopAngle = self.pan - self.zoom/2
        self.pose = (x, y, thr)
        self.color = [[0,0,0] for i in range(self.width)]
        self.range = [0 for i in range(self.width)]

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
    def __init__(self):
        # make sure outside of bb!
        LightSensor.__init__(self, ((.225,  .175, 0), (.225, -.175, 0)),
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
    def __init__(self):
        # make sure outside of bb!
        LightSensor.__init__(
            self, (
                (.225,  .175, 0),
                (.225,  .0875, 0),
                (.225, -.0875, 0),
                (.225, -.175, 0),
            ),
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

class PioneerFrontDirectionalLightSensors(DirectionalLightSensor):
    def __init__(self):
        # make sure outside of bb!
        DirectionalLightSensor.__init__(self, ((.225,  .175, -30 * PIOVER180),
                                               (.225, -.175, 30 * PIOVER180)),
                                        120 * PIOVER180,
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
