"""
Jyro Robot Base Classes.
(c) 2006-2018, Institute for Personal Robots in Education
http://www.roboteducation.org/
Distributed under a Shared Source License
"""

from builtins import input
import sys
import atexit
import time
import random
import pickle
import threading
import os
import types
import copy
import signal
import traceback
import urllib
import glob

from .songs import *
from .version import __VERSION__
from .system import *
from .scribbler import Scribbler

_timers = {}
_robot = None
_setup = None

def timer(seconds=0):
    """ A function to be used with 'for' """
    start = time.time()
    while True:
        timepast = time.time() - start
        if seconds != 0 and timepast > seconds:
            raise StopIteration()
        yield round(timepast, 3)

def timeRemaining(seconds=0):
    """ Function to be used with 'while' """
    global _timers
    if seconds == 0: return True
    now = time.time()
    stack = traceback.extract_stack()
    filename, line_no, q1, q2 = stack[-2]
    if filename.startswith("<pyshell"):
        filename = "pyshell"
    if (filename, line_no) not in _timers:
        _timers[(filename, line_no)] = (now, seconds)
        return True
    start, duration = _timers[(filename, line_no)]
    if seconds != duration:
        _timers[(filename, line_no)] = (now, seconds)
        return True
    if now - start > duration:
        del _timers[(filename, line_no)]
        return False
    else:
        return True

def wait(seconds):
    """
    Wrapper for time.sleep() so that we may later overload.
    """
    return time.sleep(seconds)

def currentTime():
    """
    Returns current time in seconds since
    """
    return time.time()

def pickOne(*args):
    """
    Randomly pick one of a list, or one between [0, arg).
    """
    if len(args) == 1:
        return random.randrange(args[0])
    else:
        return args[random.randrange(len(args))]

def pickOneInRange(start, stop):
    """
    Randomly pick one of a list, or one between [0, arg).
    """
    return random.randrange(start, stop)

def heads(): return flipCoin() == "heads"
def tails(): return flipCoin() == "tails"

def flipCoin():
    """
    Randomly returns "heads" or "tails".
    """
    return ("heads", "tails")[random.randrange(2)]

def randomNumber():
    """
    Returns a number between 0 (inclusive) and 1 (exclusive).
    """
    return random.random()

def ask(prompt):
    return input(prompt)

class BackgroundThread(threading.Thread):
    """
    A thread class for running things in the background.
    """
    def __init__(self, function, pause = 0.01):
        """
        Constructor, setting initial variables
        """
        self.function = function
        self._stopevent = threading.Event()
        self._sleepperiod = pause
        threading.Thread.__init__(self, name="JyroThread")

    def run(self):
        """
        overload of threading.thread.run()
        main control loop
        """
        while not self._stopevent.isSet():
            self.function()
            #self._stopevent.wait(self._sleepperiod)

    def join(self,timeout=None):
        """
        Stop the thread
        """
        self._stopevent.set()
        threading.Thread.join(self, timeout)

# functions:
def _cleanup():
    if _robot is not None:
        if "robot" in _robot.robotinfo:
            try:
                _robot.stop() # hangs?
                time.sleep(0.5)
            except: # catch serial.SerialException
                # port already closed
                pass
        try:
            _robot.close()
        except:
            pass

def ctrlc_handler(signum, frame):
    if _robot is not None:
        #_robot.open()
        #print "done opening"
        _robot.manual_flush()
        if "robot" in _robot.robotinfo:
            _robot.hardStop()
    #raise KeyboardInterrupt
    orig_ctrl_handler()

orig_ctrl_handler = signal.getsignal(signal.SIGINT)
# Set the signal handler and a 5-second alarm
signal.signal(signal.SIGINT, ctrlc_handler)

# Get ready for user prompt; set up environment:
if _setup is None:
    _setup = 1
    atexit.register(_cleanup)
    # Ok, now we're ready!
    print("(c) 2006-2018 Institute for Personal Robots in Education",  file=sys.stderr)
    print("[See http://www.roboteducation.org/ for more information]",  file=sys.stderr)
    print("Jyro version %s is ready!" % (__VERSION__, ),  file=sys.stderr)

## Functional interface:

def requestStop():
    if _robot is not None:
        _robot.requestStop = 1

def initialize(id = None):
    _robot = Scribbler(id)

init = initialize

def translate(amount):
    if _robot is not None:
        return _robot.translate(amount)
    else:
        raise AttributeError("need to initialize robot")
def rotate(amount):
    if _robot is not None:
        return _robot.rotate(amount)
    else:
        raise AttributeError("need to initialize robot")
def move(translate, rotate):
    if _robot is not None:
        return _robot.move(translate, rotate)
    else:
        raise AttributeError("need to initialize robot")
def forward(speed=1, seconds=None):
    if _robot is not None:
        return _robot.forward(speed, seconds)
    else:
        raise AttributeError("need to initialize robot")

def backward(speed=1, seconds=None):
    if _robot is not None:
        return _robot.backward(speed, seconds)
    else:
        raise AttributeError("need to initialize robot")
def turn(direction, amount = .8, seconds=None):
    if _robot is not None:
        return _robot.turn(direction, amount, seconds)
    else:
        raise AttributeError("need to initialize robot")
def turnLeft(speed=1, seconds=None):
    if _robot is not None:
        return _robot.turnLeft(speed, seconds)
    else:
        raise AttributeError("need to initialize robot")
def turnRight(speed=1, seconds=None):
    if _robot is not None:
        return _robot.turnRight(speed, seconds)
    else:
        raise AttributeError("need to initialize robot")
def stop():
    if _robot is not None:
        return _robot.stop()
def getPosition():
    """This returns the x and y coordinates of the scribbler 2"""
    if _robot is not None:
        return _robot.getPosition()
    else:
        raise AttributeError("need to initialize robot")
def hereIs(x=0, y=0):
    if _robot is not None:
        return _robot.setHereIs(x, y)
    else:
        raise AttributeError("need to initialize robot")
def getAngle():
    """This returns the current angle of the scribbler 2"""
    if _robot is not None:
        return _robot.getAngle()
    else:
        raise AttributeError("need to initialize robot")
def setAngle(angle):
    if _robot is not None:
        return _robot.setAngle(angle)
    else:
        raise AttributeError("need to initialize robot")
def beginPath():
    """Speed can be a value from 1 to 15"""
    if _robot is not None:
        return _robot.setBeginPath()
    else:
        raise AttributeError("need to initialize robot")
def moveTo(x, y):
    if _robot is not None:
        return _robot.setMove(x, y, "to")
    else:
        raise AttributeError("need to initialize robot")
def moveBy(x, y):
    if _robot is not None:
        return _robot.setMove(x, y, "by")
    else:
        raise AttributeError("need to initialize robot")
def turnTo(angle, radsOrDegrees):
    if _robot is not None:
        return _robot.setTurn(angle, "to", radsOrDegrees)
    else:
        raise AttributeError("need to initialize robot")
def turnBy(angle, radsOrDegrees):
    if _robot is not None:
        return _robot.setTurn(angle, "by", radsOrDegrees)
    else:
        raise AttributeError("need to initialize robot")
def arcTo(x, y, radius):
    if _robot is not None:
        return _robot.setArc(x, y, radius, "to")
    else:
        raise AttributeError("need to initialize robot")
def arcBy(x, y, radius):
    if _robot is not None:
        return _robot.setArc(x, y, radius, "by")
    else:
        raise AttributeError("need to initialize robot")
def endPath():
    if _robot is not None:
        return _robot.setEndPath()
    else:
        raise AttributeError("need to initialize robot")
def getMicEnvelope():
    """Returns a number representing the microphone envelope noise"""
    if _robot is not None:
        return _robot.getMicEnvelope()
    else:
        raise AttributeError("need to initialize robot")
def getMotorStats():
    '''Return the current motion status as a packed long and single additional byte showing if motors are ready for commands (1=ready, 0=busy):
 Left wheel and right wheel are signed, twos complement eight bit velocity values,
 Idler timer is the time in 1/10 second since the last idler edge,
 Idler spd is an unsigned six-bit velocity value, and
 Mov is non-zero iff one or more motors are turning.
 Left and right wheel velocities are instanteous encoder counts over a 1/10-second interval.
 Idler wheel wheel velocity is updated every 1/10 second and represents the idler encoder count during the last 1.6 seconds.'''
    if _robot is not None:
        return _robot.getMotorStats()
    else:
        raise AttributeError("need to initialize robot")
def getEncoders(zeroEncoders=False):
    '''Gets the values for the left and right encoder wheels.  Negative value means they have moved
    backwards from the robots perspective.  Each turn of the encoder wheel is counted as and increment or
    decrement of 2 depending on which direction the wheels moved.
    if zeroEncoders is set to True then the encoders will be set to zero after reading the values'''
    if _robot is not None:
        return _robot.getEncoders(zeroEncoders)
    else:
        raise AttributeError("need to initialize robot")
def openConnection():
    if _robot is not None:
        return _robot.open()
    else:
        raise AttributeError("need to initialize robot")
def closeConnection():
    if _robot is not None:
        return _robot.close()
    else:
        raise AttributeError("need to initialize robot")
def get(sensor = "all", *pos):
    if _robot is not None:
        return _robot.get(sensor, *pos)
    else:
        raise AttributeError("need to initialize robot")
def getVersion():
    if _robot is not None:
        return _robot.get("version")
    else:
        raise AttributeError("need to initialize robot")
def getLight(*pos):
    if _robot is not None:
        return _robot.get("light", *pos)
    else:
        raise AttributeError("need to initialize robot")
def getIR(*pos):
    if _robot is not None:
        return _robot.get("ir", *pos)
    else:
        raise AttributeError("need to initialize robot")

def getDistance(*pos):
    if _robot is not None:
        return _robot.getDistance(*pos)
    else:
        raise AttributeError("need to initialize robot")
def getLine(*pos):
    if _robot is not None:
        return _robot.get("line", *pos)
    else:
        raise AttributeError("need to initialize robot")
def getStall():
    if _robot is not None:
        return _robot.get("stall")
    else:
        raise AttributeError("need to initialize robot")
def getInfo(*item):
    if _robot is not None:
        retval = _robot.getInfo(*item)
        retval["jyro"] =  __VERSION__
        return retval
    else:
        return {"jyro": __VERSION__}
def getAll():
    if _robot is not None:
        return _robot.get("all")
    else:
        raise AttributeError("need to initialize robot")
def getName():
    if _robot is not None:
        return _robot.get("name")
    else:
        raise AttributeError("need to initialize robot")
def getPassword():
    if _robot is not None:
        return _robot.get("password")
    else:
        raise AttributeError("need to initialize robot")
def getForwardness():
    if _robot is not None:
        return _robot.get("forwardness")
    else:
        raise AttributeError("need to initialize robot")
def getStartSong():
    if _robot is not None:
        return _robot.get("startsong")
    else:
        raise AttributeError("need to initialize robot")
def getVolume():
    if _robot is not None:
        return _robot.get("volume")
    else:
        raise AttributeError("need to initialize robot")
def update():
    if _robot is not None:
        return _robot.update()
    else:
        raise AttributeError("need to initialize robot")
def beep(duration=.5, frequency1=None, frequency2=None):
    if type(duration) in [tuple, list]:
        frequency2 = frequency1
        frequency1 = duration
        duration =.5
    if frequency1 == None:
        frequency1 = random.randrange(200, 10000)
    if type(frequency1) in [tuple, list]:
        if frequency2 == None:
            frequency2 = [None for i in range(len(frequency1))]
        for (f1, f2) in zip(frequency1, frequency2):
            if _robot is not None:
                _robot.beep(duration, f1, f2)
            else:
                computer.beep(duration, f1, f2)
    else:
        if _robot is not None:
            _robot.beep(duration, frequency1, frequency2)
        else:
            computer.beep(duration, frequency1, frequency2)

def scaleDown(loopCount):
    beep(0.5, 9000 - 200 * loopCount)

def scaleUp(loopCount):
    beep(0.5, 200 + 200 * loopCount)

def set(item, position, value = None):
    if _robot is not None:
        return _robot.set(item, position, value)
    else:
        raise AttributeError("need to initialize robot")
def setLED(position, value):
    if _robot is not None:
        return _robot.set("led", position, value)
    else:
        raise AttributeError("need to initialize robot")
def setName(name):
    if _robot is not None:
        return _robot.set("name", name)
    else:
        raise AttributeError("need to initialize robot")
def setPassword(password):
    if _robot is not None:
        return _robot.set("password", password)
    else:
        raise AttributeError("need to initialize robot")
def setForwardness(value):
    if _robot is not None:
        return _robot.set("forwardness", value)
    else:
        raise AttributeError("need to initialize robot")
def setVolume(value):
    if _robot is not None:
        return _robot.set("volume", value)
    else:
        raise AttributeError("need to initialize robot")
def setS2Volume(value):
    """Level can be between 0-100 and represents the percent volume level of the speaker"""
    if _robot is not None:
        return _robot.setS2Volume(value)
    else:
        raise AttributeError("need to initialize robot")
def setStartSong(songName):
    if _robot is not None:
        return _robot.set("startsong", songName)
    else:
        raise AttributeError("need to initialize robot")
def motors(left, right):
    if _robot is not None:
        return _robot.motors(left, right)
    else:
        raise AttributeError("need to initialize robot")
def restart():
    if _robot is not None:
        return _robot.restart()
    else:
        raise AttributeError("need to initialize robot")
def joyStick(showSensors = 0):
    if _robot is not None:
        return Joystick(_robot, showSensors)
    else:
        raise AttributeError("need to initialize robot")
def calibrate():
    if _robot is not None:
        return Calibrate(_robot)
    else:
        raise AttributeError("need to initialize robot")
def playSong(song, wholeNoteDuration = .545):
    if _robot is not None:
        return _robot.playSong(song, wholeNoteDuration)
    else:
        raise AttributeError("need to initialize robot")
def playNote(tup, wholeNoteDuration = .545):
    if _robot is not None:
        return _robot.playNote(tup, wholeNoteDuration)
    else:
        raise AttributeError("need to initialize robot")

########################### New dongle commands

def getBright(position=None):
    if _robot is not None:
        return _robot.getBright(position)
    else:
        raise AttributeError("need to initialize robot")

def getBlob():
    if _robot is not None:
        return _robot.getBlob()
    else:
        raise AttributeError("need to initialize robot")

def getObstacle(position=None):
    if _robot is not None:
        return _robot.getObstacle(position)
    else:
        raise AttributeError("need to initialize robot")

def setIRPower(value):
    if _robot is not None:
        return _robot.setIRPower(value)
    else:
        raise AttributeError("need to initialize robot")

def getBattery():
    if _robot is not None:
        return _robot.getBattery()
    else:
        raise AttributeError("need to initialize robot")

def identifyRobot():
    if _robot is not None:
        return _robot.identifyRobot()
    else:
        raise AttributeError("need to initialize robot")

def getIRMessage():
    if _robot is not None:
        return _robot.getIRMessage()
    else:
        raise AttributeError("need to initialize robot")

def sendIRMessage(msg):
    if _robot is not None:
        return _robot.sendIRMessage(msg)
    else:
        raise AttributeError("need to initialize robot")

def setCommunicateLeft(on=True):
    if _robot is not None:
        return _robot.setCommunicateLeft(on)
    else:
        raise AttributeError("need to initialize robot")

def setCommunicateRight(on=True):
    if _robot is not None:
        return _robot.setCommunicateLeft(on)
    else:
        raise AttributeError("need to initialize robot")

def setCommunicateCenter(on=True):
    if _robot is not None:
        return _robot.setCommunicateCenter(on)
    else:
        raise AttributeError("need to initialize robot")

def setCommunicateAll(on=True):
    if _robot is not None:
        return _robot.setCommunicateAll(on)
    else:
        raise AttributeError("need to initialize robot")

def configureBlob(y_low=0, y_high=255,
                  u_low=0, u_high=255,
                  v_low=0, v_high=255,
                  smooth_thresh=4):
    if _robot is not None:
        return _robot.configureBlob(y_low, y_high, u_low, u_high, v_low, v_high, smooth_thresh)
    else:
        raise AttributeError("need to initialize robot")

def setWhiteBalance(value):
    if _robot is not None:
        return _robot.setWhiteBalance(value)
    else:
        raise AttributeError("need to initialize robot")

def darkenCamera(value=0):
    if _robot is not None:
        return _robot.darkenCamera(value)
    else:
        raise AttributeError("need to initialize robot")

def manualCamera(gain=0x00, brightness=0x80, exposure=0x41):
    if _robot is not None:
        return _robot.manualCamera(gain, brightness, exposure)
    else:
        raise AttributeError("need to initialize robot")

def autoCamera(value=0):
    if _robot is not None:
        return _robot.autoCamera()
    else:
        raise AttributeError("need to initialize robot")

def setLEDFront(value):
    """ Set the Light Emitting Diode on the robot's front. """
    if _robot is not None:
        return _robot.setLEDFront(value)
    else:
        raise AttributeError("need to initialize robot")

def setLEDBack(value):
    """ Set the Light Emitting Diode on the robot's back. """
    if _robot is not None:
        return _robot.setLEDBack(value)
    else:
        raise AttributeError("need to initialize robot")

################ New Fluke2 functions ###############

def setPicSize(value):
    """ Set the picture size """
    if _robot is not None:
        return _robot.setPicSize(value)
    else:
        raise AttributeError("need to initialize robot")

def servo(id, position):
    """ Commands servo number id to position position """
    if _robot is not None:
        return _robot.servo(id, position)
    else:
        raise AttributeError("need to initialize robot")

def getFlukeLog():
    """ Downloads and prints the fluke2 error log """
    if _robot is not None:
        return _robot.getFlukeLog()
    else:
        raise AttributeError("need to initialize robot")

def enablePanNetworking():
    """ Enables bluetooth PAN TCP/IP over bluetooth networking """
    if _robot is not None:
        return _robot.enablePanNetworking()
    else:
        raise AttributeError("need to initialize robot")

########################### Pictures:

def _ndim(n, *args, **kwargs):
    if not args:
        return [kwargs.get("value", 0)] * n
    A = []
    for i in range(n):
        A.append( _ndim(*args, **kwargs) )
    return A

class Column(object):
    def __init__(self, picture, column):
        self.picture = picture
        self.column = column
    def __getitem__(self, row):
        return self.picture.getPixel(self.column, row)

class Array(object):
    def __init__(self, n = 0, *args, **kwargs):
        if type(n) == Picture:
            self.data = n
        else:
            self.data = _ndim(n, *args, **kwargs)
    def __len__(self):
        return len(self.data)

    def __getitem__(self, *args):
        if type(self.data) == Picture:
            return Column(self.data, args[0])
        else:
            current = self.data
            for i in args:
                n, rest = args[0], args[1:]
                current = current[n]
            return current

def makeArray(*args, **kwargs):
    """ Returns an array of the given dimensions. """
    return Array(*args, **kwargs)

def takePicture(mode=None):
    """ Takes a picture using the camera. Mode can be 'color', 'gray', or 'blob' """
    if _robot is not None:
        return _robot.takePicture(mode)
    else:
        raise AttributeError("need to initialize robot")

def setBoard(mode=None):
    """ sets the board as 'pi' or 'fluke2' """
    if _robot is not None:
        return _robot.setBoard(mode)
    else:
        raise AttributeError("need to initialize robot")

def loadPicture(filename):
    """ Loads a picture from a filename. """
    picture = Picture()
    picture.load(filename)
    return picture

def copyPicture(picture):
    """ Takes a Picture object and returns a copy. """
    newPicture = Picture()
    newPicture.set(getWidth(picture), getHeight(picture),
                   picture.image, mode = "image")
    return newPicture

def makePicture(*args):
    """
    Takes zero or more args to make a picture.

    makePicture() - makes a 0x0 image
    makePicture(width, height)
    makePicture("filename")
    makePicture("http://image")
    makePicture(width, height, data)
    makePicture(width, height, data, "mode")
    """
    if len(args) == 0:
        retval = Picture()
    elif len(args) == 1:
        filename = args[0]
        retval = Picture()
        if filename.startswith("http://"):
            filename, message = urllib.urlretrieve(filename)
        retval.load(filename)
    elif len(args) == 2:
        x = args[0]
        y = args[1]
        retval = Picture()
        retval.set(x, y)
    elif len(args) == 3:
        x = args[0]
        y = args[1]
        if type(args[2]) in [Color, Pixel]:
            retval = Picture()
            retval.set(x, y, value=args[2].getRGB())
        elif type(args[2]) == int:
            retval = Picture()
            retval.set(x, y, value=args[2])
        elif type(args[2]) in [list, tuple]: # Undocumented
            array = args[2]
            retval = Picture()
            retval.set(x, y, value=args[2])
        else:
            raise AttributeError("unknown type: %s is '%s'; " +
                                 "should be Color, Pixel, int grayscale",
                                 args[2], type(args[2]))
    elif len(args) == 4:
        x = args[0]
        y = args[1]
        array = args[2]
        mode = args[3]
        retval = Picture()
        retval.set(x, y, array, mode)
    return retval

def writePictureTo(picture, filename):
    return picture.image.save(filename)

def savePicture(picture, filename):
    if type(picture) == type([]):
        import ImageChops
        from GifImagePlugin import getheader, getdata
        # open output file
        fp = open(filename, "wb")
        previous = None
        for im in picture:
            if type(im) == type(""): # filename
                im = Image.open(im)
                im.load()
                im = im.convert("P") # in case jpeg, etc
            else:
                im = im.image.convert("P")
            if not previous:
                for s in getheader(im) + getdata(im):
                    fp.write(s)
            else:
                delta = ImageChops.subtract_modulo(im, previous)
                bbox = delta.getbbox()
                if bbox:
                    for s in getdata(im.crop(bbox), offset = bbox[:2]):
                        fp.write(s)
            previous = im.copy()
        fp.write(";")
        fp.close()
    else:
        return picture.image.save(filename)

def getWidth(picture):
    return picture.width

def getHeight(picture):
    return picture.height

def getPixel(picture, x, y):
    return picture.getPixel(x, y)

def getPixels(picture):
    return picture.getPixels()

def setPixel(picture, x, y, color):
    return picture.setColor(x, y, color)

def getGray(picture, x, y):
    return sum((picture.getPixel(x, y)).getRGB())/3

def setGray(picture, x, y, gray):
    return getPixel(picture, x, y).setRGB([gray,gray,gray])

############################# Pixels and Colors

def getX(pixel):
    return pixel.x

def getY(pixel):
    return pixel.y

def getRed(pixel):
    return pixel.getRGB()[0]

def getGreen(pixel):
    return pixel.getRGB()[1]

def getBlue(pixel):
    return pixel.getRGB()[2]

def getColor(pixel):
    return pixel.getColor()

def getGray(pixel):
    return sum(pixel.getRGB())/3

def setRGB(pixel_or_color, rgb):
    return pixel_or_color.setRGB(rgb)

def setRGBA(pixel_or_color, rgba):
    return pixel_or_color.setRGBA(rgba)

def getRGB(pixel_or_color):
    return pixel_or_color.getRGB()

def getRGBA(pixel_or_color):
    return pixel_or_color.getRGBA()

def setRed(pixel, value):
    return pixel.setColor(Color(value, pixel.getRGB()[1], pixel.getRGB()[2]))

def setGreen(pixel, value):
    return pixel.setColor(Color(pixel.getRGB()[0], value, pixel.getRGB()[2]))

def setBlue(pixel, value):
    return pixel.setColor(Color(pixel.getRGB()[0], pixel.getRGB()[1], value))

def setGray(pixel, value):
    return pixel.setColor(Color(value, value, value))

def setAlpha(pixel, value):
    return pixel.setAlpha(value)

def getAlpha(pixel):
    return pixel.getAlpha()

def setColor(pixel, color):
    return pixel.setColor(color)

def makeColor(red, green, blue, alpha=255):
    return Color(red, green, blue, alpha)

def makeDarker(color):
    return color.makeDarker()

def makeLighter(color):
    return color.makeLighter()

def odd(n): return (n % 2) == 1
def even(n): return (n % 2) == 0
def wall(threshold=4500): return getObstacle(1) > threshold

def loop(*functions):
    """
    Calls each of the given functions sequentially, N times.
    Example:

    >>> loop(f1, f2, 10)
    will call f1() then f2(), 10 times.
    """
    assert len(functions) > 1,"loop: takes 1 (or more) functions and an integer"
    assert type(functions[-1]) == int, "loop: last parameter must be an integer"
    count = functions[-1]
    for i in range(count):
        for function in functions[:-1]:
            print("   loop #%d: running %s()... " % (i + 1, function.__name__), end="")
            try:
                retval = function()
            except TypeError:
                retval = function(i + 1)
            if retval:
                print(" => %s" % retval)
            else:
                print("")
    stop()
    return "ok"

def doTogether(*functions):
    """
    Runs each of the given functions at the same time.
    Example:

    >>> doTogether(f1, f2, f3)
    will call f1() f2() and f3() together.
    """
    thread_results = [None] * len(functions)
    def makeThread(function, position):
        def newfunction():
            result = function()
            thread_results[position] = result
            return result
        import threading
        thread = threading.Thread()
        thread.run = newfunction
        return thread
    assert len(functions) >= 2, "doTogether: takes 2 (or more) functions"
    thread_list = []
    # first make the threads:
    for i in range(len(functions)):
        thread_list.append(makeThread(functions[i], i))
    # now, start them:
    for thread in thread_list:
        thread.start()
    # wait for them to finish:
    for thread in thread_list:
        thread.join()
    if thread_results == [None] * len(functions):
        print('ok')
    else:
        return thread_results

def beepScale(duration, start, stop, factor=2):
    """
    Calls computer.beep(duration, Hz) repeatedly, where Hz is between
    the given start and stop frequencies, incrementing by the given
    factor.
    """
    hz = start
    while hz <= stop:
        computer.beep(duration, hz)
        hz *= factor

def getFilenames(pattern):
    """ Get a list of filenames via a pattern, like "z??.jpg"."""
    filenames = glob.glob(pattern)
    filenames.sort() # get in order, back to front
    return filenames

# --------------------------------------------------------
# Error handler:
# --------------------------------------------------------
def _jyroExceptionHandler(etype, value, tb):
    lines = traceback.format_exception(etype, value, tb)
    print("Jyro is stopping: -------------------------------------------",  file=sys.stderr)
    for line in lines:
        print(line.rstrip(),  file=sys.stderr)

sys.excepthook = _jyroExceptionHandler
