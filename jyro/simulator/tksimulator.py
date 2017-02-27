import tkinter

from jyro.simulator.simulator import Simulator
from jyro.simulator.tkcanvas import TkCanvas

class TkSimulator(tkinter.Toplevel, Simulator):
    def __init__(self, dimensions, offsets, scale, root=None):
        if root == None:
            root = tkinter.Tk()
            root.withdraw()
        tkinter.Toplevel.__init__(self, root)
        Simulator.__init__(self, dimensions, offsets, scale)
        self.root = root
        self.tkfont = None
        self.wm_title("Jyro Simulator")
        self.protocol('WM_DELETE_WINDOW', self.destroy)
        self.frame = tkinter.Frame(self)
        self.frame.pack(side = 'bottom', expand = "yes", anchor = "n",
                        fill = 'both')
        self.canvas = TkCanvas(self)
        self._canvas = self.canvas._canvas
        self.addMouseBindings()
        self.mBar = tkinter.Frame(self, relief=tkinter.RAISED, borderwidth=2)
        self.mBar.pack(fill=tkinter.X)
        self.lastEventRobot = None
        self.menuButtons = {}
        menu = [
            ('File', [['Reset', self.reset],
                      ['Display world details', self.printDetails],
                      ['Exit', self.destroy]]),
            ('View',[
            ['wireframe', lambda: self.simToggle("wireframe")],
            ['trail', lambda: self.toggle("trail")],                     
            ['body', lambda: self.toggle("body")],                 
            ['boundingBox', lambda: self.toggle("boundingBox")],
            ['gripper', lambda: self.toggle("gripper")],
            ['camera', lambda: self.toggle("camera")],
            ['sonar', lambda: self.toggle("sonar")],
            ['ir', lambda: self.toggle("ir")],
            ['bumper', lambda: self.toggle("bumper")],
            ['light', lambda: self.toggle("light")],                     
            ['lightBlocked', lambda: self.toggle("lightBlocked")],
            ['speech', lambda: self.toggle("speech")],
            ]
             ),
            ('Options', [['lights visible above walls',
                          lambda: self.toggleOption("lightAboveWalls")]]),
            ]
        for entry in menu:
            self.mBar.tk_menuBar(self.makeMenu(self.mBar, entry[0], entry[1]))
        self.shapes = []

    def toggleOption(self, key):
        if key == "lightAboveWalls":
            self.lightAboveWalls = not self.lightAboveWalls
        else:
            raise AttributeError("invalid key: '%s'" % key)
        self.redraw()

    def simToggle(self, key):
        self.display[key] = not self.display[key]
        self.redraw()

    def toggle(self, key):
        for r in self.robots:
            if r.subscribed == 0: continue
            if r.display[key] == 1:
                r.display[key] = 0
            else:
                r.display[key] = 1
            r._last_pose = (-1, -1, -1)
        self.redraw()

    def reset(self):
        for r in self.robots:
            r._gx, r._gy, r._ga = r._xya
            r.energy = 10000.0
        for l in self.lights:
            l.x, l.y, l.brightness = l._xyb
        self.redraw()

    def makeMenu(self, bar, name, commands):
        """ Assumes self.menuButtons exists """
        menu = tkinter.Menubutton(bar,text=name,underline=0)
        self.menuButtons[name] = menu
        menu.pack(side=tkinter.LEFT,padx="2m")
        menu.filemenu = tkinter.Menu(menu)
        for cmd in commands:
            if cmd:
                menu.filemenu.add_command(label=cmd[0],command=cmd[1])
            else:
                menu.filemenu.add_separator()
        menu['menu'] = menu.filemenu
        return menu

    def destroy(self):
        if not self.running:
            self.withdraw()
        self.done = 1 # stop processing requests, if handling
        self.quit = 1 # stop accept/bind toplevel
        self.root.quit() # kill the gui

    def dispatch_event(self, event, type):
        if self.lastEventRobot:
            return self.lastEventRobot.mouse_event(event, type, self.lastEventRobot)
        # else let's get a robot
        widget = event.widget
        x = widget.canvasx(event.x)
        y = widget.canvasy(event.y)
        d = 5 # overlap, in canvas units
        items = widget.find_overlapping(x-d, y-d, x+d, y+d)
        for item in items:
            tags = self._canvas.gettags(item)
            for tag in tags:
                if "robot-" in tag:
                    robot = self.robotsByName[tag[6:]]
                    self.lastEventRobot = robot
                    return robot.mouse_event(event, type, robot)

    def addMouseBindings(self):
        self._canvas.bind("<B1-Motion>", func=lambda event=self:self.dispatch_event(event, "motion"))
        self._canvas.bind("<Button-1>",  func=lambda event=self:self.dispatch_event(event, "down"))
        self._canvas.bind("<ButtonRelease-1>", func=lambda event=self:self.dispatch_event(event, "up"))
        self._canvas.bind("<Control-B1-Motion>", func=lambda event=self:self.dispatch_event(event, "control-motion"))
        self._canvas.bind("<Control-Button-1>", func=lambda event=self:self.dispatch_event(event, "control-down"))
        self._canvas.bind("<Control-ButtonRelease-1>", func=lambda event=self:self.dispatch_event(event, "control-up"))
        self._canvas.bind("<ButtonRelease-2>", self.click_b2_up)
        self._canvas.bind("<ButtonRelease-3>", self.click_b3_up)
        self._canvas.bind("<Button-2>", self.click_b2_down)
        self._canvas.bind("<Button-3>", self.click_b3_down)
        self._canvas.bind("<B2-Motion>", self.click_b2_move)
        self._canvas.bind("<B3-Motion>", self.click_b3_move)

    def click_b2_down(self, event):
        self.click_start = event.x, event.y

    def click_b3_down(self, event):
        self.click_start = event.x, event.y
        self.click_b3_move(event)

    def click_b2_up(self, event):
        self.click_stop = event.x, event.y
        if self.click_stop == self.click_start:
            # center on this position:
            center = self._canvas.winfo_width()/2, self._canvas.winfo_height()/2
            x_diff = self.click_start[0] - self.click_stop[0]
            y_diff = self.click_start[1] - self.click_stop[1]
            self.offset_x -= (self.click_stop[0] - center[0])
            self.offset_y -= (self.click_stop[1] - center[1])
        else: # move this much
            x_diff = self.click_start[0] - self.click_stop[0]
            y_diff = self.click_start[1] - self.click_stop[1]
            self.offset_x -= x_diff
            self.offset_y -= y_diff
        self.redraw()

    def click_b3_up(self, event):
        """
        Button handler for B3 for scaling window
        """
        stop = event.x, event.y
        center = self._canvas.winfo_width()/2, self._canvas.winfo_height()/2
        radius_stop = Segment(center, stop).length()
        radius_start = Segment(center, self.click_start).length()
        self.scale *= radius_stop/radius_start
        self.offset_x = (radius_stop/radius_start) * self.offset_x + (1 - (radius_stop/radius_start)) * center[0]
        self.offset_y = (radius_stop/radius_start) * self.offset_y + (1 - (radius_stop/radius_start)) * center[1]
        self.redraw()

    def click_b2_move(self, event):
        self.remove('arrow')
        self.click_stop = event.x, event.y
        x1, y1 = self.click_start
        x2, y2 = self.click_stop
        self._canvas.create_line(x1, y1, x2, y2, tag="arrow", fill="purple")

    def click_b3_move(self, event):
        self.remove('arrow')
        stop = event.x, event.y
        center = self._canvas.winfo_width()/2, self.canvas.winfo_height()/2
        radius = Segment(center, stop).length()
        self._canvas.create_oval(center[0] - radius, center[1] - radius,
                                center[0] + radius, center[1] + radius,
                                tag="arrow", outline="purple")
    def resetPath(self, num):
        for point in range(len(self.trail[num])):
            self.trail[num][point] = None

    def resetPaths(self):
        for t in range(len(self.trail)):
            self.resetPath(t)

    def clear(self):
        self._canvas.remove('all')
        
    def printDetails(self):
        print("Window: size=(%d,%d), offset=(%d,%d), scale=%f" % (self.winfo_width(), self.winfo_height(), self.offset_x, self.offset_y, self.scale))
        for robot in self.robots:
            print("   %s: pose = (%.2f, %.2f, %.2f)" % (robot.name, robot._gx, robot._gy, robot._ga % (2 * math.pi)))
        

    def update(self):
        self.update_idletasks()

if __name__ == "__main__":
    from jyro.simulator.robot import Pioneer
    from jyro.simulator.device import PioneerFrontSonars
    
    sim = TkSimulator((443,466), (22,420), 40.357554)
    robot = Pioneer("Pioneer", 4.99, 1.32, 6.28)
    robot.addDevice(PioneerFrontSonars())
    sim.addRobot(robot)
    sim.
