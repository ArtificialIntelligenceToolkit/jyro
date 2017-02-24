import tkinter

from jyro.simulator.canvas import Canvas

class TkCanvas(Canvas):
    def __init__(self, simulation):
        Canvas.__init__(self)
        self.frame = simulation.frame
        self._canvas = tkinter.Canvas(self.frame, bg="white",
                                      width=simulation.width,
                                      height=simulation.height)
        self._canvas.pack(expand="yes", fill="both", side="top", anchor="n")

    def move(self, name, x, y):
        pass

    def bringToTop(self):
        self._canvas.tkraise("top")
        
    def drawText(self, x, y, text, fill="black", tag="robot", **args):
        #print('called drawText with "%s"' % text)
        fontPixelHeight = 15
        if not self.tkfont:
            import tkFont
            self.font = tkFont.Font(size = -fontPixelHeight) # -n is n pixels tall
            self.tkfont = tkFont.Font(self._canvas, font=self.font)
            self.tkfont.height = self.tkfont.metrics("linespace")
            self.actual = self.font.actual() # but let's get actual
        # sizes are all in pixels
        lines = text.split("\n")
        width = 0
        for line in lines:
            w = self.tkfont.measure(line) + 10 # width of text
            if w > width:
                width = w 
        height = (abs(self.actual["size"]) * len(lines) * 1.25) + fontPixelHeight
        between = 30
        above   = 40
        roundness = 2
        xp, yp = self.scale_x(x), self.scale_y(y)
        points = [(xp, yp),
                  (xp + between, yp - above),
                  (xp + between, yp - above - 10 + roundness),
                  (xp + between + roundness, yp - above - 10),
                  (xp + between + width - roundness, yp - above - 10),
                  (xp + between + width, yp - above - 10 + roundness),
                  (xp + between + width, yp - above - 10 + height - roundness),
                  (xp + between + width - roundness, yp - above - 10 + height),
                  (xp + between + roundness, yp - above - 10 + height),
                  (xp + between, yp - above - 10 + height - roundness),
                  (xp + between, yp - above + 10),
                  ]
        self._canvas.create_polygon(points, tag=(tag,"top"), fill="white", outline="black")
        self._canvas.create_text(self.scale_x(x) + between + 5, self.scale_y(y) - above, text=text, font=self.font, tag=(tag,"top"), fill=fill, anchor="nw", **args)

    def drawLine(self, x1, y1, x2, y2, fill="", tag="robot", **args):
        return self._canvas.create_line(self.scale_x(x1), self.scale_y(y1), self.scale_x(x2), self.scale_y(y2), tag=tag, fill=fill, **args)

    def drawOval(self, x1, y1, x2, y2, **args):
        return self._canvas.create_oval(self.scale_x(x1), self.scale_y(y1),
                                       self.scale_x(x2), self.scale_y(y2),
                                       **args)

    def drawPolygon(self, points, fill="", outline="black", tag="robot", **args):
        xy = map(lambda pt: (self.scale_x(pt[0]), self.scale_y(pt[1])), points)
        xy = list(xy)
        if self.display["wireframe"]:
            if fill != "white":
                outline = fill
            else:
                outline = "black"
            fill = ""
        return self._canvas.create_polygon(xy, tag=tag, fill=fill, outline=outline)

    def remove(self, thing):
        self._canvas.delete(thing)

    def addTrail(self, pos, index, robot):
        Simulator.addTrail(self, pos, index, robot)
        if robot.display["trail"] == 1:
            xya = self.trail[pos][(index - 1) % self.maxTrailSize]
            if xya != None:
                self.drawLine(xya[0], xya[1], robot._gx, robot._gy, robot.color, "trail")
