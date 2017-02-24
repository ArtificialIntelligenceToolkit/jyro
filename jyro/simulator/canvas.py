class Canvas():
    """
    """
    def __init__(self):
        self.display = {"wireframe": 0}
        
    def clear(self):
        pass

    def drawLine(self, x1, y1, x2, y2, fill="", tag="robot", width=3):
        pass

    def drawOval(self, x1, y1, x2, y2, fill="", outline="black", tag="robot"):
        pass

    def drawPolygon(self, points, fill="", outline="black", tag="robot"):
        pass

    def drawText(self, x, y, text, fill="black", tag="robot"):
        pass

    def move(self, name, x, y):
        pass

    def bringToTop(self):
        pass

    def redraw(self):
        self.remove('all')
        for shape in self.shapes:
            if shape[0] == "box":
                name, ulx, uly, lrx, lry, fill = shape
                outline = "black"
                if self.display["wireframe"]:
                    if fill != "white":
                        outline = fill
                    else:
                        outline = "black"
                    fill = ""
                self.canvas.create_rectangle(self.scale_x(ulx), self.scale_y(uly),
                                             self.scale_x(lrx), self.scale_y(lry),
                                             tag="line", fill=fill, outline=outline)
            elif shape[0] == "polygon":
                name, points, nargs = shape
                xys = [(self.scale_x(x), self.scale_y(y)) for (x, y) in points]
                self.canvas.create_polygon(xys, tag="line", **nargs)
            elif shape[0] == "line":
                name, ((x1, y1), (x2, y2)), nargs = shape
                x1, y1, x2, y2 = self.scale_x(x1), self.scale_y(y1), self.scale_x(x2), self.scale_y(y2)
                self.canvas.create_line(x1, y1, x2, y2, tag="line", **nargs)
            elif shape[0] == "oval":
                name, ((x1, y1), (x2, y2)), nargs = shape
                x1, y1, x2, y2 = self.scale_x(x1), self.scale_y(y1), self.scale_x(x2), self.scale_y(y2)
                self.canvas.create_oval(x1, y1, x2, y2, tag="line", **nargs)
        if not self.display["wireframe"]:
            for segment in self.world:
                (x1, y1), (x2, y2) = segment.start, segment.end
                id = self.canvas.drawLine(x1, y1, x2, y2, fill="black", tag="line")
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
        if not self.running:
            self.step(run=0)


