from calysto.graphics import (Canvas as CalystoCanvas, Line,
                              Circle, Polygon, Text, Ellipse, Rectangle)
import math

def distance(p1, p2):
    """
    Return distance between two points
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 +
                     (p1[1] - p2[1]) ** 2)

class Canvas():
    """
    """
    def __init__(self, size, overlay=False):
        self.width, self.height = size
        self.display = {"wireframe": 0}
        self._canvas = CalystoCanvas((self.width, self.height))
        if not overlay:
            self._overlay = Canvas(size, overlay=True)
        else:
            self._overlay = None

    def pos_x(self, x):
        return (x * self.scale)

    def pos_y(self, y):
        return ((self.max_y - y) * self.scale)

    def reset(self):
        if self._overlay:
            self._overlay.clear()
        self.clear()

    def clear(self):
        self._canvas.clear()

    def drawLine(self, x1, y1, x2, y2, width=3, outline="black"):
        x1 = self.pos_x(x1)
        y1 = self.pos_y(y1)
        x2 = self.pos_x(x2)
        y2 = self.pos_y(y2)
        shape = Line((x1, y1), (x2, y2), stroke=outline)
        shape.draw(self._canvas)

    def drawOval(self, x1, y1, x2, y2, fill="", outline="black"):
        x1 = self.pos_x(x1)
        y1 = self.pos_y(y1)
        x2 = self.pos_x(x2)
        y2 = self.pos_y(y2)
        cx, cy = (x2 - x1)/2, (y2 - y1)/2
        radius  = distance((cx, cy), (x2, y2))
        shape = Ellipse((cx, cy), (radius, radius), fill=fill, stroke=outline)
        shape.draw(self._canvas)

    def drawRectangle(self, x1, y1, x2, y2, fill="", outline="black"):
        x1 = self.pos_x(x1)
        y1 = self.pos_y(y1)
        x2 = self.pos_x(x2)
        y2 = self.pos_y(y2)
        width = x2 - x1
        height = y2 - y1
        shape = Rectangle((x1, y1), (width, height), stroke=outline, fill=fill)
        shape.draw(self._canvas)

    def drawCircle(self, cx, cy, radius, fill="", outline="black"):
        cx = self.pos_x(cx)
        cy = self.pos_y(cy)
        shape = Circle((cx, cy), radius * self.scale, fill=fill, stroke=outline)
        shape.draw(self._canvas)

    def drawPolygon(self, points, fill="", outline="black"):
        points = [(self.pos_x(xy[0]), self.pos_y(xy[1])) for xy in points]
        shape = Polygon(points, stroke=outline, fill=fill)
        shape.draw(self._canvas)

    def _drawPolygon(self, points, fill="", outline="black"):
        shape = Polygon(points, stroke=outline, fill=fill)
        shape.draw(self._canvas)

    def drawArrow(self, cx, cy, angle, size, fill="", outline="black"):
        cx = self.pos_x(cx)
        cy = self.pos_y(cy)
        self.pushMatrix()
        self.translate(cx, cy)
        self.rotate(math.pi - angle)
        size = size * self.scale
        p = [(-size, -size), (0, 0), (size, -size), (0, size)]
        self._drawPolygon(p, fill=fill, outline=outline)
        self.popMatrix()

    def drawText(self, x, y, text, fill="black"):
        x = self.pos_x(x)
        y = self.pos_y(y)
        shape = Text(text, (x, y), stroke=fill)
        shape.draw(self._canvas)

    def save(self, filename):
        self._canvas.save(filename)

    def render(self, format="SVG", **attribs):
        format = format.upper()
        if format == "SVG":
            return self._canvas._repr_svg_(**attribs)
        elif format == "PIL":
            return self._canvas.toPIL(**attribs)
        elif format == "GIF":
            return self._canvas.toGIF(**attribs)
        elif format == "PNG":
            return self._canvas.convert(format="png", **attribs)
        else: # try it:
            return self._canvas.convert(format=format.lower(), **attribs)

    def _repr_svg_(self):
        return self._canvas._repr_svg_()

    def pushMatrix(self):
        self._canvas.pushMatrix()

    def popMatrix(self):
        self._canvas.popMatrix()

    def translate(self, x, y):
        self._canvas.translate(x, y)

    def rotate(self, r):
        self._canvas.rotate(r)
