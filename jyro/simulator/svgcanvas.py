from calysto.graphics import (Canvas as CalystoCanvas, Line,
                              Circle, Polygon, Text, Ellipse, Rectangle)
from jyro.simulator.canvas import Canvas

import math

def distance(p1, p2):
    """
    Return distance between two points
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 +
                     (p1[1] - p2[1]) ** 2)

class SVGCanvas(Canvas):
    """
    """
    def __init__(self, *args, **kwargs):
        Canvas.__init__(self, *args, **kwargs)
        self._canvas = CalystoCanvas((self.width, self.height))
        
    def clear(self):
        self._canvas.clear()
        shape = Rectangle((0, 0), (self.width, self.height), fill="white")
        shape.draw(self._canvas)

    def drawLine(self, x1, y1, x2, y2, width=3, outline="black"):
        shape = Line((x1, y1), (x2, y2), stroke=outline)
        shape.draw(self._canvas)

    def drawOval(self, x1, y1, x2, y2, fill="", outline="black"):
        cx, cy = (x2 - x1)/2, (y2 - y1)/2
        radius  = distance((cx, cy), (x2, y2))
        shape = Ellipse((cx, cy), (radius, radius), fill=fill, stroke=outline)
        shape.draw(self._canvas)

    def drawRectangle(self, x1, y1, x2, y2, fill="", outline="black"):
        width = x2 - x1
        height = y2 - y1
        shape = Rectangle((x1, y1), (width, height), stroke=outline, fill=fill)
        shape.draw(self._canvas)

    def drawCircle(self, cx, cy, radius, fill="", outline="black"):
        shape = Circle((cx, cy), radius, fill=fill, stroke=outline)
        shape.draw(self._canvas)

    def drawPolygon(self, points, fill="", outline="black"):
        shape = Polygon(points, stroke=outline, fill=fill)
        shape.draw(self._canvas)

    def drawText(self, x, y, text, fill="black"):
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
