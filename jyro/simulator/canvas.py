class Canvas():
    """
    A test Canvas class.
    """
    def __init__(self, size):
        self.width, self.height = size
        self.display = {"wireframe": 0}
        
    def clear(self):
        print("clear")

    def drawLine(self, x1, y1, x2, y2, width=3, outline="black"):
        print("line", x1, y1, x2, y2, width, outline)

    def drawOval(self, x1, y1, x2, y2, fill="", outline="black"):
        print("oval", x1, y1, x2, y2, fill, outline)

    def drawCircle(self, x, y, radius, fill="", outline="black"):
        print("circle", x, y, radius, fill, outline)

    def drawPolygon(self, points, fill="", outline="black"):
        print("polygon", points, fill, outline)

    def drawText(self, x, y, text, fill="black"):
        print("text", x, y, text, fill)

    def drawRectangle(self, x1, y1, x2, y2, fill="black",
                      outline="black", width=3):
        print("rectangle", x1, y1, x2, y1, fill, outline, width)

    def save(self, filename):
        print("save", filename)
        
    def renderSVG(self):
        print("renderSVG")
        
    def scale_x(self, x, scale):
        return (x * scale)

    def scale_y(self, y, scale):
        return (y * scale)

