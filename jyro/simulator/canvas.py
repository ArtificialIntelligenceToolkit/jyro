class Canvas():
    """
    A test Canvas class.
    """
    def __init__(self, size, debug=False):
        self.width, self.height = size
        self.display = {"wireframe": 0}
        self.debug = debug
        
    def clear(self):
        if self.debug:
            print("clear")

    def drawLine(self, x1, y1, x2, y2, width=3, outline="black"):
        if self.debug:
            print("line", x1, y1, x2, y2, width, outline)

    def drawOval(self, x1, y1, x2, y2, fill="", outline="black"):
        if self.debug:
            print("oval", x1, y1, x2, y2, fill, outline)

    def drawCircle(self, x, y, radius, fill="", outline="black"):
        if self.debug:
            print("circle", x, y, radius, fill, outline)

    def drawPolygon(self, points, fill="", outline="black"):
        if self.debug:
            print("polygon", points, fill, outline)

    def drawText(self, x, y, text, fill="black"):
        if self.debug:
            print("text", x, y, text, fill)

    def drawRectangle(self, x1, y1, x2, y2, fill="black",
                      outline="black", width=3):
        if self.debug:
            print("rectangle", x1, y1, x2, y1, fill, outline, width)

    def save(self, filename):
        if self.debug:
            print("save", filename)
        
    def renderSVG(self):
        if self.debug:
            print("renderSVG")
        
    def pos_x(self, x):
        return (x * self.scale)

    def pos_y(self, y):
        return ((self.max_y - y) * self.scale)

    def pushMatrix(self):
        if self.debug:
            print("pushMatrix")

    def popMatrix(self):
        if self.debug:
            print("popMatrix")

    def translate(self, x, y):
        if self.debug:
            print("translate", x, y)

    def rotate(self, r):
        if self.debug:
            print("rotate", r)
