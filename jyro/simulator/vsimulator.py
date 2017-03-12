import ipywidgets
from IPython.display import display

from jyro.simulator import Simulator
from jyro.simulator.svgcanvas import SVGCanvas

class VSimulator():
    def __init__(self, robot=None, worldf=None, size=None):
        self.robot = robot
        self.worldf = worldf
        self.size = size if size else (240, 240)
        self.canvas = SVGCanvas(self.size)
        self.reset()
        self.widgets = {}
        self.simulator.draw(self.canvas)
        display(self.create_widgets())
        
    def reset(self):
        self.simulator = Simulator()
        if self.worldf is not None:
            self.worldf(self.simulator)
        if self.robot is not None:
            self.robot.reset()
            self.simulator.addRobot(self.robot)

    def create_widgets(self):
        play = ipywidgets.Play(max=1000000)
        time = ipywidgets.FloatText(description="Time:")
        html_canvas = ipywidgets.HTML(value=self.canvas._repr_svg_())
        output = ipywidgets.Output()
        camera_image = ipywidgets.Image(value=self.get_image(), width=240)
        hbox = ipywidgets.HBox([play, time])
        vbox = ipywidgets.VBox([html_canvas, camera_image, hbox, output])
        play.observe(self.step, 'value')
        self.widgets.update({
            "play": play,
            "time": time,
            "html_canvas": html_canvas,
            "output": output,
            "camera_image": camera_image,
        })
        return vbox
            
    def step(self, data):
        if data["new"] == 0:
            self.simulator.reset()
        else:
            with self.widgets["output"]:
                self.simulator.step()
        self.simulator.draw(self.canvas)
        self.widgets["html_canvas"].value = self.canvas._repr_svg_()
        self.widgets["camera_image"].value = self.get_image()
        self.widgets["time"].value = data["new"]/10.0

    def get_image(self):
        import PIL.Image
        if self.simulator.robots and self.simulator.robots[0].device["camera"]:
            image = self.simulator.robots[0].device["camera"].getImage()
        else:
            image = PIL.Image.new("RGB", (120, 60))
        return image._repr_png_()
