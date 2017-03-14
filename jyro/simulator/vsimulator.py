import PIL.Image
import ipywidgets
from IPython.display import display

from jyro.simulator import Simulator
from jyro.simulator.svgcanvas import SVGCanvas

class VSimulator():
    def __init__(self, robot=None, worldf=None, size=None):
        self.current_time = 0.0
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
        step = ipywidgets.Button(icon="fa-step-forward")
        play = ipywidgets.Play(max=1000000)
        time = ipywidgets.FloatText(description="Time:")
        html_canvas = ipywidgets.HTML(value=self.canvas._repr_svg_())
        output = ipywidgets.Output()
        camera_image = ipywidgets.Image(value=self.get_image(), width=240)
        update = ipywidgets.Checkbox(description="Update GUI", value=True)
        hbox = ipywidgets.HBox([step, play, time, update])
        vbox = ipywidgets.VBox([html_canvas, camera_image, hbox, output])
        play.observe(self.step, 'value')
        step.on_click(lambda data: self.step({"new": 1}))
        update.observe(self.update_gui, 'value')
        self.widgets.update({
            "step": step,
            "play": play,
            "time": time,
            "html_canvas": html_canvas,
            "output": output,
            "camera_image": camera_image,
            "update": update,
        })
        return vbox

    def update_gui(self, data=None):
        self.simulator.draw(self.canvas)
        self.widgets["html_canvas"].value = self.canvas._repr_svg_()
        if self.simulator.robots and self.simulator.robots[0].device["camera"]:
            self.widgets["camera_image"].value = self.get_image()
            
    def step(self, data):
        ## Update Simulator:
        if data["new"] == 0:
            self.simulator.reset()
            self.current_time = 0.0
        elif not self.widgets["update"].value:
            step_size = 50 # each is 0.1 seconds
            for step in range(step_size):
                self.simulator.step()
            self.current_time += step_size/10
        else:
            with self.widgets["output"]:
                self.simulator.step()
            self.current_time += 0.1
        ## Update GUI:
        if self.widgets["update"].value:
            self.update_gui()
        self.widgets["time"].value = self.current_time

    def get_image(self):
        if self.simulator.robots and self.simulator.robots[0].device["camera"]:
            image = self.simulator.robots[0].device["camera"].getImage()
        else:
            image = PIL.Image.new("RGB", (120, 60))
        return image._repr_png_()
