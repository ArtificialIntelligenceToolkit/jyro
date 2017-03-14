import PIL.Image
import ipywidgets
from IPython.display import display

from jyro.simulator import Simulator
from jyro.simulator.svgcanvas import SVGCanvas

import math

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
        self.update_gui()
        
    def reset(self):
        self.simulator = Simulator()
        if self.worldf is not None:
            self.worldf(self.simulator)
        if self.robot is not None:
            self.robot.reset()
            self.simulator.addRobot(self.robot)

    def create_widgets(self):
        step = ipywidgets.Button(icon="fa-step-forward")
        clear = ipywidgets.Button(description="Clear")
        play = ipywidgets.Play(max=1000000)
        time = ipywidgets.Text(description="Time:")
        html_canvas = ipywidgets.HTML(value=self.canvas._repr_svg_())
        output = ipywidgets.Output()
        camera_image = ipywidgets.Image(value=self.get_image(), width=240)
        update = ipywidgets.Checkbox(description="Update GUI", value=True)
        y = ipywidgets.FloatSlider(readout=False, orientation="vertical")
        x = ipywidgets.FloatSlider(readout=False, orientation="horizontal")
        html_canvas_with_sliders =  ipywidgets.VBox(
            [ipywidgets.HBox([y, html_canvas]),
             x])
        pan = ipywidgets.FloatSlider(readout=False, orientation="horizontal")
        camera_image_with_sliders = ipywidgets.VBox([camera_image, pan])
        horz = ipywidgets.HBox([html_canvas_with_sliders,
                                camera_image_with_sliders])
        title = ipywidgets.VBox([ipywidgets.HBox([update, time]),
                                 horz])
        controls = ipywidgets.HBox([step, play, clear])
        vbox = ipywidgets.VBox([title, controls, output])
        play.observe(self.step, 'value')
        step.on_click(lambda data: self.step({"new": 1}))
        clear.on_click(lambda data: self.widgets["output"].clear_output())
        update.observe(self.update_gui, 'value')
        x.observe(self.set_x, 'value')
        y.observe(self.set_y, 'value')
        pan.observe(self.set_a, 'value')
        y.layout = ipywidgets.Layout(height="248px", padding="0px 0px 0px 0px", width="10px")
        pan.layout = ipywidgets.Layout(height="20px", padding="0px 0px 0px 0px", width="248px")
        x.layout = ipywidgets.Layout(height="30px", padding="0px 0px 0px 10px", width="260px")
        self.widgets.update({
            "step": step,
            "play": play,
            "time": time,
            "html_canvas": html_canvas,
            "output": output,
            "camera_image": camera_image,
            "update": update,
            "pan": pan,
            "x": x,
            "y": y,
        })
        return vbox

    def set_x(self, data):
        x, y, a = self.robot.getPose()
        new_x = data["new"]/100 * self.canvas.max_x # 0 to 100
        self.robot.setPose(new_x, y, a)
        self.update_gui()
    
    def set_y(self, data):
        x, y, a = self.robot.getPose()
        new_y = data["new"]/100 * self.canvas.max_y # 0 to 100
        self.robot.setPose(x, new_y, a)
        self.update_gui()
    
    def set_a(self, data):
        x, y, a = self.robot.getPose()
        new_a = (100 - data["new"])/100 * math.pi * 2
        self.robot.setPose(x, y, new_a)
        self.update_gui(set_angle=False)
    
    def update_gui(self, data=None, set_angle=True):
        self.simulator.draw(self.canvas)
        self.widgets["html_canvas"].value = self.canvas._repr_svg_()
        if self.robot and self.robot.device["camera"]:
            self.widgets["camera_image"].value = self.get_image()
        x, y, a = self.robot.getPose()
        self.widgets["x"].value = (x/self.canvas.max_x) * 100
        self.widgets["y"].value = (y/self.canvas.max_y) * 100
        if set_angle:
            self.widgets["pan"].value = 100 - ((a % (math.pi * 2))/(math.pi * 2)) * 100
            
    def step(self, data):
        ## Update Simulator:
        if data["new"] == 0:
            self.simulator.reset()
            self.simulator.time = 0.0
        elif not self.widgets["update"].value:
            step_size = 50 # each is 0.1 seconds
            for step in range(step_size):
                self.simulator.step()
        else:
            with self.widgets["output"]:
                self.simulator.step()
        ## Update GUI:
        if self.widgets["update"].value:
            self.update_gui()
        self.widgets["time"].value = "%.2f seconds" % self.simulator.time

    def get_image(self):
        if self.robot and self.robot.device["camera"]:
            image = self.robot.device["camera"].getImage()
        else:
            image = PIL.Image.new("RGB", (120, 60))
        return image._repr_png_()
