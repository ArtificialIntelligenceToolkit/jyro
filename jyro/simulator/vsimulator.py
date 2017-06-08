def main():
    from jyro.simulator import (Pioneer, Simulator, Camera,
                                PioneerFrontSonars, Gripper,
                                PioneerFrontLightSensors)
    
    sim = Simulator()
    # (443,466), (22,420), 40.357554)
    sim.addBox(0, 0, 10, 10, fill="white", wallcolor="grey") # meters
    sim.addBox(1, 1, 2, 2, "purple")
    sim.addBox(7, 7, 8, 8, "purple")
    ## brightness of 1 is radius 1 meter
    sim.addLight(7, 7, 4.25, color=Color(255, 255, 0, 64))
    robot = Pioneer("Pioneer", 5.00, 5.00, math.pi / 2) # meters, radians
    robot.addDevice(PioneerFrontSonars(maxRange=4.0))
    robot.addDevice(Gripper())
    robot.addDevice(PioneerFrontLightSensors())
    robot.addDevice(Camera())
    sim.addRobot(robot)
    return sim
    
if __name__ == "__main__":
    from jyro.simulator import Canvas
    sim = main()
    print("pose:", sim["Pioneer"].getPose())
    sim["Pioneer"].move(1, 1)
    canvas = Canvas((400, 400))
    for i in range(1):
        sim.step()
        for r in sim.robots:
            r.updateDevices()
        print("pose:", sim["Pioneer"].getPose())
        print("   light:", sim["Pioneer"].device["light"].scan)
        sim.draw(canvas)
        canvas.save("canvas%d.svg" % i)


