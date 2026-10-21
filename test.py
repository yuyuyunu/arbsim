import arbsim

sim = arbsim.sim()

state = 0

while sim.is_running():

    is_armed, is_flying, is_landing = sim.get_status()
    x, y, z = sim.get_position()

    if (state == 0):
        if (not is_armed):
            sim.send_arm_command(True)

        if (not is_flying):
            sim.send_takeoff_command(5)
        
        if (z > 4):
            sim.send_velocity((5,0))
        
        if (x > 20):
            state = 1
    elif (state ==1):
        if (x>0):
            sim.send_velocity((-5,0))
        elif (is_flying and not is_landing):
            sim.send_velocity((0,0))
            sim.send_land_command()
        elif(not is_flying):
            sim.send_arm_command(False)
            state=2
    
    elif(state==2):
        print("DONE")

    sim.process()