import pygame
import arbsim

sim = arbsim.sim()

while sim.is_running():

    is_armed, is_flying = sim.get_status()

    print(f"flying: {is_flying}\narmed: {is_armed}\n")

    sim.process()