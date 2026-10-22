uses pygame-ce library
### Basic Loop Example
```python
import arbsim

# Initialize the simulator (default size is 800x800)
sim = arbsim.sim()

sim.set_trail(True)

# The Core Loop
while sim.is_running():
    # 1. Read state
    x, y, z = sim.get_pos()
    
    # 2. Send commands (e.g., if conditions are met)
    # sim.send_vel((2.0, 0.0))
    
    # 3. Process physics and render frame (REQUIRED)
    sim.process()
