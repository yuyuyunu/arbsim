uses pygame-ce library
### Basic Loop Example
```python
import arbsim

# Initialize the simulator (default size is 800x800)
drone_sim = arbsim.sim()

# The Core Loop
while drone_sim.is_running():
    # 1. Read state
    x, y, z = drone_sim.get_pos()
    
    # 2. Send commands (e.g., if conditions are met)
    # drone_sim.send_vel((2.0, 0.0))
    
    # 3. Process physics and render frame (REQUIRED)
    drone_sim.process()