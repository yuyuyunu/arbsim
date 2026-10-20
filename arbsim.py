# Uses pygame-ce
import pygame
import math
import time
class sim:
    def __init__(self, size=800):
        """
        Initializes the ArbSim drone simulator.
        
        Args:
            size (int): The width and height of the square simulation window in pixels. 
                        Defaults to 800. Represents a fixed 100m x 100m physical area.
        """
        pygame.init()
        # Square window representing a 100m x 100m area
        self.width = size
        self.height = size
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("ArbSim Drone Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 24)

        self.running = True

        # Coordinate System & Scaling
        # Pixels per meter (dynamically calculated based on screen size for 100m area)
        self.ppm = size / 100.0 
        self.center_x = self.width / 2
        self.center_y = self.height / 2
        
        # Position (meters) - start in center
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Velocity (m/s)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        # Target Velocity for Inertia
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.xy_accel = 15.0 
        
        # Targets & Limits
        self.target_z = 0.0
        self.max_xy_speed = 10.0 
        self.z_speed = 1.0 # 1m/s for takeoff/landing

        # Status Flags
        self.armed = False
        self.flying = False

        # Ardupilot behaviors
        self.idle_start_time = time.time()
        self.auto_disarm_delay = 5.0 # seconds

        # Visuals & Trail
        self.record_trail = False
        self.trail_segments = [[]]
        self.last_trail_pos = (self.x, self.y)

    def is_running(self):
        """
        Checks if the simulation window is still active.
        
        Returns:
            bool: True if the simulation is running, False if the window was closed.
        """
        return self.running

    def send_arm_command(self, state: bool):
        """
        Arms or disarms the drone's motors. The drone must be armed to take off.
        Mid-air disarming is prevented.
        
        Args:
            state (bool): True to arm, False to disarm.
        """
        if state:
            if not self.armed:
                self.armed = True
                self.idle_start_time = time.time()
        else:
            if not self.flying: 
                self.armed = False

    def send_takeoff_command(self, alt):
        """
        Commands the drone to take off to a specified altitude.
        The drone must be armed and currently on the ground to execute this.
        
        Args:
            alt (float): The target altitude in meters.
        """
        if self.armed and not self.flying and self.z == 0.0:
            self.target_z = float(alt)
            self.flying = True

    def get_position(self):
        """
        Gets the current 3D position of the drone.
        
        Returns:
            tuple: (x, y, z) representing the drone's coordinates in meters.
        """
        return self.x, self.y, self.z

    def send_velocity(self, v):
        """
        Sends a target velocity command to the drone. The drone will use its internal
        acceleration (inertia) to smoothly reach this target velocity.
        
        Args:
            v (tuple): (vx, vy) representing target velocities along the X and Y axes in m/s.
        """
        vx, vy = v
        
        mag = math.hypot(vx, vy)
        if mag > self.max_xy_speed:
            vx = (vx / mag) * self.max_xy_speed
            vy = (vy / mag) * self.max_xy_speed

        if self.flying:
            self.target_vx = vx
            self.target_vy = vy
        else:
            self.target_vx = 0.0
            self.target_vy = 0.0

    def get_status(self):
        """
        Gets the current operational status of the drone.
        
        Returns:
            tuple: (is_armed (bool), is_flying (bool)).
        """
        return self.armed, self.flying

    def send_land_command(self):
        """
        Commands the drone to initiate a landing sequence. The drone will descend
        to an altitude of 0.0 meters.
        """
        if self.flying:
            self.target_z = 0.0

    def set_trail(self, state: bool):
        """
        Enables or disables the real-time recording of the flight trail path.
        Toggling this off and then on will create visual gaps in the rendered trail.
        
        Args:
            state (bool): True to record the path, False to pause recording.
        """
        if state and not self.record_trail:
            self.trail_segments.append([])
            self.last_trail_pos = (self.x, self.y)
        self.record_trail = state

    def process(self):
        """
        The main simulation tick. Must be called once per loop. 
        Calculates delta time, processes Pygame events, updates drone physics/inertia,
        handles auto-disarming logic, and renders the screen graphics.
        """
        dt = self.clock.tick(60) / 1000.0 

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        if self.target_z > self.z:
            self.z += self.z_speed * dt
            if self.z > self.target_z:
                self.z = self.target_z
        elif self.target_z < self.z:
            self.z -= self.z_speed * dt
            if self.z < self.target_z:
                self.z = self.target_z

        if self.flying and self.target_z == 0.0 and self.z <= 0.0:
            self.z = 0.0
            self.flying = False
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.idle_start_time = time.time()

        if self.flying:
            if self.vx < self.target_vx:
                self.vx = min(self.vx + self.xy_accel * dt, self.target_vx)
            elif self.vx > self.target_vx:
                self.vx = max(self.vx - self.xy_accel * dt, self.target_vx)
                
            if self.vy < self.target_vy:
                self.vy = min(self.vy + self.xy_accel * dt, self.target_vy)
            elif self.vy > self.target_vy:
                self.vy = max(self.vy - self.xy_accel * dt, self.target_vy)

            self.x += self.vx * dt
            self.y += self.vy * dt
        else:
            self.vx = 0.0
            self.vy = 0.0

        self.x = max(-50.0, min(50.0, self.x))
        self.y = max(-50.0, min(50.0, self.y))

        if self.armed and not self.flying and self.z == 0.0:
            if time.time() - self.idle_start_time > self.auto_disarm_delay:
                self.armed = False

        if self.flying and self.record_trail:
            dist = math.hypot(self.x - self.last_trail_pos[0], self.y - self.last_trail_pos[1])
            if dist > 0.5 or len(self.trail_segments[-1]) == 0:
                self.trail_segments[-1].append((self.x, self.y))
                self.last_trail_pos = (self.x, self.y)

        self.screen.fill((35, 40, 45))

        for i in range(-5, 6):
            offset = i * 10.0 * self.ppm
            pygame.draw.line(self.screen, (55, 60, 65), (self.center_x + offset, 0), (self.center_x + offset, self.height))
            pygame.draw.line(self.screen, (55, 60, 65), (0, self.center_y + offset), (self.width, self.center_y + offset))

        for segment in self.trail_segments:
            if len(segment) > 1:
                t_points = [(self.center_x + tx * self.ppm, self.center_y + ty * self.ppm) for tx, ty in segment]
                pygame.draw.lines(self.screen, (200, 200, 50), False, t_points, 2)

        px = int(self.center_x + self.x * self.ppm)
        py = int(self.center_y + self.y * self.ppm)

        if self.z > 0:
            shadow_offset = int(self.z * (1.5 * (self.width/800))) # Scale shadow offset with screen size
            pygame.draw.circle(self.screen, (20, 20, 20), (px + shadow_offset, py + shadow_offset), int(8 * (self.width/800)))

        if self.flying:
            color = (0, 200, 255)
        elif self.armed:
            color = (0, 255, 0)
        else:
            color = (255, 0, 0)

        # Scale drone base size with screen size to keep visual proportions identical
        base_size = 8.0 * (self.width / 800.0)
        scale = 1.0 + (self.z * 0.04)
        pygame.draw.circle(self.screen, color, (px, py), int(base_size * scale))
        
        if self.flying:
            end_x = px + int(self.vx * self.ppm)
            end_y = py + int(self.vy * self.ppm)
            pygame.draw.line(self.screen, (255, 255, 255), (px, py), (end_x, end_y), 2)

        lines = [
            f"State: {'ARMED' if self.armed else 'DISARMED'} | {'FLYING' if self.flying else 'ON GROUND'}",
            f"Pos: X:{self.x:.1f}m  Y:{self.y:.1f}m  Z:{self.z:.1f}m",
            f"Vel: VX:{self.vx:.1f}m/s  VY:{self.vy:.1f}m/s",
            f"Trail Recording: {'ON' if self.record_trail else 'OFF'}"
        ]
        
        if self.armed and not self.flying:
            time_left = max(0, self.auto_disarm_delay - (time.time() - self.idle_start_time))
            lines.append(f"Auto-disarm in: {time_left:.1f}s")

        for i, text in enumerate(lines):
            surf = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(surf, (10, 10 + (i * 24)))

        pygame.display.flip()