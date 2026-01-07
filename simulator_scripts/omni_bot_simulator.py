import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class OmniBotSimulator:
    def __init__(self, initial_pose=(0, 0, 0)):
        # --- Physical Parameters (from your code) ---
        self.WHEEL_DIAMETER_CM = 3.81
        self.TICKS_PER_REV = 520
        self.WHEEL_CIRC_CM = np.pi * self.WHEEL_DIAMETER_CM
        self.CM_PER_TICK = self.WHEEL_CIRC_CM / self.TICKS_PER_REV
        
        # Distance from center to wheel, assuming a square chassis of ~17cm side
        self.L = 8.5 
        
        # Wheel angles in radians (45, 135, 225, 315 degrees)
        self.WHEEL_ANGLES = np.deg2rad([45, 135, 225, 315])

        # --- State Variables ---
        self.x, self.y, self.theta = initial_pose
        self.wheel_ticks = np.zeros(4)
        self.path = [(self.x, self.y)]

        # --- PID Control ---
        self.target_ticks = np.zeros(4)
        self.pid_active = False
        # Tunable PID gains
        self.Kp, self.Ki, self.Kd = 2.5, 0.01, 0.8
        self.integral = np.zeros(4)
        self.last_error = np.zeros(4)
        self.max_speed_cm_s = 30.0 # Max speed of a wheel in cm/s for simulation

    def set_target_xy(self, target_x, target_y):
        """Calculates target ticks for each wheel to reach a global (x,y) coordinate."""
        print(f"New target: ({target_x:.2f}, {target_y:.2f})")
        delta_x = target_x - self.x
        delta_y = target_y - self.y

        # --- Inverse Kinematics ---
        # 1. Calculate the required robot velocity vector in the robot's frame
        # For simplicity, we assume a straight line path
        distance = np.sqrt(delta_x**2 + delta_y**2)
        move_angle = np.arctan2(delta_y, delta_x)

        # 2. Calculate the required velocity for each wheel
        # This is the standard, physically correct inverse kinematics model
        wheel_velocities_ratio = np.cos(self.WHEEL_ANGLES - move_angle)
        
        # 3. Calculate total distance each wheel needs to travel
        wheel_distances = distance * wheel_velocities_ratio
        
        # 4. Convert wheel distance to ticks
        ticks_to_move = wheel_distances / self.CM_PER_TICK

        self.target_ticks = self.wheel_ticks + ticks_to_move
        
        # Reset PID controllers and activate
        self.integral = np.zeros(4)
        self.last_error = np.zeros(4)
        self.pid_active = True
        print(f"Current Ticks: {[int(t) for t in self.wheel_ticks]}")
        print(f"Target Ticks:  {[int(t) for t in self.target_ticks]}")

    def update(self, dt):
        """Update the bot's state based on PID control."""
        if not self.pid_active:
            return

        # --- Per-Wheel PID Calculation ---
        current_error = self.target_ticks - self.wheel_ticks
        self.integral += current_error * dt
        derivative = (current_error - self.last_error) / dt
        
        # Calculate PID output (as a velocity command for each wheel)
        output_vel = (self.Kp * current_error) + \
                     (self.Ki * self.integral) + \
                     (self.Kd * derivative)

        self.last_error = current_error
        
        # Clamp output to max speed (emulates PWM limit) and handle direction
        wheel_velocities_cm_s = np.clip(output_vel, -self.max_speed_cm_s, self.max_speed_cm_s)
        
        # --- Forward Kinematics ---
        # Calculate the robot's global velocity (vx, vy, w) from wheel velocities
        # This is the inverse of the matrix used for inverse kinematics
        vx = (wheel_velocities_cm_s[0] - wheel_velocities_cm_s[1] - wheel_velocities_cm_s[2] + wheel_velocities_cm_s[3]) * (1 / (2 * np.sqrt(2)))
        vy = (wheel_velocities_cm_s[0] + wheel_velocities_cm_s[1] - wheel_velocities_cm_s[2] - wheel_velocities_cm_s[3]) * (1 / (2 * np.sqrt(2)))
        w = (wheel_velocities_cm_s[0] + wheel_velocities_cm_s[1] + wheel_velocities_cm_s[2] + wheel_velocities_cm_s[3]) / (4 * self.L)

        # --- Update State ---
        self.x += vx * dt
        self.y += vy * dt
        self.theta += w * dt
        
        # Update wheel ticks based on their simulated velocity
        self.wheel_ticks += (wheel_velocities_cm_s / self.CM_PER_TICK) * dt
        
        self.path.append((self.x, self.y))

        # Check for completion
        if np.all(np.abs(current_error) < 20): # Using the 20-tick deadband from your code
            print("Target reached!")
            self.pid_active = False

# --- Simulation Setup ---
bot = OmniBotSimulator()
target_x, target_y = 50, 20
bot.set_target_xy(target_x, target_y)

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-10, 60)
ax.set_ylim(-10, 30)
ax.grid(True)
ax.set_title("OmniBot Autonomous Movement Simulation")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")

path_line, = ax.plot([], [], 'b-', lw=1, alpha=0.7, label="Path")
bot_body, = ax.plot([], [], 'ko', ms=10, label="Bot")
bot_heading, = ax.plot([], [], 'k-', lw=2)
target_marker, = ax.plot(target_x, target_y, 'rx', ms=12, mew=2, label="Target")
ax.legend()

def animate(frame):
    bot.update(dt=0.05) # Update at 20 Hz
    
    path_x, path_y = zip(*bot.path)
    path_line.set_data(path_x, path_y)
    
    bot_body.set_data(bot.x, bot.y)
    
    # Draw heading line
    heading_x = [bot.x, bot.x + 5 * np.cos(bot.theta)]
    heading_y = [bot.y, bot.y + 5 * np.sin(bot.theta)]
    bot_heading.set_data(heading_x, heading_y)
    
    return path_line, bot_body, bot_heading

ani = FuncAnimation(fig, animate, frames=400, blit=True, interval=50, repeat=False)
plt.show()
