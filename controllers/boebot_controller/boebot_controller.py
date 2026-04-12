"""boebot_controller controller."""

from controller import Robot, Keyboard

# Initial Constants
SMOOTHING_FACTOR = 0.5
WHEEL_RADIUS = 0.034  # meters
LINE_SENSOR_THRESHOLD = 450

# Initialize the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Enable Keyboard
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Get handles to the devices and enable them.
# These sensors are provided by the Boebot proto, but we don't use them
#left_distance_sensor = robot.getDevice("lds")
#right_distance_sensor = robot.getDevice("rds")
#left_light_sensor = robot.getDevice("lls")
#right_light_sensor = robot.getDevice("rls")
#left_led = robot.getDevice("left_led")
#right_led = robot.getDevice("right_led")
#left_distance_sensor.enable(timestep)
#right_distance_sensor.enable(timestep)
#left_light_sensor.enable(timestep)
#right_light_sensor.enable(timestep)

# Initialize light sensor array (sensor0 - sensor7)
line_sensors = []
for i in range(8):
    sensor = robot.getDevice(f"sensor{i}")
    sensor.enable(timestep)
    line_sensors.append(sensor)

# Get a handle to the motors and set target position to infinity (speed control).
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Motor variables for smoothing (all in m/s)
current_left_speed = 0.0
current_right_speed = 0.0

# Driving state (Speeds in m/s)
forward_speed = 0.3  # Default to Key 5 (0.3 m/s)
turn_speed = 0.15
line_following_enabled = False

print("Keyboard commands:")
print("  Arrow keys - Move the robot")
print("  Keys 1-9   - Set speed")
print("  L          - Toggle line following")

# Main loop:
while robot.step(timestep) != -1:
    # Read keyboard
    key = keyboard.getKey()
    pressed_keys = set()
    while key != -1:
        pressed_keys.add(key)
        key = keyboard.getKey()

    # Define speed and mode keys to avoid disabling line following
    speed_keys = {ord(str(i)) for i in range(1, 10)}
    mode_keys = {ord('L'), ord('l')}

    # Dynamic Speed Control (Keys 1-9)
    # Mapping: 1 -> 0.1 m/s, 9 -> 0.5 m/s (step of 0.05)
    for i in range(1, 10):
        if ord(str(i)) in pressed_keys:
            forward_speed = 0.1 + (i - 1) * 0.05
            turn_speed = forward_speed * 0.5

    # Line Following Mode Toggle
    if mode_keys.intersection(pressed_keys):
        if not line_following_enabled:
            print("Line following: ON")
        line_following_enabled = True
    else:
        # Disable only if a key is pressed that isn't a speed key
        other_keys = pressed_keys - speed_keys
        if other_keys:
            if line_following_enabled:
                print("Line following: OFF")
            line_following_enabled = False

    target_left = 0.0
    target_right = 0.0

    if line_following_enabled:
        active_indices = [i for i, s in enumerate(line_sensors) if s.getValue() < LINE_SENSOR_THRESHOLD]
        
        if active_indices:
            avg_index = sum(active_indices) / len(active_indices)
            error = avg_index - 3.5
            
            steering_gain = 0.4
            side_adjustment = error * forward_speed * steering_gain
            
            base_vel = forward_speed * 1 
            target_left = base_vel + side_adjustment
            target_right = base_vel - side_adjustment
        else:
            target_left = 0.0
            target_right = 0.0
    else:
        # Manual Control
        forward = 0.0
        turn = 0.0

        if Keyboard.UP in pressed_keys:
            forward += 1.0
        if Keyboard.DOWN in pressed_keys:
            forward -= 1.0
        if Keyboard.LEFT in pressed_keys:
            turn -= 0.3 if forward != 0 else 1.0
        if Keyboard.RIGHT in pressed_keys:
            turn += 0.3 if forward != 0 else 1.0

        if forward != 0 and turn != 0:
            target_left = (forward * forward_speed) + (turn * forward_speed)
            target_right = (forward * forward_speed) - (turn * forward_speed)
        elif forward != 0:
            target_left = forward * forward_speed
            target_right = forward * forward_speed
        elif turn != 0:
            target_left = turn * turn_speed
            target_right = -turn * turn_speed

    # Apply softening (LERP)
    current_left_speed += (target_left - current_left_speed) * SMOOTHING_FACTOR
    current_right_speed += (target_right - current_right_speed) * SMOOTHING_FACTOR

    # Set the motor speeds (convert m/s to rad/s)
    left_motor.setVelocity(current_left_speed / WHEEL_RADIUS)
    right_motor.setVelocity(current_right_speed / WHEEL_RADIUS)

    # Print status
    #avarage_speed = (current_left_speed + current_right_speed) / 2
    #ls_values = " ".join([f"{int(s.getValue())}" for s in line_sensors])
    #state_str = "[LINE FOLLOW]" if line_following_enabled else "[MANUAL]"
    #print(f"{state_str} v: {avarage_speed:.3f} m/s | Sensor: [{ls_values}]")
