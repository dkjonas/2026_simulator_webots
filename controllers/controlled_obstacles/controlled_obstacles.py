"""controlled_obstacles controller."""
from controller import Supervisor
import math
import random


class TriggerController:
    def __init__(self, robot_instance, sensor_name, step, threshold=0.1):
        self.sensor = robot_instance.getDevice(sensor_name)
        self.sensor.enable(step)
        self.threshold = threshold
        self.is_armed = True

    def consume_trigger(self):
        is_triggered = self.sensor.getValue() > self.threshold

        if not is_triggered:
            self.is_armed = True
            return False

        if not self.is_armed:
            return False

        self.is_armed = False
        return True


class GateTimedController:
    STATE_READY = 0
    STATE_OPEN = 1
    OPEN_DURATION = 17 # Time used at roughly 0.25 m/s

    def __init__(self, robot_instance, motor_base_name):
        self.motors = [
            robot_instance.getDevice(f'{motor_base_name}{i}')
            for i in range(2)
        ]
        self.open_duration = self.OPEN_DURATION

        for motor in self.motors:
            motor.setPosition(0)

        self.state = self.STATE_READY
        self.close_time = 0.0

    def open_for(self, duration, current_time):
        if self.state != self.STATE_READY:
            return False

        self.motors[0].setPosition(1.57)
        self.motors[1].setPosition(-1.57)
        self.close_time = current_time + duration
        self.state = self.STATE_OPEN
        return True

    def open(self, current_time):
        return self.open_for(self.open_duration, current_time)

    def update(self, current_time):
        if self.state == self.STATE_OPEN and current_time > self.close_time:
            for motor in self.motors:
                motor.setPosition(0)
            self.state = self.STATE_READY

    def is_ready(self):
        return self.state == self.STATE_READY


class FitnessButtonController:
    NUM_BUTTONS = 4
    LED_OFF = 0x000000
    LED_ON = 0x00ff00
    LED_FAIL = 0xff0000
    LED_SCORE = 0x0000ff
    DEBOUNCE_SECONDS = 0.5

    STATE_INACTIVE = 0
    STATE_ACTIVE = 1
    STATE_GAME_OVER = 2
    STATE_SCORE_SHOWN = 3

    def __init__(self, robot_instance, button_name_prefix, step):
        self.num_buttons = self.NUM_BUTTONS
        self.leds = [
            robot_instance.getDevice(f'{button_name_prefix}{i}_led')
            for i in range(self.num_buttons)
        ]
        self.touch = [
            robot_instance.getDevice(f'{button_name_prefix}{i}_touch')
            for i in range(self.num_buttons)
        ]
        self.time_limits = [10.0, 5.0, 2.5]

        for led in self.leds:
            led.set(self.LED_OFF)

        for sensor in self.touch:
            sensor.enable(step)

        self.game_state = self.STATE_INACTIVE
        self.current_led_index = -1
        self.current_stage = -1
        self.correct_presses = 0
        self.deadline = None
        self.previous_touch_pressed = [False] * self.num_buttons
        self.debounce_until = 0.0
        self.debounce_release_index = -1
        self.debounce_ignore_index = -1

    def _pick_next_index(self, exclude_index):
        next_index = random.randint(0, self.num_buttons - 1)
        while next_index == exclude_index:
            next_index = random.randint(0, self.num_buttons - 1)
        return next_index

    def start_sequence(self):
        for led in self.leds:
            led.set(self.LED_OFF)

        self.game_state = self.STATE_ACTIVE
        self.current_led_index = random.randint(0, self.num_buttons - 1)
        self.current_stage = -1  # First button has no time limit yet
        self.correct_presses = 0
        self.deadline = None
        self.leds[self.current_led_index].set(self.LED_ON)
        self.previous_touch_pressed = [sensor.getValue() > 0 for sensor in self.touch]
        self.debounce_until = 0.0
        self.debounce_release_index = -1
        self.debounce_ignore_index = -1

    def _show_score(self, score):
        for led in self.leds:
            led.set(self.LED_OFF)

        for led in self.leds[:score]:
            led.set(self.LED_SCORE)

        self.game_state = self.STATE_SCORE_SHOWN
        self.current_led_index = -1
        self.current_stage = -1
        self.correct_presses = score
        self.deadline = None
        self.debounce_until = 0.0
        self.debounce_release_index = -1
        self.debounce_ignore_index = -1

    def _set_game_over(self):
        for led in self.leds:
            led.set(self.LED_FAIL)
        self.game_state = self.STATE_GAME_OVER
        self.current_led_index = -1
        self.current_stage = -1
        self.correct_presses = 0
        self.deadline = None
        self.debounce_until = 0.0
        self.debounce_release_index = -1
        self.debounce_ignore_index = -1

    def update(self, current_time):
        if self.game_state != self.STATE_ACTIVE or self.current_led_index < 0:
            return

        current_pressed = [sensor.getValue() > 0 for sensor in self.touch]

        just_pressed_indices = []
        for i, is_pressed in enumerate(current_pressed):
            if is_pressed and not self.previous_touch_pressed[i]:
                just_pressed_indices.append(i)
            self.previous_touch_pressed[i] = is_pressed

        if self.debounce_release_index >= 0 and not current_pressed[self.debounce_release_index]:
            self.debounce_until = current_time + self.DEBOUNCE_SECONDS
            self.debounce_release_index = -1

        filtered_just_pressed_indices = []
        for index in just_pressed_indices:
            is_debounced_button = (
                index == self.debounce_ignore_index and current_time < self.debounce_until
            )
            if not is_debounced_button:
                filtered_just_pressed_indices.append(index)

        if self.debounce_ignore_index >= 0 and current_time >= self.debounce_until and self.debounce_release_index < 0:
            self.debounce_ignore_index = -1

        if any(index != self.current_led_index for index in filtered_just_pressed_indices):
            self._set_game_over()
            return

        if self.current_led_index in filtered_just_pressed_indices:
            previous_led_index = self.current_led_index
            self.leds[previous_led_index].set(self.LED_OFF)
            self.debounce_release_index = previous_led_index
            self.debounce_ignore_index = previous_led_index
            self.correct_presses += 1

            if self.correct_presses >= self.num_buttons:
                self._show_score(self.num_buttons)
                return

            self.current_stage += 1
            self.current_led_index = self._pick_next_index(previous_led_index)
            self.leds[self.current_led_index].set(self.LED_ON)
            self.deadline = current_time + self.time_limits[self.correct_presses - 1]
            return

        if self.deadline is not None and current_time > self.deadline:
            self._show_score(self.correct_presses)

    def reset_sequence(self):
        for led in self.leds:
            led.set(self.LED_OFF)
        self.game_state = self.STATE_INACTIVE
        self.current_led_index = -1
        self.current_stage = -1
        self.correct_presses = 0
        self.deadline = None
        self.debounce_until = 0.0
        self.debounce_release_index = -1
        self.debounce_ignore_index = -1


class GateLockedController:
    NUM_BUTTONS = 2
    LED_GREEN = 0x00ff00
    LED_RED = 0xff0000
    STATE_INACTIVE = 0
    STATE_ACTIVE = 1
    STATE_RESOLVED = 2

    def __init__(self, robot_instance, gate_locked_name, step):
        self.motors = [
            robot_instance.getDevice(f'{gate_locked_name}_motor{i}')
            for i in range(2)
        ]
        self.leds = [
            robot_instance.getDevice(f'{gate_locked_name}_button{i}_led')
            for i in range(self.NUM_BUTTONS)
        ]
        self.touch = [
            robot_instance.getDevice(f'{gate_locked_name}_button{i}_touch')
            for i in range(self.NUM_BUTTONS)
        ]

        for motor in self.motors:
            motor.setPosition(0)

        for led in self.leds:
            led.set(0x000000)

        for sensor in self.touch:
            sensor.enable(step)

        self.state = self.STATE_INACTIVE
        self.correct_index = -1
        self.wrong_index = -1
        self.previous_touch_pressed = [False] * self.NUM_BUTTONS

    def start_sequence(self):
        for motor in self.motors:
            motor.setPosition(0)

        self.correct_index = random.randint(0, self.NUM_BUTTONS - 1)
        self.wrong_index = 1 - self.correct_index

        self.leds[self.correct_index].set(self.LED_GREEN)
        self.leds[self.wrong_index].set(self.LED_RED)

        self.state = self.STATE_ACTIVE
        self.previous_touch_pressed = [sensor.getValue() > 0 for sensor in self.touch]

    def update(self):
        if self.state != self.STATE_ACTIVE:
            return

        current_pressed = [sensor.getValue() > 0 for sensor in self.touch]

        just_pressed_indices = []
        for i, is_pressed in enumerate(current_pressed):
            if is_pressed and not self.previous_touch_pressed[i]:
                just_pressed_indices.append(i)
            self.previous_touch_pressed[i] = is_pressed

        if self.wrong_index in just_pressed_indices:
            for led in self.leds:
                led.set(self.LED_RED)
            self.state = self.STATE_RESOLVED
            return

        if self.correct_index in just_pressed_indices:
            self.motors[0].setPosition(-1.57)
            self.motors[1].setPosition(1.57)
            for led in self.leds:
                led.set(self.LED_GREEN)
            self.state = self.STATE_RESOLVED
            return


class BallPositionerController:
    BALL3A_TARGET = [-2.25, 1.27, 0.02]
    BALL3B_TARGET = [-2.25, -0.01, 0.02]
    BALL3C_CURVE_POINTS = [
        [-1.21, 1.28, 0.02],
        [-1.07, 1.30, 0.02],
        [-0.83, 1.49, 0.02],
        [-0.50, 2.14, 0.02],
        [-0.32, 2.35, 0.02],
        [-0.04, 2.44, 0.02]
    ]

    def __init__(self, supervisor_instance):
        self.root_node = supervisor_instance.getRoot()
        self.ball_nodes = {
            'ball3a': self._get_named_world_node('ball3a'),
            'ball3b': self._get_named_world_node('ball3b'),
            'ball3c': self._get_named_world_node('ball3c'),
        }
        self.translation_fields = {
            ball_name: node.getField('translation')
            for ball_name, node in self.ball_nodes.items()
        }

    def _find_node_by_name(self, node, node_name):
        name_field = node.getField('name')
        if name_field is not None and name_field.getSFString() == node_name:
            return node

        children_field = node.getField('children')
        if children_field is None:
            return None

        for i in range(children_field.getCount()):
            child_node = children_field.getMFNode(i)
            result = self._find_node_by_name(child_node, node_name)
            if result is not None:
                return result

        return None

    def _get_named_world_node(self, node_name):
        children = self.root_node.getField('children')

        for i in range(children.getCount()):
            node = children.getMFNode(i)
            result = self._find_node_by_name(node, node_name)
            if result is not None:
                return result

        raise RuntimeError(f'Could not find node named {node_name}')

    def _random_point_on_curve(self):
        segments = []
        total_length = 0.0

        for i in range(len(self.BALL3C_CURVE_POINTS) - 1):
            start = self.BALL3C_CURVE_POINTS[i]
            end = self.BALL3C_CURVE_POINTS[i + 1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dz = end[2] - start[2]
            length = (dx * dx + dy * dy + dz * dz) ** 0.5
            segments.append((start, end, length))
            total_length += length

        distance = random.uniform(0.0, total_length)

        for start, end, length in segments:
            if distance <= length:
                ratio = 0.0 if length == 0 else distance / length
                return [
                    start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio,
                    start[2] + (end[2] - start[2]) * ratio,
                ]
            distance -= length

        return list(self.BALL3C_CURVE_POINTS[-1])

    def reposition_balls(self):
        target_positions = {
            'ball3a': self.BALL3A_TARGET,
            'ball3b': self.BALL3B_TARGET,
            'ball3c': self._random_point_on_curve(),
        }

        for ball_name, target in target_positions.items():
            self.translation_fields[ball_name].setSFVec3f(target)
            self.ball_nodes[ball_name].resetPhysics()


class FreeGatePositioner:
    ARENA_SQUARE_SIZE = 1.8
    GATE_LEG_DISTANCE = 0.472
    GATE_FOOT_EXTEND_X = 0.046
    GATE_FOOT_LENGHT = 0.115

    LEG_DISTANCE_TO_SQUARE = GATE_FOOT_LENGHT + GATE_LEG_DISTANCE / 2
    CONSTRAINED_HALF_SIZE = ARENA_SQUARE_SIZE / 2 - LEG_DISTANCE_TO_SQUARE
    MAX_RANDOM_ATTEMPTS = 2000

    def __init__(self, supervisor_instance):
        self.root_node = supervisor_instance.getRoot()
        self.gate_nodes = {
            'gate_free0': self._get_named_world_node('gate_free0'),
            'gate_free1': self._get_named_world_node('gate_free1'),
        }
        self.translation_fields = {
            gate_name: node.getField('translation')
            for gate_name, node in self.gate_nodes.items()
        }
        self.rotation_fields = {
            gate_name: node.getField('rotation')
            for gate_name, node in self.gate_nodes.items()
        }

    def _find_node_by_name(self, node, node_name):
        name_field = node.getField('name')
        if name_field is not None and name_field.getSFString() == node_name:
            return node

        children_field = node.getField('children')
        if children_field is None:
            return None

        for i in range(children_field.getCount()):
            child_node = children_field.getMFNode(i)
            result = self._find_node_by_name(child_node, node_name)
            if result is not None:
                return result

        return None

    def _get_named_world_node(self, node_name):
        children = self.root_node.getField('children')

        for i in range(children.getCount()):
            node = children.getMFNode(i)
            result = self._find_node_by_name(node, node_name)
            if result is not None:
                return result

        raise RuntimeError(f'Could not find node named {node_name}')

    def _rotate_position(self, u, v):
        # Rotate 45 degrees (π/4 radians) around center (0, 0)
        cos_45 = 2 ** 0.5 / 2
        sin_45 = 2 ** 0.5 / 2
        x = u * cos_45 - v * sin_45
        y = u * sin_45 + v * cos_45
        return [x, y, 0.0]

    def _rotate_yaw(self, yaw):
        # Rotate the gate orientation with the same 45-degree world transform.
        return yaw + math.pi / 4

    def _random_gate_in_square(self):
        """Generates a random mathematically valid gate strictly inside the constrained square bounds."""
        u_min = -self.CONSTRAINED_HALF_SIZE
        u_max = self.CONSTRAINED_HALF_SIZE
        v_min = -self.CONSTRAINED_HALF_SIZE
        v_max = self.CONSTRAINED_HALF_SIZE
        L = self.GATE_LEG_DISTANCE

        yaw = random.uniform(-math.pi, math.pi)

        span_u = L * math.cos(yaw)
        span_v = L * math.sin(yaw)

        # Permissible start bounds to ensure both legs are inside the square
        u_start_min = u_min if span_u >= 0 else u_min - span_u
        u_start_max = u_max - span_u if span_u >= 0 else u_max

        v_start_min = v_min if span_v >= 0 else v_min - span_v
        v_start_max = v_max - span_v if span_v >= 0 else v_max

        u = random.uniform(u_start_min, u_start_max)
        v = random.uniform(v_start_min, v_start_max)
        return (u, v, yaw)

    def _get_gate_legs(self, u, v, yaw):
        offset_u = self.GATE_LEG_DISTANCE * math.cos(yaw)
        offset_v = self.GATE_LEG_DISTANCE * math.sin(yaw)
        return [
            (u, v),
            (u + offset_u, v + offset_v),
        ]

    def _is_leg_in_exclusion_zone(self, leg_u, leg_v, gate_u, gate_v, gate_yaw):
        L = self.GATE_LEG_DISTANCE
        dx = leg_u - gate_u
        dy = leg_v - gate_v

        cos_y = math.cos(gate_yaw)
        sin_y = math.sin(gate_yaw)

        # Project vector onto gate parallel (lateral) and normal (longitudinal) axes
        lateral = dx * cos_y + dy * sin_y
        longitudinal = -dx * sin_y + dy * cos_y

        # Lateral fits extended gate width (-FOOT_OFFSET to L+FOOT_OFFSET). Longitudinal clearance 1xL behind and 1xL ahead, total length 2L
        return (-self.GATE_FOOT_EXTEND_X <= lateral <= L + self.GATE_FOOT_EXTEND_X) and (-L <= longitudinal <= L)

    def _gates_compatible(self, pose1, pose2):
        legs1 = self._get_gate_legs(*pose1)
        legs2 = self._get_gate_legs(*pose2)

        for leg in legs2:
            if self._is_leg_in_exclusion_zone(leg[0], leg[1], *pose1):
                return False
        for leg in legs1:
            if self._is_leg_in_exclusion_zone(leg[0], leg[1], *pose2):
                return False
        return True

    def reposition_gates(self):
        gate0_pose = None
        gate1_pose = None

        # Sample both simultaneously to prevent locking Gate0 into an unresolvable pose.
        # This resolves the 2000 attempt trap naturally for complex custom zone layouts.
        for _ in range(self.MAX_RANDOM_ATTEMPTS):
            p0 = self._random_gate_in_square()
            p1 = self._random_gate_in_square()
            if self._gates_compatible(p0, p1):
                gate0_pose = p0
                gate1_pose = p1
                break

        if gate0_pose is None or gate1_pose is None:
            print('FreeGatePositioner: failed to place gate pair')
            return

        # Rotate positions and orientations 45 degrees into world space.
        position_gate0 = self._rotate_position(gate0_pose[0], gate0_pose[1])
        position_gate1 = self._rotate_position(gate1_pose[0], gate1_pose[1])
        rotation_gate0 = [0, 0, 1, self._rotate_yaw(gate0_pose[2])]
        rotation_gate1 = [0, 0, 1, self._rotate_yaw(gate1_pose[2])]

        positions = {
            'gate_free0': position_gate0,
            'gate_free1': position_gate1,
        }
        rotations = {
            'gate_free0': rotation_gate0,
            'gate_free1': rotation_gate1,
        }

        for gate_name, position in positions.items():
            self.translation_fields[gate_name].setSFVec3f(position)
            self.rotation_fields[gate_name].setSFRotation(rotations[gate_name])
            self.gate_nodes[gate_name].resetPhysics()


class SeesawPositioner:
    POSITIONS = [-0.197, 0.197]

    def __init__(self, supervisor_instance):
        self.root_node = supervisor_instance.getRoot()
        self.seesaw_node = self._get_named_world_node('seesaw')
        self.position_field = self.seesaw_node.getField('position')

    def _find_node_by_name(self, node, node_name):
        name_field = node.getField('name')
        if name_field is not None and name_field.getSFString() == node_name:
            return node

        children_field = node.getField('children')
        if children_field is None:
            return None

        for i in range(children_field.getCount()):
            child_node = children_field.getMFNode(i)
            result = self._find_node_by_name(child_node, node_name)
            if result is not None:
                return result

        return None

    def _get_named_world_node(self, node_name):
        children = self.root_node.getField('children')
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            result = self._find_node_by_name(node, node_name)
            if result is not None:
                return result
        raise RuntimeError(f'Could not find node named {node_name}')

    def reposition_seesaw(self):
        target = random.choice(self.POSITIONS)
        self.position_field.setSFFloat(target)
        self.seesaw_node.resetPhysics()


# Get the Supervisor instance.
robot = Supervisor()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(timestep)
r_key_armed = True

start_trigger = TriggerController(robot, 'distance_sensor_start', timestep)
gate_timed = GateTimedController(robot, 'gate_timed_motor')
fitness_controller = FitnessButtonController(robot, 'fitness_button', timestep)
gate_locked_controller = GateLockedController(robot, 'gate_locked', timestep)
ball_positioner_controller = BallPositionerController(robot)
free_gate_positioner = FreeGatePositioner(robot)
seesaw_positioner = SeesawPositioner(robot)

print("Obstacle Controller Keyboard commands:")
print("  R - Reset all dynamic obstacles randomly")


def run_start_actions(current_time):
    gate_timed.open(current_time)
    fitness_controller.start_sequence()
    gate_locked_controller.start_sequence()
    ball_positioner_controller.reposition_balls()
    free_gate_positioner.reposition_gates()
    seesaw_positioner.reposition_seesaw()


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    current_time = robot.getTime()

    r_is_down = False
    key = keyboard.getKey()
    while key != -1:
        if key in (ord('R'), ord('r')):
            r_is_down = True
        key = keyboard.getKey()

    keyboard_triggered = r_is_down and r_key_armed
    if r_is_down:
        r_key_armed = False
    else:
        r_key_armed = True

    if start_trigger.consume_trigger() or keyboard_triggered:
        run_start_actions(current_time)

    gate_timed.update(current_time)
    fitness_controller.update(current_time)
    gate_locked_controller.update()
