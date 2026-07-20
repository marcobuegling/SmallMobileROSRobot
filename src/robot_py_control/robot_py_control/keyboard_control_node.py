import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, Parameter, SetParametersResult
from geometry_msgs.msg import Twist
from robot_interfaces.srv import StartStop
from pynput import keyboard
import threading

# Define all possible control keys - possible change to node parameters in the future for customization
CONTROL_KEYS = {keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right}

# Max and min frequency at which control command messages can be published (in Hz)
MAX_UPDATE_FREQUENCY = 100.0
MIN_UPDATE_FREQUENCY = 1.0

class KeyboardControlNode(Node):
    """
    ROS node used for handling user input for mobile robots.
    This node has two main functions:
    Control using arrow keys: publishes on topic 'car_control'
    Further actions: 'q': quit, 's': stop motors, 'd': start motors
    """
    def __init__(self):
        super().__init__('keyboard_control')

        # Node parameter declaration
        # potentially add key bindings as parameters to allow rebinding
        self.declare_parameter(
            'key_control_active', 
            True
        )
        self.declare_parameter(
            'acceleration_sensitivity', 
            1.0, 
            ParameterDescriptor(floating_point_range=[FloatingPointRange(0.0, 1.0, 0.0)])
        )
        self.declare_parameter(
            'steering_sensitivity', 
            1.0, 
            ParameterDescriptor(floating_point_range=[FloatingPointRange(0.0, 1.0, 0.0)])
        )
        self.declare_parameter(
            'update_frequency', 
            20.0, 
            ParameterDescriptor(description='Frequency of command updates in Hz.', floating_point_range=[FloatingPointRange(MIN_UPDATE_FREQUENCY, MAX_UPDATE_FREQUENCY, 0.0)])
        )
        # Cache values locally and calculate update interval
        self._key_control_active = self.get_parameter('key_control_active').value
        self._acceleration_sensitivity = self.get_parameter('acceleration_sensitivity').value
        self._steering_sensitivity = self.get_parameter('steering_sensitivity').value
        self._update_frequency = self.get_parameter('update_frequency').value

        # Register callback for external changes of parameters
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self._calculate_update_strength()
        
        # Track current speed and steering
        self._speed = 0.0 # values from -1.0 (full speed backwards) to 1.0 (full speed forwards)
        self._steering = 0.0 # values from -1.0 (full right turn) to 1.0 (full left turn)

        # Track keys that are currently held down
        self._held_keys = set()

        # Create publisher for control commands as Ackermann drive commands (i.e. just speed and steering)
        self._control_pub = self.create_publisher(Twist, '/car_control', 10)
        # Create service client for toggling motors activity
        self._start_stop_cli = self.create_client(StartStop, '/toggle_motors')

        # Create timer for publishing control commands
        self._update_interval = 1 / self._update_frequency
        self.timer = self.create_timer(timer_period_sec=self._update_interval, callback=self._publish_command)

        # Create a lock for thread-safe updates, start thread for listener and listener itself
        self.lock = threading.Lock()
        self.listener_thread = threading.Thread(target=self._start_keyboard_listener)
        self.listener_thread.start()

    def _calculate_update_strength(self):
        """
        Effective speed and steering update strength in a single step
        Takes the update frequency relative to the maximum update frequency into account
        Clipped to 1 (i.e. instant update in a single step)
        """
        self._speed_update_strength = max(1.0, self._acceleration_sensitivity * MAX_UPDATE_FREQUENCY / self._update_frequency)
        self._steering_update_strength = max(1.0, self._steering_sensitivity * MAX_UPDATE_FREQUENCY / self._update_frequency)

    def _on_parameters_changed(self, params):
        for p in params:
            if p.name == 'key_control_active':
                self._key_control_active = p.value
            elif p.name == 'acceleration_sensitivity':
                self._acceleration_sensitivity = p.value
            elif p.name == 'steering_sensitivity':
                self._steer_sensitivity = p.value
            elif p.name == 'update_frequency':
                self._update_frequency = self.get_parameter('update_frequency').value
                self._update_interval = 1 / self._update_frequency
                self.timer.destroy()
                self.timer = self.create_timer(timer_period_sec=self._update_interval, callback=self._publish_command)
        self._calculate_update_strength()
        return SetParametersResult(successful=True)

    def toggle_key_control(self):
        """
        Activate or deactivate speed and steering control using the keyboard.
        All other actions (quit, motor start/stop) are not affected.
        Initial state can be set through parameter 'key_control_active'.
        """
        if self._key_control_active:
            self._speed = 0.0
            self._steering = 0.0
            self._held_keys.clear()
        self.set_parameters([Parameter('key_control_active', not self._key_control_active)])

    def on_press(self, key: keyboard.Key):
        """Callback for key press events."""

        # Stop the keyboard listener to quit program
        if key in (keyboard.Key.esc, keyboard.KeyCode.from_char('q')):
            return False
        
        # Handle start and stop of motors
        if key == keyboard.KeyCode.from_char('d'):
            req = StartStop.Request()
            req.start_stop_signal = True
            self._start_stop_cli.call_async(req)
            return
        if key == keyboard.KeyCode.from_char('s'):
            req = StartStop.Request()
            req.start_stop_signal = False
            self._start_stop_cli.call_async(req)
            return
        
        # Handle control input if active
        if self._key_control_active and key in CONTROL_KEYS:
            with self.lock:
                self._held_keys.add(key)

    def on_release(self, key: keyboard.Key):
        """Callback for key release events."""
        with self.lock:
            self._held_keys.discard(key) 

    def _update_speed_and_steering(self):
        """Updates the speed and steering values locally"""
        # Copy currently held keys
        with self.lock:
            held = set(self._held_keys)

        # Calculate target speed based on currently held keys
        target_speed = 0.0
        target_steering = 0.0
        if keyboard.Key.up in held:
            target_speed += 1.0
        if keyboard.Key.down in held:
            target_speed -= 1.0
        if keyboard.Key.left in held:
            target_steering += 1.0
        if keyboard.Key.right in held:
            target_steering -= 1.0

        # Update speed with respect to update sensitivity
        self._speed = self._speed + self._update_interval * self._acceleration_sensitivity * (target_speed - self._speed)
        self._steering = self._steering + self._update_interval * self._steering_sensitivity * (target_steering - self._steering)

    def _publish_command(self):
        """Publishes a new message with a drive command."""
        self._update_speed_and_steering()
        msg = Twist()
        msg.linear.x = self._speed
        msg.angular.z = self._steering
        self._control_pub.publish(msg)

    def _start_keyboard_listener(self):
        """Start the keyboard listener. Suppresses passing of all input events to rest of the system."""
        # also possible without with..as
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=True) as listener: 
            listener.join()


def main(args=None):
    rclpy.init(args=args)

    keyboard_control_node = KeyboardControlNode()

    rclpy.spin(keyboard_control_node)

    keyboard_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
