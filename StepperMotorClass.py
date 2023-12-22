import RPi.GPIO as GPIO
import time

class StepperMotor:
    def __init__(self, dir_pin=7, step_pin=13, steps_per_revolution=400, max_speed_delay=30, min_speed_delay=50):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.steps_per_revolution = steps_per_revolution
        self.max_speed_delay = max_speed_delay
        self.min_speed_delay = min_speed_delay

        # Initialize GPIO settings
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)

    def distance2step(self, displacement):
        step = (displacement / 5) * 40000
        return int(step)
    
    def _step(self, delay):
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(self.step_pin, GPIO.LOW)
        time.sleep(delay)

    def delay_microseconds(self, microseconds):
        start_time = time.time()
        while (time.time() - start_time) * 1000000 < microseconds:
            pass

    def move(self, direction, distance):
        # Set motor direction, 1 down, 0 up
        if direction == 1:
            direction = 0
        elif direction == -1:
            direction = 1
        else:
            raise ValueError("Invalid direction value, must be 1 or -1")
        GPIO.output(self.dir_pin, direction)

        # Convert distance to steps
        total_steps = self.distance2step(distance)

        # Calculate acceleration and deceleration steps
        acceleration_time = 1000    # in microseconds
        acceleration_steps = int(0.5 * (self.max_speed_delay ** 2) / (acceleration_time ** 2))
        deceleration_steps = acceleration_steps
        constant_speed_steps = total_steps - acceleration_steps - deceleration_steps

        delay = self.max_speed_delay  # Start with max speed delay
        
        # Acceleration
        for step in range(acceleration_steps):
            current_speed_delay = self.max_speed_delay * (step / acceleration_steps)
            delay = max(self.max_speed_delay, current_speed_delay)
            self._step(delay)

        # Constant speed
        for step in range(constant_speed_steps):
            self._step(self.max_speed_delay)

        # Deceleration
        for step in range(deceleration_steps):
            current_speed_delay = self.max_speed_delay * ((deceleration_steps - step) / deceleration_steps)
            delay = max(self.max_speed_delay, current_speed_delay)
            self._step(delay)

        time.sleep(0.5)

    def rotate(self, direction, distance):
        # Set motor direction, 1 down, 0 up
        if direction == 1:
            direction = 0
        elif direction == -1:
            direction = 1
        else:
            raise ValueError("Invalid direction value, must be 1 or -1")
        GPIO.output(self.dir_pin, direction)

        # Convert distance to steps
        steps = self.distance2step(distance)

        # Control motor speed and steps
        for i in range(steps):
            if i < steps / 2:
                delay = self.min_speed_delay - (self.min_speed_delay - self.max_speed_delay) * (i / (steps / 2))
            else:
                delay = self.max_speed_delay + (self.min_speed_delay - self.max_speed_delay) * ((i - steps / 2) / (steps / 2))

            GPIO.output(self.step_pin, GPIO.HIGH)
            self.delay_microseconds(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            self.delay_microseconds(delay)

        time.sleep(0.5)

# Example usage

if __name__ == "__main__":
    motor = StepperMotor()
    motor.rotate(direction=1, distance=5)  # Example call to rotate the motor
    motor.move(direction=1, distance=10)  # Example call to rotate the motor
