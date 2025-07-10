from multiprocessing import Process, Value
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from keys import MQTT_URL, MQTT_USERNAME, MQTT_PASSWORD
import time

# Define the number of steps per revolution for NEMA 17
STEPS_PER_REV = 200

# Set the microstepping resolution (e.g., 16 for 1/16 microsteps)
MICROSTEP = 16

# Calculate total steps for a full rotation
TOTAL_STEPS = STEPS_PER_REV * MICROSTEP

# Set the delay between steps (adjust for speed)
STEP_DELAY = 0.001  # milliseconds

BODY_POSITION = 0.5
CURRENT_POSITION = 1
ACTIVATED = False
NEED_TO_MOVE = False
CLOCKWISE = GPIO.LOW
COUNTERCLOCKWISE = GPIO.HIGH
DIRECTION_TO_MOVE = CLOCKWISE

class MQTTClient:
    def __init__(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.tls_set()
        self.connected = False

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            print("Connected to MQTT broker.")
            self.connected = True
            self.client.subscribe("/body_x")
        else:
            print(f"Failed to connect to MQTT broker. Reason code: {reason_code}")

    def on_message(self, client, userdata, msg):
        global BODY_POSITION
        # global CURRENT_POSITION
        BODY_POSITION = float(msg.payload)
            

    def connect(self):
        try:
            print("Connecting to MQTT broker...")
            self.client.connect(MQTT_URL, 8883, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")

mqtt_client = MQTTClient()
mqtt_client.connect()


def motor_control_x():
    print("Motor control process started!")
    STEP_PIN = 24
    DIR_PIN = 25
    ENABLE_PIN = 23
    LEFT_LIMIT = 19
    RIGHT_LIMIT = 26
    global CLOCKWISE
    global COUNTERCLOCKWISE
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(ENABLE_PIN, GPIO.OUT)
    GPIO.setup(LEFT_LIMIT, GPIO.IN) # Left limit switch when looking from perspective of barrel
    GPIO.setup(RIGHT_LIMIT, GPIO.IN) # Right limit switch when looking from perspective of barrel

    global ACTIVATED
    global CURRENT_POSITION
    global NEED_TO_MOVE
    global BODY_POSITION
    global DIRECTION_TO_MOVE

    def move_motor(direction, steps, speed): # TODO: Look into moving into separate process if the overhead of calling a function one-at-a-time makes things slow
    # False -> left, True -> right
        GPIO.output(DIR_PIN, direction)  # 1 for forward, 0 for backward
        for i in range(steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(speed)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(speed)

    LEFT_BOUND = 0 # Left coordinate, normalized
    RIGHT_BOUND = 1 # Right coordinate, normalized
    STEP_DISTANCE = 0 # Number of steps to go from far left to far right

    ###### Startup sequence ######
    left_switch_state = GPIO.input(LEFT_LIMIT)
    right_switch_state = GPIO.input(RIGHT_LIMIT)
    # Enable motor
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    print("Starting calibration sequence!")
    # Move to the left until hit switch
    while left_switch_state != GPIO.LOW: # TODO: ENSURE normally open switch
        move_motor(CLOCKWISE, 1, STEP_DELAY) # TODO: ENSURE THAT CLOCKWISE MOVES BARREL LEFT
        left_switch_state = GPIO.input(LEFT_LIMIT)

    # Now we are at the full left of the machine, or x-coordinate (0)
    # Move to the right until hit switch
    while right_switch_state != GPIO.LOW: # TODO: ENSURE normally open switch
        move_motor(COUNTERCLOCKWISE, 1, STEP_DELAY) # TODO: ENSURE THAT COUNTERCLOCKWISE MOVES BARREL RIGHT
        right_switch_state = GPIO.input(RIGHT_LIMIT)
        STEP_DISTANCE += 1

    # Now, STEP_DISTANCE is the number of steps between LEFT BOUND and RIGHT BOUND
    print("STEP DISTANCE: " + str(STEP_DISTANCE))

    CURRENT_POSITION = RIGHT_BOUND
    STEP_VALUE = 1.0 / STEP_DISTANCE # Normalizes to a value 0-1 - Could also multiply the BODY VALUE by STEP_DISTANCE, since that is 0-1 
    SMOOTHING_THRESHOLD = 0.02 # TODO: Tune smoothing threshold
    SAFETY_THRESHOLD = 0.05 # Threshold for how far away from it's limit it can g

    # Center within frame
    while abs(CURRENT_POSITION - 0.5) > 0.05:
        move_motor(CLOCKWISE, 1, STEP_DELAY)
        CURRENT_POSITION = CURRENT_POSITION - STEP_VALUE
        print("CURRENT_POSITION: " + str(CURRENT_POSITION))

    print((((((("Beginning body tracking loop in 3 seconds!")))))))
    time.sleep(3)
    ACTIVATED = True
    # speed definitions
    SUPER_SLOW = 0.05
    SLOW = 0.005
    MODERATE = 0.001
    FAST = 0.001

    ###### Body tracking loop ######

    # TODO: Identify if motor has to be disabled whenever possible in order to reduce overheating - currently just always-on because torque may be necessary
        # TMC2209 has current-scaling modes, but requires UART
    try:
        while True:
            global BODY_POSITION
            # print("CURRENT POSITION: " + str(CURRENT_POSITION))
            print("BODY POSITION: " + str(BODY_POSITION))
            # Distance from detected face to the middle of the frame
            BODY_DIFFERENCE = abs(BODY_POSITION - 0.5)
            if (BODY_DIFFERENCE > SMOOTHING_THRESHOLD): # If the body's location is away from 0.5
                # direction = to the left if cannon aiming to the right of where it wants to be  - CLOCKWISE = LEFT, COUNTER = RIGHT
                DIRECTION = CLOCKWISE if BODY_POSITION > 0.5 else COUNTERCLOCKWISE
                # Bounds check 
                if (DIRECTION == CLOCKWISE and CURRENT_POSITION > LEFT_BOUND + SAFETY_THRESHOLD) or (DIRECTION == COUNTERCLOCKWISE and CURRENT_POSITION < RIGHT_BOUND - SAFETY_THRESHOLD):
                    print("Moving motor 1 step to " + ("LEFT" if DIRECTION==CLOCKWISE else "RIGHT"))
                    if BODY_DIFFERENCE > 0.3:
                        move_motor(DIRECTION, 1, FAST)
                    elif BODY_DIFFERENCE > 0.1:
                        move_motor(DIRECTION, 1, MODERATE)
                    elif BODY_DIFFERENCE > 0.05:
                        move_motor(DIRECTION, 1, SLOW)
                    else:
                        move_motor(DIRECTION, 1, SUPER_SLOW)
                    CURRENT_POSITION = (CURRENT_POSITION - STEP_VALUE) if DIRECTION == CLOCKWISE else (CURRENT_POSITION + STEP_VALUE) 
                else:
                    print("REACHED EDGE OF BOUNDS! CANNOT MOVE ANY FURTHER!")

            # time.sleep(0.015)

    finally:
        print("Program exited!")
        # Disable motor
        GPIO.output(ENABLE_PIN, GPIO.HIGH)
        GPIO.cleanup()
        


if __name__ == "__main__":
    print("Motor auto-aiming program started!")
    motor_control_x()