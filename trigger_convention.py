import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from keys import MQTT_URL, MQTT_USERNAME, MQTT_PASSWORD
import signal
import pigpio
import time
import threading
# Global stage flag

# This file fires the cannon. The cannon must be armed to fire, at which point the green LED 
# on the trigger control board will light. Only when this LED is lit may the cannon fire. 
# The cannon can fire from a second button press OR from a wireless trigger signal via MQTT.



SOLENOID_PIN = 16
MAGNET_PIN = 12 # PWM-capable pin
BUTTON_PIN = 20
LED_PIN = 21

PWM_FREQ = 1000
PWM_DUTYCYCLE = 255
ready_to_fire = False
# Initializations for GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(SOLENOID_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Change depending on whether or not we choose NC or NO for button
GPIO.setup(LED_PIN, GPIO.OUT)

# pigpio for PWN w/ the magnet
PI = pigpio.pi()
PI.hardware_PWM(MAGNET_PIN, PWM_FREQ, PWM_DUTYCYCLE)
if not PI.connected:
    print("Failed to connect to pigpio daemon")
    exit()

# To set to 10% power:
# pi.set_PWM_dutycycle(MAGNET_PIN, int(0.10 * 255))
# To set to full power:
# pi.set_PWM_dutycycle(MAGNET_PIN, 255)
# To turn off:
# pi.set_PWM_dutycycle(MAGNET_PIN, 0)


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
            print("Connected to MQTT broker. Subscribing to endpoints: ")
            self.connected = True
            self.client.subscribe("/pull_back")
            self.client.subscribe("/trigger")
            i=0
            while i<6:
                GPIO.output(LED_PIN, GPIO.HIGH)
                time.sleep(0.25)
                GPIO.output(LED_PIN, GPIO.LOW)
                time.sleep(0.25)
                i += 1
        else:
            print(f"Failed to connect to MQTT broker. Reason code: {reason_code}")

    def on_message(self, client, userdata, msg):
        global ready_to_fire
        # print(f"Received message: {msg.topic} {msg.payload.decode()}")
        if msg.topic == "/trigger":
            print("Received trigger message! Ready to fire: " + ("TRUE" if ready_to_fire else "FALSE"))
            if ready_to_fire:
                ready_to_fire = False
                trigger_cannon()         

    def connect(self):
        try:
            print("Connecting to MQTT broker...")
            self.client.connect(MQTT_URL, 8883, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")

mqtt_client = MQTTClient()
mqtt_client.connect()



def wait_for_button():
    global ready_to_fire
    print("Program began! Waiting for button press.")
    while True:
        button_status = GPIO.input(BUTTON_PIN)
        if button_status == GPIO.LOW:
            print("Button pressed! Ready to fire: " + ("TRUE" if ready_to_fire else "FALSE"))
            if ready_to_fire:
                ready_to_fire = False
                try:
                    mqtt_client.client.publish("/cannon_fired", "null")
                except Exception as e:
                    print("MQTT publishing failed!")
                trigger_cannon()
            else:
                ready_to_fire = True
                GPIO.output(LED_PIN, GPIO.HIGH)
                try:
                    mqtt_client.client.publish("/button", "null")
                except Exception as e:
                    print("MQTT publishing failed!")

                time.sleep(1)
            while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                time.sleep(0.05)

def trigger_cannon():
    global cooldown
    # Activate magnet
    PI.set_PWM_dutycycle(MAGNET_PIN, PWM_DUTYCYCLE)
    time.sleep(0.1) # Note - not necessary
    # 2. Activate solenoid
    GPIO.output(SOLENOID_PIN, GPIO.HIGH)
    time.sleep(0.5)
    # 4. Let go of magnet, launching tomato
    PI.set_PWM_dutycycle(MAGNET_PIN, 0)
    time.sleep(0.25)
    GPIO.output(SOLENOID_PIN, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.LOW)
    print("Cannon activation sequence complete! Entering cooldown mode.")
    # //// COOLDOWN SEQUENCE ////
    time.sleep(3)
    PI.set_PWM_dutycycle(MAGNET_PIN, (0.2 * PWM_DUTYCYCLE)) # Sets magnet to 20% power to allow reloading
    print("Magnet set to 20% power!")


try:
    PI.set_PWM_dutycycle(MAGNET_PIN, (0.20 * PWM_DUTYCYCLE)) # Sets magnet to 20% power to allow reloading\
    print("Program began! Waiting for button press.")
    mqtt_client.client.loop_start()
    wait_for_button()
except Exception as e:
    print(str(e))
finally:
    
    GPIO.cleanup()
    PI.set_PWM_dutycycle(MAGNET_PIN, 0)

    PI.stop()
    
    print("Cleaned up - exiting")