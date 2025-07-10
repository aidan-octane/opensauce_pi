import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
from keys import MQTT_URL, MQTT_USERNAME, MQTT_PASSWORD

#Start cv2 video capturing through CSI port
cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)

#Initialise Media Pipe Pose features
mp_pose=mp.solutions.pose
mpDraw=mp.solutions.drawing_utils
pose=mp_pose.Pose()

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
        else:
            print(f"Failed to connect to MQTT broker. Reason code: {reason_code}")

    def on_message(self, client, userdata, msg):
        print(f"Received message: {msg.topic} {msg.payload.decode()}")

    def connect(self):
        try:
            print("Connecting to MQTT broker...")
            self.client.connect(MQTT_URL, 8883, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")

mqtt_client = MQTTClient()
mqtt_client.connect()

#Start endless loop to create video frame by frame Add details about video size and image post-processing to better identify bodies
def pose_tracking():
    # print("Pose tracking process started!")
    # while True:
    #     ret,frame=cap.read()
    #     flipped=cv2.flip(frame,flipCode=1)
    #     frame1 = cv2.resize(flipped,(640,480))
    #     rgb_img=cv2.cvtColor(frame1,cv2.COLOR_BGR2RGB)
    #     result=pose.process(rgb_img)
    #     #Print general details about observed body
    #     landmarks = result.pose_landmarks
        
    #     if landmarks:
    #         x_value = 1 - landmarks.landmark[mp_pose.PoseLandmark.NOSE].x
    #         if mqtt_client.connected:
    #             mqtt_client.client.publish("/body_x", x_value)
    #             print("Published data: " + str(x_value))
    #         else:
    #             print("Couldn't publish data. MQTT client not connected.")
    
    #     #Draw the framework of body onto the processed image and then show it in the preview window
    #         mpDraw.draw_landmarks(frame1,result.pose_landmarks,mp_pose.POSE_CONNECTIONS)
    #         cv2.putText(frame1, "NOSE X: " + str(x_value), (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    #     else:
    #         print("No body found!")
    
    #     cv2.imshow("frame",frame1)
        
    #     # #At any point if the | q | is pressed on the keyboard then the system will stop
    #     key = cv2.waitKey(1) & 0xFF
    #     if key ==ord("q"):
    #         break


    mp_face_detection = mp.solutions.face_detection
    mp_drawing = mp.solutions.drawing_utils

    with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.4) as face_detection:
        x_center = -1
        while True:
                ret, frame = cap.read()
            
                # Convert to RGB
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = face_detection.process(image)

                if results.detections:
                    for detection in results.detections:
                        bbox = detection.location_data.relative_bounding_box
                        x_center = bbox.xmin + bbox.width / 2.0
                        cv2.putText(frame, "Face X: " + str(float(f"{x_center:.2f}")), (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)    

                        # print(f"X position of face (normalized): {x_center:.2f}")
                        if mqtt_client.connected:
                            mqtt_client.client.publish("/body_x", x_center)
                            print("Published data: " + str(x_center))
                        else:
                            print("Couldn't publish data. MQTT client not connected.")
                        mp_drawing.draw_detection(frame, detection)
                else: # publish 0.5 if no face detected
                     x_center = 0.5
                     if mqtt_client.connected:
                            mqtt_client.client.publish("/body_x", x_center)
                            print("Published data: " + str(x_center))
                     else:
                            print("Couldn't publish data. MQTT client not connected.")
                cv2.imshow("frame", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

    cap.release()
    cv2.destroyAllWindows()
                    

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/BODY_POSITION")


if __name__ == "__main__":
    print("Running auto aim camera!")
    # mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    # mqttc.on_connect = on_connect
    # mqttc.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    # mqttc.client.tls_set()
    # mqttc.connect(MQTT_URL, 8883, 30)
    # mqttc.loop_start()
    pose_tracking()