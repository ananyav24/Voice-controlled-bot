import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading

class VoicePublisher(Node):
    def __init__(self):
        super().__init__('voice_publisher')
        self.publisher_ = self.create_publisher(String, 'voice_cmd', 10)
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()

        # Calibrate to ambient noise once
        with self.mic as source:
            self.get_logger().info("Calibrating for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=2)

        thread = threading.Thread(target=self.listen_loop, daemon=True)
        thread.start()

    def listen_loop(self):
        while rclpy.ok():
            with self.mic as source:
                self.get_logger().info("Say a command (forward/left/right/stop)...")
                audio = self.recognizer.listen(source, phrase_time_limit=2)
            try:
                text = self.recognizer.recognize_google(audio).lower()
                if text in ["forward", "stop", "left", "right"]:
                    msg = String()
                    msg.data = text
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {text}")
                else:
                    self.get_logger().warn(f"Ignored: {text}")
            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Google API error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoicePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

