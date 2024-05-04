import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import time
from threading import Thread
import os

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.last_time = time.time()
        self.data = []
        self.recording = False
        self.start_time = None

        # Configurable paths and file name
        self.base_path = "/home/loop/Desktop/trajectory_data"  # Adjust this path as needed
        self.recording_details = {}

        # Keyboard thread
        self.keyboard_thread = Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()

    def listener_callback(self, msg):
        if time.time() - self.last_time < 0.04:  # Throttle messages to 25 Hz
            return
        self.last_time = time.time()

        if self.recording:
            self.data.append({
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
                "positions": msg.position,
                "velocities": msg.velocity,
                "efforts": msg.effort
            })

    def keyboard_listener(self):
        try:
            while True:
                key = input("Press 'r' to start/stop recording, 'q' to quit: ")
                if key == 'r':
                    if not self.recording:
                        self.recording_details = self.get_recording_details()
                        self.recording = True
                        self.data = []
                        self.start_time = time.time()
                        print("Recording started...")
                    else:
                        print("Recording is started...")
                elif key == 'q':
                    duration = time.time() - self.start_time
                    print(f"Recording stopped. Duration: {duration:.2f} seconds")
                    self.recording = False
                    self.save_to_csv()

        except Exception as e:
            self.get_logger().info(f"Keyboard listener stopped: {str(e)}")
    

    def get_recording_details(self):
        while True:
            valid_approach = False
            valid_task = False
            valid_count = False
            valid_userid = False

            while not valid_approach:
                approach = input("Kinesthetic teaching or teleoperation? [k/t]: ").lower()
                if approach in ['k', 't']:
                    valid_approach = True
                else:
                    print("Invalid input for approach. Please enter 'k' for kinesthetic or 't' for teleoperation.")

            while not valid_task:
                task = input("Task ID [1-3]: ")
                if task.isdigit() and 1 <= int(task) <= 3:
                    valid_task = True
                else:
                    print("Invalid task ID. Please enter a number from 1 to 3.")

            while not valid_count:
                count = input("Round [1-3]: ")
                if count.isdigit() and 1 <= int(count) <= 3:
                    valid_count = True
                else:
                    print("Invalid round number. Please enter a number from 1 to 3.")

            while not valid_userid:
                userid = input("Participant ID: ")
                if userid.isdigit() and int(userid) > 0:
                    valid_userid = True
                else:
                    print("Invalid participant ID. Please enter a positive integer.")

            approach_str = "kinesthetic" if approach == 'k' else "teleoperation"
            filename = f"task{task}-{count}-{userid}.csv"
            directory = os.path.join(self.base_path, approach_str)
            full_path = os.path.join(directory, filename)

            # Check if file exists and ask user if they want to overwrite
            if os.path.exists(full_path):
                overwrite = input(f"The file '{filename}' already exists. Do you want to overwrite it? [y/n]: ").lower()
                if overwrite == 'y':
                    return {'directory': directory, 'filename': filename}
                else:
                    print("Please re-enter the details.")
            else:
                confirm = input(f"Recording will be saved to '{filename}'. Confirm? [y/n]: ").lower()
                if confirm == 'y':
                    return {'directory': directory, 'filename': filename}
                else:
                    print("Please re-enter the details.")


    def save_to_csv(self):
        if not self.data:
            print("No data to save.")
            return

        directory = self.recording_details['directory']
        filename = self.recording_details['filename']
        os.makedirs(directory, exist_ok=True)
        full_path = os.path.join(directory, filename)

        with open(full_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=["sec", "nanosec", "positions", "velocities", "efforts"])
            writer.writeheader()
            for entry in self.data:
                writer.writerow(entry)

        print(f"Data saved to {full_path}")

def main(args=None):
    rclpy.init(args=args)
    data_recorder = DataRecorder()
    rclpy.spin(data_recorder)

    data_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
