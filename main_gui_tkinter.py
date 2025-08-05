import tkinter as tk
from tkinter import scrolledtext
from PIL import Image, ImageTk, ImageDraw
import subprocess
import signal
import threading
import rclpy
import cv2
import os
from ros_image_subscriber import RosImageSubscriber

class FallDetectionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Fall Detection System")
        self.root.geometry("800x640")
        self.ros_proc = None
        self.running = False
        self.image_received = False

        button_frame = tk.Frame(root)
        button_frame.pack(pady=10)

        self.start_btn = tk.Button(button_frame, text="Start", command=self.start_detection, width=15)
        self.start_btn.pack(side=tk.LEFT, padx=10)

        self.stop_btn = tk.Button(button_frame, text="Stop", command=self.stop_detection, width=15)
        self.stop_btn.pack(side=tk.LEFT, padx=10)

        self.ss_btn = tk.Button(button_frame, text="Screenshots", command=self.open_screenshot_folder, width=15)
        self.ss_btn.pack(side=tk.LEFT, padx=10)

        self.image_label = tk.Label(root)
        self.image_label.pack()

        self.display_waiting_image()

        self.log_box = scrolledtext.ScrolledText(root, height=10, width=100)
        self.log_box.pack(pady=10)

        self.subscriber_thread = threading.Thread(target=self.start_ros_image_node, daemon=True)
        self.subscriber_thread.start()

        self.root.after(1000, self.check_image_timeout)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def display_waiting_image(self):
        img = Image.new('RGB', (640, 480), color='white')
        draw = ImageDraw.Draw(img)
        text = "Waiting for camera..."
        bbox = draw.textbbox((0, 0), text)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        draw.text(((640 - text_width) / 2, (480 - text_height) / 2), text, fill='gray')
        img_tk = ImageTk.PhotoImage(image=img)
        self.image_label.configure(image=img_tk)
        self.image_label.image = img_tk

    def check_image_timeout(self):
        if not self.image_received:
            self.display_waiting_image()
        self.image_received = False
        self.root.after(1000, self.check_image_timeout)

    def start_ros_image_node(self):
        rclpy.init()
        self.ros_image_node = RosImageSubscriber()
        self.ros_image_node.set_callback(self.update_image)
        rclpy.spin(self.ros_image_node)

    def update_image(self, frame):
        self.image_received = True
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)
        img = img.resize((640, 480))
        img_tk = ImageTk.PhotoImage(image=img)
        self.image_label.configure(image=img_tk)
        self.image_label.image = img_tk

    def start_detection(self):
        if self.ros_proc and self.ros_proc.poll() is None:
            self.log_box.insert(tk.END, "Already running...\n")
            return

        self.log_box.insert(tk.END, "Starting ROS2 fall detection launch file...\n")
        self.log_box.see(tk.END)

        self.ros_proc = subprocess.Popen(
            ['bash', '-c', 'source ~/fall_detection_ws/install/setup.bash && ros2 launch human_fall_detection fall_detection.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        threading.Thread(target=self.read_ros_logs, daemon=True).start()

    def read_ros_logs(self):
        for line in iter(self.ros_proc.stdout.readline, b''):
            if not line:
                break
            log = line.decode("utf-8")
            self.log_box.insert(tk.END, log)
            self.log_box.see(tk.END)

    def stop_detection(self):
        if self.ros_proc and self.ros_proc.poll() is None:
            self.log_box.insert(tk.END, "Stopping detection...\n")
            try:
                os.killpg(os.getpgid(self.ros_proc.pid), signal.SIGINT)
                self.ros_proc.wait(timeout=5)
            except Exception as e:
                self.log_box.insert(tk.END, f"Stop failed: {e}\n")
            self.ros_proc = None
        else:
            self.log_box.insert(tk.END, "No running detection process\n")
        self.log_box.see(tk.END)

    def open_screenshot_folder(self): 
        folder_path = os.path.expanduser('~/fall_detection_ws/screenshots')
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        subprocess.call(['xdg-open', folder_path])
  
    def on_close(self):
        self.stop_detection()
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = FallDetectionApp(root)
    root.mainloop()
