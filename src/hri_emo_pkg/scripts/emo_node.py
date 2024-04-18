#!/usr/bin/python3

import sys, os
import rclpy
import queue
from rclpy.node import Node
from std_msgs.msg import String, Bool
from emotions import load_emo_frames, show_emo_thread
import tkinter as tk
import numpy as np
import threading

frame_count = {
    'blink': 39, 'happy': 45, 'sad': 47, 'dizzy': 67, 'excited': 24, 'neutral': 61,
    'happy2': 20, 'angry': 20, 'happy3': 26, 'bootup3': 124, 'blink2': 20
}
emotion_queue = queue.Queue()

cname = "emo_module: "

def _change_emotion(new_emotion):
    global emotion_queue
    emotion_queue.put(new_emotion)

def change_emotion(times, new_emotion):

    for _ in range(times):
        _change_emotion(new_emotion)
    _change_emotion('neutral')

def start_tkinter():
    root = tk.Tk()
    root.title("Emotion Animation")
    canvas = tk.Canvas(root, width=320, height=240)
    canvas.pack()
    
    frames_emo = load_emo_frames()
    
    threading.Thread(target=show_emo_thread, args=(frames_emo, emotion_queue, canvas), daemon=True).start()
    
    root.mainloop()

class EMO_Node(Node):

    def __init__(self):
        super().__init__('EMO_Node')
        self.current_emotion = 'neutral'

        self.subscriber = self.create_subscription(String, '/hearts/chat_to_emo', self.listen,10)


    def listen(self, emo_msg):
        try:
            times, new_emo = emo_msg.data.split('@')
            times = int(times)  # 确保这是一个整数
            change_emotion(times, new_emo)
        except ValueError:
            self.get_logger().error("Received invalid emotion data format: {}".format(emo_msg.data))


def main(args=None):
    rclpy.init(args=args)

    gui_thread = threading.Thread(target=start_tkinter, daemon=True)
    gui_thread.start()

    my_node = EMO_Node()
 
    rclpy.spin(my_node)  



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(cname+" Cancelelld by user !")
