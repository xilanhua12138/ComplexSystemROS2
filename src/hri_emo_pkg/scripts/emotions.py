import threading
import os
from PIL import Image, ImageTk
import tkinter as tk
import queue

frame_count = {
    'blink': 39, 'happy': 45, 'sad': 47, 'dizzy': 67, 'excited': 24, 'neutral': 61,
    'happy2': 20, 'angry': 20, 'happy3': 26, 'bootup3': 124, 'blink2': 20
}

emotion_list = ['happy', 'angry', 'sad', 'excited', 'neutral', 'blink2']
current_emotion = 'neutral'

def load_emo_frames():
    frames_emo = {}
    for emo in emotion_list:
        one_frames = []
        for i in range(frame_count[emo]):
            image_path = os.path.join('/home/xilanhua/ros_ws/src/hri_emo_pkg/scripts/emotions', emo, f'frame{i}.png')
            img = Image.open(image_path)
            photo = ImageTk.PhotoImage(img)
            one_frames.append(photo)
        frames_emo[emo] = one_frames
    return frames_emo

def show_emo_thread(frames_emo, emotion_queue, canvas):
    global current_emotion
    while True:
        # 检查队列是否有新的情绪，如果有，更新当前情绪
        if not emotion_queue.empty():
            current_emotion = emotion_queue.get()
        for i in range(frame_count[current_emotion]):
            photo = frames_emo[current_emotion][i]
            canvas.create_image(0, 0, anchor=tk.NW, image=photo)
            canvas.update()
            canvas.after(50)