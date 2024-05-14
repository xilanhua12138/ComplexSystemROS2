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
    from screeninfo import get_monitors

    monitors = get_monitors()

    primary_monitor = monitors[0]
    screen_width = primary_monitor.width
    screen_height = primary_monitor.height
    
    for emo in emotion_list:
        one_frames = []
        for i in range(frame_count[emo]):
            image_path = os.path.join('/root/ros_ws/src/hri_emo_pkg/scripts/emotions', emo, f'frame{i}.png')
            original_image = Image.open(image_path)

            image_width, image_height = original_image.size
            if image_width / image_height > screen_width / screen_height:
                new_height = screen_height
                new_width = int(image_width * (new_height / image_height))
                resized_image = original_image.resize((new_width, new_height), Image.ANTIALIAS)
            else:
                new_width = screen_width
                new_height = int(image_height * (new_width / image_width))
                resized_image = original_image.resize((new_width, new_height), Image.ANTIALIAS)

            photo = ImageTk.PhotoImage(resized_image)
            one_frames.append(photo)
        frames_emo[emo] = one_frames
    return frames_emo
current_image = None  
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
        canvas.delete('all')
        
