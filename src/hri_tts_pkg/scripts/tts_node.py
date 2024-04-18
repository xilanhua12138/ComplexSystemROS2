#!/usr/bin/env python3

import rclpy
from   rclpy.node   import Node
from   std_msgs.msg import String,Bool
import os
import sherpa_onnx
import sounddevice as sd
import queue
import threading
import numpy as np
import soundfile as sf
import time
cname = " tts_module"
buffer = queue.Queue()

started = False
stopped = False
killed = False

event = threading.Event()

import subprocess

def mute_microphone():
    subprocess.run(['amixer', 'set', 'Capture', 'nocap'])

def unmute_microphone():
    subprocess.run(['amixer', 'set', 'Capture', 'cap'])


def generated_audio_callback(samples: np.ndarray, progress: float):
    buffer.put(samples)
    
def play_audio_callback(outdata: np.ndarray, frames: int, time, status: sd.CallbackFlags):
    global started, stopped, event
    if (started and buffer.empty() and stopped):
        event.set()
        started = False
        stopped = False

    if buffer.empty():
        outdata.fill(0)
        return

    n = 0
    while n < frames and not buffer.empty():
        remaining = frames - n
        k = buffer.queue[0].shape[0]

        if remaining <= k:
            outdata[n:, 0] = buffer.queue[0][:remaining]
            buffer.queue[0] = buffer.queue[0][remaining:]
            n = frames
            if buffer.queue[0].shape[0] == 0:
                buffer.get()
            break

        outdata[n : n + k, 0] = buffer.get()
        n += k

    if n < frames:
        outdata[n:, 0] = 0

def play_audio():
    with sd.OutputStream(channels=1, callback=play_audio_callback, dtype="float32", samplerate=22050, blocksize=1024,):
        event.wait()
        event.clear()

def play_text_audio(text, tts):
    play_back_thread = threading.Thread(target=play_audio)
    play_back_thread.start()
    audio = tts.generate(text, sid=0, speed=0.9, callback=generated_audio_callback,)
    global started
    started = True
    global stopped
    stopped = True

    if len(audio.samples) != 0:
        sf.write("./generated.wav", audio.samples, samplerate=audio.sample_rate, subtype="PCM_16",)
    play_back_thread.join()

class TTS_Node(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.listen = self.create_subscription(
            String, '/hearts/chat_to_tts',self.listen_callback,10)

        self.pub_is_playing = self.create_publisher(
            Bool, '/hearts/tts_is_playing', 10)

        self.tts = self.create_tts()
        print('tts node is started')

    def create_tts(self):

        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model='/home/xilanhua/Voice_Assistant/model/tts/theresa.onnx',
                    lexicon='/home/xilanhua/Voice_Assistant/model/tts/lexicon.txt',
                    data_dir="",
                    tokens="/home/xilanhua/Voice_Assistant/model/tts/tokens.txt",
                ),
                provider="cpu",
                debug=False,
                num_threads=1,
            ),
            rule_fsts="",
            max_num_sentences=1,
        )

        if not tts_config.validate():
            raise ValueError("Please check your config")

        tts = sherpa_onnx.OfflineTts(tts_config)
        return tts
    
    def listen_callback(self, msg):
        mute_microphone()

        tts_is_playing = Bool()
        text_result = msg.data

        tts_is_playing.data = True
        self.pub_is_playing.publish(tts_is_playing)
        
        play_text_audio(text_result, self.tts)
        
        tts_is_playing.data = False
        self.pub_is_playing.publish(tts_is_playing)

        time.sleep(0.5)
        unmute_microphone()

def main(args=None):
    rclpy.init(args=args)

    tts_node  = TTS_Node()
    rclpy.spin(tts_node)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(cname+" Cancelelld by user !")
