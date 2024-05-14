#!/usr/bin/python3

import sys, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import soundfile as sf
import sounddevice as sd
import numpy as np
import sherpa_onnx
from typing import Tuple
from collections import defaultdict
import threading
import time
import librosa
def load_audio(filename: str) -> Tuple[np.ndarray, int]:
    samples, sample_rate = librosa.load(filename,sr=None)
    return samples, sample_rate

def load_speaker_file(speaker_file):
    ans = defaultdict(list)
    with open(speaker_file) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            fields = line.split()
            if len(fields) != 2:
                raise ValueError(f"Invalid line: {line}. Fields: {fields}")

            speaker_name, filename = fields
            ans[speaker_name].append(filename)
    return ans

def compute_speaker_embedding(filenames, extractor: sherpa_onnx.SpeakerEmbeddingExtractor,):
    assert len(filenames) > 0, "filenames is empty"

    ans = None
    for filename in filenames:
        print(f"processing {filename}")
        samples, sample_rate = load_audio(filename)
        stream = extractor.create_stream()
        stream.accept_waveform(sample_rate=sample_rate, waveform=samples)
        stream.input_finished()

        assert extractor.is_ready(stream)
        embedding = extractor.compute(stream)
        embedding = np.array(embedding)
        if ans is None:
            ans = embedding
        else:
            ans += embedding

    return ans / len(filenames)

cname = "stt_module: "
class STT_Node(Node):

    def __init__(self):
        super().__init__('STT_Node')
        self.name = 'User'
        self.publisher = self.create_publisher(String, '/hearts/stt', 10)
        self.subscriber = self.create_subscription(Bool, '/hearts/tts_is_playing', self.listen,10)

        self.recognizer = self.create_recognizer()
        self.extractor, self.manager = self.create_speaker_recognition()

        self.sample_rate = 16000
        self.samples_per_read = int(0.1 * self.sample_rate)  # 0.1 second = 100 ms

        self.stream = self.recognizer.create_stream()

        self.is_active = False
        self.tts_is_playing = False

        self.audio_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.audio_thread.start()

    def create_speaker_recognition(self):

        speaker_file = '/root/Voice_Assistant/speaker_audio/speakers.txt'
        config = sherpa_onnx.SpeakerEmbeddingExtractorConfig(
            model='/root/Voice_Assistant/model/speaker_recogition/3dspeaker_speech_eres2net_base_200k_sv_zh-cn_16k-common.onnx',
            num_threads=1,
            debug=False,
            provider='cpu'
        )

        if not config.validate():
            raise ValueError(f"Invalid config. {config}")
        extractor = sherpa_onnx.SpeakerEmbeddingExtractor(config)
        speaker_file = load_speaker_file(speaker_file)
        manager = sherpa_onnx.SpeakerEmbeddingManager(extractor.dim)

        for name, filename_list in speaker_file.items():
            embedding = compute_speaker_embedding(
                filenames=filename_list,
                extractor=extractor,
            )
            status = manager.add(name, embedding)
            if not status:
                raise RuntimeError(f"Failed to register speaker {name}")

        return extractor, manager

    def create_recognizer(self):

        recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens='/root/Voice_Assistant/model/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/tokens.txt',
            encoder='/root/Voice_Assistant/model/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/encoder-epoch-99-avg-1.onnx',
            decoder='/root/Voice_Assistant/model/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/decoder-epoch-99-avg-1.onnx',
            joiner='/root/Voice_Assistant/model/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/joiner-epoch-99-avg-1.onnx',
            num_threads=1,
            sample_rate=16000,
            feature_dim=80,
            enable_endpoint_detection=True,
            rule1_min_trailing_silence=2.4,
            rule2_min_trailing_silence=1.2,
            rule3_min_utterance_length=300,  # it essentially disables this rule
            decoding_method="greedy_search",
            provider="cpu",
            hotwords_file="",
            hotwords_score=1.5,
            blank_penalty=0.0,
        )
        return recognizer

    def recognize_name(self, speaker_buffer, extractor, sample_rate, manager):
        extractor_stream = extractor.create_stream()
        extractor_stream.accept_waveform(
            sample_rate=sample_rate, waveform=speaker_buffer
        )
        extractor_stream.input_finished()
        embedding = extractor.compute(extractor_stream)
        embedding = np.array(embedding)
        name = manager.search(embedding, threshold=0.5)

        if not name:
            name = "User"
        return name

    def listen(self, msg):
        self.tts_is_playing = msg.data

    def process_audio(self):

        pub_msg = String()
        last_result = ''
        speaker_buffer = []

        print('Please say it')
        with sd.InputStream(channels=1, dtype="float32", samplerate=self.sample_rate) as s:
            while True:
                if not self.tts_is_playing:
                    samples, _ = s.read(self.samples_per_read)  # a blocking read
                    samples = samples.reshape(-1)
                    self.stream.accept_waveform(self.sample_rate, samples)

                    speaker_buffer = np.concatenate([speaker_buffer, samples])

                    while self.recognizer.is_ready(self.stream):
                        self.recognizer.decode_stream(self.stream)

                    is_endpoint = self.recognizer.is_endpoint(self.stream)

                    result = self.recognizer.get_result(self.stream)

                    # global current_emotion

                    if result and (last_result != result):
                        last_result = result
                        print("\r{}:{}".format(self.name, result), end="", flush=True)

                    if is_endpoint:
                        if result:
                            if not self.is_active:
                                name = self.recognize_name(speaker_buffer, self.extractor, self.sample_rate, self.manager)
                                speaker_buffer = []
                            print("\r{}:{}".format(name,result), flush=True)

                            if '小白小白' in result and name=='syh':
                                self.is_active = True

                                text_result = "你好，请问有什么需要我帮助的吗"
                                pub_msg.data = result
                                self.publisher.publish(pub_msg)
                                # change_emotion(text_result, 'happy')

                                # play_text_audio(text_result, tts)

                                self.recognizer.reset(self.stream)

                                continue

                            if self.is_active:
                                pub_msg.data = result
                                self.publisher.publish(pub_msg)
                                # text_result, memory = chat(result, memory)
                                # new_emotion = text_result.split('@')[0]

                                # text_result = ''.join(text_result.split('@')[1:])

                                # pattern = re.compile(r'[*#$.""@%^&()+-><、]')
                                # text_result = pattern.sub('', text_result)

                                # change_emotion(text_result, new_emotion)
                                # play_text_audio(text_result, tts)

                        self.recognizer.reset(self.stream)
                else:
                    time.sleep(0.1)
def main(args=None):
    rclpy.init(args=args)

    my_node = STT_Node()

    rclpy.spin(my_node)



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(cname+" Cancelelld by user !")
