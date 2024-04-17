#!/usr/bin/env python3

import rclpy
from   rclpy.node    import Node
from   std_msgs.msg  import String
from chat_module import chat
from prompt import sys_prompt
import re

frame_count = {
    'blink': 39, 'happy': 45, 'sad': 47, 'dizzy': 67, 'excited': 24, 'neutral': 61,
    'happy2': 20, 'angry': 20, 'happy3': 26, 'bootup3': 124, 'blink2': 20
}
cname = "chat_module"

class Chat_Node(Node):

    def __init__(self):
        super().__init__('chat_node')
        self.memory = [{"role":"user", "content":f'{sys_prompt}'}, 
              {"role":"assistant", "content":'neutral@好的，主人'}]
        
        self.subscription = self.create_subscription(
            String, '/hearts/stt', self.listener_callback, 10)

        self.publisher_for_tts = self.create_publisher(
            String, '/hearts/chat_to_tts', 10)
        
        self.publisher_for_emotion = self.create_publisher(
            String, '/hearts/chat_to_emo', 10)


    def listener_callback(self, msg):
        txt = msg.data

        # pass text to get an appropriate answer
        text_result, memory = chat(txt, self.memory)
        emotion = text_result.split('@')[0] 
        text_result = ''.join(text_result.split('@')[1:])
        pattern = re.compile(r'[*#$.""@%^&()+-><、]')
        text_result = pattern.sub('', text_result)

        self.memory = memory

        msg = String()
        msg.data = text_result
        self.publisher_for_tts.publish(msg)

        msg = String()

        times = int((len(text_result)/7) // (frame_count[emotion]*0.05)) + 1
        msg.data = emotion + f'@{times}'
        self.publisher_for_emotion.publish(msg)

        return

def main(args=None):
    rclpy.init(args=args)

    waffle = Chat_Node()
    rclpy.spin(waffle)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:

        rclpy.shutdown()

        print(cname+" Cancelelld by user !")
