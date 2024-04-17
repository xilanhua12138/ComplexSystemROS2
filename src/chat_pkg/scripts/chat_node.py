#!/usr/bin/env python3

import rclpy
from   rclpy.node    import Node
from   std_msgs.msg  import String
from chat_module import chat
from prompt import sys_prompt
import re

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
        return

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
        msg.data = emotion
        self.publisher_for_emotion.publish(msg)

        return

def main(args=None):
    rclpy.init(args=args)

    waffle = Chat()
    rclpy.spin(waffle)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:

        rclpy.shutdown()

        print(cname+" Cancelelld by user !")
