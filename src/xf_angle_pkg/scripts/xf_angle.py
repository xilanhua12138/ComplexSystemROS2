#!/usr/bin/env python3
import serial 
import json
import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

woshuo = b'\xa5\x01\x01\x04\x00\x00\x00\xa5\x00\x00\x00\xb0'
queren = b'\xA5\x01\xff\x04\x00\x00\x00\xA5\x00\x00\x00\xB2'

len_r = 0
data_list = []
buf = []
buf_flag = 0
first = 0

def deal_file():
    if os.path.getsize('./file.txt') == 0:
        with open("./file.txt", mode='w+') as f:
            f.write("text!")


def deal(data_list):
    global first
    str1 = str(data_list)

    f_data = str1.find('{')
    l_data = str1.rfind('}')
    str1 = str1[f_data:l_data+1]

    str1 = str1.replace("\\", "")
    str1 = str1.replace("', b'", "")
    str1 = str1.replace('"{', "{")
    str1 = str1.replace('}"', "}")

    json_str = json.loads(str1)

    if 'code' in json_str and first == 0:
        sss = json_str['content']
        print(json_str['content'])
        first = 1
    else:
        angle = json_str['content']['info']['ivw']['angle']
        print("angle: ", angle)
        return int(angle)


class RecordAudioNode(Node):
    def __init__(self):
        super().__init__('record_audio_node')
        self.angle_publisher = self.create_publisher(Int32, '/xf_angle', 100)
        self.ser = serial.Serial("/dev/ttyUSB0", 115200, 8, 'N', 1, timeout=5)
        self.buf = []
        self.buf_flag = 0
        self.first = 0
        self.timer = self.create_timer(0.5, self.timer_callback)
        print('xf_angle started')
        
    def timer_callback(self):
        rcv = self.ser.read_all()
        len_r = len(rcv)
        if rcv == woshuo:
            self.ser.write(queren)
        elif len_r > 1:
            self.buf.append(rcv)
            self.buf_flag = 1
        elif len_r < 1 and self.buf_flag == 1:
            self.buf_flag = 0
            data_list = self.buf
            self.buf = []
            angle_msg = deal(data_list)
            self.angle_publisher.publish(Int32(data=angle_msg))


def main(args=None):
    rclpy.init(args=args)
    node = RecordAudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"---Error---: {e}")
    finally:
        node.ser.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
