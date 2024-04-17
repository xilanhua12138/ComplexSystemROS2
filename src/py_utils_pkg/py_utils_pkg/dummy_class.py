#!/usr/bin/env python3

import string
import random

class Dummy(object):

    def __init__(self):
        print("This is a Dummy Class Init ...")
        print("This is a Dummy Class Init .... Done")

    def talk(self):
        letters = string.ascii_lowercase
        random_letters = ''.join(random.choice(letters) for i in range(10))
        print("Class dummy : "+random_letters)
        return random_letters
