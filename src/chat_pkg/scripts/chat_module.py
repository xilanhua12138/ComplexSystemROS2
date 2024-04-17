import requests
import json
from typing import List
from chat_api import request_api 

def chat(text:str, memory:List):
    
    user_message = {"role":"user", "content":f'{text}'}
    
    memory.append(user_message)
    
    text_result = request_api(memory)
    
    assistant_message = {"role":"assistant", "content":f'{text_result}'}
    memory.append(assistant_message)
    
    return text_result, memory


if __name__ == '__main__':
    print(chat('你好',[])[0])
