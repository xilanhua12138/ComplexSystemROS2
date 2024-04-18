import requests
import json
from typing import List

API_KEY = "EoJIpkw0iiKOCP9G3wewpG4O"
SECRET_KEY = "DeYpGKwkFiogXmLUEodoQQWIAcXrf8Qg"

def get_access_token():
    url = "https://aip.baidubce.com/oauth/2.0/token"
    params = {"grant_type": "client_credentials", "client_id": API_KEY, "client_secret": SECRET_KEY}
    return str(requests.post(url, params=params).json().get("access_token"))

def request_api(messages):
    url = "https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/completions?access_token=" + get_access_token()
    payload =  json.dumps({
        "messages": messages,
        "disable_search": False,
        "enable_citation": False
    })
    headers = {'Content-Type': 'application/json'}
    response = requests.request("POST", url, headers=headers, data=payload)
    text_result = json.loads(response.text)['result']

    return text_result