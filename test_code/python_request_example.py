import requests

URL = 'http://192.168.0.3'
response = requests.get(URL)
print(response.status_code)
print(response.text)