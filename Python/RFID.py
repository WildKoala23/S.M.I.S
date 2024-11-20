import requests
import json
from time import sleep

post_url = "http://localhost:8000/products/create"
myobj = {'quantity': 2, 'desc':"DHT-22"}

post_request = requests.post(post_url, myobj)
print(post_request.text)

sleep(2)

response = requests.get("http://localhost:8000/products/list")
json_data = json.loads(response.text)['data']

for data in json_data:
    print(f"ID: [{data['id']}] Quantity: [{data['quantity']}] Description: [{data['desc']}]")


