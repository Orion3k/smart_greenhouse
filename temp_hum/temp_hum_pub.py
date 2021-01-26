#!/usr/bin/python3
import paho.mqtt.client as mqtt
import time
import Adafruit_DHT
from datetime import datetime
from gpiozero import LED

def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))

dht11 = Adafruit_DHT.DHT11
dht11_pin = 4
nebulizer = LED(27)

client = mqtt.Client()
client.on_connect = on_connect
client.connect("localhost", 1881, 60)
client.loop_start()

last_msg_pub = datetime.now()
while True:
	humidity,temperature = Adafruit_DHT.read_retry(dht11,dht11_pin,retries=4,delay_seconds=1)
	if (humidity is not None and temperature is not None):
		print("T: ",temperature,"H: ",humidity)
		client.publish("greenhouse/temperature", temperature)
		time.sleep(30.)
		client.publish("greenhouse/humidity",humidity)
		last_msg_pub = datetime.now()
		time.sleep(30.)
	else:
		time.sleep(2.0)
