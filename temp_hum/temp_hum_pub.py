#!/usr/bin/python3
import paho.mqtt.client as mqtt
import adafruit_dht
import time
import Adafruit_DHT

def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))

dht11 = Adafruit_DHT.DHT11
dht11_pin = 4

client = mqtt.Client()
client.on_connect = on_connect

client.connect("localhost", 1881, 60)

client.loop_start()
while True:
	try:
		humidity,temperature = Adafruit_DHT.read_retry(dht11,dht11_pin,retries=4,delay_seconds=1)
		print("T: ",temperature,"H: ",humidity)
		client.publish("greenhouse/temperature", temperature)
		time.sleep(30.)
		client.publish("greenhouse/humidity",humidity)
	except RuntimeError as error:
		print(error.args[0])
		time.sleep(2.0)
		continue
	except Exception as Error:
		raise error
	time.sleep(900.0)

