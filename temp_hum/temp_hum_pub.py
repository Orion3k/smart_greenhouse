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

hum_threshold = 70
last_msg_pub = datetime.now()
while True:
	try:
		humidity,temperature = Adafruit_DHT.read_retry(dht11,dht11_pin,retries=4,delay_seconds=1)
		if (humidity is not None and temperature is not None):
			print("T: ",temperature,"H: ",humidity)

			if (humidity < hum_threshold):
				#nebulizer.on()
				client.publish("greenhouse/humidity/nebulizer", 1, qos=2)
				print('Nebulizer ON')
			else:
				client.publish("greenhouse/humidity/nebulizer", 0, qos=2)
				#nebulizer.off()
				print('Nebulizer OFF')

			if ((datetime.now() - last_msg_pub).total_seconds() > 900. ):
				client.publish("greenhouse/temperature", temperature)
				time.sleep(30.)
				client.publish("greenhouse/humidity",humidity)
				last_msg_pub = datetime.now()
			time.sleep(60.)
	except RuntimeError as error:
		time.sleep(2.0)
		continue
	except Exception as Error:
		raise error

