#!/usr/bin/python3
import paho.mqtt.client as mqtt
from gpiozero import PWMLED

def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))

water_pump_topic = "greenhouse/water_pump"
water_pump = PWMLED(22)

def on_subscribe(client, userdata, msg):
	# on/off water pump
	value = float(msg.payload.decode())
	print(f"Water pump message received: {value}")
	water_pump.value = value

def run():
	client = mqtt.Client()
	client.on_connect = on_connect
	client.connect("localhost", 1881, 60)
	# subscribe to water pump topic
	client.subscribe(water_pump_topic)
	client.on_message = on_subscribe
	# keep spinning subs
	client.loop_forever()

if __name__ == '__main__':
   	run()
