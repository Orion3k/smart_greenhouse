#!/usr/bin/python3
import paho.mqtt.client as mqtt
from gpiozero import PWMLED

def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))

led_strip_topic = "greenhouse/led_strip/#"
led_strip = {
	"greenhouse/led_strip/red" : PWMLED(21),
	"greenhouse/led_strip/green" : PWMLED(19),
	"greenhouse/led_strip/blue" : PWMLED(23)
	}

def on_subscribe(client, userdata, msg):
	# on/off led strip
	value = float(msg.payload.decode())
	print(f"led strip message received from {msg.topic}: {value}")
	print(led_strip[msg.topic].value)
	led_strip[msg.topic].value = value / 100.

def run():
	client = mqtt.Client()
	client.on_connect = on_connect
	client.connect("localhost", 1881, 60)
	# subscribe to led strip topic
	client.subscribe(led_strip_topic)
	client.on_message = on_subscribe
	# keep spinning subs
	client.loop_forever()

if __name__ == '__main__':
   	run()
