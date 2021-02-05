# smart_Greenhouse

Let's build your Smart Greenhouse!
Monitor and contorl light, temperature, airflow and humidity in your greenhouse in order to recreate the natural optimal conditions for your plants.

Components:
- Raspberry pi Zero
- DHT11 Temperature/Humidity Sensor
- Led Strip RGB
- PC fan
- Nebulizer
- Transistors / Mosfet
- DC/DC Step down 5V and 12V

Tools to install (on Raspberry pi Zero):
- node-red
  - webhookrelay palette
  - simpletime palette
- mosquitto
- python3
  - paho-mqtt
  - gpiozero
  - Adafruit_DHT
  
Services:
 - ThingSpeak
  
