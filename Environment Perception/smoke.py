#!/usr/bin/python
# encoding:utf-8
import json
import sys
import os
import paho.mqtt.client as mqtt
import time


import RPi.GPIO as GPIO
import time



TASK_TOPIC = "$hw/events/device/" + "smoke" + "/twin/update"                      #mqtt topic

client_id = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))

client = mqtt.Client(client_id, transport='tcp')

client.connect("192.168.1.204", 1883, 60)                                         #mqtt broker ip
client.loop_start()


def clicent_main(message: str):

    time_now = time.strftime('%Y-%m-%d %H-%M-%S', time.localtime(time.time()))
    payload = {"event_id":"","timestamp":0,"twin":{"smoke-status":{"actual": {"value": "%s"%message}, "metadata": {"type":"Updated"}}}}

    client.publish(TASK_TOPIC, json.dumps(payload, ensure_ascii=False))

    return True

pin_pqrs=21                    #The signal output pin of the sensor is connected to the BCM code of the raspberry pie pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_pqrs, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
try:
    while True:
        status = GPIO.input(pin_pqrs)
        if status == False:
            print('1')
            clicent_main("have_smoke")
        else:
            print('0')
            clicent_main("no_smoke")
        time.sleep(0.5)
except KeyboradInterrupt:
    GPIO.cleanup()
