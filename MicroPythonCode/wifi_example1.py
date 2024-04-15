# A simple example that:
# - Connects to a WiFi Network defined by "ssid" and "password"
# - Performs a GET request (loads a webpage)
# - Queries the current time from a server

import network   # handles connecting to WiFi
import socket
import urequests # handles making and servicing network requests
from time import sleep
from picozero import pico_temp_sensor, pico_led
import machine

# Fill in your network name (ssid) and password here:
ssid = 'HOME 2.4G'
password = 'surajsuraj'

# Connect to network
def connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        print('Waiting for connection...')
        sleep(1)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    return ip
    
 
try:
    ip = connect()

except KeyboardInterrupt:
    machine.reset()

