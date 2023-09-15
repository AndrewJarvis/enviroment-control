import sys
from machine import Pin,PWM
import network
from picozero import pico_temp_sensor, pico_led
import urequests as requests
from secrets import secrets
from threading import Thread
import ubinascii
import socket
import time
from micropython import const
import random

machine.freq(100000000)
speed=machine.freq()
fan=Pin(3,Pin.OUT)
ultra_son=Pin(4,Pin.OUT)
pwm=PWM(Pin(2))
pwm.freq(25000)
fan.value(0)
ultra_son(0)
rp2.country('GB')
wlan=network.WLAN(network.STA_IF)
wlan.active(True)
wlan.config(pm=0xa11140)
time.sleep(30)
wlan.ifconfig(('192.168.0.37','255.255.255.0','192.168.0.1','1.1.1.1'))
mac=ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
stat=wlan.ifconfig()[0]
count=0
ssid=secrets['ssid']
pw=secrets['pw']
wlan.connect(ssid,pw)
timeout = 10

while timeout > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        pico_led.on()
        break
    timeout -= 1
    if timeout==0:
        while True:
            pico_led.off()
            print('not connected')
    time.sleep(1)

def get_html(html_name):
    with open(html_name, 'r') as file:
        html=file.read()
    return html

addr=socket.getaddrinfo('0.0.0.0', 80)[0][-1]
try:
    s=socket.socket()
except:
    machine.reset()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.settimeout(30) # may need adjusting
s.bind(addr)
s.listen(1)

class SCD4X:
    """
    Based on https://github.com/adafruit/Adafruit_CircuitPython_SCD4X
    Copyright (c) 2021 ladyada for Adafruit Industries
    MIT License
    """
    DEFAULT_ADDRESS=0x62
    DATA_READY=const(0xE4B8)
    STOP_PERIODIC_MEASUREMENT=const(0x3F86)
    START_PERIODIC_MEASUREMENT=const(0x21B1)
    READ_MEASUREMENT=const(0xEC05)

    def __init__(self, i2c_bus, address=DEFAULT_ADDRESS):
        self.i2c = i2c_bus
        self.address = address
        self._buffer = bytearray(18)
        self._cmd = bytearray(2)
        self._crc_buffer = bytearray(2)

        # cached readings
        self._temperature = None
        self._relative_humidity = None
        self._co2 = None

        self.stop_periodic_measurement()

    @property
    def co2(self):
        """Returns the CO2 concentration in PPM (parts per million)
        .. note::
            Between measurements, the most recent reading will be cached and returned.
        """
        if self.data_ready:
            self._read_data()
        return self._co2

    @property
    def temperature(self):
        """Returns the current temperature in degrees Celsius
        .. note::
            Between measurements, the most recent reading will be cached and returned.
        """
        if self.data_ready:
            self._read_data()
        return self._temperature

    @property
    def relative_humidity(self):
        """Returns the current relative humidity in %rH.
        .. note::
            Between measurements, the most recent reading will be cached and returned.
        """
        if self.data_ready:
            self._read_data()
        return self._relative_humidity

    def _read_data(self):
        """Reads the temp/hum/co2 from the sensor and caches it"""
        self._send_command(self.READ_MEASUREMENT, cmd_delay=0.001)
        self._read_reply(self._buffer, 9)
        self._co2 = (self._buffer[0] << 8) | self._buffer[1]
        temp = (self._buffer[3] << 8) | self._buffer[4]
        self._temperature = -45 + 175 * (temp / 2 ** 16)
        humi = (self._buffer[6] << 8) | self._buffer[7]
        self._relative_humidity = 100 * (humi / 2 ** 16)

    @property
    def data_ready(self):
        """Check the sensor to see if new data is available"""
        self._send_command(self.DATA_READY, cmd_delay=0.001)
        self._read_reply(self._buffer, 3)
        return not ((self._buffer[0] & 0x03 == 0) and (self._buffer[1] == 0))

    def stop_periodic_measurement(self):
        """Stop measurement mode"""
        self._send_command(self.STOP_PERIODIC_MEASUREMENT, cmd_delay=0.5)

    def start_periodic_measurement(self):
        """Put sensor into working mode, about 5s per measurement"""
        self._send_command(self.START_PERIODIC_MEASUREMENT, cmd_delay=0.01)

    def _send_command(self, cmd, cmd_delay=0.0):
        self._cmd[0] = (cmd >> 8) & 0xFF
        self._cmd[1] = cmd & 0xFF
        self.i2c.writeto(self.address, self._cmd)
        time.sleep(cmd_delay)

    def _read_reply(self, buff, num):
        self.i2c.readfrom_into(self.address, buff, num)
        self._check_buffer_crc(self._buffer[0:num])

    def _check_buffer_crc(self, buf):
        for i in range(0, len(buf), 3):
            self._crc_buffer[0] = buf[i]
            self._crc_buffer[1] = buf[i + 1]
            if self._crc8(self._crc_buffer) != buf[i + 2]:
                raise RuntimeError("CRC check failed while reading data")
        return True

    @staticmethod
    def _crc8(buffer):
        crc = 0xFF
        for byte in buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc & 0xFF  # return the bottom 8 bits

co2_ppm=0
co2_ppmone=0
humidity_percent=0
temp_deg=0
humidtarget=0
droprate=0
co2btarget=0
flag=0
co2target=0

def data():
    global scd4x,co2_ppm,temp_deg,humidity_percent,neartarg,co2target
    i2c0=machine.I2C(0,sda=machine.Pin(0),scl=machine.Pin(1),freq=100000)
    scd4x=SCD4X(i2c0)
    scd4x.start_periodic_measurement()
    count=1
    humiddiff=0
    while True:
        time.sleep(60)
        count-=1
        if count==0:
            break
    while True:
        f=open('humidity_target.txt','r')
        humidtarget=f.readline()
        humidtarget=float(humidtarget)
        co2target=int(co2target)
        pico_led.off()
        humidity_percent=scd4x.relative_humidity
        time.sleep_ms(2500)
        pico_led.on()
        co2_ppm=scd4x.co2
        time.sleep_ms(2500)
        co2_ppmone=scd4x.co2
        temp_deg=scd4x.temperature
        humidity_rising=scd4x.relative_humidity
        try:
            if co2_ppm>=co2_ppmone:
                droprate=co2_ppm-co2_ppmone
            elif co2_ppmone>=co2_ppm:
                autoraise=co2_ppmone-co2_ppm
                droprate=0
        except:
            continue
        try:
            if co2target<=co2_ppm: # automatically raise and lower co2 max value based on if c02 maxes out before reaching target maybe in a given time rate
                if autoraise==0:
                    co2target=co2_ppm # may need to write to file maybe co2btarget
                onoff=random.randint(0,1)
                ultra_son.value(onoff)
                fan.value(1)
        except Exception as e:
            continue
        if droprate<=2 and co2_ppm<=co2target:
            ultra_son.value(0)
            fan.value(0)
            pwm.duty_u16(0)
        elif droprate>10 and co2_ppm<=co2target: 
                co2btarget=scd4x.co2
                try:
                    z=open('co2btarget.txt','r')
                    oldco2target=z.readline()
                    z.close
                    oldco2target=int(oldco2target)
                    if co2btarget<oldco2target:
                        z=open('co2btarget.txt', 'w') # use with open to cut lines down tidy up
                        z.write(str(co2btarget))
                        z.close()
                except:
                    continue
        if humidity_rising>humidity_percent:
            humiddiff=humidity_rising-humidity_percent
        else:
            humiddiff=0
        if humidity_percent<humidtarget and humiddiff==0:
            ultra_son.value(1)
            time.sleep(10)
            ultra_son.value(0)
            pwm.duty_u16(65025)
            fan.value(1)
            time.sleep_ms(3000)
            fan.value(0)

Thread.daemon=True
Thread(target=data,args=()).start()

if __name__ == "__main__": # change refresh time on web html make it longer
    while True:
        try:
            u=open('co2btarget.txt','r')
            co2btarget=u.readline()
            u.close()
            openfile=0
            co2btarget=int(co2btarget)
        except Exception as e:
            continue
        try:
            cl, addr=s.accept()  
            request=cl.recv(1024)
            request=str(request)
            lowco2=request.find("nco2_action=1")
            raiseco2=request.find("nco2_action=2")
            low=request.find("nhumidity_action=8")
            raisee=request.find("nhumidity_action=9")
        except KeyboardInterrupt:
            sys.exit(0)
        except Exception as e:
            continue
        try:
            f=open('humidity_target.txt','r')
            humidtarget=f.readline()
            f.close()
            humidtarget=float(humidtarget)
            e=open('co2_target.txt','r')
            co2target=e.readline()
            e.close()
            co2target=int(co2target)
        except:
            continue
        if raisee>=519:
            if humidtarget>=95:
                humidtarget+=1
            else:
                humidtarget+=5
            if humidtarget>99.9:
                humidtarget=99.99847
        if low>=519:
            if humidtarget==99.99847:
                humidtarget=humidtarget-0.99847
                humidtarget+=1
            if humidtarget>95:
                humidtarget-=1
            else:
                humidtarget-=5
            if humidtarget<6:
                humidtarget=5
        if raiseco2>=519:
            flag=0
            co2target+=100
            if co2target>=2000:
                co2target=2000
        if lowco2>=519:
            if flag==0:
                if co2target<=co2btarget:
                    difference=co2btarget-co2target
                    flag=1
                    co2target=co2btarget
                    co2target=round(co2target,-2)
                    co2target=co2target+100
                elif co2target>=co2btarget:
                    difference=co2target-co2btarget
                co2target-=100
        try:
            f= open('humidity_target.txt','w')
            f.write(str(humidtarget))
            f.close()
            e= open('co2_target.txt','w')
            e.write(str(co2target))
            e.close()
        except:
            continue
        response = get_html('index.html')
        response = response.replace('AccX',str(co2_ppm))
        response = response.replace('AccY',str(temp_deg))
        response = response.replace('AccZ',str(humidity_percent))
        response = response.replace('lala',str(humidtarget))
        response = response.replace('co2la',str(co2target))
        try:
            cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n') # econnreset error 104
        except:
            print(e)
            continue # maybe reconnect using ai code and send again? if not add to try above
        cl.send(response)
        cl.close()
