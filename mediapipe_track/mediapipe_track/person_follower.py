import serial 
import time 

arduino = serial.Serial(
    port='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_7513330323235120C040-if00',
    baudrate=9600,
    timeout=0.1
)

def write_read(x): 
    arduino.write(bytes([x]))  # Send as raw byte
    time.sleep(0.05)
    data = arduino.readline()
    return data

while True: 
    num = int(input("Enter a number (0-255): "))  # Ensure it's in byte range
    if 0 <= num <= 255:
        value = write_read(num)
        print("Received:", value)
    else:
        print("Please enter a number between 0 and 255.")
