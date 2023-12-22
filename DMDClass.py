import smbus
import spidev
from PIL import Image, ImageOps
import RPi.GPIO as GPIO
import time

class DigitalMicromirrorDevice:
    def __init__(self):
        self.head1 = [0x04, 0x60, 0x02, 0x00, 0xF1, 0x00, 0x00, 0x40, 0x38, 0x00]
        self.head2 = [0x04, 0x60, 0x02, 0x00, 0xF1, 0x00]
        self.CRC = [0x00, 0x00]
        self.Bus = smbus.SMBus(1)
        self.CS_PIN = 12

        # Initialize GPIO settings
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.CS_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    @staticmethod
    def time_to_hex_pair(time: float) -> tuple:
        num_frame = int(time * 60)
        high, low = divmod(num_frame, 256)
        return low, high

    def img_transfer(self, img):
        # Initialize SPI interface
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.mode = 0b00
        spi.max_speed_hz = 15000000
        spi.lsbfirst = False

        # Disable CRC, select buffer0
        self.Bus.write_byte_data(0x1B, 0xCA, 0x00)
        self.Bus.write_byte_data(0x1B, 0xC5, 0x00)

        # Pull chip select pin low before SPI transfer
        GPIO.setup(self.CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.output(self.CS_PIN, GPIO.LOW)

        im = list(img.getdata())
        im = self.head1 + im + self.CRC

        spi.xfer3(im)

        # Pull chip select pin high after SPI transfer
        GPIO.output(self.CS_PIN, GPIO.HIGH)
        GPIO.setup(self.CS_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Switching to buffer1 and enabling FPGA buffer read and write
        self.Bus.write_byte_data(0x1B, 0xC5, 0x01)
        self.Bus.write_byte_data(0x1B, 0xC3, 0x01)
        spi.close()

    def exposure_start(self, exposure_time, wait_time, sleep_time):
        self.Bus.write_byte_data(0x1B, 0x05, 0x06)
        self.Bus.write_i2c_block_data(0x1B, 0x54, [0x32, 0x00, 0x32, 0x00, 0x6C, 0x02])
        expo_low_byte, expo_high_byte = self.time_to_hex_pair(exposure_time)
        wait_low_byte, wait_high_byte = self.time_to_hex_pair(wait_time)
        self.Bus.write_i2c_block_data(0x1B, 0xC1, [0x00, wait_low_byte, wait_high_byte, expo_low_byte, expo_high_byte])
        time.sleep(sleep_time)
        self.Bus.write_i2c_block_data(0x1B, 0x54, [0x32, 0x00, 0x32, 0x00, 0x64, 0x00])

# Example usage
# dmd = DigitalMicromirrorDevice()
# image = Image.open("path_to_your_image.jpg")  # Replace with your image path
# dmd.img_transfer(image.tobytes())  # Example call to transfer an image
# dmd.exposure_start(5, 3, 2)  # Example call to start exposure
