import serial

class HardwareUART:
    """Hardware UART interface for high-speed communication"""
    def __init__(self, device='/dev/ttyAMA3', baudrate=115200, parity=serial.PARITY_ODD):
        self.device = device
        self.baudrate = baudrate
        self.parity = parity
        self.serial = None
        
    def setup_hardware_uart(self):
        """Configure hardware UART with optimal settings"""
        try:
            self.serial = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                parity=self.parity,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1,
                write_timeout=0.1,
                # Hardware flow control
                rtscts=False,
                dsrdtr=False
                )
            # Configure buffers for high throughput
            #self.serial.set_buffer_size(rx_size=4096, tx_size=4096)
            print("UART setup successful")
            return True
        
        except serial.SerialException as e:
            print(f"UART setup failed: {e}")
            return False
    
if __name__ == '__main__':
    uart = HardwareUART('/dev/ttyAMA3', 115200, parity=serial.PARITY_ODD)
    print(uart)
    if uart.setup_hardware_uart():
        print("hello")
        uart.serial.write(b'AA')
       