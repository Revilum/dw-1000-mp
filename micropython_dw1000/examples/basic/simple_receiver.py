"""
Simple Continuous Receiver - Compact version
Just prints received messages without detailed stats

Usage: mpremote connect /dev/ttyACM0 run simple_receiver.py
"""

import dw1000
import machine
import time

# Quick setup
spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0, 
                  sck=machine.Pin(18), mosi=machine.Pin(19), miso=machine.Pin(16))
dwt = dw1000.DW1000(spi, machine.Pin(17, machine.Pin.OUT), 
                    machine.Pin(21, machine.Pin.OUT), machine.Pin(20, machine.Pin.IN))
dwt.init()
dwt.configure({'channel': 2, 'data_rate': dw1000.BR_850K})

# Receiver setup
dwt.force_trx_off()
dwt.rx_reset()
dwt.sync_rx_bufptrs()
dwt.rx_enable()

print("Simple receiver listening...")
frame_count = 0

try:
    while True:
        status = dwt.get_status()
        
        if status & (0x2000 | 0x0400 | 0x8000):  # Any frame activity
            frame_count += 1
            rx_finfo = dwt.get_rx_finfo()
            frame_length = rx_finfo & 0x000003FF
            frame_good = (status & 0x2000) != 0
            
            if frame_good and frame_length > 0:
                try:
                    data = dwt.read_rx_data()
                    try:
                        message = data.decode('utf-8')
                    except:
                        message = str(data)  # Fallback to bytes representation
                    print(f"#{frame_count:04d}: {message}")
                except Exception as e:
                    print(f"#{frame_count:04d}: READ ERROR - {e}")
            else:
                print(f"#{frame_count:04d}: BAD FRAME (Status: 0x{status:08X}, Length: {frame_length})")
            
            # Reset for next frame
            dwt.force_trx_off()
            dwt.rx_reset()
            dwt.sync_rx_bufptrs()
            dwt.rx_enable()
        
        time.sleep_ms(10)

except KeyboardInterrupt:
    print(f"\nStopped. Received {frame_count} frames total.")
    dwt.force_trx_off()
    dwt.deinit()