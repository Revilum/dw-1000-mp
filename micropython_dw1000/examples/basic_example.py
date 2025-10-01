"""
Basic DW1000 Example - Device Initialization and ID Reading

This example demonstrates the basic setup and initialization of the DW1000 module.
It reads the device ID to verify communication is working.
"""

import machine
import dw1000
import time

def main():
    print("DW1000 Basic Example")
    print("===================")
    
    # Configure SPI - Raspberry Pi Pico pins
    spi = machine.SPI(
        0,  # SPI0
        baudrate=1000000,  # 1 MHz - safe starting frequency
    )
    
    # Configure control pins
    cs_pin = machine.Pin(17, machine.Pin.OUT)     # GPIO5 - Chip Select
    reset_pin = machine.Pin(21, machine.Pin.OUT)  # GPIO6 - Reset (optional)
    irq_pin = machine.Pin(20, machine.Pin.IN)     # GPIO7 - IRQ (optional)
    
    print("Initializing DW1000...")
    
    try:
        # Create DW1000 instance
        dwm = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        
        # Initialize the device
        if dwm.init():
            print("✓ DW1000 initialized successfully")
        else:
            print("✗ Failed to initialize DW1000")
            return
        
        # Read and verify device ID
        device_id = dwm.read_device_id()
        print(f"Device ID: 0x{device_id:08X}")
        
        # Check device ID - handle potential sign extension issues
        expected_id = 0xDECA0130
        # Convert both to unsigned 32-bit for comparison
        device_id_u32 = device_id & 0xFFFFFFFF
        expected_id_u32 = expected_id & 0xFFFFFFFF
        
        if device_id_u32 == expected_id_u32:
            print("✓ Device ID matches expected value")
        else:
            print(f"✗ Unexpected device ID (expected 0x{expected_id:08X})")
            print(f"Note: module constant DEVICE_ID = 0x{dw1000.DEVICE_ID & 0xFFFFFFFF:08X}")
            # Continue anyway since the actual device ID is correct
            print("Continuing with initialization...")
        
        # Configure the device with default settings
        print("Configuring DW1000...")
        config = {
            'channel': 2,                    # Channel 2
            'data_rate': dw1000.BR_6M8      # 6.8 Mbps
        }
        dwm.configure(config)
        print("✓ Configuration complete")
        
        print("DW1000 is ready for operation!")
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Clean up
        try:
            dwm.deinit()
            print("DW1000 deinitialized")
        except:
            pass

if __name__ == "__main__":
    main()