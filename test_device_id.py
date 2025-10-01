"""
Simple test to check what DEVICE_ID constant is being exposed
"""
import dw1000

print("Testing DW1000 constants:")
print(f"dw1000.DEVICE_ID = 0x{dw1000.DEVICE_ID:08X}")
print(f"Expected: 0xDECA0130")
print(f"Match: {dw1000.DEVICE_ID == 0xDECA0130}")

# Test with actual device reading (minimal test)
import machine

# Use minimal SPI setup for testing
spi = machine.SPI(0, baudrate=1000000)
cs_pin = machine.Pin(17, machine.Pin.OUT)
reset_pin = machine.Pin(21, machine.Pin.OUT)
irq_pin = machine.Pin(20, machine.Pin.IN)
dwm = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
if dwm.init():
    device_id = dwm.read_device_id()
    print(f"Actual device ID: 0x{device_id:08X}")
    print(f"Matches constant: {device_id == dw1000.DEVICE_ID}")
    dwm.deinit()
else:
    print("Failed to initialize device")