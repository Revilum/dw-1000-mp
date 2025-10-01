#!/usr/bin/env python3
"""
DW1000 Step-by-Step Initialization Diagnostic

This script tests each initialization step individually to identify
exactly what prerequisite is missing for successful RX enable.
"""

import time
import dw1000
from machine import SPI, Pin
import machine

# Initialize SPI and DW1000
spi = SPI(0, baudrate=1000000)
cs_pin = machine.Pin(17, machine.Pin.OUT)
reset_pin = machine.Pin(21, machine.Pin.OUT)
irq_pin = machine.Pin(20, machine.Pin.IN)

print("=== DW1000 STEP-BY-STEP INITIALIZATION DIAGNOSTIC ===")

def check_register(name, reg_id, expected_bits=None):
    """Check a register and report its value"""
    try:
        value = dw.read_register(reg_id)
        print(f"{name}: 0x{value:08X}")
        if expected_bits:
            for bit_name, bit_mask in expected_bits.items():
                if value & bit_mask:
                    print(f"  ✓ {bit_name} bit SET")
                else:
                    print(f"  ✗ {bit_name} bit CLEAR")
        return value
    except Exception as e:
        print(f"{name}: ERROR - {e}")
        return None

def test_rx_enable_step():
    """Test RX enable with detailed checking"""
    print("\n--- Testing RX Enable Sequence ---")
    
    # Check initial state
    print("1. Pre-enable state:")
    sys_ctrl_before = check_register("SYS_CTRL", 0x0D, {"RXENAB": 0x0100})
    sys_state_before = check_register("SYS_STATE", 0x19)
    
    # Force off first
    print("\n2. Force TRX off:")
    dw.force_trx_off()
    time.sleep_ms(5)
    check_register("SYS_CTRL after force_off", 0x0D, {"RXENAB": 0x0100})
    
    # Clear status
    print("\n3. Clear status:")
    status_before = check_register("SYS_STATUS before clear", 0x0F)
    dw.get_status()  # This should clear status in our implementation
    status_after = check_register("SYS_STATUS after clear", 0x0F)
    
    # Reset RX
    print("\n4. RX reset:")
    dw.rx_reset()
    time.sleep_ms(2)
    check_register("SYS_STATE after rx_reset", 0x19)
    
    # Sync buffers
    print("\n5. Sync RX buffers:")
    dw.sync_rx_bufptrs()
    time.sleep_ms(1)
    check_register("RX_FINFO after sync", 0x10)
    
    # Try to enable RX
    print("\n6. Enable RX:")
    try:
        # Call the raw dwt_rxenable equivalent - direct register write
        print("Writing SYS_CTRL RXENAB bit directly...")
        
        # Read current SYS_CTRL
        sys_ctrl = dw.read_register(0x0D)
        print(f"SYS_CTRL before: 0x{sys_ctrl:08X}")
        
        # Set RXENAB bit (0x0100)
        new_sys_ctrl = sys_ctrl | 0x0100
        print(f"Writing SYS_CTRL: 0x{new_sys_ctrl:08X}")
        
        # This is what dwt_rxenable() should do internally
        # We can't write directly, but we can call rx_enable and see what happens
        result = dw.rx_enable()
        print(f"rx_enable() returned: {result}")
        
    except Exception as e:
        print(f"RX enable FAILED: {e}")
        return False
    
    # Check final state
    print("\n7. Post-enable state:")
    sys_ctrl_after = check_register("SYS_CTRL after enable", 0x0D, {"RXENAB": 0x0100})
    sys_state_after = check_register("SYS_STATE after enable", 0x19)
    
    # Analyze results
    print("\n--- ANALYSIS ---")
    if sys_ctrl_after and (sys_ctrl_after & 0x0100):
        print("✓ SUCCESS: RXENAB bit is SET")
        return True
    else:
        print("✗ FAILURE: RXENAB bit is NOT SET")
        print("  This means dwt_rxenable() was rejected by hardware")
        return False

# Step 1: Initialize DW1000
print("\nStep 1: Basic Initialization")
dw = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
dw.init()
device_id = dw.read_device_id()
print(f"Device ID: 0x{device_id:08X}")

if device_id != 0xDECA0130:
    print("ERROR: Invalid device ID")
    exit()

# Step 2: Check initial register state
print("\nStep 2: Initial Register State")
check_register("SYS_STATUS", 0x0F)
check_register("SYS_STATE", 0x19)
check_register("SYS_CTRL", 0x0D, {"RXENAB": 0x0100})
check_register("RX_FINFO", 0x10)

# Step 3: Configure DW1000
print("\nStep 3: Configure DW1000")
dw.configure()
print("Configuration complete")

# Check post-config state
print("\nPost-configuration state:")
check_register("SYS_STATUS", 0x0F)
check_register("SYS_STATE", 0x19)
check_register("SYS_CTRL", 0x0D, {"RXENAB": 0x0100})

# Step 4: Test RX enable
print("\nStep 4: RX Enable Test")
success = test_rx_enable_step()

# Step 5: If failed, try alternative approaches
if not success:
    print("\n=== TRYING ALTERNATIVE APPROACHES ===")
    
    print("\nAlternative 1: Re-initialize completely")
    dw.init()
    dw.configure()
    success = test_rx_enable_step()
    
    if not success:
        print("\nAlternative 2: Check for missing setup steps")
        print("This suggests a fundamental configuration issue")
        print("Possible causes:")
        print("- Missing clock setup")
        print("- Missing PLL configuration") 
        print("- Missing LDO setup")
        print("- Hardware timing issue")
        print("- SPI communication problem")

print("\n=== DIAGNOSTIC COMPLETE ===")
if success:
    print("✓ DW1000 RX enable is working correctly")
else:
    print("✗ DW1000 RX enable is failing - configuration issue identified")