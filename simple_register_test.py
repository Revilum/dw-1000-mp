#!/usr/bin/env python3
"""
Simple register-level test for DW1000 SYS_CTRL register
This will help determine if we can write to the register at all
"""

import dw1000
import time

def test_sys_ctrl_register():
    """Test if we can read and manipulate SYS_CTRL register"""
    print("=== SYS_CTRL REGISTER TEST ===")
    
    # Initialize device
    dwt = dw1000.DW1000(None, None)  # SPI=None, CS=None (use defaults)
    dwt.init()
    
    print(f"Device ID: 0x{dwt.get_device_id():08X}")
    
    # Read initial SYS_CTRL
    sys_ctrl_initial = dwt.read_register(0x0D, 4)
    print(f"Initial SYS_CTRL: 0x{sys_ctrl_initial:08X}")
    
    # Try forcing TRX off (this should clear SYS_CTRL)
    print("Forcing TRX OFF...")
    dwt.force_trx_off()
    time.sleep(0.1)
    
    sys_ctrl_after_off = dwt.read_register(0x0D, 4)
    print(f"SYS_CTRL after TRX_OFF: 0x{sys_ctrl_after_off:08X}")
    
    # Check SYS_STATE to see transceiver state
    sys_state = dwt.get_sys_state()
    rx_state = (sys_state >> 8) & 0x1F
    print(f"SYS_STATE: 0x{sys_state:08X} (RX state: {rx_state})")
    
    # Now try the actual dwt_rxenable call and monitor what happens
    print("\nCalling dwt_rxenable()...")
    
    # Call our rx_enable function which calls dwt_rxenable internally
    result = dwt.rx_enable()
    print(f"rx_enable() returned: {result}")
    
    # Check registers immediately after
    sys_ctrl_after_enable = dwt.read_register(0x0D, 4)
    sys_state_after = dwt.get_sys_state()
    
    print(f"SYS_CTRL after rx_enable(): 0x{sys_ctrl_after_enable:08X}")
    print(f"SYS_STATE after rx_enable(): 0x{sys_state_after:08X}")
    
    # Check if RXENAB bit is set
    rxenab_set = (sys_ctrl_after_enable & 0x0100) != 0
    print(f"RXENAB bit set: {rxenab_set}")
    
    if not rxenab_set:
        print("\nREGISTER WRITE FAILURE DETECTED!")
        print("This suggests either:")
        print("1. SPI communication issue")
        print("2. Hardware rejecting the register write")
        print("3. Missing prerequisite for RX enable")
        
        # Let's try reading some other registers to check SPI
        print("\nReading other registers for comparison:")
        device_id = dwt.read_register(0x00, 4)
        sys_cfg = dwt.read_register(0x04, 4)
        sys_status = dwt.read_register(0x0F, 4)
        
        print(f"DEV_ID:     0x{device_id:08X}")
        print(f"SYS_CFG:    0x{sys_cfg:08X}")
        print(f"SYS_STATUS: 0x{sys_status:08X}")
        
        if device_id == 0xDECA0130:
            print("SPI reads are working, so this is a register write issue")
        else:
            print("SPI communication appears to be failing")
    
    return rxenab_set

if __name__ == "__main__":
    try:
        success = test_sys_ctrl_register()
        if success:
            print("\n✓ SYS_CTRL register test PASSED")
        else:
            print("\n✗ SYS_CTRL register test FAILED")
    except Exception as e:
        print(f"\nERROR: {e}")
        # Traceback not available in MicroPython