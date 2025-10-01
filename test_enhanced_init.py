#!/usr/bin/env python3
"""
Test enhanced DW1000 initialization with complete Polypoint-based setup
This should finally resolve the SYS_CTRL RXENAB bit issue
"""

import dw1000
import time

def test_enhanced_initialization():
    """Test the enhanced initialization with Polypoint configuration"""
    print("=== ENHANCED DW1000 INITIALIZATION TEST ===")
    
    # Initialize with default pins for Pico (adjust as needed)
    dwt = dw1000.DW1000(None, None)  # SPI=None, CS=None (use defaults)
    
    # Initialize with enhanced Polypoint configuration
    print("Initializing DW1000 with enhanced Polypoint configuration...")
    dwt.init()
    
    # Test device ID
    device_id = dwt.get_device_id()
    print(f"DW1000 Device ID: 0x{device_id:08X}")
    
    if device_id != 0xDECA0130:
        print("ERROR: Invalid device ID!")
        return False
    
    # CRITICAL TEST: Check if rx_enable() now works
    print("\nTesting RX enable functionality...")
    
    # Get initial state
    sys_ctrl_before = dwt.read_register(0x0D, 4)  # SYS_CTRL register
    print(f"SYS_CTRL before rx_enable(): 0x{sys_ctrl_before:08X}")
    
    # Try to enable receiver
    try:
        result = dwt.rx_enable()
        print(f"rx_enable() returned: {result}")
        
        # Check SYS_CTRL after rx_enable
        time.sleep(0.1)  # Brief delay
        sys_ctrl_after = dwt.read_register(0x0D, 4)
        print(f"SYS_CTRL after rx_enable():  0x{sys_ctrl_after:08X}")
        
        # Check if RXENAB bit (0x0100) is set
        rxenab_bit = (sys_ctrl_after & 0x0100) != 0
        print(f"RXENAB bit set: {rxenab_bit}")
        
        if rxenab_bit:
            print("SUCCESS: RX enable is working properly!")
            
            # Check receiver state
            sys_state = dwt.get_sys_state()
            print(f"SYS_STATE: 0x{sys_state:08X}")
            
            # Extract state information
            rx_state_code = (sys_state >> 8) & 0x1F
            rx_states = {
                0x00: "IDLE",
                0x01: "RX_EN", 
                0x02: "PREAMBLE_SEARCH",
                0x03: "PREAMBLE_FOUND",
                0x04: "SFD_SEARCH",
                0x05: "SFD_FOUND",
                0x06: "PHR_RX",
                0x07: "DATA_RX"
            }
            rx_state = rx_states.get(rx_state_code, f"UNKNOWN({rx_state_code})")
            print(f"Receiver State: {rx_state}")
            
            if rx_state in ["RX_EN", "PREAMBLE_SEARCH"]:
                print("SUCCESS: Receiver is properly listening for messages!")
                return True
            else:
                print(f"WARNING: Receiver in unexpected state: {rx_state}")
                return False
        else:
            print("FAILURE: RXENAB bit still not set - initialization issue remains")
            return False
            
    except Exception as e:
        print(f"ERROR during rx_enable(): {e}")
        return False

if __name__ == "__main__":
    try:
        success = test_enhanced_initialization()
        if success:
            print("\n🎉 INITIALIZATION TEST PASSED!")
            print("The enhanced Polypoint-based configuration has resolved the RX enable issue.")
        else:
            print("\n❌ INITIALIZATION TEST FAILED")
            print("Further investigation needed.")
    except Exception as e:
        print(f"\nUNEXPECTED ERROR: {e}")