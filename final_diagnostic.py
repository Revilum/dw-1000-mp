#!/usr/bin/env python3
"""
Final diagnostic: Test if the dwt_rxenable function is actually being called
and what it's trying to write to the SYS_CTRL register
"""

import dw1000
import time

def test_rx_enable_step_by_step():
    """Step through the exact dwt_rxenable process"""
    print("=== STEP-BY-STEP RX ENABLE DIAGNOSTIC ===")
    
    # Initialize
    dwt = dw1000.DW1000(None, None)  # SPI=None, CS=None (use defaults)
    dwt.init()
    
    print(f"Device ID: 0x{dwt.get_device_id():08X}")
    
    # Step 1: Check initial state
    print("\n1. Initial State Check:")
    sys_ctrl = dwt.read_register(0x0D, 4)
    sys_state = dwt.get_sys_state()
    print(f"   SYS_CTRL: 0x{sys_ctrl:08X}")
    print(f"   SYS_STATE: 0x{sys_state:08X}")
    
    # Step 2: Force TRX off
    print("\n2. Forcing TRX OFF:")
    dwt.force_trx_off()
    time.sleep(0.1)
    
    sys_ctrl = dwt.read_register(0x0D, 4)
    sys_state = dwt.get_sys_state()
    print(f"   SYS_CTRL: 0x{sys_ctrl:08X}")
    print(f"   SYS_STATE: 0x{sys_state:08X}")
    
    # Step 3: Clear status and reset buffers
    print("\n3. Clear status and reset buffers:")
    dwt.clear_all_status()
    dwt.rx_reset()
    dwt.sync_rx_bufptrs()
    time.sleep(0.1)
    
    sys_ctrl = dwt.read_register(0x0D, 4)
    sys_state = dwt.get_sys_state()
    print(f"   SYS_CTRL: 0x{sys_ctrl:08X}")
    print(f"   SYS_STATE: 0x{sys_state:08X}")
    
    # Step 4: The critical test - call rx_enable
    print("\n4. Calling rx_enable():")
    result = dwt.rx_enable()
    print(f"   rx_enable() returned: {result}")
    
    # Check immediately
    sys_ctrl = dwt.read_register(0x0D, 4)
    sys_state = dwt.get_sys_state()
    print(f"   SYS_CTRL: 0x{sys_ctrl:08X}")
    print(f"   SYS_STATE: 0x{sys_state:08X}")
    
    # Step 5: Wait a bit and check again
    print("\n5. After 100ms delay:")
    time.sleep(0.1)
    
    sys_ctrl = dwt.read_register(0x0D, 4)
    sys_state = dwt.get_sys_state()
    print(f"   SYS_CTRL: 0x{sys_ctrl:08X}")
    print(f"   SYS_STATE: 0x{sys_state:08X}")
    
    # Final analysis
    print("\n=== ANALYSIS ===")
    if (sys_ctrl & 0x0100) != 0:
        print("✓ RXENAB bit is SET - RX enable worked!")
        rx_state = (sys_state >> 8) & 0x1F
        print(f"  RX State: {rx_state}")
        return True
    else:
        print("✗ RXENAB bit is CLEAR - RX enable failed")
        print("  This means dwt_rxenable() is not successfully writing to SYS_CTRL")
        print("  Possible causes:")
        print("  1. Hardware is rejecting the register write")
        print("  2. Something immediately clears the register after writing")
        print("  3. SPI communication issue specific to writes")
        print("  4. Missing hardware prerequisite (power, clock, etc.)")
        return False

def test_register_read_write_capability(dwt):
    """Test if we can write to ANY register successfully"""
    print("\n=== REGISTER WRITE CAPABILITY TEST ===")
    
    # We can't easily test writes without potentially damaging config,
    # but we can test reads of various registers to ensure SPI is working
    registers_to_test = {
        'DEV_ID': 0x00,
        'SYS_CFG': 0x04,
        'SYS_TIME': 0x06,
        'SYS_CTRL': 0x0D,
        'SYS_STATUS': 0x0F,
        'SYS_STATE': 0x19,
        'CHAN_CTRL': 0x1F,
    }
    
    print("Reading various registers:")
    for name, reg_id in registers_to_test.items():
        try:
            value = dwt.read_register(reg_id, 4)
            print(f"  {name:12}: 0x{value:08X}")
        except Exception as e:
            print(f"  {name:12}: ERROR - {e}")
    
    print("\nIf all reads work but SYS_CTRL never gets RXENAB bit,")
    print("then the issue is specifically with the dwt_rxenable() function")
    print("or its prerequisites in the lab11 driver.")

if __name__ == "__main__":
    try:
        success = test_rx_enable_step_by_step()
        
        if not success:
            # If RX enable failed, do additional testing
            dwt = dw1000.DW1000(None, None)  # SPI=None, CS=None (use defaults)
            dwt.init()
            test_register_read_write_capability(dwt)
            
            print("\n=== RECOMMENDATION ===")
            print("Since RX enable consistently fails despite proper initialization,")
            print("the issue is likely one of:")
            print("1. Bug in the lab11 dw1000-driver dwt_rxenable() function")
            print("2. Hardware-specific issue (power, crystal, etc.)")
            print("3. Missing driver configuration that's required for this specific DW1000 chip")
            print("4. Timing issue in the SPI communication")
            
    except Exception as e:
        print(f"ERROR: {e}")
        # Traceback not available in MicroPython