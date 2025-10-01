#!/usr/bin/env python3
"""
Deep hardware diagnostic for DW1000 RX enable failure
This will systematically check every aspect of the hardware setup
"""

import dw1000
import machine
import time

def check_spi_communication(dwt):
    """Verify SPI communication is working correctly"""
    print("=== SPI COMMUNICATION TEST ===")
    
    # Test 1: Read device ID multiple times
    print("Reading device ID 5 times...")
    ids = []
    for i in range(5):
        device_id = dwt.read_device_id()  # Correct method name
        ids.append(device_id)
        print(f"  Read {i+1}: 0x{device_id:08X}")
        time.sleep(0.1)
    
    if all(id == 0xDECA0130 for id in ids):
        print("✓ SPI communication is stable")
        return True
    else:
        print("✗ SPI communication is unstable!")
        return False

def check_core_registers(dwt):
    """Check critical DW1000 registers"""
    print("\n=== CORE REGISTER CHECK ===")
    
    # Critical registers to check - read_register only takes reg_id, not length
    registers = {
        'DEV_ID': (0x00, 0xDECA0130),
        'SYS_CFG': (0x04, None),  # No expected value
        'SYS_TIME': (0x06, None),  # Should be changing
        'SYS_STATUS': (0x0F, None),
        'SYS_STATE': (0x19, None),
        'SYS_CTRL': (0x0D, None),
    }
    
    all_good = True
    for name, (reg_id, expected) in registers.items():
        try:
            value = dwt.read_register(reg_id)  # Only pass reg_id
            
            print(f"{name:12}: 0x{value:08X}", end="")
            
            if expected is not None:
                if value == expected:
                    print(" ✓")
                else:
                    print(f" ✗ (expected 0x{expected:08X})")
                    all_good = False
            else:
                print("")
                
        except Exception as e:
            print(f"{name:12}: ERROR - {e}")
            all_good = False
    
    return all_good

def check_initialization_sequence(dwt):
    """Check if initialization was done correctly"""
    print("\n=== INITIALIZATION SEQUENCE CHECK ===")
    
    # Read key configuration registers
    sys_cfg = dwt.read_register(0x04)  # SYS_CFG
    chan_ctrl = dwt.read_register(0x1F)  # CHAN_CTRL
    tx_power = dwt.read_register(0x1E)  # TX_POWER
    
    print(f"SYS_CFG:     0x{sys_cfg:08X}")
    print(f"CHAN_CTRL:   0x{chan_ctrl:08X}")
    print(f"TX_POWER:    0x{tx_power:08X}")
    
    # Check if these look reasonable
    if sys_cfg == 0 and chan_ctrl == 0 and tx_power == 0:
        print("✗ Registers look uninitialized!")
        return False
    else:
        print("✓ Configuration registers have been set")
        return True

def test_force_transceiver_states(dwt):
    """Test if we can manually control transceiver states"""
    print("\n=== TRANSCEIVER STATE CONTROL TEST ===")
    
    try:
        # Get initial state
        initial_state = dwt.get_sys_state()
        print(f"Initial SYS_STATE: 0x{initial_state:08X}")
        
        # Try to force transceiver off
        print("Forcing transceiver OFF...")
        dwt.force_trx_off()
        time.sleep(0.1)
        
        off_state = dwt.get_sys_state()
        print(f"After TRX_OFF: 0x{off_state:08X}")
        
        # Check if state changed to IDLE
        rx_state = (off_state >> 8) & 0x1F
        if rx_state == 0:  # IDLE
            print("✓ Successfully forced to IDLE state")
            return True
        else:
            print(f"✗ Still not in IDLE state (rx_state={rx_state})")
            return False
            
    except Exception as e:
        print(f"✗ Error during state control: {e}")
        return False

def test_simple_register_write(dwt):
    """Test if we can write to registers"""
    print("\n=== REGISTER WRITE TEST ===")
    
    try:
        # Try to write to a safe register (like LED control)
        # Note: This would need a write_register function
        print("Register write test not implemented (no write_register function)")
        return None
        
    except Exception as e:
        print(f"✗ Error during register write test: {e}")
        return False

def analyze_sys_state_details(dwt):
    """Detailed analysis of SYS_STATE register"""
    print("\n=== DETAILED SYS_STATE ANALYSIS ===")
    
    sys_state = dwt.get_sys_state()
    print(f"SYS_STATE: 0x{sys_state:08X}")
    
    # Extract individual fields
    rx_state = (sys_state >> 8) & 0x1F
    tx_state = (sys_state >> 16) & 0x0F
    pmsc_state = (sys_state >> 20) & 0x0F
    
    rx_states = {
        0: "IDLE", 1: "RX_EN", 2: "PREAMBLE_SEARCH", 3: "PREAMBLE_FOUND",
        4: "SFD_SEARCH", 5: "SFD_FOUND", 6: "PHR_RX", 7: "DATA_RX"
    }
    
    tx_states = {
        0: "IDLE", 1: "TX_EN", 2: "TX_PREAMBLE", 3: "TX_SFD",
        4: "TX_PHR", 5: "TX_DATA", 6: "TX_CRC", 7: "TX_WAIT_ACK"
    }
    
    print(f"RX State:   {rx_states.get(rx_state, 'UNKNOWN(' + str(rx_state) + ')')}")
    print(f"TX State:   {tx_states.get(tx_state, 'UNKNOWN(' + str(tx_state) + ')')}")
    print(f"PMSC State: {pmsc_state}")
    
    # Check if stuck in any particular state
    if rx_state == 0 and tx_state == 0:
        print("✓ Transceiver in proper IDLE state")
        return True
    elif rx_state == 0 and tx_state == 1:
        print("⚠ TX is still enabled - this may block RX enable!")
        print("  This could be the root cause of the RX enable failure")
        return True  # This is informative, not a failure
    else:
        print("? Transceiver in unexpected state")
        return False

def deep_hardware_diagnostic():
    """Run complete hardware diagnostic"""
    print("=== DEEP DW1000 HARDWARE DIAGNOSTIC ===")
    print("This will systematically check hardware setup and driver state")
    print()
    
    spi = machine.SPI(0, baudrate=1000000)
    cs_pin = machine.Pin(17, machine.Pin.OUT)
    reset_pin = machine.Pin(21, machine.Pin.OUT)
    irq_pin = machine.Pin(20, machine.Pin.IN)
    dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
    dwt.init()
    
    # Run all tests
    tests = [
        ("SPI Communication", check_spi_communication),
        ("Core Registers", check_core_registers),
        ("Initialization Sequence", check_initialization_sequence),
        ("SYS_STATE Analysis", analyze_sys_state_details),
        ("Transceiver Control", test_force_transceiver_states),
    ]
    
    results = {}
    for test_name, test_func in tests:
        print(f"\n{'='*50}")
        try:
            result = test_func(dwt)
            results[test_name] = result
        except Exception as e:
            print(f"TEST FAILED: {e}")
            results[test_name] = False
    
    # Final summary
    print(f"\n{'='*50}")
    print("DIAGNOSTIC SUMMARY:")
    for test_name, result in results.items():
        status = "✓ PASS" if result else ("✗ FAIL" if result is False else "? UNKNOWN")
        print(f"  {test_name:25}: {status}")
    
    # Overall assessment
    failed_tests = [name for name, result in results.items() if result is False]
    if failed_tests:
        print(f"\nFAILED TESTS: {', '.join(failed_tests)}")
        print("These areas need investigation.")
    else:
        print("\nAll tests passed - the issue may be in the RX enable logic itself.")
    
    return dwt, results

if __name__ == "__main__":
    try:
        dwt, results = deep_hardware_diagnostic()
        
        # If basic tests pass, try one more targeted test
        if all(r is not False for r in results.values()):
            print(f"\n{'='*50}")
            print("FINAL RX ENABLE TEST:")
            
            # Try RX enable one more time with detailed monitoring
            sys_ctrl_before = dwt.read_register(0x0D)
            print(f"SYS_CTRL before: 0x{sys_ctrl_before:08X}")
            
            # Enable RX
            result = dwt.rx_enable()
            print(f"rx_enable() returned: {result}")
            
            # Check immediately after
            sys_ctrl_after = dwt.read_register(0x0D)
            print(f"SYS_CTRL after:  0x{sys_ctrl_after:08X}")
            
            if (sys_ctrl_after & 0x0100) != 0:
                print("SUCCESS: RXENAB bit is finally set!")
            else:
                print("FAILURE: RXENAB bit still not set")
                print("This indicates a fundamental driver or hardware issue.")
        
    except Exception as e:
        print(f"DIAGNOSTIC FAILED: {e}")
        # Remove traceback import since it's not available in MicroPython