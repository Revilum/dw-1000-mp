# DW1000 Receiver Diagnostic Summary

## **Problem Diagnosed**: Receiver stops receiving after some time

From your log:
```
[0003] 'Test 638' (8B) [E:0 L:0 F:0]
Performing full recovery: 60s inactivity (preventing 'stopping')
[0004] 'Test 698' (8B) [E:0 L:0 F:1]
```

**Gap**: Test 638 → Test 698 = 60 missing messages indicates receiver entered non-responsive state.

## **Root Cause Analysis**

The receiver likely entered one of these stuck states:
1. **Clock Recovery Loss** - Lost synchronization with transmitter
2. **AGC Lock-up** - Automatic Gain Control stuck in wrong state  
3. **Buffer Overflow** - Internal buffers full, stops accepting frames
4. **RX State Machine Stuck** - Hardware state machine in invalid state
5. **Frame Filtering Issues** - Rejecting all frames due to configuration

## **Enhanced Diagnostic Tools Implemented**

### **1. New DW1000 Module Functions** (using lab11 driver constants)
```c
// Status monitoring
dw.get_status()        // SYS_STATUS register
dw.get_sys_state()     // SYS_STATE register  
dw.get_rx_finfo()      // RX_FINFO register
dw.read_register(id)   // Any 32-bit register

// Recovery functions
dw.force_trx_off()     // Shutdown + clear status
dw.rx_reset()          // Reset receiver
dw.sync_rx_bufptrs()   // Sync buffer pointers
```

### **2. Comprehensive Error Detection** (lab11 driver masks)
```python
# Uses existing driver constants
SYS_STATUS_ALL_RX_ERR   # All error conditions
SYS_STATUS_ALL_RX_GOOD  # All success conditions  
SYS_STATUS_ALL_RX_TO    # All timeout conditions

# Individual error flags
SYS_STATUS_RXFCE     # CRC Error
SYS_STATUS_RXPHE     # PHY Header Error
SYS_STATUS_RXRFSL    # Reed Solomon Error
SYS_STATUS_RXRFTO    # Frame Wait Timeout
SYS_STATUS_RXOVRR    # Buffer Overrun
SYS_STATUS_RXSFDTO   # SFD Timeout
SYS_STATUS_LDEERR    # Leading Edge Detection Error
SYS_STATUS_AFFREJ    # Frame Filtering Rejection
```

### **3. RX State Machine Monitoring**
```python
# System state decoding
RX_STATE_IDLE         # 0x00 - Not receiving
RX_STATE_RX_READY     # 0x01 - Ready to receive
RX_STATE_RX_WAIT_SFD  # 0x02 - Waiting for SFD (normal)
RX_STATE_RX_DATA      # 0x03 - Receiving data
RX_STATE_RX_VALIDATE  # 0x05 - Validating frame

# State validation
sys_state = dw.get_sys_state()
rx_state = sys_state & dw1000.SYS_STATE_RX_STATE
```

## **Diagnostic Receiver Features**

### **Real-time State Monitoring**
- **RX State Tracking**: Detects when receiver leaves listening state
- **Error Classification**: Identifies specific failure modes
- **State Transitions**: Monitors receiver state machine changes
- **Health Checks**: Periodic validation every 10 seconds

### **Smart Recovery Strategies**
```python
def smart_recovery(dw, issues, reason=""):
    if "RECEIVER NOT ENABLED" in issues:
        dw.rx_enable()  # Simple re-enable
    elif "RECEIVER STUCK" in issues:
        # Full reset sequence
        dw.force_trx_off()
        dw.rx_reset() 
        dw.sync_rx_bufptrs()
        dw.rx_enable()
    elif "BUFFER OVERRUN" in issues:
        # Buffer-specific recovery
        dw.force_trx_off()
        dw.sync_rx_bufptrs()
        dw.rx_enable()
```

### **Proactive Issue Detection**
- **IDLE State Detection**: Receiver not listening when it should be
- **Invalid State Detection**: Hardware in undefined state
- **Buffer Status Monitoring**: Overflow and pointer issues
- **Inactivity Timeouts**: Extended periods without activity

## **Usage Instructions**

### **1. Deploy Enhanced Firmware**
```bash
# Build with diagnostic functions
cd /home/nils/dw100-new-test/micropython/ports/rp2
make BOARD=RPI_PICO_W USER_C_MODULES=/home/nils/dw100-new-test/micropython_dw1000 -j16

# Flash to Pico
cp build-RPI_PICO_W/firmware.uf2 /media/USER/RPI-RP2/
```

### **2. Run Diagnostic Receiver**
```bash
# Copy to Pico
cp examples/diagnostic_receiver.py /media/USER/PICO/

# Run on Pico
python diagnostic_receiver.py
```

### **3. Monitor Output**
```
=== DW1000 COMPREHENSIVE DIAGNOSIS ===
SYS_STATUS: 0x00004000
SYS_STATE:  0x00000002  
RX_FINFO:   0x00000000
RX State: WAIT_SFD
Status: NO FRAME
Errors: NONE
STATE: Normal operation
========================================

# If receiver gets stuck:
PROBLEM: Receiver in IDLE state - should be listening!
SMART RECOVERY: receiver idle when should be listening
- Re-enabling receiver
```

## **Expected Results**

### **Normal Operation**
```
[0001] 'Test 636' (8B) RX:WAIT_SFD [E:0 R:0]
[0002] 'Test 637' (8B) RX:WAIT_SFD [E:0 R:0]
RX State changed: WAIT_SFD -> RX_DATA
RX State changed: RX_DATA -> WAIT_SFD
[0003] 'Test 638' (8B) RX:WAIT_SFD [E:0 R:0]
```

### **Problem Detection**
```
[0003] 'Test 638' (8B) RX:WAIT_SFD [E:0 R:0]
RX State changed: WAIT_SFD -> IDLE
PROBLEM: Receiver in IDLE state - should be listening!
SMART RECOVERY: receiver idle when should be listening
- Re-enabling receiver
[0004] 'Test 639' (8B) RX:WAIT_SFD [E:0 R:1]
```

## **Benefits**

✅ **Real-time Diagnosis**: Immediate detection of receiver state issues  
✅ **Targeted Recovery**: Specific fixes for specific problems  
✅ **Professional Integration**: Uses lab11 driver constants and functions  
✅ **Comprehensive Coverage**: All major DW1000 error conditions  
✅ **Proactive Prevention**: Detects issues before they cause stopping  

## **Next Steps**

1. **Test with Current Setup**: Run diagnostic_receiver.py to see what's happening
2. **Analyze Patterns**: Look for specific state transitions that precede stopping
3. **Optimize Recovery**: Fine-tune recovery strategies based on observed issues
4. **Long-term Monitoring**: Run extended tests to validate reliability improvements

The diagnostic tools should now reveal exactly why your receiver stops and provide targeted recovery mechanisms to maintain continuous operation!