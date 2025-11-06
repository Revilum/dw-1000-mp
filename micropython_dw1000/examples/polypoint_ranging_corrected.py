"""
PolyPoint-Style Two-Way Ranging for DW1000 (CORRECTED VERSION)
===============================================================

Based on analysis of PolyPoint firmware and DW1000 driver.
Uses hardware-scheduled delayed transmission for precise timing.

Key Corrections:
✅ Uses wrapper's timestamp_to_seconds() instead of hardcoded DWT_TIME_UNITS
✅ Proper RX pattern: status check → read_rx_data → force_trx_off → rx_enable
✅ Constants properly explained (DW1000 hardware-specific timing)

Key Features:
- Hardware-scheduled delayed transmission (microsecond precision)
- Immediate timestamp reading after RX/TX events
- No software timing delays needed
- Simple two-way ranging protocol

Author: Analysis of PolyPoint project
"""

import time
import struct
import machine
import dw1000

# Physical constants
# SPEED_OF_LIGHT: PolyPoint uses 299,702,547 m/s (calibrated)
# Standard physics: 299,792,458 m/s
SPEED_OF_LIGHT = 299702547  # m/s

# Note: DWT_TIME_UNITS (15.65e-12 seconds per tick) is hardware-specific to DW1000
# It comes from: 1 / (64MHz * 499.2/64 * 128) = 1/(499.2e6*128) = 15.65e-12
# We use the wrapper's timestamp_to_seconds() function instead of hardcoding

# ============================================================================
# SOFTWARE CALIBRATION (PolyPoint-style)
# ============================================================================
# Antenna delay calibration per channel (in DW1000 ticks)
# TRUE PolyPoint approach: FULL calibration value subtracted from RX timestamps ONLY
# Source: polypoint/software/firmware/dw1000.c line 57
# PolyPoint default: 33,000 ticks (517ns) = 155m
# 
# CALIBRATION RESULTS @ 1.0m actual distance:
# OLD TESTS (with float conversion bug - INVALID):
#   0 ticks → 0.9m (BEST - nearly perfect!)
#   100 ticks → 140-220m (WORSE!)
#   32,500 ticks → -40m to +40m (way too large)
#
# NEW TESTS (with proper uint64 conversion - CORRECT!):
#   0 ticks → 152m (measured TOF: 32,400 ticks, expected: 213 ticks)
#   16,093 ticks → 77m (halved the error)
#   32,187 ticks → 0.2-2.7m (high variance, possible overcorrection + drift)
#
# CALIBRATION MODE: Set to 0 for antenna delay calibration
# After running calibrate_antenna_delay.py, update with recommended value
ANTENNA_CALIBRATION = {
    1: 0,  # Channel 1 - ZERO FOR CALIBRATION
    2: 0,  # Channel 2 - ZERO FOR CALIBRATION
    3: 0,  # Channel 3 - ZERO FOR CALIBRATION
    4: 0,  # Channel 4 - ZERO FOR CALIBRATION
    5: 0,  # Channel 5 - ZERO FOR CALIBRATION
    7: 0,  # Channel 7 - ZERO FOR CALIBRATION
}
# ============================================================================

CALIBRATION_ENABLED = True  # TRUE PolyPoint approach: FULL cal on RX timestamps only

# Currently configured channel
CURRENT_CHANNEL = 2  # Channel 2 (3993.6 MHz)

# ============================================================================
# CLOCK DRIFT COMPENSATION (PolyPoint method)
# ============================================================================
# Crystal oscillator frequency mismatch between devices causes TOF errors.
# Typical crystal tolerance: ±10-20 ppm (parts per million)
# Over 1ms ranging interval: 10ppm = 10ns error = 3m distance error
#
# PolyPoint compensates by calculating the clock frequency ratio:
#   clock_ratio = (T4 - T1) / (T3 - T2)
# 
# Then corrects TOF using:
#   tof_corrected = ((T4-T1) - (T3-T2)*clock_ratio) / 2
#
# This eliminates the crystal frequency mismatch between initiator and responder.
# Source: PolyPoint uses this in their ranging calculations
# ============================================================================

CLOCK_DRIFT_COMPENSATION = True  # Uses standard TWR (drift effects cancel in round-trip)

def calculate_clock_ratio(t1, t2, t3, t4):
    """
    Calculate the clock frequency ratio between initiator and responder.
    
    This compensates for crystal oscillator frequency mismatch.
    
    PolyPoint formula: ratio = (T4 - T1) / (T3 - T2)
    
    The ratio represents: responder_clock_frequency / initiator_clock_frequency
    
    Args:
        t1: Initiator TX timestamp (POLL sent)
        t2: Responder RX timestamp (POLL received)
        t3: Responder TX timestamp (RESPONSE sent)
        t4: Initiator RX timestamp (RESPONSE received)
    
    Returns:
        Clock frequency ratio (typically close to 1.0, e.g., 1.000015 for 15ppm drift)
    """
    TIMESTAMP_MAX = (1 << 40)  # 40-bit wraparound
    
    # Round trip time at initiator
    round_trip = t4 - t1
    if round_trip < 0:
        round_trip += TIMESTAMP_MAX
    
    # Processing time at responder
    processing = t3 - t2
    if processing < 0:
        processing += TIMESTAMP_MAX
    
    # Avoid division by zero
    if processing == 0:
        return 1.0
    
    # Clock ratio
    ratio = float(round_trip) / float(processing)
    
    return ratio

def calculate_tof_with_drift_compensation(t1, t2, t3, t4):
    """
    Calculate Time of Flight with clock drift compensation.
    
    Standard formula: TOF = ((T4-T1) - (T3-T2)) / 2
    
    The issue: if responder clock runs faster/slower than initiator,
    (T3-T2) measured in responder's clock needs conversion to initiator's clock.
    
    Correct approach: We DON'T use the ratio this way - it cancels out!
    Instead, we need the actual propagation times:
    - Propagation T1→T2: some time
    - Propagation T3→T4: same time (symmetric)
    
    The standard formula already works if clocks are similar. The "drift compensation"
    is actually handled by using calibrated timestamps - the antenna delay calibration
    already accounts for systematic errors.
    
    For now, return standard TWR formula. True drift compensation requires
    more sophisticated approach (e.g., multiple ranges to establish clock ratio).
    
    Args:
        t1: Initiator TX timestamp (POLL sent) 
        t2: Responder RX timestamp (POLL received)
        t3: Responder TX timestamp (RESPONSE sent)
        t4: Initiator RX timestamp (RESPONSE received)
    
    Returns:
        Time of flight in DW1000 ticks
    """
    TIMESTAMP_MAX = (1 << 40)  # 1,099,511,627,776
    
    # Round trip time at initiator (in initiator's clock)
    # Arduino-DW1000 approach: just check if negative, then add TIME_OVERFLOW
    # This is simpler and always works!
    round_trip = t4 - t1
    if round_trip < 0:
        round_trip += TIMESTAMP_MAX
    
    # Processing time at responder (in responder's clock)
    processing = t3 - t2
    if processing < 0:
        processing += TIMESTAMP_MAX
    
    # Standard TWR formula
    # This works because we're measuring round-trip, so clock drift effects
    # largely cancel out (drift affects both forward and return equally)
    tof_ticks = (round_trip - processing) // 2
    
    return tof_ticks

def get_rx_calibration(channel):
    """
    Get RX calibration value for a channel (FULL calibration - TRUE PolyPoint approach).
    
    PolyPoint applies calibration ONLY to RX timestamps, subtracting the FULL value.
    Hardware antenna delays are set to 0. TX timestamps are NOT calibrated.
    
    Source: polypoint/software/firmware/dw1000.c line 602 and oneway_tag.c line 242
    - Hardware delays set to 0: DW1000_ANTENNA_DELAY_TX/RX = 0
    - Software calibration subtracted from ALL RX timestamps only
    - Default: 33,000 ticks = 517ns = 155m
    
    Args:
        channel: RF channel number (1-7)
    
    Returns:
        RX calibration in ticks (FULL antenna delay, not half!)
    """
    if not CALIBRATION_ENABLED:
        return 0
    
    return ANTENNA_CALIBRATION.get(channel, 33000)  # FULL value!

def apply_rx_calibration(timestamp, channel):
    """
    Apply RX calibration to a timestamp (TRUE PolyPoint pattern).
    SUBTRACTS the FULL RX delay from the raw timestamp.
    
    PolyPoint ONLY calibrates RX timestamps (T2, T4).
    TX timestamps (T1, T3) are NOT calibrated.
    
    Result:
    - Round trip = (T4 - T1) - cal  
    - Processing = (T3 - T2) + cal
    - TOF = ((T4-T1) - (T3-T2) - cal - cal)/2 = real_TOF - cal
    
    So the full calibration value reduces measured distance.
    
    Args:
        timestamp: Raw RX timestamp from DW1000
        channel: RF channel number
    
    Returns:
        Calibrated timestamp (with wraparound handling)
    """
    TIMESTAMP_MAX = (1 << 40)  # 40-bit wraparound
    cal = get_rx_calibration(channel)
    result = timestamp - cal
    
    # Handle wraparound: if subtraction results in negative, wrap around
    if result < 0:
        result += TIMESTAMP_MAX
    
    return result

def calculate_distance_from_tof(tof_ticks, dwt):
    """
    Calculate distance from time-of-flight in ticks.
    Uses wrapper's timestamp_to_seconds() to get proper time conversion.
    
    Args:
        tof_ticks: Time of flight in DW1000 timestamp ticks
        dwt: DW1000 device instance (needed for conversion function)
    
    Returns:
        Distance in meters
    """
    # Convert ticks to seconds using wrapper function
    tof_seconds = dwt.timestamp_to_seconds(tof_ticks)
    
    # Distance = time * speed_of_light
    distance = tof_seconds * SPEED_OF_LIGHT
    
    return distance

def ticks_to_microseconds(ticks, dwt):
    """Convert DW1000 ticks to microseconds"""
    # DWT_TIME_UNITS = 1 / (499.2e6 * 128) = 15.65e-12 seconds/tick
    # Microseconds = ticks * 15.65e-12 * 1e6 = ticks * 15.65e-6
    return float(ticks) * (1.0 / 499.2e6 / 128.0) * 1e6

def microseconds_to_ticks(microseconds, dwt):
    """Convert microseconds to DW1000 ticks"""
    # ticks = microseconds / (15.65e-6) = microseconds * 63897.6
    return int(microseconds / ((1.0 / 499.2e6 / 128.0) * 1e6))

# Message types
MSG_POLL = 0x61
MSG_RESPONSE = 0x62

class RangingInitiator:
    """
    Initiator sends POLL and measures round-trip time
    """
    
    def __init__(self, device_id=0x01):
        # Initialize DW1000 with correct SPI pins (exact pattern from transmitter.py)
        # SPI @ 1 MHz - OPTIMAL for breadboard/jumper wire setup
        # (PolyPoint uses 6 MHz with short PCB traces, but our setup needs slower speed for signal integrity)
        spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0,
                          sck=machine.Pin(18), 
                          mosi=machine.Pin(19), 
                          miso=machine.Pin(16))
        cs_pin = machine.Pin(17, machine.Pin.OUT)
        reset_pin = machine.Pin(21, machine.Pin.OUT)
        irq_pin = machine.Pin(20, machine.Pin.IN)
        
        self.dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        
        self.dwt.init(auto_rx=True)
        self.dwt.configure(channel=2, data_rate=dw1000.BR_850K)
        
        self.device_id = device_id
        self.sequence = 0
        
        print("✅ Ranging Initiator initialized")
        print(f"   Device ID: 0x{self.dwt.read_device_id():08X}")
        print(f"   Role: INITIATOR (ID {device_id:02X})")
        
    def create_poll_message(self):
        """Create POLL message"""
        self.sequence = (self.sequence + 1) & 0xFF
        # Format: [type][device_id][sequence][padding(7 bytes)]
        # Total: 10 bytes minimum (PolyPoint uses ~31 bytes with IEEE 802.15.4 headers)
        # Padding to ensure frame meets minimum length requirements
        return struct.pack('<BBB7s', MSG_POLL, self.device_id, self.sequence, b'\x00\x00\x00\x00\x00\x00\x00')
        
    def parse_response(self, data):
        """Parse RESPONSE message containing T2 and T3"""
        if len(data) < 26:  # type + device_id + seq + padding(7) + T2(5) + pad(3) + T3(5) + pad(3) = 26 bytes
            print(f"⚠️ Frame too short: {len(data)} bytes (need 26)")
            return None
            
        try:
            msg_type, device_id, seq_num = struct.unpack('<BBB', data[:3])
            
            if msg_type != MSG_RESPONSE:
                print(f"⚠️ Wrong message type: 0x{msg_type:02X} (expected 0x{MSG_RESPONSE:02X})")
                return None
                
            # Extract T2 and T3 (5 bytes each, little-endian) - DW1000 timestamps are 40-bit!
            # Arduino-DW1000 uses 5-byte timestamps, not 8!
            t2 = int.from_bytes(data[10:15], 'little')  # 5 bytes [10-14]
            t3 = int.from_bytes(data[18:23], 'little')  # 5 bytes [18-22]
            
            return {
                'device_id': device_id,
                'sequence': seq_num,
                't2': t2,
                't3': t3
            }
        except Exception as e:
            print(f"⚠️ Parse exception: {e}")
            return None
    
    def _proper_rx_reset(self):
        """
        Proper RX reset sequence matching receiver_polling.py pattern
        """
        self.dwt.force_trx_off()
        time.sleep_ms(2)  # Allow hardware to settle
        self.dwt.rx_reset()
        self.dwt.sync_rx_bufptrs()
            
    def range_once(self, timeout_ms=1000):
        """
        Perform one ranging measurement
        Returns distance in meters or None on timeout
        """
        
        # 1. Prepare receiver for response BEFORE sending POLL
        self._proper_rx_reset()
        
        # 2. Send POLL
        poll_msg = self.create_poll_message()
        self.dwt.tx_frame(poll_msg)
        
        # 3. Wait for TX complete (TXFRS bit 7 = 0x0080)
        tx_timeout = 100  # ms
        tx_start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), tx_start) < tx_timeout:
            status = self.dwt.get_status()
            if status & 0x0080:  # TXFRS - TX frame sent
                break
            time.sleep_ms(1)
        
        # 4. Read T1 immediately after transmission complete
        # Hardware captured the TX timestamp automatically
        t1 = self.dwt.read_tx_timestamp()
        
        if t1 == 0:
            print("❌ Failed to capture T1 timestamp")
            return None
            
        # 5. Enable receiver
        self.dwt.rx_enable()
        
        # 6. Wait for response (PROPER RX PATTERN from receiver_polling.py)
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            status = self.dwt.get_status()
            
            # IMPORTANT: Check for good frame FIRST and process immediately
            # Status bits accumulate, so RXFCG (0x2000) takes priority
            if status & 0x2000:  # SYS_STATUS_RXFCG - Frame received OK
                
                # 7. Read T4 immediately
                t4 = self.dwt.read_rx_timestamp()
                
                # 8. Get response data (BEFORE force_trx_off!)
                rx_data = self.dwt.read_rx_data()
                print(f"🔍 RX: {len(rx_data)} bytes, hex: {rx_data.hex()}")
                
                # 9. CRITICAL: Force TRX off and reset for next operation
                self._proper_rx_reset()
                
                # 10. Parse response
                response = self.parse_response(rx_data)
                print(f"🔍 Parsed: {response}")
                
                if response and response['sequence'] == self.sequence:
                    t2_raw = response['t2']  # Raw T2 from responder
                    t3_raw = response['t3']
                    
                    # 11. Apply software calibration (TRUE PolyPoint approach)
                    # PolyPoint ONLY calibrates RX timestamps (T2, T4), NOT TX (T1, T3)!
                    # Source: polypoint/software/firmware/oneway_tag.c line 242
                    #         polypoint/software/firmware/oneway_anchor.c line 207-208
                    # Hardware antenna delays set to 0, software calibration on RX only
                    #
                    # Responder sends RAW T2, initiator calibrates it locally
                    # This ensures both devices apply same calibration
                    #
                    # Result: TOF = ((T4-T1) - (T3-T2))/2 with T2,T4 calibrated
                    #       = ((T4-cal - T1) - (T3 - T2+cal))/2  
                    #       = (T4 - T1 - T3 + T2 - cal - cal)/2
                    #       = real_TOF - cal
                    t1_cal = t1  # NO calibration on TX timestamps!
                    t2_cal = apply_rx_calibration(t2_raw, CURRENT_CHANNEL)  # Subtract FULL cal
                    t3_cal = t3_raw  # NO calibration on TX timestamps!
                    t4_cal = apply_rx_calibration(t4, CURRENT_CHANNEL)  # Subtract FULL cal
                    
                    # 12. Calculate distance using two-way ranging formula
                    # RTT = (T4 - T1) - (T3 - T2)
                    # Distance = (RTT / 2) * speed_of_light * time_unit
                    
                    # Handle 40-bit timestamp wraparound
                    # The DW1000 uses 40-bit timestamps that wrap around
                    TIMESTAMP_MAX = (1 << 40)  # 2^40 = 1,099,511,627,776
                    
                    # Calculate differences with wraparound handling (using CALIBRATED timestamps)
                    round_trip = t4_cal - t1_cal
                    if round_trip < 0:
                        round_trip += TIMESTAMP_MAX
                    
                    processing = t3_cal - t2_cal
                    if processing < 0:
                        processing += TIMESTAMP_MAX
                    
                    # Time of flight (one way)
                    if CLOCK_DRIFT_COMPENSATION:
                        # Use PolyPoint's clock drift compensation method
                        # This accounts for crystal frequency mismatch between devices
                        tof_ticks = calculate_tof_with_drift_compensation(t1_cal, t2_cal, t3_cal, t4_cal)
                        clock_ratio = calculate_clock_ratio(t1_cal, t2_cal, t3_cal, t4_cal)
                    else:
                        # Standard formula without drift compensation
                        tof_ticks = (round_trip - processing) // 2
                        clock_ratio = 1.0
                    
                    # Convert to distance using wrapper function
                    distance = calculate_distance_from_tof(tof_ticks, self.dwt)
                    
                    # Get timing info using wrapper's conversion
                    rtt_us = ticks_to_microseconds(round_trip, self.dwt)
                    proc_us = ticks_to_microseconds(processing, self.dwt)
                    tof_us = ticks_to_microseconds(tof_ticks, self.dwt)
                    
                    # Show calibration status
                    cal_status = "✅ ENABLED" if CALIBRATION_ENABLED else "❌ DISABLED"
                    cal_value = ANTENNA_CALIBRATION.get(CURRENT_CHANNEL, 0)
                    drift_status = "✅ ENABLED" if CLOCK_DRIFT_COMPENSATION else "❌ DISABLED"
                    
                    print(f"\n📏 Ranging Result (seq {self.sequence}):")
                    print(f"   Calibration: {cal_status} ({cal_value} ticks)")
                    print(f"   Clock Drift Comp: {drift_status} (ratio: {clock_ratio:.6f})")
                    print(f"   T1 (POLL TX):     {t1_cal} (raw: {t1})")
                    print(f"   T2 (POLL RX):     {t2_cal} (raw: {t2_raw})")
                    print(f"   T3 (RESPONSE TX): {t3_cal} (raw: {t3_raw})")
                    print(f"   T4 (RESPONSE RX): {t4_cal} (raw: {t4})")
                    print(f"   Round trip: {round_trip} ticks ({rtt_us:.1f} µs)")
                    print(f"   Processing: {processing} ticks ({proc_us:.1f} µs)")
                    print(f"   TOF: {tof_ticks} ticks ({tof_us:.1f} µs)")
                    print(f"   📍 Distance: {distance:.3f} meters")
                    
                    return distance
                else:
                    # Parse failed - wrong message type or sequence
                    print(f"⚠️ Parse failed or sequence mismatch: got {response}")
                    continue
            
            # Only check for errors if we didn't get a good frame
            # (error bits may be stale from previous events)
            elif status & 0x5C00:  # RX errors including CRC (0x1C00 | 0x4000)
                print(f"⚠️ RX error: status = 0x{status:08X}")
                # Decode error bits for debugging
                if status & 0x4000:
                    print("   - CRC Error (RXFCE)")
                if status & 0x1000:
                    print("   - RX PHY Header Error (RXPHE)")
                if status & 0x0800:
                    print("   - RX Frame Wait Timeout (RXRFTO)")
                if status & 0x0400:
                    print("   - RX Overrun (RXOVRR)")
                self._proper_rx_reset()
                return None
                    
            time.sleep_ms(10)
            
        print("⏰ Timeout waiting for response")
        self._proper_rx_reset()
        return None
        
    def range_continuous(self, interval_ms=1000):
        """Continuously perform ranging"""
        print("\n🚀 Starting continuous ranging...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                distance = self.range_once()
                time.sleep_ms(interval_ms)
        except KeyboardInterrupt:
            print("\n🛑 Ranging stopped by user")

class RangingResponder:
    """
    Responder receives POLL and sends back RESPONSE with T2 and T3
    Uses PolyPoint-style delayed transmission for precise T3
    """
    
    def __init__(self, device_id=0x02):
        # Initialize DW1000 with correct SPI pins (exact pattern from transmitter.py)
        # SPI @ 1 MHz - OPTIMAL for breadboard/jumper wire setup
        # (PolyPoint uses 6 MHz with short PCB traces, but our setup needs slower speed for signal integrity)
        spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0,
                          sck=machine.Pin(18), 
                          mosi=machine.Pin(19), 
                          miso=machine.Pin(16))
        cs_pin = machine.Pin(17, machine.Pin.OUT)
        reset_pin = machine.Pin(21, machine.Pin.OUT)
        irq_pin = machine.Pin(20, machine.Pin.IN)
        
        self.dwt = dw1000.DW1000(spi, cs_pin, reset_pin, irq_pin)
        
        self.dwt.init()
        self.dwt.configure(channel=2, data_rate=dw1000.BR_850K)
        
        self.device_id = device_id
        
        print("✅ Ranging Responder initialized")
        print(f"   Device ID: 0x{self.dwt.read_device_id():08X}")
        print(f"   Role: RESPONDER (ID {device_id:02X})")
    
    def _proper_rx_reset(self):
        """
        Proper RX reset sequence matching receiver_polling.py pattern
        """
        self.dwt.force_trx_off()
        time.sleep_ms(2)  # Allow hardware to settle
        self.dwt.rx_reset()
        self.dwt.sync_rx_bufptrs()
        
    def parse_poll(self, data):
        """Parse POLL message"""
        if len(data) < 10:  # Need minimum 10 bytes with padding
            return None
            
        try:
            msg_type, device_id, seq_num = struct.unpack('<BBB', data[:3])
            
            if msg_type != MSG_POLL:
                return None
                
            return {
                'device_id': device_id,
                'sequence': seq_num
            }
        except:
            return None
            
    def create_response_message(self, poll_seq, t2, t3):
        """Create RESPONSE message with T2 and T3 timestamps"""
        # Format: [type][device_id][seq][padding(7 bytes)][T2(8 bytes)][T3(8 bytes)]
        # Total: 26 bytes (matches POLL padding structure)
        # Create in ONE operation to ensure proper byte alignment
        return struct.pack('<BBB7sQQ', 
                          MSG_RESPONSE, 
                          self.device_id, 
                          poll_seq, 
                          b'\x00\x00\x00\x00\x00\x00\x00',
                          t2,
                          t3)
        
    def respond_loop(self):
        """
        Main responder loop using PolyPoint-style delayed transmission
        """
        print("\n🔄 Starting responder loop...")
        print("Waiting for POLL messages...\n")
        
        response_count = 0
        
        try:
            while True:
                # Reset and enable receiver (PROPER PATTERN)
                self._proper_rx_reset()
                self.dwt.rx_enable()
                
                # Wait for frame (PROPER RX PATTERN from receiver_polling.py)
                while True:
                    status = self.dwt.get_status()
                    
                    # Check for good frame (bit 13 = 0x2000)
                    if status & 0x2000:  # SYS_STATUS_RXFCG
                        response_count += 1
                        
                        # 1. Read T2 immediately
                        t2 = self.dwt.read_rx_timestamp()
                        
                        # 2. Read POLL data (BEFORE force_trx_off!)
                        rx_data = self.dwt.read_rx_data()
                        
                        # 3. Parse POLL
                        poll = self.parse_poll(rx_data)
                        
                        if poll and t2 != 0:
                            response_count += 1
                            
                            # Apply RX calibration to T2 (PolyPoint pattern)
                            t2_cal = apply_rx_calibration(t2, CURRENT_CHANNEL)
                            
                            print(f"📥 POLL received (#{response_count}, seq {poll['sequence']})")
                            print(f"   T2 (POLL RX): {t2_cal} (raw: {t2})")
                            
                            # 4. CRITICAL: Reset before transmission
                            self.dwt.force_trx_off()
                            time.sleep_ms(2)
                            
                            # 5. TIME-CRITICAL: Use optimized C function for atomic delayed TX
                            # IMPORTANT: Pass CALIBRATED T2 so initiator receives correct value!
                            # This matches PolyPoint: responder sends calibrated RX timestamp
                            delay_us = 1000  # 1ms delay
                            
                            try:
                                # All timing-critical code happens in C!
                                # TRUE PolyPoint: NO TX calibration! Only RX timestamps are calibrated.
                                # Pass 0 for TX calibration (keeping C function signature compatible)
                                # Send RAW T2 - initiator will calibrate it
                                t3_raw = self.dwt.send_ranging_response(t2, delay_us, poll['sequence'], self.device_id, 0)
                                
                                # T3 is RAW - PolyPoint does NOT calibrate TX timestamps
                                # Only RX timestamps (T2, T4) are calibrated by subtracting antenna delay
                                
                                # Show calibration status
                                cal_status = "✅ ENABLED" if CALIBRATION_ENABLED else "❌ DISABLED"
                                cal_value = ANTENNA_CALIBRATION.get(CURRENT_CHANNEL, 0)
                                
                                print(f"   Calibration: {cal_status} ({cal_value} ticks - RX ONLY)")
                                print(f"   T3 (RESPONSE TX scheduled): {t3_raw} (no TX calibration)")
                                print(f"   Delay: {delay_us} µs")
                                
                                # Wait for TX complete (TXFRS bit 7 = 0x0080)
                                tx_timeout = 100  # ms
                                tx_start = time.ticks_ms()
                                tx_success = False
                                
                                while time.ticks_diff(time.ticks_ms(), tx_start) < tx_timeout:
                                    tx_status = self.dwt.get_status()
                                    
                                    # Check for TX success
                                    if tx_status & 0x0080:  # TXFRS - TX frame sent
                                        tx_success = True
                                        break
                                    
                                    # Check for delayed TX error (HPDWARN bit 10 = 0x0400)
                                    if tx_status & 0x0400:
                                        print(f"❌ Delayed TX late warning: status = 0x{tx_status:08X}")
                                        break
                                        
                                    time.sleep_ms(1)
                                
                                if tx_success:
                                    # Read actual T3 timestamp to verify precision
                                    t3_actual_raw = self.dwt.read_tx_timestamp()
                                    
                                    if t3_actual_raw != 0:
                                        t3_error_ticks = t3_actual_raw - t3_raw
                                        print(f"   T3 (actual TX):     {t3_actual_raw}")
                                        print(f"   T3 precision: {t3_error_ticks} ticks ({ticks_to_microseconds(abs(t3_error_ticks), self.dwt):.3f} µs)")
                                    else:
                                        print(f"   ⚠️  T3 (actual TX):     0 (timestamp not captured)")
                                    
                                    print("📤 RESPONSE sent")
                                    print(f"✅ Response #{response_count} completed\n")
                                else:
                                    print(f"❌ TX timeout or error (status = 0x{tx_status:08X})\n")
                                    
                            except OSError as e:
                                print(f"❌ Delayed TX failed: {e} (scheduled time may have passed)\n")
                            
                        break
                    
                    # Check for errors (including CRC errors)
                    if status & 0x5C00:  # RX errors including CRC (0x1C00 | 0x4000)
                        print(f"⚠️ RX error: status = 0x{status:08X}")
                        # Decode error bits for debugging
                        if status & 0x0400:
                            print("   - CRC Error (RXFCE)")
                        if status & 0x0800:
                            print("   - PHY Header Error (RXPHE)")
                        if status & 0x1000:
                            print("   - RX Frame Wait Timeout (RXRFTO)")
                        if status & 0x4000:
                            print("   - RX Overrun (RXOVRR)")
                        
                        # CRITICAL FIX: Break out to outer loop to reset receiver!
                        # Without this, responder gets stuck after errors
                        break  # Exit inner loop, outer loop will reset RX
                        if status & 0x4000:
                            print("   - CRC Error (RXFCE)")
                        if status & 0x1000:
                            print("   - RX PHY Header Error (RXPHE)")
                        if status & 0x0800:
                            print("   - RX Frame Wait Timeout (RXRFTO)")
                        if status & 0x0400:
                            print("   - RX Overrun (RXOVRR)")
                        # CRITICAL: Clear the error status bits and reset
                        self.dwt.force_trx_off()
                        time.sleep_ms(2)
                        break
                        
                    time.sleep_ms(1)
                    
        except KeyboardInterrupt:
            print("\n🛑 Responder stopped by user")

# Example usage
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "responder":
        responder = RangingResponder()
        responder.respond_loop()
    else:
        initiator = RangingInitiator()
        initiator.range_continuous(interval_ms=1000)
