"""
ArduPilot Pixhawk 6C Mini Connection Test with Pre-arm Diagnostics
Fixed for Python 3 string handling
"""
 
import time
from pymavlink import mavutil
 
class ArduPilotController:
    def __init__(self, connection_string, baud=57600):
        self.connection_string = connection_string
        self.baud = baud
        self.master = None
    def connect(self):
        """Connect to ArduPilot"""
        print("🚁 Connecting to ArduPilot Pixhawk 6C Mini...")
        try:
            self.master = mavutil.mavlink_connection(
                self.connection_string, 
                baud=self.baud
            )
            print("   Waiting for heartbeat...")
            self.master.wait_heartbeat()
            print("✅ Connected to ArduPilot successfully!")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    def safe_decode_text(self, text):
        """Safely decode text message"""
        if isinstance(text, str):
            return text
        elif isinstance(text, bytes):
            return text.decode('utf-8', errors='ignore')
        else:
            return str(text)
    def get_detailed_status(self):
        """Get detailed status including pre-arm checks"""
        print("\n📊 Detailed ArduPilot Status:")
        # Request all data streams
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4, 1
        )
        # Collect messages for 10 seconds
        end_time = time.time() + 10
        status_messages = []
        while time.time() < end_time:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    mode = mavutil.mode_string_v10(msg)
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    print(f"   Mode: {mode}")
                    print(f"   Armed: {armed}")
                elif msg_type == 'SYS_STATUS':
                    voltage = msg.voltage_battery / 1000.0
                    remaining = msg.battery_remaining
                    print(f"   Battery: {voltage:.1f}V ({remaining}%)")
                elif msg_type == 'GPS_RAW_INT':
                    print(f"   GPS: Fix type {msg.fix_type}, Satellites: {msg.satellites_visible}")
                elif msg_type == 'GLOBAL_POSITION_INT':
                    altitude = msg.relative_alt / 1000.0
                    print(f"   Altitude: {altitude:.1f}m")
                elif msg_type == 'STATUSTEXT':
                    # Safely decode status messages
                    text = self.safe_decode_text(msg.text)
                    status_messages.append(text)
                elif msg_type == 'VIBRATION':
                    print(f"   Vibration: X={msg.vibration_x:.2f}, Y={msg.vibration_y:.2f}, Z={msg.vibration_z:.2f}")
        # Print recent status messages
        if status_messages:
            print("\n📋 Recent Status Messages:")
            for msg in status_messages[-5:]:  # Last 5 messages
                print(f"   {msg}")
    def try_arm_with_errors(self):
        """Try to arm and capture error messages"""
        print("\n🔓 Attempting to arm...")
        # Clear any old messages
        while self.master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.1):
            pass
        # Try to arm
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("   Listening for arm result...")
        # Listen for results
        timeout = time.time() + 8
        arm_messages = []
        while time.time() < timeout:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        print("✅ Armed successfully!")
                        return True
                elif msg_type == 'STATUSTEXT':
                    text = self.safe_decode_text(msg.text)
                    arm_messages.append(text)
                    print(f"   {text}")
        print("❌ Arming failed")
        return False
    def close(self):
        """Close connection"""
        if self.master:
            self.master.close()
        print("🔌 Connection closed")
 
def main():
    """Test with pre-arm diagnostics"""
    # Use COM6
    drone = ArduPilotController('COM3', baud=57600)
    try:
        # Connect
        if not drone.connect():
            print("🔧 Connection failed - check:")
            print("   1. COM6 is correct port")
            print("   2. Pixhawk 6C Mini powered on")
            print("   3. USB cable connected")
            print("   4. Mission Planner closed")
            return
        # Get status
        drone.get_detailed_status()
        # Try to arm
        drone.try_arm_with_errors()
        print("\n✅ Diagnostic completed!")
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted")
    finally:
        drone.close()
 
if __name__ == "__main__":
    print("🚁 ArduPilot Pre-arm Diagnostics (Fixed)")
    print("=" * 50)
    main()