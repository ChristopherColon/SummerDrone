"""
Simple ArduPilot Flight Control
For Pixhawk 6C Mini with ArduPilot
"""
 
import time
from pymavlink import mavutil
 
class ArduPilotFlight:
    def __init__(self, connection_string, baud=57600):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        self.master.wait_heartbeat()
        print("✅ Connected to ArduPilot")
    def pre_arm_checks(self):
        """Perform pre-arm safety checks"""
        print("🔍 Performing pre-arm checks...")
        # Request system status
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            2, 1
        )
        # Wait for status messages
        checks = {
            'heartbeat': False,
            'battery': False,
            'mode_set': False
        }
        timeout = time.time() + 10
        while time.time() < timeout and not all(checks.values()):
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    mode = mavutil.mode_string_v10(msg)
                    print(f"   Current mode: {mode}")
                    checks['heartbeat'] = True
                elif msg_type == 'SYS_STATUS':
                    voltage = msg.voltage_battery / 1000.0
                    if voltage > 10.0:  # Minimum safe voltage
                        print(f"   Battery OK: {voltage:.1f}V")
                        checks['battery'] = True
                    else:
                        print(f"   ⚠️ Low battery: {voltage:.1f}V")
        return all(checks.values())
    def set_mode_and_arm(self, mode="STABILIZE"):
        """Set mode and arm for manual flight"""
        print(f"🎯 Setting {mode} mode and arming...")
        # Set mode
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        # Wait for mode confirmation
        timeout = time.time() + 5
        while time.time() < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and mavutil.mode_string_v10(msg) == mode:
                print(f"✅ {mode} mode set")
                break
            time.sleep(0.1)
        else:
            print(f"❌ Failed to set {mode} mode")
            return False
        # Arm vehicle
        print("🔓 Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        # Wait for arm confirmation
        timeout = time.time() + 10
        while time.time() < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("✅ Armed - ready for manual flight")
                return True
            time.sleep(0.1)
        print("❌ Arming failed")
        return False
    def monitor_flight(self, duration=30):
        """Monitor flight status"""
        print(f"📊 Monitoring flight for {duration} seconds...")
        end_time = time.time() + duration
        while time.time() < end_time:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    mode = mavutil.mode_string_v10(msg)
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    print(f"   Status: {mode}, Armed: {armed}")
                    if not armed:
                        print("🛬 Vehicle disarmed - flight ended")
                        break
                elif msg_type == 'GLOBAL_POSITION_INT':
                    alt = msg.relative_alt / 1000.0
                    print(f"   Altitude: {alt:.1f}m")
            time.sleep(2)
 
def main():
    """Main flight sequence"""
    print("⚠️  SAFETY CHECKLIST:")
    print("✓ RC transmitter ON and ready")
    print("✓ Props attached securely") 
    print("✓ Clear flight area")
    print("✓ Manual override ready")
    proceed = input("\nAll safety checks complete? (y/n): ")
    if proceed.lower() != 'y':
        print("🛑 Flight aborted")
        return
    # Connect to ArduPilot
    flight = ArduPilotFlight('COM5')  # Change your COM port
    try:
        # Pre-flight checks
        if not flight.pre_arm_checks():
            print("❌ Pre-arm checks failed")
            return
        # Set mode and arm
        if not flight.set_mode_and_arm("STABILIZE"):
            print("❌ Failed to set mode and arm")
            return
        print("\n🚁 VEHICLE ARMED - USE RC TRANSMITTER TO FLY")
        print("   Monitoring flight status...")
        # Monitor flight
        flight.monitor_flight(60)  # Monitor for 60 seconds
    except KeyboardInterrupt:
        print("\n🛑 Monitoring stopped by user")
 
if __name__ == "__main__":
    main()