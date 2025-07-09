"""
Enhanced Pre-arm Diagnostics for ArduPilot
Forces all pre-arm checks and shows detailed results
"""
 
import time
from pymavlink import mavutil
 
class PreArmDiagnostics:
    def __init__(self, connection_string='COM6', baud=57600):
        print("🔌 Connecting to ArduPilot...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        self.master.wait_heartbeat()
        print("✅ Connected successfully")
    def safe_decode(self, text):
        """Safely decode text"""
        if isinstance(text, str):
            return text
        elif hasattr(text, 'decode'):
            return text.decode('utf-8', errors='ignore')
        else:
            return str(text)
    def clear_messages(self):
        """Clear old messages"""
        while self.master.recv_match(blocking=False, timeout=0.1):
            pass
    def force_prearm_checks(self):
        """Force pre-arm checks and collect all messages"""
        print("\n🔍 Forcing pre-arm checks...")
        self.clear_messages()
        # Method 1: Try to arm (will trigger pre-arm checks)
        print("   Attempting arm to trigger pre-arm checks...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        # Collect messages for 10 seconds
        messages = []
        prearm_messages = []
        end_time = time.time() + 10
        while time.time() < end_time:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'STATUSTEXT':
                    text = self.safe_decode(msg.text).strip()
                    messages.append(text)
                    # Look for pre-arm specific messages
                    if any(keyword in text.lower() for keyword in ['prearm', 'pre-arm', 'check', 'fail', 'error']):
                        prearm_messages.append(text)
                        print(f"   PreArm: {text}")
                elif msg_type == 'HEARTBEAT':
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        print("✅ Vehicle armed successfully!")
                        return True, []
        print(f"\n📋 Found {len(prearm_messages)} pre-arm related messages")
        return False, prearm_messages
    def check_individual_systems(self):
        """Check individual systems that commonly cause pre-arm failures"""
        print("\n🔧 Checking individual systems...")
        # Request system status
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4, 1
        )
        systems_checked = {
            'battery': False,
            'gps': False,
            'compass': False,
            'accelerometer': False,
            'gyro': False,
            'barometer': False
        }
        timeout = time.time() + 15
        while time.time() < timeout and not all(systems_checked.values()):
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'SYS_STATUS' and not systems_checked['battery']:
                    voltage = msg.voltage_battery / 1000.0
                    remaining = msg.battery_remaining
                    if voltage > 10.5:
                        print(f"✅ Battery: {voltage:.1f}V ({remaining}%) - OK")
                    else:
                        print(f"❌ Battery: {voltage:.1f}V ({remaining}%) - TOO LOW")
                    systems_checked['battery'] = True
                elif msg_type == 'GPS_RAW_INT' and not systems_checked['gps']:
                    if msg.satellites_visible >= 6 and msg.fix_type >= 3:
                        print(f"✅ GPS: {msg.satellites_visible} sats, fix type {msg.fix_type} - OK")
                    else:
                        print(f"⚠️  GPS: {msg.satellites_visible} sats, fix type {msg.fix_type} - POOR SIGNAL")
                    systems_checked['gps'] = True
                elif msg_type == 'RAW_IMU' and not systems_checked['accelerometer']:
                    print(f"✅ IMU: Accel X={msg.xacc} Y={msg.yacc} Z={msg.zacc} - DETECTED")
                    systems_checked['accelerometer'] = True
                    systems_checked['gyro'] = True
                elif msg_type == 'SCALED_PRESSURE' and not systems_checked['barometer']:
                    print(f"✅ Barometer: {msg.press_abs:.1f} mbar - DETECTED")
                    systems_checked['barometer'] = True
                elif msg_type == 'ATTITUDE' and not systems_checked['compass']:
                    yaw_deg = msg.yaw * 180 / 3.14159
                    print(f"✅ Attitude: Yaw {yaw_deg:.1f}° - COMPASS WORKING")
                    systems_checked['compass'] = True
        # Report missing systems
        for system, checked in systems_checked.items():
            if not checked:
                print(f"❌ {system.upper()}: No data received")
    def get_parameter_issues(self):
        """Check for common parameter issues"""
        print("\n⚙️  Checking critical parameters...")
        critical_params = [
            'ARMING_CHECK',
            'FS_THR_VALUE', 
            'FS_THR_ENABLE',
            'COMPASS_USE',
            'GPS_TYPE',
            'EK3_ENABLE'
        ]
        for param in critical_params:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                param.encode('utf-8'),
                -1
            )
            # Wait for response
            timeout = time.time() + 2
            while time.time() < timeout:
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if msg and self.safe_decode(msg.param_id).strip() == param:
                    print(f"   {param}: {msg.param_value}")
                    break
            else:
                print(f"   {param}: TIMEOUT")
    def run_full_diagnostics(self):
        """Run complete pre-arm diagnostics"""
        print("🚁 Running Full Pre-arm Diagnostics")
        print("=" * 50)
        # Check systems
        self.check_individual_systems()
        # Check parameters
        self.get_parameter_issues()
        # Force pre-arm checks
        armed, prearm_errors = self.force_prearm_checks()
        if armed:
            print("\n🎉 SUCCESS: Vehicle armed successfully!")
            # Disarm for safety
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            print("🔒 Disarmed for safety")
        else:
            print(f"\n❌ FAILED: {len(prearm_errors)} pre-arm issues found")
            if not prearm_errors:
                print("   No specific error messages received")
                print("   Try running diagnostics in Mission Planner")
        return armed, prearm_errors
 
def main():
    """Run diagnostics"""
    try:
        diagnostics = PreArmDiagnostics('COM3')
        armed, errors = diagnostics.run_full_diagnostics()
        if errors:
            print("\n🔧 Next steps to fix pre-arm failures:")
            for i, error in enumerate(errors, 1):
                print(f"   {i}. {error}")
    except Exception as e:
        print(f"❌ Error: {e}")
    except KeyboardInterrupt:
        print("\n🛑 Diagnostics interrupted")
 
if __name__ == "__main__":
    main()