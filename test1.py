"""
Simple Arm Test - Check if vehicle actually arms
"""
 
import time
from pymavlink import mavutil
 
def test_arm():
    print("🔌 Connecting to COM3...")
    master = mavutil.mavlink_connection('COM3', baud=57600)
    master.wait_heartbeat()
    print("✅ Connected")
    # Check current status
    print("\n📊 Current Status:")
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"   Mode: {mode}")
        print(f"   Armed: {armed}")
    # Set STABILIZE mode first
    print("\n🎯 Setting STABILIZE mode...")
    mode_id = master.mode_mapping()['STABILIZE']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    # Wait for mode change
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and mavutil.mode_string_v10(msg) == 'STABILIZE':
            print("✅ STABILIZE mode set")
            break
    # Try to arm
    print("\n🔓 Attempting to arm...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    # Check if armed
    timeout = time.time() + 10
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                print("🎉 SUCCESS: Vehicle ARMED!")
                # Wait 3 seconds then disarm
                print("   Waiting 3 seconds...")
                time.sleep(3)
                print("🔒 Disarming for safety...")
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                print("✅ Test completed successfully!")
                master.close()
                return True
    print("❌ Vehicle did NOT arm")
    master.close()
    return False
 
if __name__ == "__main__":
    print("🚁 Simple Arm Test")
    print("=" * 30)
    # Safety check
    print("⚠️  SAFETY: Ensure props are OFF or removed!")
    proceed = input("Props removed/safe? (y/n): ")
    if proceed.lower() == 'y':
        success = test_arm()
        if success:
            print("\n🎯 GREAT! Pre-arm checks are passing!")
            print("   Ready to move to autonomous flight code")
        else:
            print("\n🔧 Pre-arm checks may be failing silently")
            print("   Check Mission Planner for detailed errors")
    else:
        print("🛑 Test aborted for safety")