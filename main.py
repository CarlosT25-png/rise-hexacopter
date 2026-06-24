import asyncio
import sys
import time
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# Global Parameters
CONNECTION_ADDRESS = "udp://:14551"  
TIMEOUT_CONNECTION = 15.0
TARGET_ALTITUDE_METERS = 5.0
TAKEOFF_DELAY=2.0 # change to 20.0 for a real test
HOVER_TIME = 10.0
OFFBOARD_SETPOINT_HZ = 10
OFFBOARD_PRIME_SECONDS = 2.0
CLIMB_SPEED_M_S = 1.5

async def run_resilient_flight():
    drone = System()
    await drone.connect(system_address=CONNECTION_ADDRESS)
    
    print("Connecting to autopilot...")
    try:
        await asyncio.wait_for(verify_drone_connection(drone), TIMEOUT_CONNECTION)
        print("Connection successful!")
    except asyncio.TimeoutError:
        print("CRITICAL ERROR: Connection timed out.")
        return

    # Pre flight checks
    print("Waiting for preflight checks...")
    if not await wait_for_preflight(drone):
        return
      
    # Take off delay
    print(f"{TAKEOFF_DELAY}s takeoff delay")
    await asyncio.sleep(TAKEOFF_DELAY)

    # Arm
    if not await robust_arm(drone, max_attempts=5):
        return

    # Takeoff
    if not await robust_takeoff(drone, TARGET_ALTITUDE_METERS, max_attempts=3):
        await drone.action.land()
        return

    # Fly movements
    print("\nStart back and forth movements")
    hover_command = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)

    try:
        await engage_offboard(drone, hover_command)
        print("Offboard Mode Engaged.")
    except OffboardError as error:
        print(f"Failed to engage offboard: {error}")
        await drone.action.land()
        return
        
    print(f"Hovering for {HOVER_TIME} seconds...")
    await asyncio.sleep(HOVER_TIME)
    
    print("Flying forward...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(2.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5) 

    print("Flying backward...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-2.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("Stopping (Hovering)...")
    await drone.offboard.set_velocity_body(hover_command)
    await asyncio.sleep(2)

    print("Stopping Offboard mode and Landing...")
    await drone.offboard.stop()
    await drone.action.land()

async def verify_drone_connection(drone: System):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return True

async def wait_for_preflight(drone: System, timeout_s=120):
    for _ in range(timeout_s):
        async for health in drone.telemetry.health():
            # Add GPS checks when needed:
            # health.is_global_position_ok and health.is_local_position_ok and
            if (health.is_accelerometer_calibration_ok and
                health.is_magnetometer_calibration_ok):
                print("Sensors ready")
                return True
            break
        await asyncio.sleep(1)
    
    print("ERROR: Preflight checks timed out.", file=sys.stderr)
    return False

async def robust_arm(drone: System, max_attempts=5):
    print("Attempting to arm...")
    for attempt in range(1, max_attempts + 1):
        try:
            await drone.action.arm()
            print("Armed successfully.")
            return True
        except ActionError as e:
            print(f"Arm attempt {attempt} failed: {e}. Retrying in 2s...")
            await asyncio.sleep(2)
            
    print("CRITICAL ERROR: Failed to arm after maximum attempts.", file=sys.stderr)
    return False

async def stream_offboard_setpoint(drone: System, setpoint: VelocityBodyYawspeed, duration_s: float):
    interval = 1.0 / OFFBOARD_SETPOINT_HZ
    for _ in range(int(duration_s * OFFBOARD_SETPOINT_HZ)):
        await drone.offboard.set_velocity_body(setpoint)
        await asyncio.sleep(interval)

async def engage_offboard(drone: System, setpoint: VelocityBodyYawspeed):
    # ArduPilot requires setpoints streamed before accepting offboard/GUIDED.
    await stream_offboard_setpoint(drone, setpoint, OFFBOARD_PRIME_SECONDS)
    await drone.offboard.start()

async def safe_offboard_stop(drone: System):
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass

async def robust_takeoff(drone: System, target_alt: float, max_attempts=3):
    hover = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    climb = VelocityBodyYawspeed(0.0, 0.0, -CLIMB_SPEED_M_S, 0.0)

    print(f"Attempting takeoff to {target_alt}m via offboard climb...")
    for attempt in range(1, max_attempts + 1):
        print(f"Takeoff attempt {attempt}/{max_attempts}...")
        try:
            await engage_offboard(drone, hover)
            print("Offboard engaged, climbing...")

            deadline = time.monotonic() + (target_alt / CLIMB_SPEED_M_S) * 3 + 30
            while time.monotonic() < deadline:
                await drone.offboard.set_velocity_body(climb)
                async for position in drone.telemetry.position():
                    current_alt = position.relative_altitude_m
                    print(f"Climbing... {current_alt:.2f}m / {target_alt}m")
                    if current_alt >= (target_alt * 0.95):
                        await drone.offboard.set_velocity_body(hover)
                        print("Target altitude reached")
                        await safe_offboard_stop(drone)
                        return True
                    break
                await asyncio.sleep(0.2)

            print("Climb timed out, retrying...")
            await safe_offboard_stop(drone)
        except OffboardError as e:
            print(f"Takeoff attempt {attempt} failed: {e}. Retrying in 5s...")
            await safe_offboard_stop(drone)
            await asyncio.sleep(5)

    print("CRITICAL ERROR: Takeoff failed after maximum attempts.", file=sys.stderr)
    print(
        "ArduPilot needs GUIDED/offboard + a position estimate. "
        "Check EKF/GPS or indoor no-GPS params in GCS.",
        file=sys.stderr,
    )
    return False

if __name__ == "__main__":
    asyncio.run(run_resilient_flight())