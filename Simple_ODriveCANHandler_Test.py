import asyncio
from ODriveCANHandler import ODriveCANHandler, AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, REBOOT_ACTION_REBOOT # Import the class and constants

async def main():
    """
    Main function to test the ODriveCANHandler class.
    """
    print("Starting ODriveCANHandler test program...")

    # ---- 1. Configuration ----
    can_channel = 'can0'  # Replace with your CAN channel
    odrive_node_id = 1     # Replace with your ODrive node ID

    print(f"Using CAN channel: '{can_channel}', ODrive Node ID: {odrive_node_id}")

    # ---- 2. Instantiate ODriveCANHandler ----
    try:
        with ODriveCANHandler(channel=can_channel, node_id=odrive_node_id) as motor1:
            print("ODriveCANHandler initialized.")

            # ---- 3. Test Connection and Basic Commands ----
            print("\n--- Testing Connection and Basic Commands ---")

            if motor1.bus: # Check if bus connection was successful
                print("Successfully connected to CAN bus.")

                # --- Get Version ---
                print("\n--- Getting Version ---")
                motor1.get_version(remote_transmission_request=True) # Request version info
                await asyncio.sleep(0.1) # Allow time for message to be received and processed
                print(f"Protocol Version: {motor1.protocol_version}")
                print(f"Hardware Version: {motor1.hw_version_major}.{motor1.hw_version_minor}.{motor1.hw_version_variant}")
                print(f"Firmware Version: {motor1.fw_version_major}.{motor1.fw_version_minor}.{motor1.fw_version_revision}.{motor1.fw_version_unreleased}")

                # --- Clear Errors ---
                print("\n--- Clearing Errors ---")
                motor1.clear_errors_msg()
                print("Clear Errors command sent.")
                await asyncio.sleep(0.1)

                # --- Get Error ---
                print("\n--- Getting Errors ---")
                motor1.get_error(remote_transmission_request=True)
                await asyncio.sleep(0.1)
                print(f"Active Errors: {motor1.active_errors}")
                print(f"Disarm Reason: {motor1.disarm_reason}")

                # --- Set Axis State to IDLE ---
                print("\n--- Setting Axis State to IDLE ---")
                motor1.set_axis_state_msg(AXIS_STATE_IDLE)
                print(f"Set Axis State command sent: IDLE ({AXIS_STATE_IDLE})")
                await asyncio.sleep(0.1)

                # --- Get Heartbeat ---
                print("\n--- Getting Heartbeat ---")
                motor1.get_heartbeat(remote_transmission_request=True)
                await asyncio.sleep(0.1)
                print(f"Axis State: {motor1.state}")
                print(f"Axis Error: {motor1.axis_error}")
                print(f"Trajectory Done Flag: {motor1.traj_done_flag}")

                # --- Set Axis State to CLOSED_LOOP_CONTROL ---
                print("\n--- Setting Axis State to CLOSED_LOOP_CONTROL ---")
                motor1.set_axis_state_msg(AXIS_STATE_CLOSED_LOOP_CONTROL)
                print(f"Set Axis State command sent: CLOSED_LOOP_CONTROL ({AXIS_STATE_CLOSED_LOOP_CONTROL})")
                await asyncio.sleep(0.1)

                # --- Get Encoder Estimates ---
                print("\n--- Getting Encoder Estimates ---")
                motor1.get_encoder_estimates(remote_transmission_request=True)
                await asyncio.sleep(0.1)
                print(f"Encoder Position Estimate: {motor1.encoder_pos_estimate}")
                print(f"Encoder Velocity Estimate: {motor1.encoder_vel_estimate}")

                # --- Get Temperature ---
                print("\n--- Getting Temperature ---")
                motor1.get_temperature(remote_transmission_request=True)
                await asyncio.sleep(0.1)
                print(f"FET Temperature: {motor1.fet_temp}")
                print(f"Motor Temperature: {motor1.motor_temp}")

                # --- Get Bus Voltage and Current ---
                print("\n--- Getting Bus Voltage and Current ---")
                motor1.get_bus_voltage_current(remote_transmission_request=True)
                await asyncio.sleep(0.1)
                print(f"DC Voltage: {motor1.dc_voltage}")
                print(f"DC Current: {motor1.dc_current}")

                # --- Get Torques ---
                print("\n--- Getting Torques ---")
                motor1.get_torques(remote_transmission_request=True)
                await asyncio.sleep(0.1)
                print(f"Torque Setpoint: {motor1.torque_setpoint}")
                print(f"Torque Estimate: {motor1.torque_estimate}")

                # --- Reboot ODrive (Reboot action) ---
                print("\n--- Rebooting ODrive (Reboot action) ---")
                print("Sending Reboot command...")
                motor1.reboot_msg(REBOOT_ACTION_REBOOT)
                print("Reboot command sent. ODrive will reboot.")
                await asyncio.sleep(1) # Give time for reboot to be initiated.

            else:
                print("Failed to connect to CAN bus. Please check your CAN interface and configuration.")

    except Exception as e:
        print(f"An error occurred during the test: {e}")
    finally:
        print("\n--- Test program finished ---")

if __name__ == "__main__":
    asyncio.run(main())