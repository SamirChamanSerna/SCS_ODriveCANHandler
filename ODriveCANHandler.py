import asyncio
import can
import struct
import time
import math

ADDRESS_CMD = 0x06
SET_AXIS_STATE_CMD = 0x07
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1
REBOOT_ACTION_ERASE = 2

ODRIVE_TIMEOUT = 1.0

AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


class ODriveCANHandler:
    """
    A class to handle CAN bus communication with an ODrive device using CANSimple protocol.
    Includes methods for all CANSimple messages.
    """

    def __init__(self, channel, node_id):
        """
        Initialize the CAN handler.

        :param channel: can.Bus instance or channel string
        :param node_id: ODrive node ID on the CAN bus.
        """
        self.channel = channel
        self.node_id = node_id
        self.bus = None
        self.notifier = None
        self._is_running = False
        self.reader = can.BufferedReader()  # Initialize the reader here

        self.last_timestamp = 0.0

        # telemetry - populated by on_message
        self.error = None
        self.state = AXIS_STATE_UNDEFINED
        self.vel = None
        self.dc_voltage = None
        self.dc_current = None
        self.torque_setpoint = None
        self.torque_estimate = None
        self.fet_temp = None
        self.motor_temp = None
        self.iq_setpoint = None
        self.iq_measured = None
        self.encoder_pos_estimate = None
        self.encoder_vel_estimate = None
        self.active_errors = None
        self.disarm_reason = None
        self.traj_done_flag = None
        self.protocol_version = None
        self.hw_version_major = None
        self.hw_version_minor = None
        self.hw_version_variant = None
        self.fw_version_major = None
        self.fw_version_minor = None
        self.fw_version_revision = None
        self.fw_version_unreleased = None


    def __enter__(self):
        if self.bus is None:
            self.connect() # Ensure bus is connected if not already
        if self.bus is not None and self.notifier is None: # Prevent creating notifier if bus connection failed
            self.notifier = can.Notifier(self.bus, [self.reader, self.on_message]) # Removed asyncio loop as it's managed by can.Notifier internally if needed
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.notifier:
            self.notifier.stop()
            self.notifier = None # Ensure notifier is reset
        if self.bus:
            self.bus.shutdown() # Shutdown the bus to release resources
            self.bus = None # Ensure bus is reset


    def flush_rx(self):
        self.reader.flush() # Use built-in flush method for BufferedReader

    async def await_msg(self, cmd_id: int): # Removed timeout parameter
        async def _impl():
            while True: # Changed to while True and break inside for better control
                msg = await self.reader.get_message() # Use async get_message, blocks indefinitely
                if msg is not None and msg.arbitration_id == (self.node_id << 5 | cmd_id):
                    return msg
        return await _impl() # Await the _impl coroutine, no timeout now

    def clear_errors_msg(self, identify: bool = False):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | CLEAR_ERRORS_CMD,
            data=b'\x01' if identify else b'\x00',
            is_extended_id=False
        ))

    def reboot_msg(self, action: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | REBOOT_CMD,
            data=[action],
            is_extended_id=False
        ))

    def set_axis_state_msg(self, state: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | SET_AXIS_STATE_CMD),
            data=struct.pack('<I', state),
            is_extended_id=False
        ))

    def send_message(self, arbitration_id, data, remote_transmission_request=False):
        """
        Sends a CAN message on the bus.

        :param arbitration_id: The arbitration ID of the CAN message.
        :param data: The data payload of the CAN message (bytes).
        :param remote_transmission_request: Set to True to send RTR frame (request data)
        """
        if not self.connected(): # Corrected method name here
            print("Not connected to CAN bus. Cannot send message.")
            return False
        if self.bus is None: # Added check if bus is initialized
            print("CAN bus is not initialized. Call connect() first.")
            return False
        try:
            message = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=False,
                is_remote_frame=remote_transmission_request  # Set RTR bit if requested
            )
            self.bus.send(message)
            return True
        except can.CanError as e:
            print(f"Error sending CAN message: {e}")
            return False

    def on_message(self, msg: can.Message):
        """Handles incoming CAN messages and updates telemetry data."""
        cmd_id = msg.arbitration_id & 0x1F  # Extract command ID from message ID

        if msg.arbitration_id >> 5 != self.node_id:
            return # ignore messages not for this node

        if cmd_id == 0x00:  # Get_Version
            try:
                (self.protocol_version, self.hw_version_major, self.hw_version_minor, self.hw_version_variant,
                self.fw_version_major, self.fw_version_minor, self.fw_version_revision, self.fw_version_unreleased) = struct.unpack('<BBBBBBBB', msg.data)
            except struct.error as e:
                print(f"Error unpacking Get_Version message: {e}, data: {msg.data}")
        elif cmd_id == 0x01:  # Heartbeat
            try:
                # Assuming Heartbeat message data is 4 bytes as per variable names (error, state, procedure_result, trajectory_done_flag)
                self.axis_error, self.state, self.procedure_result, self.traj_done_flag = struct.unpack('<IBBB', msg.data[:4]) # Unpack first 4 bytes
            except struct.error as e:
                print(f"Error unpacking Heartbeat message: {e}, data: {msg.data}")
            self.last_timestamp = time.monotonic()
        elif cmd_id == 0x03:  # Get_Error
            try:
                self.active_errors, self.disarm_reason = struct.unpack('<II', msg.data)
            except struct.error as e:
                print(f"Error unpacking Get_Error message: {e}, data: {msg.data}")
        elif cmd_id == 0x05:  # TxSdo (SDO Response) - You'll likely want to handle SDO responses elsewhere, based on requests
            pass # Handle SDO responses if needed, maybe using a queue or callback.
        elif cmd_id == 0x09:  # Get_Encoder_Estimates
            try:
                self.encoder_pos_estimate, self.encoder_vel_estimate = struct.unpack('<ff', msg.data)
            except struct.error as e:
                print(f"Error unpacking Get_Encoder_Estimates message: {e}, data: {msg.data}")
        elif cmd_id == 0x14:  # Get_Iq
            try:
                self.iq_setpoint, self.iq_measured = struct.unpack('<ff', msg.data)
            except struct.error as e:
                print(f"Error unpacking Get_Iq message: {e}, data: {msg.data}")
        elif cmd_id == 0x15:  # Get_Temperature
            try:
                self.fet_temp, self.motor_temp = struct.unpack('<ff', msg.data) # Removed redundant bytes()
                self.fet_temp = None if math.isnan(self.fet_temp) else self.fet_temp
                self.motor_temp = None if math.isnan(self.motor_temp) else self.motor_temp
            except struct.error as e:
                print(f"Error unpacking Get_Temperature message: {e}, data: {msg.data}")
        elif cmd_id == 0x17: # Get_Bus_Voltage_Current
            try:
                self.dc_voltage, self.dc_current = struct.unpack('<ff', msg.data)
            except struct.error as e:
                print(f"Error unpacking Get_Bus_Voltage_Current message: {e}, data: {msg.data}")
        elif cmd_id == 0x1C: # Get_Torques
            try:
                self.torque_setpoint, self.torque_estimate = struct.unpack('<ff', msg.data)
            except struct.error as e:
                print(f"Error unpacking Get_Torques message: {e}, data: {msg.data}")

    def connect(self):
        """
        Connect to the CAN bus and start the notifier.
        """
        try:
            self.bus = can.interface.Bus(self.channel, bustype='socketcan') # Bus is created here
            self.notifier = can.Notifier(self.bus, [self.reader, self.on_message]) # Notifier is created here after bus is ready
            self._is_running = True
            print(f"Connected to CAN bus '{self.channel}' with ODrive node ID {self.node_id}")
        except Exception as e:
            print(f"Error connecting to CAN bus: {e}")
            self.bus = None
            self.notifier = None

    def connected(self, now=None):
        if now is None:
            now = time.monotonic()
        return now - self.last_timestamp < ODRIVE_TIMEOUT if self.last_timestamp else False

    # ------------------------------------------
    # CANSimple Message Implementations
    # ------------------------------------------

    def get_version(self, remote_transmission_request=False):
        """Requests the Get_Version message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x00  # 0x00: Get_Version
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def get_heartbeat(self, remote_transmission_request=False):
        """Requests the Heartbeat message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x01  # 0x01: Heartbeat
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def send_estop(self):
        """Sends the Estop (Emergency Stop) message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x02  # 0x02: Estop
        return self.send_message(arbitration_id, b'')

    def get_error(self, remote_transmission_request=False):
        """Requests the Get_Error message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x03  # 0x03: Get_Error
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def send_rxsdo(self, opcode, endpoint_id, value):
        """Sends an RxSdo (SDO Request) message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x04  # 0x04: RxSdo
        data = struct.pack('<BHB', opcode, endpoint_id, 0) + value # Assuming value is already packed bytes
        return self.send_message(arbitration_id, data)

    def get_txsdo(self, remote_transmission_request=False): # Typically you don't request TxSDO, it's a response
        """Requests the TxSdo (SDO Response) message from the ODrive (uncommon to request)."""
        arbitration_id = (self.node_id << 5) | 0x05  # 0x05: TxSdo
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def send_address(self, node_id_new, serial_number):
        """Sends an Address message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x06  # 0x06: Address
        data = struct.pack('<Bq', node_id_new, serial_number)
        return self.send_message(arbitration_id, data)

    def set_axis_state(self, axis_state):
        """Sets the axis state using Set_Axis_State message."""
        arbitration_id = (self.node_id << 5) | 0x07  # 0x07: Set_Axis_State
        data = struct.pack('<I', axis_state)
        return self.send_message(arbitration_id, data)

    def get_encoder_estimates(self, remote_transmission_request=False):
        """Requests the Get_Encoder_Estimates message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x09  # 0x09: Get_Encoder_Estimates
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def set_controller_mode(self, control_mode, input_mode):
        """Sends a Set_Controller_Mode message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x0B  # 0x0B: Set_Controller_Mode
        data = struct.pack('<II', control_mode, input_mode)
        return self.send_message(arbitration_id, data)

    def set_input_pos(self, input_pos, vel_feedforward, torque_feedforward):
        """Sends a Set_Input_Pos message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x0C  # 0x0C: Set_Input_Pos
        data = struct.pack('<fff', input_pos, vel_feedforward, torque_feedforward)
        return self.send_message(arbitration_id, data)

    def set_input_vel(self, input_vel, input_torque_ff):
        """Sends a Set_Input_Vel message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x0D  # 0x0d: Set_Input_Vel
        data = struct.pack('<ff', input_vel, input_torque_ff)
        return self.send_message(arbitration_id, data)

    def set_input_torque(self, input_torque):
        """Sends a Set_Input_Torque message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x0E  # 0x0e: Set_Input_Torque
        data = struct.pack('<f', input_torque)
        return self.send_message(arbitration_id, data)

    def set_limits(self, velocity_limit, current_limit):
        """Sends a Set_Limits message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x0F  # 0x0f: Set_Limits
        data = struct.pack('<ff', velocity_limit, current_limit)
        return self.send_message(arbitration_id, data)

    def set_traj_vel_limit(self, traj_vel_limit):
        """Sends a Set_Traj_Vel_Limit message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x11  # 0x11: Set_Traj_Vel_Limit
        data = struct.pack('<f', traj_vel_limit)
        return self.send_message(arbitration_id, data)

    def set_traj_accel_limits(self, traj_accel_limit, traj_decel_limit):
        """Sends a Set_Traj_Accel_Limits message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x12  # 0x12: Set_Traj_Accel_Limits
        data = struct.pack('<ff', traj_accel_limit, traj_decel_limit)
        return self.send_message(arbitration_id, data)

    def set_traj_inertia(self, traj_inertia):
        """Sends a Set_Traj_Inertia message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x13  # 0x13: Set_Traj_Inertia
        data = struct.pack('<f', traj_inertia)
        return self.send_message(arbitration_id, data)

    def get_iq(self, remote_transmission_request=False):
        """Requests the Get_Iq message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x14  # 0x14: Get_Iq
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def get_temperature(self, remote_transmission_request=False):
        """Requests the Get_Temperature message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x15  # 0x15: Get_Temperature
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def reboot(self, action=REBOOT_ACTION_REBOOT):
        """Sends a Reboot message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x16  # 0x16: Reboot
        data = struct.pack('<B', action)
        return self.send_message(arbitration_id, data)

    def get_bus_voltage_current(self, remote_transmission_request=False):
        """Requests the Get_Bus_Voltage_Current message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x17  # 0x17: Get_Bus_Voltage_Current
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def set_absolute_position(self, position):
        """Sends a Set_Absolute_Position message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x19  # 0x19: Set_Absolute_Position
        data = struct.pack('<f', position)
        return self.send_message(arbitration_id, data)

    def set_pos_gain(self, pos_gain):
        """Sends a Set_Pos_Gain message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x1A  # 0x1a: Set_Pos_Gain
        data = struct.pack('<f', pos_gain)
        return self.send_message(arbitration_id, data)

    def set_vel_gains(self, vel_gain, vel_integrator_gain):
        """Sends a Set_Vel_Gains message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x1B  # 0x1b: Set_Vel_Gains
        data = struct.pack('<ff', vel_gain, vel_integrator_gain)
        return self.send_message(arbitration_id, data)

    def get_torques(self, remote_transmission_request=False):
        """Requests the Get_Torques message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x1C  # 0x1c: Get_Torques
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def get_powers(self, remote_transmission_request=False):
        """Requests the Get_Powers message from the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x1D  # 0x1d: Get_Powers
        return self.send_message(arbitration_id, b'', remote_transmission_request)

    def enter_dfu_mode(self):
        """Sends the Enter_DFU_Mode message to the ODrive."""
        arbitration_id = (self.node_id << 5) | 0x1F  # 0x1f: Enter_DFU_Mode
        return self.send_message(arbitration_id, b'')