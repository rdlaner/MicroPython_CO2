"""CO2 Sensor Application"""
# pylint: disable=import-error, wrong-import-position, c-extension-no-member, no-member, no-name-in-module, import-outside-toplevel
# TODO: Apply received miniot data when received.
# TODO: Convert to settings.toml instead of secrets file...?
# TODO: Battery improvement - collect several samples and publish them at longer intervals
# TODO: Add base class for HA types (eg for sensor, number, etc.)

# Using lazy loading of modules to reduce "boot" time as not all are needed for every boot
# Standard imports
import time
app_start = now = time.monotonic()
from micropython import const

# Third party imports
import adafruit_logging as logging
from cp_libs.memory import BackupRAM

# Local imports
from config import config

# Constants
FORCE_CAL_DISABLED = const(-1)
SCD4X_DEFAULT_ADDR = const(0x62)
SCD41_CMD_SINGLE_SHOT = const(0x219D)
SINGLE_SHOT_SLEEP_SEC = const(5)
SENSOR_READ_TIMEOUT_SEC = const(5)
STATE_LIGHT_SLEEP = const(0)
STATE_DEEP_SLEEP_SAMPLING = const(1)
STATE_DEEP_SLEEP = const(2)
BPI_BATT_DXN_VOLTAGE = 4.20
BACKUP_NAME_PRESSURE = "pressure"
BACKUP_NAME_CAL = "forced cal"
BACKUP_NAME_TEMP_OFFSET = "temp offset"
BACKUP_NAME_DISPLAY_TIME = "display"
BACKUP_NAME_TIME_SYNC_TIME = "time sync"
BACKUP_NAME_UPLOAD_TIME = "upload"
BACKUP_NAME_RX_TIME = "receive"
BACKUP_NAME_STATE = "state"
BACKUP_NAME_ENABLE_NETWORK = "network"
BACKUP_NAME_SEND_DISCOVERY_TIME = "discovery"
NUMBER_NAME_TEMP_OFFSET = "Temp Offset"
NUMBER_NAME_PRESSURE = "Pressure"
NUMBER_NAME_CO2_REF = "CO2 Ref"
SENSOR_NAME_BATTERY = "Batt Voltage"
SENSOR_NAME_SCD41_CO2 = "SCD41 CO2"
SENSOR_NAME_SCD41_HUM = "SCD41 Humidity"
SENSOR_NAME_SCD41_TEMP = "SCD41 Temperature"

# Globals
backup_ram = BackupRAM()
logger = logging.getLogger("CO2")
logger.setLevel(config["logging_level"])


# MQTT Callbacks
# pylint: disable=unused-argument
def mqtt_disconnected(client: MQTT.MQTT, user_data, return_code: int) -> None:
    """Callback for when MQTT client is disconnected from the broker"""
    logger.debug("MQTT disconnected callback")


def mqtt_message(client: MQTT.MQTT, topic: str, message: str) -> None:
    """Callback for when MQTT client's subscribed topic receives new data"""
    logger.debug(f"New message on topic {topic}: {message}")

    if config["topics"]["pressure_topic"] == topic:
        try:
            pressure = round(float(message))
        except ValueError as exc:
            import traceback
            logger.error("Ambient pressure value invalid:")
            logger.error(f"{''.join(traceback.format_exception(exc, chain=True))}")
            return

        logger.info(f"Updating backup pressure to {pressure}")
        backup_ram.set_element(BACKUP_NAME_PRESSURE, pressure)

    elif config["topics"]["cmd_topic"] == topic:
        import json
        try:
            obj = json.loads(message)
        except ValueError as exc:
            import traceback
            logger.error("Forced calibration value invalid:")
            logger.error(f"{''.join(traceback.format_exception(exc, chain=True))}")
            return

        if NUMBER_NAME_CO2_REF in obj:
            cal_val = obj[NUMBER_NAME_CO2_REF]
            backup_ram.set_element(BACKUP_NAME_CAL, cal_val)
            logger.info(f"Received new cal val: {cal_val}")

        if NUMBER_NAME_TEMP_OFFSET in obj:
            temp_offset = obj[NUMBER_NAME_TEMP_OFFSET]
            backup_ram.set_element(BACKUP_NAME_TEMP_OFFSET, temp_offset)
            logger.info(f"Received new temp offset: {temp_offset}")


def backup_ram_init():
    """Initialize persistent data"""
    from supervisor import runtime

    backup_ram.reset()
    backup_ram.add_element(BACKUP_NAME_PRESSURE, "I", config["ambient_pressure"])
    backup_ram.add_element(BACKUP_NAME_CAL, "i", FORCE_CAL_DISABLED)
    backup_ram.add_element(BACKUP_NAME_TEMP_OFFSET, "f", config["temp_offset_c"])
    backup_ram.add_element(BACKUP_NAME_ENABLE_NETWORK, "B", config["enable_network"])
    backup_ram.add_element(BACKUP_NAME_UPLOAD_TIME, "I", 0)
    backup_ram.add_element(BACKUP_NAME_TIME_SYNC_TIME, "I", 0)
    backup_ram.add_element(BACKUP_NAME_SEND_DISCOVERY_TIME, "I", 0)
    backup_ram.add_element(BACKUP_NAME_RX_TIME, "I", 0)

    if config["display_enable"]:
        backup_ram.add_element(BACKUP_NAME_DISPLAY_TIME, "I", 0)

    if runtime.usb_connected and not config["force_deep_sleep"]:
        backup_ram.add_element(BACKUP_NAME_STATE, "I", STATE_LIGHT_SLEEP)
    else:
        backup_ram.add_element(BACKUP_NAME_STATE, "I", STATE_DEEP_SLEEP_SAMPLING)


def c_to_f(temp_cels: float) -> float:
    """"Convert celsius to fahrenheit"""
    return (temp_cels * 1.8) + 32.0 if temp_cels else None


def deep_sleep(sleep_time: float) -> None:
    """Deep sleep for specified time.

    Assuming eps32 hardware where device reboots after deep sleep.

    Args:
        sleep_time (float): Deep sleep duration in seconds
    """
    import alarm
    logger.info("Sleeping...\n")
    time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + sleep_time)
    alarm.exit_and_deep_sleep_until_alarms(time_alarm)


def display_init() -> "Display":
    """Initialize and return a display object.

    Returns:
        Display: New display object
    """
    from display import Display
    return Display()


def display_update(display_time: int, sensor_data: list, display: Display = None) -> "Display":
    """Perform display update if time has passed defined refresh rate.

    Args:
        display_time (int): Time of last display update.
        sensor_data (list): CO2, temp, hum, and battery data.
        display (Display, optional): Display instance. Defaults to None.

    Returns:
        Display: Returns display instance in case it was created here. None if not used.
    """
    if config["display_enable"] and (time.time() - display_time) >= config["display_refresh_rate_sec"]:
        logger.info("Updating display...")
        from cp_libs.time import get_fmt_time
        if not display:
            display = display_init()

        if (
            config["enable_network"] and
            config["network_transport"] != "espnow" and
            config["network_transport"] != "miniot"
        ):
            now = get_fmt_time()
            uploaded_time = get_fmt_time(backup_ram.get_element(BACKUP_NAME_UPLOAD_TIME))
            display.update_datetime(f"Updated: {now}. Uploaded: {uploaded_time}")

        display.update_co2(sensor_data[SENSOR_NAME_SCD41_CO2])
        display.update_temp(c_to_f(sensor_data[SENSOR_NAME_SCD41_TEMP]))
        display.update_hum(sensor_data[SENSOR_NAME_SCD41_HUM])
        display.update_batt(sensor_data[SENSOR_NAME_BATTERY])
        display.refresh(delay=False)
        backup_ram.set_element(BACKUP_NAME_DISPLAY_TIME, time.time())

    return display


def network_connect(net: Network) -> bool:
    """Connect network.

    If network is using an MQTT transport, will also sub to topics.

    Args:
        net (Network): Network object.

    Returns:
        bool: True if connected, False if failed.
    """
    if not net.connect():
        return False

    if config["network_transport"] == "mqtt":
        for topic in config["topics"].values():
            logger.info(f"Subscribing to {topic}...")
            net.subscribe(topic)

    return True


def network_init(device_state: int) -> "Network":
    """Creates and returns a Network instance specified in config file

    Args:
        device_state (int): Current device state.

    Raises:
        RuntimeError: Unsupported network transport from config

    Returns:
        Network: New Network instance.
    """
    logger.info("Initializing network ")
    from cp_libs.network import Network

    if config["network_transport"] == "espnow":
        network = Network.create_espnow(config["network_prefix_id"])
    elif config["network_transport"] == "miniot":
        network = Network.create_min_iot(config["network_prefix_id"])
    elif config["network_transport"] == "mqtt":
        if device_state == STATE_LIGHT_SLEEP:
            keep_alive_sec = config["light_sleep_sec"] + config["keep_alive_margin_sec"]
        else:
            keep_alive_sec = config["deep_sleep_sec"] + config["keep_alive_margin_sec"]

        network = Network.create_mqtt(config["network_prefix_id"],
                                      keep_alive_sec=keep_alive_sec,
                                      on_disconnect_cb=mqtt_disconnected,
                                      on_message_cb=mqtt_message)
    else:
        raise RuntimeError(f"Unsupported network transport: {config['network_transport']}")

    return network


def light_sleep(sleep_time: float) -> "Alarm":
    """Light sleep for specified time.

    Args:
        sleep_time (float): Light sleep duration in seconds

    Returns:
        Alarm: Any alarm that caused wake from sleep
    """
    import alarm
    import gc
    logger.info("Sleeping...\n")
    time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + sleep_time)
    triggered_alarm = alarm.light_sleep_until_alarms(time_alarm)
    gc.collect()

    return triggered_alarm


def process_calibration(scd41: SCD4X, current_cal_val: int) -> bool:
    """Perform forced recal if received new cal value.

    Args:
        scd41 (SCD4X): SCD4x instance.
        current_cal_val (int): current calibration value.

    Returns:
        bool: True if updated, False if not.
    """
    updated = False
    expected_cal_val = backup_ram.get_element(BACKUP_NAME_CAL)

    if expected_cal_val != FORCE_CAL_DISABLED and expected_cal_val != current_cal_val:
        logger.info(f"Updating cal reference from {current_cal_val} to {expected_cal_val}")
        scd41.force_calibration(expected_cal_val)
        updated = True

    return updated


def process_pressure(scd41: SCD4X, current_pressure: int) -> bool:
    """Update pressure if received a new value.

    Args:
        scd41 (SCD4X): SCD4x instance.
        current_pressure (int): Current pressure value.

    Returns:
        bool: True if updated, False if not.
    """
    updated = False
    expected_pressure = backup_ram.get_element(BACKUP_NAME_PRESSURE)

    if expected_pressure != current_pressure:
        logger.info(f"Updating pressure from {current_pressure} to {expected_pressure}")
        scd41.set_ambient_pressure(expected_pressure)
        updated = True

    return updated


def process_temp_offset(scd41: SCD4X, current_temp_offset: float) -> bool:
    """Update temp offset if received a new value

    Args:
        scd41 (SCD4X): SCD4x instance.
        current_temp_offset (float): Current temperature offset value.

    Returns:
        bool: True if updated, False if not.
    """
    updated = False
    expected_temp_offset = backup_ram.get_element(BACKUP_NAME_TEMP_OFFSET)

    if expected_temp_offset != current_temp_offset:
        logger.info(f"Updating temp offset from {current_temp_offset} to {expected_temp_offset}")
        scd41.stop_periodic_measurement()
        scd41.temperature_offset = expected_temp_offset
        updated = True

    return updated


def reboot() -> None:
    """Reboot device.

    Should be used for non-failure, intended device reboots. If attempting to recover a device
    due to some failure, use `reset` instead.
    """
    from supervisor import reload
    logger.warning("Rebooting...")
    reload()


def reset(msg: str = None) -> None:
    """Reset device.

    Should be used for attempting to recover a device due to unrecoverable failures.

    Instead of resetting directly here, we will throw an exception that will get caught by main.py.
    main.py will write the optional message to the file system and then clean up before rebooting.

    Args:
        msg (str, optional): Optional reboot message.
    """
    logger.warning("Rebooting...")
    raise RuntimeError(msg)


def receive_data(receive_time: int,
                 device_state: int,
                 net: Network = None,
                 recover: bool = False) -> "Network":
    """Attempt to receive any network data available if configured receive rate has elapsed.

    Initializing a network instance can take somewhere around 0.5 second. In order to avoid
    unneeded initialization, this function will initialize a network instance if none is provided
    but only when it is needed to receive data.
    If it does init a new network instance, it will return is so that the caller can use the
    single instance.
    If data is received, this function will also update the receive time in backup ram.

    Args:
        receive_time (int): Time of last receive.
        device_state (int): Current device state.
        net (Network, optional): Network instance. Defaults to None.
        recover (bool, optional): Attempt to recover network if receive fails. Defaults to False.

    Returns:
        Network: Returns network instance in case it was initialized here. None if not used.
    """
    if (time.time() - receive_time) >= config["receive_rate_sec"]:
        logger.info("Receiving data...")

        if not net:
            net = network_init(device_state)

        if not net.is_connected():
            network_connect(net)

        # Receive any available data. If using MQTT, this will call its loop function.
        rxed_data = []
        rx_win = config["receive_window_sec"]
        start = time.monotonic()

        while (start + rx_win) > time.monotonic():
            try:
                net.receive(rxed_data, recover=recover)
            except Exception as exc:
                import traceback
                reset(f"Failed receiving data\n{''.join(traceback.format_exception(exc, chain=True))}")
            else:
                backup_ram.set_element(BACKUP_NAME_RX_TIME, time.time())

        if rxed_data:
            logger.info(f"Received data: {rxed_data}")

    return net


def scd41_init(first_boot: bool, device_state: int) -> "SCD4X":
    """Create and initialize an scd41 instance.

    Note:
    Measurements must be stopped in order to make configuration changes, however, config
    updates persist across reboots.
    Therefore, only stop measurements and perform initializations once in order to prevent
    unnecessary latency in deep sleep mode.

    Args:
        first_boot (bool): True if first boot from power-on, False if not.
        device_state (int): Current device state.

    Returns:
        SCD4X: New SCD4X instance.
    """
    from adafruit_scd4x import SCD4X
    from board import SCL, SDA
    from busio import I2C
    i2c = I2C(SCL, SDA, frequency=100000)
    scd41 = SCD4X(i2c)

    if first_boot:
        logger.info("Initializing SCD41:")
        scd41.stop_periodic_measurement()

        logger.info("Updating self cal mode to OFF")
        scd41.self_calibration_enabled = False

        temp_offset = backup_ram.get_element(BACKUP_NAME_TEMP_OFFSET)
        logger.info(f"Updating temperature offset to {temp_offset}")
        scd41.temperature_offset = temp_offset

        pressure = backup_ram.get_element(BACKUP_NAME_PRESSURE)
        logger.info(f"Updating pressure to {pressure}")
        scd41.set_ambient_pressure(pressure)

        logger.info("Persisting settings to EEPROM")
        scd41.persist_settings()

    if device_state == STATE_LIGHT_SLEEP:
        scd41.start_periodic_measurement()  # High performance mode

    return scd41


def send_data(upload_time: int,
              co2_device: HomeAssistantDevice,
              device_state: int,
              net: Network = None,) -> "Network":
    """Perform sensor data upload if configured upload rate has elapsed.

    Initializing a network instance can take somewhere around 0.5 second. In order to avoid
    unneeded initialization, this function will initialize a network instance if none is provided
    but only when it is needed to send data.
    If it does init a new network instance, it will return is so that the caller can use the
    single instance.
    If data is sent, this function will also update the upload time in backup ram.

    Args:
        upload_time (int): Time of last upload.
        co2_device (HomeAssistantDevice): CO2 device instance.
        device_state (int): Current device state.
        net (Network, optional): Network instance. Defaults to None.

    Returns:
        Network: Returns network instance in case it was initialized here. None if not used.
    """
    if (time.time() - upload_time) >= config["upload_rate_sec"]:
        logger.info("Publishing data...")

        if not net:
            net = network_init(device_state)

        if not net.is_connected():
            network_connect(net)

        # Lazy init of network instance means we have to set the send fxn here
        co2_device.network_send_fxn = net.send

        # Publish data
        try:
            co2_device.publish_numbers()
            co2_device.publish_sensors()
        except Exception as exc:
            import traceback
            reset(f"Failed sending data\n{''.join(traceback.format_exception(exc, chain=True))}")
        else:
            backup_ram.set_element(BACKUP_NAME_UPLOAD_TIME, time.time())

    return net


def send_discovery(discovery_time: int,
                   co2_device: HomeAssistantDevice,
                   device_state: int,
                   net: Network = None) -> "Network":
    """Send Home Assistant discovery data if configured send rate has elapsed.

    Initializing a network instance can take somewhere around 0.5 second. In order to avoid
    unneeded initialization, this function will initialize a network instance if none is provided
    but only when it is needed to send discovery data.
    If it does init a new network instance, it will return is so that the caller can use the
    single instance.
    If discovery data is sent, this function will also update the send time in backup ram.

    Args:
        discovery_time (int): Time of last discovery sending.
        co2_device (HomeAssistantDevice): CO2 device instance.
        device_state (int): Current device state.
        net (Network, optional): Network instance. Defaults to None.

    Returns:
        Network: Returns network instance in case it was initialized here. None if not used.
    """
    if (time.time() - discovery_time) >= config["send_discovery_rate_sec"]:
        logger.info("Sending HA discovery data...")

        if not net:
            net = network_init(device_state)

        if not net.is_connected():
            network_connect(net)

        # Lazy init of network instance means we have to set the send fxn here
        co2_device.network_send_fxn = net.send

        # Send discovery data
        try:
            co2_device.send_discovery()
        except Exception as exc:
            import traceback
            reset(f"Failed sending discovery\n{''.join(traceback.format_exception(exc, chain=True))}")
        else:
            backup_ram.set_element(BACKUP_NAME_SEND_DISCOVERY_TIME, time.time())

    return net


def time_sync(time_sync_time: int, device_state: int, net: Network = None) -> "Network":
    """Perform ntp time sync if configured update rate has elapsed.

    Initializing a network instance can take somewhere around 0.5 second. In order to avoid
    unneeded initialization, this function will initialize a network instance if none is provided
    but only when it is needed to perform time sync.
    If it does init a new network instance, it will return is so that the caller can use the
    single instance.
    If time is synced, this function will also update the sync time in backup ram.

    Args:
        time_sync_time (int): Time of last sync.
        device_state (int): Current device state.
        net (Network, optional): Network instance. Defaults to None.

    Returns:
        Network: Returns network instance in case it was initialized here. None if not used.
    """
    if (time.time() - time_sync_time) >= config["time_sync_rate_sec"]:
        logger.info("Time syncing...")

        if not net:
            net = network_init(device_state)

        if not net.is_connected():
            network_connect(net)

        if net.ntp_time_sync():
            from cp_libs.time import get_fmt_date, get_fmt_time
            backup_ram.set_element(BACKUP_NAME_TIME_SYNC_TIME, time.time())
            logger.info(f"Time: {get_fmt_time()}")
            logger.info(f"Date: {get_fmt_date()}")

    return net


def update_state(new_state: int):
    """Update state variable stored in backup ram

    Args:
        new_state (int): New state.
    """
    backup_ram.set_element(BACKUP_NAME_STATE, new_state)


def main() -> None:
    """Main function"""
    logger.debug(f"App start time: {app_start}")
    logger.debug(f"Time main: {time.monotonic()}")

    import alarm
    import microcontroller
    from supervisor import runtime, RunReason
    if runtime.run_reason == RunReason.SUPERVISOR_RELOAD:
        first_boot = False
    else:
        first_boot = not alarm.wake_alarm

    logger.info(f"Reset reason: {microcontroller.cpu.reset_reason}")
    logger.info(f"Run reason: {runtime.run_reason}")
    logger.info(f"Wake alarm: {alarm.wake_alarm}")
    logger.info(f"First boot: {first_boot}")
    logger.info("Initializing...")
    if first_boot:
        backup_ram_init()

    # Load state variable
    device_state = backup_ram.get_element(BACKUP_NAME_STATE)

    # Init device
    logger.info("Initializing device...")
    import peripherals
    periphs = peripherals.Peripherals(device_state == STATE_LIGHT_SLEEP)

    # Initialize SCD41
    logger.info("Initializing scd41...")
    scd41 = scd41_init(first_boot, device_state)

    # If in STATE_DEEP_SLEEP_SAMPLING, we don't need to take up time with the
    # rest of initialization. So, handle STATE_DEEP_SLEEP_SAMPLING here and skip main loop.
    if device_state == STATE_DEEP_SLEEP_SAMPLING:
        # Send single shot command and then go to sleep
        logger.info("Sending single shot...")

        scd41._send_command(SCD41_CMD_SINGLE_SHOT)

        update_state(STATE_DEEP_SLEEP)
        deep_sleep(SINGLE_SHOT_SLEEP_SEC)

    # Sensor read functions
    def read_co2():
        start = time.monotonic()
        while not scd41.data_ready and time.monotonic() - start < SENSOR_READ_TIMEOUT_SEC:
            time.sleep(0.1)
        return scd41.CO2

    def read_batt():
        volts = periphs.battery
        return volts if volts < BPI_BATT_DXN_VOLTAGE else 0

    # Create home assistant sensors
    logger.info("Initializing home assistant...")
    from homeassistant.device import HomeAssistantDevice
    from homeassistant.number import HomeAssistantNumber
    from homeassistant.sensor import HomeAssistantSensor
    from homeassistant.device_class import DeviceClass
    sensor_battery = HomeAssistantSensor(
        SENSOR_NAME_BATTERY, read_batt, 2, DeviceClass.BATTERY, "V")
    sensor_scd41_co2 = HomeAssistantSensor(
        SENSOR_NAME_SCD41_CO2, read_co2, 0, DeviceClass.CARBON_DIOXIDE, "ppm")
    sensor_scd41_hum = HomeAssistantSensor(
        SENSOR_NAME_SCD41_HUM, lambda: scd41.relative_humidity, 0, DeviceClass.HUMIDITY, "%")
    sensor_scd41_temp = HomeAssistantSensor(
        SENSOR_NAME_SCD41_TEMP, lambda: scd41.temperature, 1, DeviceClass.TEMPERATURE, "°C")

    # Create home assistant numbers
    number_temp_offset = HomeAssistantNumber(
        NUMBER_NAME_TEMP_OFFSET,
        lambda: backup_ram.get_element(BACKUP_NAME_TEMP_OFFSET),
        precision=1,
        unit="°C",
        min_value=0,
        mode="box")
    number_pressure = HomeAssistantNumber(
        NUMBER_NAME_PRESSURE,
        lambda: backup_ram.get_element(BACKUP_NAME_PRESSURE),
        device_class=DeviceClass.PRESSURE,
        unit="mbar",
        min_value=100,
        max_value=1100,
        mode="box")
    number_co2_ref = HomeAssistantNumber(
        NUMBER_NAME_CO2_REF,
        lambda: backup_ram.get_element(BACKUP_NAME_CAL),
        device_class=DeviceClass.CARBON_DIOXIDE,
        unit="ppm",
        min_value=400,
        max_value=5000,
        mode="box")

    # Lazy init network and display
    if device_state == STATE_LIGHT_SLEEP:
        # Init right away for use in main loop
        net = network_init(device_state)
        display = display_init()
        send_fxn = net.send
    else:
        # Skip init here - init later on first use
        net = None
        display = None
        send_fxn = None

    # Create home assistant device
    co2_device = HomeAssistantDevice(config["device_name"], config["model"], send_fxn, debug=False)
    # Add co2 sensor first since its read fxn waits for data ready
    co2_device.add_sensor(sensor_scd41_co2)
    co2_device.add_sensor(sensor_scd41_hum)
    co2_device.add_sensor(sensor_scd41_temp)
    co2_device.add_sensor(sensor_battery)
    co2_device.add_number(number_temp_offset)
    co2_device.add_number(number_pressure)
    co2_device.add_number(number_co2_ref)

    config["topics"]["cmd_topic"] = f"{co2_device.number_topic}/cmd"

    logger.info("Backup Ram:")
    backup_ram.print_elements()
    color_index = 0

    # Main Loop
    while True:
        logger.info("Processing...")

        # Load backup RAM data and state variables
        # State changes are processed on each pass through loop or on reboot
        time_sync_time = backup_ram.get_element(BACKUP_NAME_TIME_SYNC_TIME)
        upload_time = backup_ram.get_element(BACKUP_NAME_UPLOAD_TIME)
        receive_time = backup_ram.get_element(BACKUP_NAME_RX_TIME)
        discovery_time = backup_ram.get_element(BACKUP_NAME_SEND_DISCOVERY_TIME)
        current_pressure = backup_ram.get_element(BACKUP_NAME_PRESSURE)
        current_temp_offset = backup_ram.get_element(BACKUP_NAME_TEMP_OFFSET)
        current_cal_val = backup_ram.get_element(BACKUP_NAME_CAL)
        network_enabled = backup_ram.get_element(BACKUP_NAME_ENABLE_NETWORK)
        if config["display_enable"]:
            display_time = backup_ram.get_element(BACKUP_NAME_DISPLAY_TIME)
        else:
            display_time = None

        # State Machine
        if STATE_LIGHT_SLEEP == device_state:
            logger.info("Reading sensors...")
            sensor_data = co2_device.read_sensors(cache=True)
            logger.info(str(sensor_data))

            if network_enabled:
                config["upload_rate_sec"] = config["light_sleep_sec"]
                config["receive_rate_sec"] = config["light_sleep_sec"]
                send_discovery(discovery_time, co2_device, device_state, net=net)
                send_data(upload_time, co2_device, device_state, net=net)
                time_sync(time_sync_time, device_state, net=net)
                display_update(display_time, sensor_data, display=display)
                receive_data(receive_time, device_state, net=net, recover=True)

                if process_calibration(scd41, current_cal_val):
                    scd41.start_periodic_measurement()

                if process_temp_offset(scd41, current_temp_offset):
                    scd41.start_periodic_measurement()

                process_pressure(scd41, current_pressure)

                # Live color wheel as aliveness indicator
                periphs.neopixels[0] = peripherals.rgb_color_wheel(color_index)
                periphs.neopixels.show()
                color_index += 25

            if not runtime.usb_connected or config["force_deep_sleep"]:
                # State transition, reboot into deep sleep state
                update_state(STATE_DEEP_SLEEP_SAMPLING)
                reboot()
            else:
                light_sleep(config["light_sleep_sec"])

        elif STATE_DEEP_SLEEP == device_state:
            logger.info("Reading sensors...")
            sensor_data = co2_device.read_sensors(cache=True)
            logger.info(str(sensor_data))

            if network_enabled:
                net = send_discovery(discovery_time, co2_device, device_state, net=net)
                net = send_data(upload_time, co2_device, device_state, net=net)
                net = time_sync(time_sync_time, device_state, net=net)
                net = receive_data(receive_time, device_state, net=net)
                display_update(display_time, sensor_data, display=display)
                process_calibration(scd41, current_cal_val)
                process_temp_offset(scd41, current_temp_offset)
                process_pressure(scd41, current_pressure)

                if net and net.is_connected():
                    net.disconnect()

            if runtime.usb_connected and not config["force_deep_sleep"]:
                # State transition, reboot into light sleep state
                update_state(STATE_LIGHT_SLEEP)
                reboot()
            else:
                update_state(STATE_DEEP_SLEEP_SAMPLING)
                deep_sleep(config["deep_sleep_sec"])
        else:
            raise RuntimeError(f"Unknown state: {device_state}")
