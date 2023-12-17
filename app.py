"""CO2 Sensor Application"""
# pylint: disable=import-error, wrong-import-position, c-extension-no-member, no-member, no-name-in-module, import-outside-toplevel, logging-fstring-interpolation
# pyright: reportGeneralTypeIssues=false
# TODO: Write a button handler to wake the device from deep sleep and halt so we can get a repl when device is deep sleeping.
# TODO: Use scd eeprom to persist settings. Use backup ram as buffer for storing transients only.
# TODO: Switch to async MQTT?
# TODO: Apply received miniot data when received.
# TODO: Convert to settings.toml instead of secrets file...?
# TODO: Battery improvement - collect several samples and publish them at longer intervals
# TODO: Add base class for HA types (eg for sensor, number, etc.)

# Using lazy loading of modules to reduce "boot" time as not all are needed for every boot
# Standard imports
import time
app_start = now = time.ticks_ms()
from micropython import const

# Third party imports
from mp_libs import logging
from mp_libs.memory import BackupRAM
from mp_libs.sleep import deep_sleep, light_sleep

# Local imports
from config import config

# Constants
FORCE_CAL_DISABLED = const(-1)
SINGLE_SHOT_SLEEP_SEC = const(5)
SENSOR_READ_TIMEOUT_SEC = const(5)
STATE_LIGHT_SLEEP = const(0)
STATE_DEEP_SLEEP_SAMPLING = const(1)
STATE_DEEP_SLEEP = const(2)
BPI_BATT_DXN_VOLTAGE = 4.20
BACKUP_NAME_PRESSURE = "pressure"
BACKUP_NAME_CAL = "forced cal"
BACKUP_NAME_TEMP_OFFSET = "temp offset"
BACKUP_NAME_DISPLAY_INIT = "disp init"
BACKUP_NAME_DISPLAY_TIME = "disp time"
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
SENSOR_NAME_CO2 = "CO2"
SENSOR_NAME_HUM = "Humidity"
SENSOR_NAME_TEMP = "Temperature"

# Update root logger
logging.getLogger().setLevel(config["logging_level"])
for handler in logging.getLogger().handlers:
    handler.setLevel(config["logging_level"])
if config["log_to_fs"]:
    file_handler = logging.FileHandler("root_log.txt", "a")
    file_handler.setLevel(config["logging_level"])
    file_handler.setFormatter(logging.Formatter("%(mono)d %(name)s-%(levelname)s:%(message)s"))
    logging.getLogger().addHandler(file_handler)

# Globals
backup_ram = BackupRAM()
logger = logging.getLogger("CO2")
logger.setLevel(config["logging_level"])


# MQTT Callbacks
# pylint: disable=unused-argument
def mqtt_disconnected(user_data, return_code: int) -> None:
    """Callback for when MQTT client is disconnected from the broker"""
    logger.debug("MQTT disconnected callback")


def mqtt_message(topic: str, message: str) -> None:
    """Callback for when MQTT client's subscribed topic receives new data"""
    logger.debug(f"New message on topic {topic}: {message}")

    if config["topics"]["pressure_topic"] == topic:
        try:
            pressure = round(float(message))
        except ValueError as exc:
            logger.exception("Ambient pressure value invalid:", exc_info=exc)
            return

        logger.info(f"Updating backup pressure to {pressure}")
        backup_ram.set_element(BACKUP_NAME_PRESSURE, pressure)

    elif config["topics"]["cmd_topic"] == topic:
        import json
        try:
            obj = json.loads(message)
        except ValueError as exc:
            logger.exception("Forced calibration value invalid:", exc_info=exc)
            return

        if NUMBER_NAME_CO2_REF in obj:
            cal_val = obj[NUMBER_NAME_CO2_REF]
            backup_ram.set_element(BACKUP_NAME_CAL, cal_val)
            logger.info(f"Received new cal val: {cal_val}")

        if NUMBER_NAME_TEMP_OFFSET in obj:
            temp_offset = obj[NUMBER_NAME_TEMP_OFFSET]
            backup_ram.set_element(BACKUP_NAME_TEMP_OFFSET, temp_offset)
            logger.info(f"Received new temp offset: {temp_offset}")


def backup_ram_init(usb_connected: bool):
    """Initialize persistent data"""
    logger.debug("Initializing backup ram...")
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
        backup_ram.add_element(BACKUP_NAME_DISPLAY_INIT, "B", False)

    if usb_connected and not config["force_deep_sleep"]:
        backup_ram.add_element(BACKUP_NAME_STATE, "I", STATE_LIGHT_SLEEP)
    else:
        backup_ram.add_element(BACKUP_NAME_STATE, "I", STATE_DEEP_SLEEP_SAMPLING)


def backup_ram_is_valid() -> bool:
    """Check if backup ram has been properly initialized"""
    try:
        backup_ram.get_element(BACKUP_NAME_PRESSURE)
        backup_ram.get_element(BACKUP_NAME_CAL)
        backup_ram.get_element(BACKUP_NAME_TEMP_OFFSET)
        backup_ram.get_element(BACKUP_NAME_ENABLE_NETWORK)
        backup_ram.get_element(BACKUP_NAME_UPLOAD_TIME)
        backup_ram.get_element(BACKUP_NAME_TIME_SYNC_TIME)
        backup_ram.get_element(BACKUP_NAME_SEND_DISCOVERY_TIME)
        backup_ram.get_element(BACKUP_NAME_RX_TIME)
        backup_ram.get_element(BACKUP_NAME_STATE)

        if config["display_enable"]:
            backup_ram.get_element(BACKUP_NAME_DISPLAY_TIME)
            backup_ram.get_element(BACKUP_NAME_DISPLAY_INIT)
    except KeyError:
        return False

    return True


def c_to_f(temp_cels: float) -> float:
    """"Convert celsius to fahrenheit"""
    return (temp_cels * 1.8) + 32.0 if temp_cels else 0.0


def display_init() -> "Display":
    """Initialize and return a display object.

    Returns:
        Display: New display object
    """
    logger.info("Initializing display...")
    if config["display_enable"]:
        import display

        # Do one-time initiliazation of display
        if not backup_ram.get_element(BACKUP_NAME_DISPLAY_INIT):
            display.init()
            backup_ram.set_element(BACKUP_NAME_DISPLAY_INIT, True)

        return display
    return None


def display_update(display_time: int, sensor_data: dict, display: Display = None) -> "Display":
    """Perform display update if time has passed defined refresh rate.

    Args:
        display_time (int): Time of last display update.
        sensor_data (dict): CO2, temp, hum, and battery data.
        display (Display, optional): Display instance. Defaults to None.

    Returns:
        Display: Returns display instance in case it was created here. None if not used.
    """
    if config["display_enable"] and (time.time() - display_time) >= config["display_refresh_rate_sec"]:
        logger.info("Updating display...")
        from mp_libs.time import get_fmt_time
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

        display.update_co2(sensor_data[SENSOR_NAME_CO2])
        display.update_temp(c_to_f(sensor_data[SENSOR_NAME_TEMP]))
        display.update_hum(sensor_data[SENSOR_NAME_HUM])
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
    from mp_libs.network import Network

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

    if expected_cal_val not in (FORCE_CAL_DISABLED, current_cal_val):
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
    from machine import soft_reset
    logger.warning("Rebooting...")
    soft_reset()


def reset(msg: str = "", exc_info=None) -> None:
    """Reset device.

    Should be used for attempting to recover a device due to unrecoverable failures.

    Instead of resetting directly here, we will throw an exception that will get caught by main.py.
    main.py will write the optional message to the file system and then clean up before rebooting.

    Args:
        msg (str, optional): Optional reboot message.
        exc_info (Exception, optional): Optional exception instance.
    """
    logger.warning("Rebooting...")

    if exc_info:
        import io
        import sys
        buf = io.StringIO()
        sys.print_exception(exc_info, buf)
        msg = f"{msg}\n{buf.getvalue()}"

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
        rx_win = config["receive_window_sec"] * 1000.0
        start = time.ticks_ms()

        while (start + rx_win) > time.ticks_ms():
            try:
                net.receive(rxed_data, recover=recover)
            except Exception as exc:
                reset("Failed receiving data", exc_info=exc)
            else:
                backup_ram.set_element(BACKUP_NAME_RX_TIME, time.time())

        if rxed_data:
            logger.info(f"Received data: {rxed_data}")

    return net


def scd41_init(first_boot: bool, device_state: int, i2c_scl_pin: int, i2c_sda_pin: int) -> "SCD4X":
    """Create and initialize an scd41 instance.

    Note:
    Measurements must be stopped in order to make configuration changes, however, config
    updates persist across reboots.
    Therefore, only stop measurements and perform initializations once in order to prevent
    unnecessary latency in deep sleep mode.

    Args:
        first_boot (bool): True if first boot from power-on, False if not.
        device_state (int): Current device state.
        i2c_scl_pin (int): I2C clock pin number.
        i2c_sda_pin (int): I2C data pin number.

    Returns:
        SCD4X: New SCD4X instance.
    """
    from mp_libs.sensors.scd4x import SCD4X
    from machine import I2C, Pin
    i2c = I2C(0, scl=Pin(i2c_scl_pin), sda=Pin(i2c_sda_pin), freq=100000)
    scd41 = SCD4X(i2c, stop_measurements=False)

    if first_boot:
        logger.info("Initializing SCD41:")
        scd41.stop_periodic_measurement(delay_sec=1.0)

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
        scd41.stop_periodic_measurement()
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
            co2_device.publish_numbers(recover=True)
            co2_device.publish_sensors(recover=True)
        except Exception as exc:
            reset("Failed sending data", exc_info=exc)
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
            co2_device.send_discovery(recover=True)
        except Exception as exc:
            reset("Failed sending discovery", exc_info=exc)
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
            from mp_libs.time import get_fmt_date, get_fmt_time
            backup_ram.set_element(BACKUP_NAME_TIME_SYNC_TIME, time.time())
            logger.info(f"Time: {get_fmt_time()}")
            logger.info(f"Date: {get_fmt_date()}")

    return net


def update_state(new_state: int):
    """Update state variable stored in backup ram

    Args:
        new_state (int): New state.
    """
    logger.debug(f"Updating state to {new_state}")
    backup_ram.set_element(BACKUP_NAME_STATE, new_state)


def main() -> None:
    """Main function"""
    logger.debug("Main start")
    logger.debug(f"App start time: {app_start}")

    import machine
    first_boot = machine.reset_cause() in [machine.PWRON_RESET, machine.HARD_RESET]

    logger.info(f"Reset cause: {machine.reset_cause()}")
    logger.info(f"Wake reason: {machine.wake_reason()}")
    logger.info(f"First boot: {first_boot}")

    # Init device
    logger.info("Initializing device...")
    from platforms.unexpected_maker_feather_s3 import peripherals
    periphs = peripherals.Peripherals(False)

    if first_boot:
        backup_ram_init(periphs.usb_connected())
    elif not backup_ram_is_valid():
        backup_ram_init(periphs.usb_connected())

    # Load state variable
    device_state = backup_ram.get_element(BACKUP_NAME_STATE)
    logger.info(f"Initial device state: {device_state}")

    # Initialize SCD41
    logger.info("Initializing scd41...")
    scd41 = scd41_init(first_boot, device_state, peripherals.I2C_SCL_PIN, peripherals.I2C_SDA_PIN)

    # If in STATE_DEEP_SLEEP_SAMPLING, we don't need to take up time with the
    # rest of initialization. So, handle STATE_DEEP_SLEEP_SAMPLING here and skip main loop.
    if device_state == STATE_DEEP_SLEEP_SAMPLING:
        # Send single shot command and then go to sleep
        logger.info("Sending single shot...")
        scd41.single_shot()
        update_state(STATE_DEEP_SLEEP)
        deep_sleep(SINGLE_SHOT_SLEEP_SEC, lambda: config["fake_sleep"])
    elif device_state == STATE_LIGHT_SLEEP:
        periphs.ld02 = True

    # Sensor read functions
    def read_co2():
        start = time.ticks_ms()
        while not scd41.data_ready and time.ticks_ms() - start < SENSOR_READ_TIMEOUT_SEC:
            time.sleep_ms(5)
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
        SENSOR_NAME_CO2, read_co2, 0, DeviceClass.CARBON_DIOXIDE, "ppm")
    sensor_scd41_hum = HomeAssistantSensor(
        SENSOR_NAME_HUM, lambda: scd41.relative_humidity, 0, DeviceClass.HUMIDITY, "%")
    sensor_scd41_temp = HomeAssistantSensor(
        SENSOR_NAME_TEMP, lambda: scd41.temperature, 1, DeviceClass.TEMPERATURE, "°C")

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
    logger.debug("Creating home assistant device...")
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

            # Update display
            display_update(display_time, sensor_data, display=display)

            # Live color wheel as aliveness indicator
            periphs.neopixels[0] = peripherals.rgb_color_wheel(color_index)
            periphs.neopixels.write()
            color_index += 25

            # Handle network related activities
            if network_enabled:
                config["upload_rate_sec"] = config["light_sleep_sec"]
                config["receive_rate_sec"] = config["light_sleep_sec"]
                send_discovery(discovery_time, co2_device, device_state, net=net)
                send_data(upload_time, co2_device, device_state, net=net)
                time_sync(time_sync_time, device_state, net=net)
                receive_data(receive_time, device_state, net=net, recover=True)

                if process_calibration(scd41, current_cal_val):
                    scd41.start_periodic_measurement()

                if process_temp_offset(scd41, current_temp_offset):
                    scd41.start_periodic_measurement()

                process_pressure(scd41, current_pressure)

            logger.warning(f"Run time: {time.ticks_ms() - app_start}")
            if not periphs.usb_connected() or config["force_deep_sleep"]:
                # State transition, reboot into deep sleep state
                scd41.stop_periodic_measurement(delay_sec=0.0)
                update_state(STATE_DEEP_SLEEP_SAMPLING)
                reboot()
            else:
                light_sleep(config["light_sleep_sec"], periphs.usb_connected)

        elif STATE_DEEP_SLEEP == device_state:
            logger.info("Reading sensors...")
            sensor_data = co2_device.read_sensors(cache=True)
            logger.info(str(sensor_data))

            # Update display
            display_update(display_time, sensor_data, display=display)

            # Handle network related activities
            if network_enabled:
                net = send_discovery(discovery_time, co2_device, device_state, net=net)
                net = send_data(upload_time, co2_device, device_state, net=net)
                net = time_sync(time_sync_time, device_state, net=net)
                net = receive_data(receive_time, device_state, net=net)
                process_calibration(scd41, current_cal_val)
                process_temp_offset(scd41, current_temp_offset)
                process_pressure(scd41, current_pressure)

                if net and net.is_connected():
                    net.disconnect()

            logger.warning(f"Run time: {time.ticks_ms() - app_start}")
            if periphs.usb_connected() and not config["force_deep_sleep"]:
                # State transition, reboot into light sleep state
                update_state(STATE_LIGHT_SLEEP)
                reboot()
            else:
                update_state(STATE_DEEP_SLEEP_SAMPLING)
                deep_sleep(config["deep_sleep_sec"], lambda: config["fake_sleep"])
        else:
            raise RuntimeError(f"Unknown state: {device_state}")
