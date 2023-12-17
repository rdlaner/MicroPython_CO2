"""CO2 Config File"""
# pyright: reportGeneralTypeIssues=false
# Standard imports
from micropython import const

# Third party imports
from mp_libs import logging

config = {
    # Platform Configuration
    "platform": "unexpected_maker_feather_s3",
    "model": "UM",

    # App Configuration
    "device_name": "Bedroom_CO2",
    "light_sleep_sec": const(5),
    "deep_sleep_sec": const(120),
    "upload_rate_sec": const(120),
    "receive_rate_sec": const(120),
    "receive_window_sec": const(0.5),
    "time_sync_rate_sec": const(600),
    "send_discovery_rate_sec": const(600),
    "display_refresh_rate_sec": const(120),
    "ambient_pressure": const(1000),
    "temp_offset_c": const(0.5),
    "force_deep_sleep": False,
    "logging_level": logging.WARNING,
    "fake_sleep": False,
    "log_to_fs": False,

    # Display Configuration
    "display_enable": True,

    # Transport Configuration
    # supported transports are: "espnow", "miniot", or "mqtt"
    "enable_network": False,
    "network_transport": "miniot",
    "network_prefix_id": "CO2",

    # Wifi configuration parameters
    "wifi_channel": const(1),

    # Mqtt configuration parameters
    "topics": {
        "pressure_topic": "homeassistant/aranet/pressure",
        "cmd_topic": "homeassistant/number/generic-device/cmd",
    },
    "keep_alive_sec": const(60),
    "keep_alive_margin_sec": const(20),
    "connect_retries": const(5),
    "recv_timeout_sec": const(10),

    # Espnow configuration parameters
    "epn_peer_mac": b'|\xdf\xa1\xf2\xa2\xd0',
    "epn_channel": const(1)
}
