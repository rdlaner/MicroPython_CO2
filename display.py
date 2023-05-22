"""CO2 Display"""
#pylint: disable=c-extension-no-member, no-member
# Standard imports
import board
import displayio
import terminalio
import time
import traceback

# Third party imports
import adafruit_logging as logging
from adafruit_display_text import label

# Local imports
try:
    from config import config
except ImportError:
    config = {"logging_level": logging.INFO}

# Globals
logger = logging.getLogger("display")
logger.setLevel(config["logging_level"])


class Display:
    """High level display object for CO2 project.

    Assumes board module has a DISPLAY
    """
    # Constants
    CO2_PREFIX = "CO2: "
    CO2_SUFFIX = ""
    TEMP_PREFIX = ""
    TEMP_SUFFIX = " F"
    HUM_PREFIX = ""
    HUM_SUFFIX = "%"
    BATT_PREFIX = ""
    BATT_SUFFIX = " V"

    def __init__(self) -> None:
        self.display = board.DISPLAY
        color_bitmap = displayio.Bitmap(self.display.width, self.display.height, 1)
        color_palette = displayio.Palette(1)
        color_palette[0] = 0xFFFFFF
        self.bg_sprite = displayio.TileGrid(
            color_bitmap,
            pixel_shader=color_palette,
            x=0,
            y=0
        )
        wifi_enabled_bitmap = displayio.OnDiskBitmap("/bmps/wifi_enabled.bmp")
        wifi_disabled_bitmap = displayio.OnDiskBitmap("/bmps/wifi_disabled.bmp")
        self.wifi_enabled = displayio.TileGrid(
            wifi_enabled_bitmap,
            pixel_shader=wifi_enabled_bitmap.pixel_shader,
            x=5,
            y=5)
        self.wifi_disabled = displayio.TileGrid(
            wifi_disabled_bitmap,
            pixel_shader=wifi_disabled_bitmap.pixel_shader,
            x=5,
            y=5)

        self.co2_label = label.Label(
            font=terminalio.FONT, text=self.CO2_PREFIX, scale=3, color=0
        )
        self.temp_label = label.Label(
            font=terminalio.FONT, text=self.TEMP_PREFIX, scale=2, color=0
        )
        self.hum_label = label.Label(
            font=terminalio.FONT, text=self.HUM_PREFIX, scale=2, color=0
        )
        self.batt_label = label.Label(
            font=terminalio.FONT, text=self.BATT_PREFIX, scale=2, color=0
        )
        self.datetime_label = label.Label(
            font=terminalio.FONT, text="", scale=1, color=0
        )

        self.co2_label.anchor_point = (0.5, 0.0)
        self.co2_label.anchored_position = (self.display.width // 2, 10)
        self.temp_label.anchor_point = (0.5, 0)
        self.temp_label.anchored_position = (self.display.width // 6, 60)
        self.hum_label.anchor_point = (0.5, 0)
        self.hum_label.anchored_position = (3 * self.display.width // 6, 60)
        self.batt_label.anchor_point = (0.5, 0)
        self.batt_label.anchored_position = (5 * self.display.width // 6, 60)
        self.datetime_label.anchor_point = (0.5, 0)
        self.datetime_label.anchored_position = (3 * self.display.width // 6, 100)

        self.main_group = displayio.Group()
        self.main_group.append(self.bg_sprite)
        self.main_group.append(self.wifi_disabled)
        self.main_group.append(self.co2_label)
        self.main_group.append(self.temp_label)
        self.main_group.append(self.hum_label)
        self.main_group.append(self.batt_label)
        self.main_group.append(self.datetime_label)
        self.display.show(self.main_group)

    def _build_text_batt(self, batt_val: float) -> str:
        try:
            text = f"{self.BATT_PREFIX}{batt_val:.2f}{self.BATT_SUFFIX}"
        except ValueError as exc:
            text = None
            logger.error(f"Invalid display value: {batt_val}")
            logger.error(f"{''.join(traceback.format_exception(exc, chain=True))}")

        return text

    def _build_text_co2(self, co2_val: float) -> str:
        try:
            text = f"{self.CO2_PREFIX}{co2_val:.0f}{self.CO2_SUFFIX}"
        except ValueError as exc:
            text = None
            logger.error(f"Invalid display value: {co2_val}")
            logger.error(f"{''.join(traceback.format_exception(exc, chain=True))}")

        return text

    def _build_text_hum(self, hum_val: float) -> str:
        try:
            text = f"{self.HUM_PREFIX}{hum_val:.0f}{self.HUM_SUFFIX}"
        except ValueError as exc:
            text = None
            logger.error(f"Invalid display value: {hum_val}")
            logger.error(f"{''.join(traceback.format_exception(exc, chain=True))}")

        return text

    def _build_text_temp(self, temp_val: float) -> str:
        try:
            text = f"{self.TEMP_PREFIX}{temp_val:.1f}{self.TEMP_SUFFIX}"
        except ValueError as exc:
            text = None
            logger.error(f"Invalid display value: {temp_val}")
            logger.error(f"{''.join(traceback.format_exception(exc, chain=True))}")

        return text

    def _build_text_datetime(self, datetime_val) -> str:
        try:
            text = f"{datetime_val}"
        except ValueError as exc:
            text = None
            logger.error(f"Invalid display value: {datetime_val}")
            logger.error(f"{''.join(traceback.format_exception(exc, chain=True))}")

        return text

    def refresh(self, delay: bool = True) -> None:
        """Refresh display with latest written elements.

        Args:
            delay (bool, optional): Delay before and after refresh. Defaults to True.
        """
        if delay:
            time.sleep(self.display.time_to_refresh + 1)
        self.display.refresh()
        if delay:
            time.sleep(self.display.time_to_refresh + 1)

    def update_batt(self, val: float) -> None:
        """Update battery label text.

        Args:
            val (float): New battery voltage
        """
        if text := self._build_text_batt(val):
            self.batt_label.text = text

    def update_co2(self, val: float) -> None:
        """Update CO2 label text.

        Args:
            val (float): New CO2 value.
        """
        if text := self._build_text_co2(val):
            self.co2_label.text = text

    def update_hum(self, val: float) -> None:
        """Update humidity label text.

        Args:
            val (float): New humidity value.
        """
        if text := self._build_text_hum(val):
            self.hum_label.text = text

    def update_temp(self, val: float) -> None:
        """Update temperature label text.

        Args:
            val (float): New temperature value in F.
        """
        if text := self._build_text_temp(val):
            self.temp_label.text = text

    def update_datetime(self, val) -> None:
        """Update datetime label text.

        Args:
            val (Any): New datetime value.
        """
        if text := self._build_text_datetime(val):
            self.datetime_label.text = text

    def show_wifi(self, enabled: bool) -> None:
        """Show requested wifi tile grid.

        Args:
            enabled (bool): Show wifi enabled TF if True, disabled TG if False.
        """
        if enabled:
            remove = self.wifi_disabled
            append = self.wifi_enabled
        else:
            remove = self.wifi_enabled
            append = self.wifi_disabled

        # Remove inactive TG
        try:
            self.main_group.remove(remove)
        except ValueError:
            pass

        # Add active TG
        try:
            self.main_group.append(append)
        except ValueError:
            pass
