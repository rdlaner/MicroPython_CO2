"""CO2 Display

This is designed specifically for the Adafruit 2.9" eink display.
Pixel dimensions: 296*128 pixels

# TODO: Bring in BitMap from micro-gui for wifi symbol
"""
# pylint: disable=c-extension-no-member, no-member, logging-fstring-interpolation
# pyright: reportGeneralTypeIssues=false
# Standard imports
import time
from micropython import const

# Third party imports
import mp_libs.nano_gui.gui.fonts.arial10 as small_font
import mp_libs.nano_gui.gui.fonts.courier20 as med_font
import mp_libs.nano_gui.gui.fonts.arial35 as large_font
from mp_libs import logging
from mp_libs.nano_gui.color_setup import ssd
from mp_libs.nano_gui.gui.core.writer import Writer
from mp_libs.nano_gui.gui.widgets.label import Label, ALIGN_CENTER

# Local imports
try:
    from config import config
except ImportError:
    config = {"logging_level": logging.INFO}

# Constants
DISP_WIDTH = const(296)
DISP_HEIGHT = const(128)

CO2_PREFIX = "CO2: "
CO2_SUFFIX = ""
CO2_COL_START = const(0)
CO2_LABEL_WIDTH = DISP_WIDTH
CO2_ROW = const(20)

TEMP_PREFIX = ""
TEMP_SUFFIX = "F"
TEMP_COL_START = const(0)
TEMP_LABEL_WIDTH = DISP_WIDTH // const(3)
TEMP_ROW = const(80)

HUM_PREFIX = ""
HUM_SUFFIX = "%"
HUM_COL_START = TEMP_COL_START + TEMP_LABEL_WIDTH
HUM_LABEL_WIDTH = DISP_WIDTH // const(3)
HUM_ROW = const(80)

BATT_PREFIX = ""
BATT_SUFFIX = "V"
BATT_COL_START = HUM_COL_START + HUM_LABEL_WIDTH
BATT_LABEL_WITHD = DISP_WIDTH // const(3)
BATT_ROW = const(80)

DATE_COL_START = const(0)
DATE_LABEL_WIDTH = DISP_WIDTH
DATE_ROW = const(115)

REFRESH_TIME_SEC = const(2)
LARGE_FONT_WIDTH = const(19)

# Globals
logger = logging.getLogger("display")
logger.setLevel(config["logging_level"])

# Text renderers
small_writer = Writer(ssd, small_font, verbose=False)
med_writer = Writer(ssd, med_font, verbose=False)
large_writer = Writer(ssd, large_font, verbose=False)

# Labels
label_co2 = Label(large_writer, CO2_ROW, CO2_COL_START, text=CO2_LABEL_WIDTH, align=ALIGN_CENTER)
label_temp = Label(med_writer, TEMP_ROW, TEMP_COL_START, text=TEMP_LABEL_WIDTH, align=ALIGN_CENTER)
label_hum = Label(med_writer, HUM_ROW, HUM_COL_START, text=HUM_LABEL_WIDTH, align=ALIGN_CENTER)
label_batt = Label(med_writer, BATT_ROW, BATT_COL_START, text=BATT_LABEL_WITHD, align=ALIGN_CENTER)
label_datetime = Label(small_writer, DATE_ROW, DATE_COL_START, text=DATE_LABEL_WIDTH, align=ALIGN_CENTER)
labels = [label_co2, label_temp, label_hum, label_batt, label_datetime]


def _build_text_batt(batt_val: float) -> str:
    try:
        text = f"{BATT_PREFIX}{batt_val:.2f}{BATT_SUFFIX}"
    except ValueError as exc:
        text = ""
        logger.exception(f"Invalid display value: {batt_val}", exc_info=exc)

    return text


def _build_text_co2(co2_val: float) -> str:
    try:
        text = f"{CO2_PREFIX}{co2_val:.0f}{CO2_SUFFIX}"
    except ValueError as exc:
        text = ""
        logger.exception(f"Invalid display value: {co2_val}", exc_info=exc)

    return text


def _build_text_hum(hum_val: float) -> str:
    try:
        text = f"{HUM_PREFIX}{hum_val:.0f}{HUM_SUFFIX}"
    except ValueError as exc:
        text = ""
        logger.exception(f"Invalid display value: {hum_val}", exc_info=exc)

    return text


def _build_text_temp(temp_val: float) -> str:
    try:
        text = f"{TEMP_PREFIX}{temp_val:.1f}{TEMP_SUFFIX}"
    except ValueError as exc:
        text = ""
        logger.exception(f"Invalid display value: {temp_val}", exc_info=exc)

    return text


def _build_text_datetime(datetime_val) -> str:
    try:
        text = f"{datetime_val}"
    except ValueError as exc:
        text = ""
        logger.exception(f"Invalid display value: {datetime_val}", exc_info=exc)

    return text


def init() -> None:
    """Initialize display."""
    ssd.init()
    ssd.fill(0)
    ssd.show()
    while not ssd.ready():
        time.sleep_ms(10)


def refresh(delay: bool = False) -> None:
    """Refresh display with latest written elements.

    Args:
        delay (bool, optional): Delay before and after refresh. Defaults to False.
    """
    if delay:
        time.sleep(REFRESH_TIME_SEC)

    logger.info("Refreshing display...")
    for label in labels:
        label.show()
    ssd.show()
    logger.debug("Display update completed.")

    if delay:
        time.sleep(REFRESH_TIME_SEC)


def update_batt(val: float) -> None:
    """Update battery label text.

    Args:
        val (float): New battery voltage
    """
    if text := _build_text_batt(val):
        label_batt.value(text=text)


def update_co2(val: float) -> None:
    """Update CO2 label text.

    Args:
        val (float): New CO2 value.
    """
    if text := _build_text_co2(val):
        label_co2.value(text=text)


def update_hum(val: float) -> None:
    """Update humidity label text.

    Args:
        val (float): New humidity value.
    """
    if text := _build_text_hum(val):
        label_hum.value(text=text)


def update_temp(val: float) -> None:
    """Update temperature label text.

    Args:
        val (float): New temperature value in F.
    """
    if text := _build_text_temp(val):
        label_temp.value(text=text)


def update_datetime(val) -> None:
    """Update datetime label text.

    Args:
        val (Any): New datetime value.
    """
    if text := _build_text_datetime(val):
        label_datetime.value(text=text)


def show_wifi(enabled: bool) -> None:
    # TODO
    # """Show requested wifi tile grid.

    # Args:
    #     enabled (bool): Show wifi enabled TF if True, disabled TG if False.
    # """
    # if enabled:
    #     remove = wifi_disabled
    #     append = wifi_enabled
    # else:
    #     remove = wifi_enabled
    #     append = wifi_disabled

    # # Remove inactive TG
    # try:
    #     main_group.remove(remove)
    # except ValueError:
    #     pass

    # # Add active TG
    # try:
    #     main_group.append(append)
    # except ValueError:
    #     pass
    pass
