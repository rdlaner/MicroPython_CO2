#pylint: disable=c-extension-no-member, no-member, no-name-in-module
from app import main

try:
    main()
except Exception as exc:
    import io
    import sys
    import time
    from config import config
    from mp_libs import logging
    from platforms.unexpected_maker_feather_s3 import peripherals

    # Create logger
    logger = logging.getLogger("main")
    logger.setLevel(config["logging_level"])
    file_handler = logging.FileHandler("Exception_Log.txt", "a")
    file_handler.setLevel(config["logging_level"])
    file_handler.setFormatter(logging.Formatter("%(mono)d %(name)s-%(levelname)s:%(message)s"))
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(config["logging_level"])
    stream_handler.setFormatter(logging.Formatter("%(mono)d %(name)s-%(levelname)s:%(message)s"))
    logger.addHandler(stream_handler)
    logger.addHandler(file_handler)

    # Log message to file
    logger.critical("Caught unexpected exception:")
    buf = io.StringIO()
    sys.print_exception(exc, buf)
    logger.critical(f"{buf.getvalue()}")
    logger.critical("Rebooting...")

    # Close/flush file handler
    file_handler.close()

    # NOTE: A hard reset will cause RTC memory to be lost
    periphs = peripherals.Peripherals(False)
    while True:
        periphs.led_toggle()
        time.sleep(0.25)
