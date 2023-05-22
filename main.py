#pylint: disable=c-extension-no-member, no-member, no-name-in-module
from app import main

try:
    main()
except Exception as exc:
    import adafruit_logging as logging
    import storage
    import traceback
    from supervisor import reload, runtime

    # Create logger
    logger = logging.getLogger("main")

    if not runtime.serial_connected:
        # Allow writing to file system
        storage.remount("/", readonly=False, disable_concurrent_write_protection=True)

        # Create and add file  handler
        file_handler = logging.FileHandler("Exception_Log.txt", "a")
        logger.addHandler(file_handler)

    # Log message to file
    logger.critical("Caught unexpected exception:")
    logger.critical(f"{''.join(traceback.format_exception(exc, chain=True))}")

    if not runtime.serial_connected:
        # Close/flush file handler
        file_handler.close()

        # Set file system back to read-only
        storage.remount("/", readonly=True, disable_concurrent_write_protection=False)

        logger.critical("Rebooting...")
        reload()
