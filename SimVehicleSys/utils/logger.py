from __future__ import annotations
import logging

def setup_logger(level: int = logging.INFO) -> logging.Logger:
    logger = logging.getLogger("SimVehicleSys")
    if not logger.handlers:
        h = logging.StreamHandler()
        fmt = logging.Formatter("[%(asctime)s] %(levelname)s %(name)s: %(message)s")
        h.setFormatter(fmt)
        logger.addHandler(h)
    logger.setLevel(level)
    return logger