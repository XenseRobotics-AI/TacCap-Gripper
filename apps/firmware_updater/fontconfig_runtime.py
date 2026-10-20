"""Select the host's font configuration before Qt initializes Fontconfig."""

import os
import sys
from pathlib import Path

SYSTEM_CONFIG = Path("/etc/fonts/fonts.conf")


def configure_fontconfig():
    """Avoid compiled-in build-machine prefixes; respect explicit overrides.

    Only the current process environment changes. System configuration and the
    user's fonts/cache remain untouched; missing configuration is not hidden.
    """
    if not sys.platform.startswith("linux"):
        return False
    if os.environ.get("FONTCONFIG_FILE") or os.environ.get("FONTCONFIG_PATH"):
        return False
    if not SYSTEM_CONFIG.is_file():
        return False
    os.environ["FONTCONFIG_FILE"] = str(SYSTEM_CONFIG)
    os.environ["FONTCONFIG_PATH"] = str(SYSTEM_CONFIG.parent)
    return True
