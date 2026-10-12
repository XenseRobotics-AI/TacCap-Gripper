"""User-local GNOME launcher for AppImage; never overrides the deb launcher."""

import os
import shutil
from pathlib import Path

APPIMAGE_ID = "taccap-firmware-updater-appimage"
MARKER = "X-Xense-Generated=true"


def desktop_id():
    return APPIMAGE_ID if os.environ.get("APPIMAGE") else "taccap-firmware-updater"


def integrate_appimage(icon, data_home=None):
    image = os.environ.get("APPIMAGE")
    if not image or not Path(image).is_file():
        return False
    root = Path(
        data_home or os.environ.get("XDG_DATA_HOME") or Path.home() / ".local/share"
    )
    if not root.is_absolute():
        raise ValueError("XDG_DATA_HOME must be absolute")
    launcher = root / "applications" / f"{APPIMAGE_ID}.desktop"
    if launcher.exists() and MARKER not in launcher.read_text():
        raise ValueError("Existing desktop entry is not owned by this application")
    # Desktop Exec has its own escaping (not shell quoting); % introduces fields.
    executable = str(Path(image).resolve()).replace("%", "%%")
    for char in ("\\", '"', "`", "$"):
        executable = executable.replace(char, "\\" + char)
    if "\n" in executable or "\r" in executable:
        raise ValueError("Unsupported newline in AppImage path")
    icons = root / "icons/hicolor/512x512/apps"
    icons.mkdir(parents=True, exist_ok=True)
    shutil.copy2(icon, icons / f"{APPIMAGE_ID}.png")
    launcher.parent.mkdir(parents=True, exist_ok=True)
    launcher.write_text(
        "[Desktop Entry]\nType=Application\n"
        "Name=Xense TacCap Firmware Updater (AppImage)\n"
        "Name[zh_CN]=Xense TacCap 固件更新 (AppImage)\n"
        f'Exec="{executable}"\nIcon={APPIMAGE_ID}\n'
        f"StartupWMClass={APPIMAGE_ID}\nTerminal=false\nCategories=Utility;\n"
        f"{MARKER}\n",
        encoding="utf-8",
    )
    return True
