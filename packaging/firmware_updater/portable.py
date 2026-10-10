"""Single-file, no-install launcher around the already verified frozen bundle."""

import shutil
import tarfile
from pathlib import Path


def build_portable(frozen, output):
    output = Path(output)
    archive = output.with_suffix(".payload.tmp")
    try:
        with tarfile.open(archive, "w:gz", compresslevel=6) as tar:
            tar.add(frozen, arcname="app")
        header = """#!/bin/sh
set -eu
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
if [ "$(id -u)" -eq 0 ]; then
    echo "Please run Xense TacCap as a normal user, not root." >&2
    exit 1
fi
work=$(mktemp -d "${TMPDIR:-/tmp}/xense-taccap.XXXXXXXX")
cleanup() {
    case "$work" in */xense-taccap.????????) rm -rf -- "$work" ;; esac
}
trap cleanup EXIT
tail -n +PAYLOAD_LINE "$0" | tar -xz --no-same-owner -C "$work"
"$work/app/taccap-firmware-updater" "$@"
exit $?
"""
        header = header.replace("PAYLOAD_LINE", str(header.count("\n") + 1))
        with output.open("wb") as stream, archive.open("rb") as payload:
            stream.write(header.encode("utf-8"))
            shutil.copyfileobj(payload, stream)
        output.chmod(0o755)
    finally:
        archive.unlink(missing_ok=True)
