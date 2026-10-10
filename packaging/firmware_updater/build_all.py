#!/usr/bin/env python3
"""Build deb, run and AppImage together from one frozen SDK snapshot."""

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

from build_appimage import RUNTIME_SHA, TOOL_SHA, verify_file

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PACKAGE = "taccap-firmware-updater"


def dependency(cache, name, repository, digest):
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / name
    if path.exists():
        verify_file(path, digest)
        return path
    url = (
        f"https://github.com/AppImage/{repository}/releases/download/continuous/{name}"
    )
    with tempfile.TemporaryDirectory(dir=cache) as temp:
        candidate = Path(temp) / name
        with (
            urllib.request.urlopen(url, timeout=60) as response,
            candidate.open("wb") as stream,
        ):
            shutil.copyfileobj(response, stream)
        verify_file(candidate, digest)
        candidate.replace(path)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    parser.add_argument("--tools-dir", type=Path, default=ROOT / "build/appimage-tools")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    tool = dependency(
        args.tools_dir.resolve(),
        "appimagetool-x86_64.AppImage",
        "appimagetool",
        TOOL_SHA,
    )
    runtime = dependency(
        args.tools_dir.resolve(), "runtime-x86_64", "type2-runtime", RUNTIME_SHA
    )
    with tempfile.TemporaryDirectory(prefix="taccap-all-") as temp:
        staging = Path(temp)
        subprocess.run(
            [sys.executable, str(HERE / "build_deb.py"), "--output", str(staging)],
            check=True,
        )
        (deb,) = staging.glob(f"{PACKAGE}_*_amd64.deb")
        subprocess.run(
            [
                sys.executable,
                str(HERE / "build_appimage.py"),
                str(deb),
                "--tool",
                str(tool),
                "--runtime",
                str(runtime),
                "--output",
                str(staging),
            ],
            check=True,
        )
        artifacts = [deb, *staging.glob("*.run"), *staging.glob("*.AppImage")]
        if len(artifacts) != 3:
            raise RuntimeError("Expected exactly three artifacts")
        # Verify real frozen payload, without discovery or hardware access.
        reports = []
        for artifact in artifacts[1:]:
            command = [str(artifact)]
            if artifact.suffix == ".AppImage":
                command.append("--appimage-extract-and-run")
            result = subprocess.check_output(command + ["--self-test"], text=True)
            reports.append(json.loads(result))
        if reports[0]["build"] != reports[1]["build"]:
            raise RuntimeError("Artifacts have different build identities")
        if reports[0]["build"]["dirty"]:
            raise RuntimeError("Commit source changes before packaging")
        existing = [
            output / item.name for item in artifacts if (output / item.name).exists()
        ]
        if existing:
            backup = Path(tempfile.mkdtemp(prefix="previous-build-", dir=output))
            for item in existing:
                shutil.move(str(item), backup / item.name)
            print(f"Previous artifacts preserved: {backup}")
        for item in artifacts:
            destination = output / item.name
            shutil.move(str(item), destination)
            print(destination)


if __name__ == "__main__":
    main()
