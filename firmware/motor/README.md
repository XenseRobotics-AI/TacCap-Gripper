# Local motor firmware

Place EL05 and RS00 vendor `.bin` files directly in this directory.
Keep their original names and verify that each image matches the physical motor.
These binaries are ignored by Git and are not distributed with this SDK.
`../manifest.json` describes only the released MCU images in `../gripper/`.

From the repository root:

```bash
python python/examples/full_ota_update.py firmware/motor/rs00-0.0.3.32.bin YOUR_FOLLOWER_SN
```

Moving a file here does not change which image/version formats the updater supports.
