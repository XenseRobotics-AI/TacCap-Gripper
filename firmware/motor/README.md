# Local motor firmware

Place EL05 and RS00 vendor `.bin` files directly in this directory.
Keep their original names and verify that each image matches the physical motor.
The EL05 1.0.5.0.4 image is tracked with its release metadata in `../manifest.json`.
RS00 and other locally supplied motor images remain ignored by Git.
The user-facing updater deb bundles the explicitly approved EL05 1.0.5.0.4 and
RS00 0.0.3.32 images after SHA-256 verification; other versions remain manual inputs.
`../manifest.json` describes MCU images in `../gripper/` and the shipped EL05 image.

From the repository root:

```bash
python python/examples/full_ota_update.py firmware/motor/rs00-0.0.3.32.bin YOUR_FOLLOWER_SN
```

Moving a file here does not change which image/version formats the updater supports.
