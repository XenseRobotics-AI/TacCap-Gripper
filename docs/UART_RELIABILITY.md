# UART reliability candidate (2026-10-28)

This source change addresses reproducible code risks; it does **not** establish
that the EL05 short frames are fixed. No firmware flash, live-port access, camera
access or host installation is part of the offline verification.

## Transport

- Reserve pending ACK sequence slots until the requesting call releases them,
  including SameSeq retry delays. No-ACK allocation skips reserved slots.
- Match echoed command as well as sequence; keep legacy cmd=0, one-byte ACK/NACK
  support. A mismatched response cannot consume a waiting request.
- Clear inherited INPCK/IGNPAR in raw serial setup. Existing full-frame write
  locking, one serial reader, retries and independent dispatch remain.
- The wire has only 8 sequence bits. An arbitrarily late response with the same
  sequence **and command** after slot reuse remains indistinguishable without a
  protocol transaction/session identifier. This patch does not claim otherwise.
- Target writes are counted after successful full writes, under the writer lock;
  four target commands (position, velocity, torque, impedance) share the counter.
  ACK retries count as wire attempts. These are not MCU acceptance counts.

## Explicit observations (C++ and Python)

Existing `diagnostics.uart_stats()` sends an empty 0x54 request and retains the
44-byte struct. `diagnostics.uart_rx_errors()` sends payload `01` and requires a
72-byte reply. It returns overrun, framing, noise, parity, receiver_timeout,
last_error (HAL bitmask) and last_error_ms (32-bit boot milliseconds). Missing
support is an error, never zero-filled. Multiple error bits can count in one event.
Successful HAL TX counts are not pin-level proof and cannot exclude firmware.

`motor.target_writes()` returns local count, last_write_ns (host steady clock),
last_gap_us and max_gap_us. It sends nothing. Gaps include deliberate pauses and
are cumulative for the Transport lifetime; start a new baseline for a new run.

`motor.delivery_snapshot()` makes one explicit 0x51 control-stat query and returns
`before`, `after` local write snapshots plus `firmware` (target_seq, applied_seq,
loop/error counts etc.). Call from a monitoring thread, never from a controller
or a serial callback. No automatic polling or change to the motor stop policy.

For snapshots a and b on the same boot/connection:

```python
# g is the application's already-open follower. No discovery/open in this example.
a = g.motor.delivery_snapshot()
# ... application continues its normal control workload ...
b = g.motor.delivery_snapshot()
host_min = max(0, b.before.count - a.after.count)
host_max = b.after.count - a.before.count
accepted = (b.firmware.target_seq - a.firmware.target_seq) & 0xffffffff
print(f"Host write window: {host_min}..{host_max}, MCU accepted: {accepted}")
print(f"Host max write gap: {b.after.max_gap_us / 1000:.3f} ms")
```

This range brackets **query overlap**, not arbitrary bytes already buffered in
the OS/bridge/MCU. It cannot prove exact loss for a short window. Invalidate the
baseline after restart, counter reset, reconnect, other command producers or
control-mode changes. Do not equate target_seq-applied_seq with packet loss:
multiple targets can legitimately be replaced before a 500 Hz controller step.

Feedback-paced controllers remain feedback-paced. A status frame does not prove
an idle interval: IMU/ACK frames may immediately follow it. Average 100 Hz is not
a guarantee of 10 ms spacing. No stale target replay is introduced.

## Validation and deployment

Run the PTY reliability cases plus existing transport, controller, IMU and OTA
regressions; compile the Python bindings. PTYs cannot reproduce UART electrical
errors, physical FIFO overrun, bridge loss, USB contention or camera throughput.
Build with `-DSKBUILD=ON` into a separate directory to avoid replacing an SDK
library used by a running test. New Python sources and native libraries must come
from the same build. Do not install or replace a running test's module.

Use the firmware's optional error classification in the next matched hardware
run. Keep logs off; retain raw host input and, if authorized, follower-only
usbmon data. Test main/modified cross combinations separately. This patch does
not change camera capture/Rectify performance or claim to recover the observed
101 fps to the configured 120 fps.
