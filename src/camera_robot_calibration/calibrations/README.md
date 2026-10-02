# Hand-eye calibrations

easy_handeye2 YAML results. Keep a copy here after every successful run.

At publish time easy_handeye2 still reads
`~/.ros2/easy_handeye2/calibrations/<name>.calib`. Sync from this folder:

```bash
mkdir -p ~/.ros2/easy_handeye2/calibrations
cp src/camera_robot_calibration/calibrations/<name>.calib \
  ~/.ros2/easy_handeye2/calibrations/
```

| File | Cell | Date | `base` → camera optical (m) |
|------|------|------|------------------------------|
| `niryo_d455_eob.calib` | Niryo Ned3 Pro + table D455 | 2026-09-08 | `[0.343, -0.017, 0.773]` (42 mm A4-fill squares; see package README issue log) |

JAKA / Lexium files should use `jaka_d455_eob.calib` (see package README).
