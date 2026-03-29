## rockpi-server: HackRF WSPR Receiver

This stack now replaces OpenWebRX with a local build of
[Guenael/rtlsdr-wsprd](https://github.com/Guenael/rtlsdr-wsprd), patched to use
HackRF (`libhackrf`) instead of RTL-SDR (`librtlsdr`).

### What changed

- `owrx` service removed from [docker-compose.yml](docker-compose.yml).
- New `wsprd` service added, built from [rtlsdr-wsprd/Dockerfile](rtlsdr-wsprd/Dockerfile).
- SDR backend in [rtlsdr-wsprd/rtlsdr_wsprd.c](rtlsdr-wsprd/rtlsdr_wsprd.c) now uses HackRF APIs.

### Configure

Set these environment variables before starting the stack (or in a `.env` file):

- `WSPR_BAND` (example: `20m`, `40m`, `2m`)
- `WSPR_CALLSIGN` (example: `M0ABC`)
- `WSPR_LOCATOR` (example: `IO91aa`)
- `WSPR_GAIN` (`0` to `62`, default `32`)
- `WSPR_RF_AMP` (`0` or `1`, HackRF RF amp enable)
- `WSPR_IF_GAIN` (`0` to `40`, step `8`, HackRF IF/LNA gain)
- `WSPR_BASEBAND_GAIN` (`0` to `62`, step `2`, HackRF baseband/VGA gain)
- `WSPR_FREQ_SHIFT` (Hz fine correction passed to `-o`, useful for calibration)
- `WSPR_PPM` (accepted for compatibility, currently ignored by HackRF backend)
- `WSPR_DEVICE_INDEX` (default `0`)
- `WSPR_USE_HASHTABLE` (`0` or `1`, enables `-H` to improve decode of hashed calls)
- `WSPR_QUICK_MODE` (`0` or `1`, enables `-Q`; faster but can reduce weak-signal decode)
- `WSPR_SINGLE_PASS` (`0` or `1`, enables `-S`; compatible with original `wsprd`, lower sensitivity)
- `WSPR_EXTRA_ARGS` (advanced passthrough for extra CLI flags)

### Start

```bash
docker compose up -d --build
```

### Notes

- Container needs USB pass-through: `/dev/bus/usb` is already configured.
- `wsprd` runs as root in compose to avoid HackRF USB permission errors (`Access denied`) when opening the device.
- `-a` (auto gain), `-d` (direct sampling), and `-p` (ppm correction) are kept for CLI compatibility but are ignored or warning-only on HackRF.

### Fewer spots after switching to `wsprd`

If spot count dropped after migration, these settings usually help on HF bands:

- Keep `WSPR_RF_AMP=0` initially to avoid front-end overload.
- Start with only `WSPR_GAIN` set (do not set `WSPR_IF_GAIN` / `WSPR_BASEBAND_GAIN` yet).
- Calibrate frequency using `WSPR_FREQ_SHIFT` in small steps (for example `-40`, `-20`, `0`, `20`, `40`) and compare 1-2 hour spot totals.
- Keep `WSPR_QUICK_MODE=0` and `WSPR_SINGLE_PASS=0` for maximum sensitivity.
- Optionally test `WSPR_USE_HASHTABLE=1` if many decodes are hashed/compound calls.

