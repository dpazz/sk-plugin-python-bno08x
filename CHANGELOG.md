## Unreleased
- Fixed crash when no BNO08X is on the I2C bus: the scan error was logged and then `NameError: name 'addr' is not defined` killed the python child. The plugin now logs the missing sensor once and rescans the bus every 30 s, so a sensor connected later is picked up.
- Fixed `scan_for_bno()` leaving the I2C bus locked when no sensor was found (`i2c.unlock` was referenced but not called).
- Fixed `TypeError: 'builtin_function_or_method' object is not subscriptable` (`hex[...]` instead of `hex(...)`) when the configured address differs from the one found on the bus.
- The configured I2C address is now used when that device is on the bus (before, 0x4B always won over 0x4A when both answered).
- Fixed the NOAA declination query never running: `getDeclination()` read a global `resp` that was never assigned (the `NameError` was swallowed). `SignalkPositionAvailable()` now returns the response.
- The python virtualenv is no longer hard-coded to `/home/pi/.env`: it defaults to `~/.env` of the user running Signal K and can be overridden with `SK_PY_BNO08X_VENV`. Editing `index.js` after every install/update is no longer needed.
- Failures to start `python3` or an abnormal exit of `plugin.py` are shown in the plugin status (admin UI); `stdin` errors no longer risk an `EPIPE` crash and `stop()` no longer throws when the child never started.

## 1.0.6
- Corrected stop of plugin if start calibration goes timeout (it goes on signalling it bat doesn't stop anyway)
- Improved management of 'position' key  equal to 'null' in Signalk data when declination for the position has to be queried to NOAA 
## 1.0.5
- Added management of pid of child already not running in 'Stop' branch of index.js 
## 1.0.4
- Added management of exception caused by missing of a valid 'position' in SignalK when query of NOAA declination calculator is active in schema
- updated the note in README indicating the required availabiity of a valid 'position' in SignalK when query of NOAA declination calculator is acyive in schema
## 1.0.3
- some code refactoring/cleaning
- some minor bug removed in declination management
## 1.0.2
- Added management of the exception in NOAA calculator returning malformed/unexpected response
- Added a note in README indicating the required availabiity of a valid 'position' in SignalK when query of NOAA declination calculator is active in schema