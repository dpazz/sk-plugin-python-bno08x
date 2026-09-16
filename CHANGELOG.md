
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