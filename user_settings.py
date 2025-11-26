import getpass

whitelist = list()
dot_basename = "movella"
username = getpass.getuser().lower()
whitelist = {}
dot_basename = "Movella DOT"
FS = 60

## Sensor Address List
# DOT #1: D4:22:CD:00:8D:05
# DOT #2: D4:22:CD:00:8D:01
# DOT #3: D4:22:CD:00:8C:BB
# DOT #4: D4:22:CD:00:86:04
# DOT #5: D4:22:CD:00:86:14

SENSOR_MAPPING = {
    "D4:22:CD:00:8D:01": "left",
    "D4:22:CD:00:86:04": "right",
}