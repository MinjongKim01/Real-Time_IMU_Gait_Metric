"""
User settings for IMU gait analysis system.

This module provides backward compatibility while using the new config system.
Settings are now loaded from config.yaml, but can still be imported from this module.
"""

import getpass
from config_loader import get_config

# Initialize config
config = get_config()

# Legacy variables (for backward compatibility)
whitelist = {}
dot_basename = "Movella DOT"
username = getpass.getuser().lower()

# Load settings from config
FS = config.get('sensors.sampling_rate', 60)
SENSOR_MAPPING = config.get('sensors.mapping', {
    "D4:22:CD:00:8D:01": "left",
    "D4:22:CD:00:86:04": "right",
})

## Sensor Address List
# DOT #1: D4:22:CD:00:8D:05
# DOT #2: D4:22:CD:00:8D:01
# DOT #3: D4:22:CD:00:8C:BB
# DOT #4: D4:22:CD:00:86:04
# DOT #5: D4:22:CD:00:86:14