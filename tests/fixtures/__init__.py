"""                                                                                                                                                                 
Test fixtures for MARCO test suite.

Provides reusable navigation, mobility, and sensor configurations.
"""

from .sensors import create_sensors, SensorConfig
from .navigation import create_navigator, NavigationConfig
from .mobility import create_motion_controller, MobilityConfig