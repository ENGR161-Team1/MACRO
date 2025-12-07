import asyncio
from systems.state import State


class Cargo:
    """
    Cargo detection and deployment system using magnetic field sensing.
    
    Uses the IMU magnetometer (via State) to detect magnetic cargo markers.
    Magnetic field values are updated by SensorInput.
    
    Args:
        state (State): Centralized state object for sensor data
        edge_threshold (float): Magnetic threshold for edge detection in µT (default: 400)
        semi_threshold (float): Magnetic threshold for semi-detection in µT (default: 1000)
        full_threshold (float): Magnetic threshold for full detection in µT (default: 3000)
    """
    
    def __init__(self, **kwargs):
        self.state = kwargs.get("state", State())

        self.edge_mag_threshold = kwargs.get("edge_threshold", 400)  # µT
        self.semi_mag_threshold = kwargs.get("semi_threshold", 1000)  # µT
        self.full_mag_threshold = kwargs.get("full_threshold", 3000)  # µT

    def get_magnetic_delta(self):
        """
        Return the magnetic field difference from baseline.
        
        Returns:
            float: Difference from baseline in micro-tesla
        """
        if not self.state.calibrated_mag or self.state.bias is None or "mag" not in self.state.bias:
            return self.state.magnetic_field
        return self.state.magnetic_field - self.state.bias["mag"]

    def detect_cargo_level(self):
        """
        Detect the cargo proximity level based on magnetic field.
        
        Returns:
            str: "none", "edge", "semi", or "full" based on magnetic reading
        """
        mag_delta = abs(self.get_magnetic_delta())
        
        if mag_delta >= self.full_mag_threshold:
            return "full"
        elif mag_delta >= self.semi_mag_threshold:
            return "semi"
        elif mag_delta >= self.edge_mag_threshold:
            return "edge"
        else:
            return "none"

    def is_cargo_detected(self):
        """
        Check if any cargo is detected.
        
        Returns:
            bool: True if cargo detected at any level
        """
        return self.detect_cargo_level() != "none"