import asyncio
from buildhat import Motor
from systems.state import State


class Cargo:
    """
    Cargo detection and deployment system using magnetic field sensing.
    
    Uses the IMU magnetometer (via State) to detect magnetic cargo markers.
    Magnetic field values are updated by SensorInput.
    Payload motor controls cargo deployment.
    
    Args:
        state (State): Centralized state object for sensor data
        motor_port (str): Build HAT port for payload motor (default: "B")
        motor_speed (int): Motor speed for deployment (default: 50)
        deploy_angle (int): Degrees to turn for deployment (default: 180)
        edge_threshold (float): Magnetic threshold for edge detection in µT (default: 400)
        semi_threshold (float): Magnetic threshold for semi-detection in µT (default: 1000)
        full_threshold (float): Magnetic threshold for full detection in µT (default: 3000)
    """
    
    def __init__(self, **kwargs):
        self.state = kwargs.get("state", State())
        
        # Payload motor
        self.motor = Motor(kwargs.get("motor_port", "B"))
        self.motor_speed = kwargs.get("motor_speed", 50)
        self.deploy_angle = kwargs.get("deploy_angle", 180)
        self.deployed = False  # Tracks if cargo has been deployed (one-time deployment)

        self.edge_mag_threshold = kwargs.get("edge_threshold", 400)  # µT
        self.semi_mag_threshold = kwargs.get("semi_threshold", 1000)  # µT
        self.full_mag_threshold = kwargs.get("full_threshold", 3000)  # µT
        
        # Debounce settings for false positive prevention
        self.full_detection_count = 0
        self.required_detections = kwargs.get("required_detections", 5)  # Number of consecutive detections needed

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
    
    async def deploy(self):
        """
        Deploy the cargo by turning the payload motor +180 degrees.
        Sets deploying_cargo state to pause motion during deployment.
        """
        if self.deployed:
            print("Cargo already deployed")
            return
        
        print("Deploying cargo...")
        self.state.deploying_cargo = True
        self.motor.run_for_degrees(self.deploy_angle, self.motor_speed, blocking=True)
        self.deployed = True
        self.state.deploying_cargo = False
        print("Cargo deployed")
    
    async def close(self):
        """
        Close the payload by turning the motor -180 degrees.
        Does not reset deployed flag to prevent re-deployment.
        """
        print("Closing cargo bay...")
        self.state.deploying_cargo = True
        self.motor.run_for_degrees(-self.deploy_angle, self.motor_speed, blocking=True)
        self.state.deploying_cargo = False
        print("Cargo bay closed")
    
    def stop(self):
        """Stop the cargo motor."""
        self.motor.stop()
    
    async def run_cargo_update_loop(self, update_interval: float = 0.1):
        """
        Continuously monitor cargo level and update State.
        Auto-deploys cargo when full level is detected (one-time only).
        Uses debouncing to prevent false positives from motor EMF.
        
        Args:
            update_interval (float): Time between checks in seconds (default: 0.1)
        """
        while True:
            self.state.mag_delta = self.get_magnetic_delta()
            self.state.cargo_level = self.detect_cargo_level()
            
            # Debounce full detection to prevent false positives from motor EMF
            if self.state.cargo_level == "full" and not self.deployed:
                self.full_detection_count += 1
                if self.full_detection_count >= self.required_detections:
                    print(f"Full cargo confirmed after {self.full_detection_count} consecutive detections")
                    await self.deploy()
                    await asyncio.sleep(0.5)  # Brief pause between deploy and close
                    await self.close()
                    # deployed remains True to prevent future deployments
            else:
                # Reset counter if not full detection
                self.full_detection_count = 0
            
            await asyncio.sleep(update_interval)
    
