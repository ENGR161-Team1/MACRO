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
        self.deploy_distance = kwargs.get("deploy_distance", 0.0) * 100 # In cm

        # Maximum magnetic delta observed
        self.max_mag_delta = 0.0

        # Maximum cargo level observed
        self.max_cargo_level = "none"

        # Distance of maximum magnetic level detected
        self.max_mag_distance = 0.0

    def get_magnetic_delta(self):
        """
        Return the magnetic field difference from baseline.
        
        Returns:
            float: Difference from baseline in micro-tesla
        """
        if not self.state.calibrated_mag or self.state.bias is None or "mag" not in self.state.bias:
            if self.state.magnetic_field > self.max_mag_delta:
                self.max_mag_delta = self.state.magnetic_field
                self.max_mag_distance = self.state.distance_traveled
            return self.state.magnetic_field
        if self.state.magnetic_field - self.state.bias["mag"] > self.max_mag_delta:
            self.max_mag_delta = self.state.magnetic_field - self.state.bias["mag"]
            self.max_mag_distance = self.state.distance_traveled
        return self.state.magnetic_field - self.state.bias["mag"]

    def detect_cargo_level(self):
        """
        Detect the cargo proximity level based on magnetic field.
        
        Returns:
            str: "none", "edge", "semi", or "full" based on magnetic reading
        """
        mag_delta = abs(self.get_magnetic_delta())
        
        if mag_delta >= self.full_mag_threshold:
            if self.max_cargo_level != "full":
                self.max_cargo_level = "full"
            return "full"
        elif mag_delta >= self.semi_mag_threshold:
            if self.max_cargo_level != "semi" and self.max_cargo_level != "full":
                self.max_cargo_level = "semi"
            return "semi"
        elif mag_delta >= self.edge_mag_threshold:
            if self.max_cargo_level == "none":
                self.max_cargo_level = "edge"
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
        else:
            while True:
                if self.state.distance_traveled == self.max_mag_distance + self.deploy_distance:
                    break
        
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
    
    async def deploy_and_close(self):
        """Deploy and then close the cargo bay."""
        await self.deploy()
        await asyncio.sleep(0.5)  # Brief pause between deploy and close
        await self.close()
    
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
            if not self.deployed:
                if self.state.cargo_level == "full":
                    print(f"Full cargo confirmed")
                    await self.deploy()
                elif self.state.cargo_level == "semi":
                    if self.max_mag_delta > self.state.mag_delta:
                        print(f"Semi cargo confirmed")
                        await self.deploy()
                elif self.state.cargo_level == "edge":
                    print(f"Edge cargo detected at distance {self.max_mag_distance:.2f} m")
                    if self.max_cargo_level != "edge":
                        if self.max_mag_delta > self.state.mag_delta:
                            print(f"Semi cargo confirmed")
                            await self.deploy()
                else:
                    if self.max_cargo_level != "none":
                        if self.max_mag_delta > self.state.mag_delta:
                            print(f"Edge cargo confirmed")
                            await self.deploy()
            else:
                # Reset counter if not full detection
                self.full_detection_count = 0
            
            await asyncio.sleep(update_interval)
    
