"""
navigation_display.py

Visual display for rover navigation showing position, heading, and grade.

This module provides a real-time visualization of the rover's position
in the X-Y plane with heading arrow and navigation data from Navigation3D.
"""

import tkinter as tk
import math
import asyncio
from systems.navigation_system import Navigation3D


class NavigationDisplay:
    """
    Visual navigation display showing position, orientation, and acceleration.
    
    Displays the rover's position on a 2D grid with:
    - X-Y coordinate system (positive X right, positive Y up)
    - Black dot for rover position with blue heading arrow
    - Major grid lines every 1 meter (gray 64)
    - Minor grid lines every 0.1 meters (gray 32)
    - Info panel showing position, orientation, velocity, acceleration
    
    Args:
        width (int): Initial window width in pixels (default: 800)
        height (int): Initial window height in pixels (default: 800)
        scale (float): Pixels per meter (default: 50.0)
        title (str): Window title
        navigator (Navigation3D): Navigation3D instance for data
    """
    
    def __init__(self, **kwargs):
        self.width = kwargs.get("width", 800)
        self.height = kwargs.get("height", 800)
        self.scale = kwargs.get("scale", 50.0)  # pixels per meter
        self.title = kwargs.get("title", "MACRO Navigation Display")
        self.navigator = kwargs.get("navigator", None)
        
        # Position and orientation state
        self.position = (0.0, 0.0, 0.0)  # (x, y, z) in meters
        self.orientation = (0.0, 0.0, 0.0)  # (yaw, pitch, roll) in degrees
        self.velocity = (0.0, 0.0, 0.0)  # (vx, vy, vz) in m/s
        self.acceleration = (0.0, 0.0, 0.0)  # (ax, ay, az) in m/s^2
        
        # Center of display (origin) - will be updated on resize
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        
        # Grid settings
        self.major_grid_spacing = 1.0  # meters
        self.minor_grid_spacing = 0.1  # meters
        self.major_grid_color = "#404040"  # Gray 64
        self.minor_grid_color = "#202020"  # Gray 32
        self.axis_color = "#606060"
        
        # Arrow properties
        self.arrow_length = 40  # pixels
        self.arrow_width = 3
        self.arrow_color = "#0066FF"  # Blue
        self.rover_color = "#000000"  # Black
        self.rover_radius = 8
        
        # Initialize tkinter
        self.root = None
        self.canvas = None
        self.info_frame = None
        self.running = False
        
        # Info labels
        self.pos_label = None
        self.orient_label = None
        self.vel_label = None
        self.accel_label = None
    
    def _format_value(self, value, sig_figs):
        """Format a value to specified significant figures."""
        if value == 0:
            return "0.0"
        magnitude = math.floor(math.log10(abs(value))) if value != 0 else 0
        decimal_places = sig_figs - 1 - magnitude
        decimal_places = max(0, decimal_places)
        return f"{value:.{decimal_places}f}"
    
    def _format_tuple_sigfigs(self, values, sig_figs):
        """Format a tuple of values to specified significant figures."""
        formatted = []
        for v in values:
            if abs(v) < 1e-10:
                formatted.append("0.0")
            else:
                formatted.append(self._format_value(v, sig_figs))
        return f"({', '.join(formatted)})"
    
    def _world_to_screen(self, x, y):
        """
        Convert world coordinates (meters) to screen coordinates (pixels).
        
        Positive X is to the right, positive Y is up.
        """
        screen_x = self.center_x + x * self.scale
        screen_y = self.center_y - y * self.scale  # Y is inverted in screen coords
        return screen_x, screen_y
    
    def _on_resize(self, event):
        """Handle window resize events."""
        self.width = event.width
        self.height = event.height
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        self._refresh()
    
    def _draw_grid(self):
        """Draw the coordinate grid with major and minor lines."""
        # Calculate grid range
        x_range = self.width / (2 * self.scale)
        y_range = self.height / (2 * self.scale)
        
        # Draw minor grid lines (0.1m spacing, gray 32)
        x = 0
        while x <= x_range + self.minor_grid_spacing:
            for sign in [1, -1]:
                sx, _ = self._world_to_screen(sign * x, 0)
                if 0 <= sx <= self.width:
                    self.canvas.create_line(sx, 0, sx, self.height,
                                          fill=self.minor_grid_color, width=1,
                                          tags="grid")
            x += self.minor_grid_spacing
        
        y = 0
        while y <= y_range + self.minor_grid_spacing:
            for sign in [1, -1]:
                _, sy = self._world_to_screen(0, sign * y)
                if 0 <= sy <= self.height:
                    self.canvas.create_line(0, sy, self.width, sy,
                                          fill=self.minor_grid_color, width=1,
                                          tags="grid")
            y += self.minor_grid_spacing
        
        # Draw major grid lines (1m spacing, gray 64)
        x = 0
        while x <= x_range + self.major_grid_spacing:
            for sign in [1, -1]:
                sx, _ = self._world_to_screen(sign * x, 0)
                if 0 <= sx <= self.width:
                    self.canvas.create_line(sx, 0, sx, self.height,
                                          fill=self.major_grid_color, width=1,
                                          tags="grid")
            x += self.major_grid_spacing
        
        y = 0
        while y <= y_range + self.major_grid_spacing:
            for sign in [1, -1]:
                _, sy = self._world_to_screen(0, sign * y)
                if 0 <= sy <= self.height:
                    self.canvas.create_line(0, sy, self.width, sy,
                                          fill=self.major_grid_color, width=1,
                                          tags="grid")
            y += self.major_grid_spacing
        
        # Draw axes (thicker)
        sx, _ = self._world_to_screen(0, 0)
        _, sy = self._world_to_screen(0, 0)
        self.canvas.create_line(sx, 0, sx, self.height,
                               fill=self.axis_color, width=2, tags="axis")
        self.canvas.create_line(0, sy, self.width, sy,
                               fill=self.axis_color, width=2, tags="axis")
        
        # Axis labels
        label_offset = 20
        self.canvas.create_text(self.width - label_offset, self.center_y - label_offset,
                               text="+X", fill=self.axis_color, font=("Arial", 12, "bold"),
                               tags="axis")
        self.canvas.create_text(self.center_x + label_offset, label_offset,
                               text="+Y", fill=self.axis_color, font=("Arial", 12, "bold"),
                               tags="axis")
        self.canvas.create_text(self.center_x + 20, self.center_y + 20,
                               text="(0,0)", fill=self.axis_color, font=("Arial", 10),
                               tags="axis")
    
    def _draw_rover(self):
        """Draw the rover as a black dot with blue heading arrow."""
        # Get screen position (use x, y from position)
        sx, sy = self._world_to_screen(self.position[0], self.position[1])
        
        # Calculate arrow endpoint based on yaw (heading)
        heading_rad = math.radians(self.orientation[0])  # yaw
        end_x = sx + self.arrow_length * math.cos(heading_rad)
        end_y = sy - self.arrow_length * math.sin(heading_rad)  # Y inverted
        
        # Draw blue arrow (below dot)
        self.canvas.create_line(
            sx, sy, end_x, end_y,
            fill=self.arrow_color, width=self.arrow_width,
            arrow=tk.LAST, arrowshape=(12, 15, 5),
            tags="rover"
        )
        
        # Draw black dot (on top)
        self.canvas.create_oval(
            sx - self.rover_radius, sy - self.rover_radius,
            sx + self.rover_radius, sy + self.rover_radius,
            fill=self.rover_color, outline=self.rover_color,
            tags="rover"
        )
    
    def _update_info_panel(self):
        """Update the info panel with current navigation data."""
        if self.pos_label:
            pos_text = f"Position (x, y, z): {self._format_tuple_sigfigs(self.position, 4)} m"
            self.pos_label.config(text=pos_text)
        
        if self.orient_label:
            orient_text = f"Orientation (yaw, pitch, roll): {self._format_tuple_sigfigs(self.orientation, 3)}°"
            self.orient_label.config(text=orient_text)
        
        if self.vel_label:
            vel_text = f"Velocity (x, y, z): {self._format_tuple_sigfigs(self.velocity, 4)} m/s"
            self.vel_label.config(text=vel_text)
        
        if self.accel_label:
            accel_text = f"Acceleration (x, y, z): {self._format_tuple_sigfigs(self.acceleration, 4)} m/s²"
            self.accel_label.config(text=accel_text)
    
    def update(self, **kwargs):
        """
        Update the display with new navigation data.
        
        Args:
            position (tuple): (x, y, z) position in meters
            orientation (tuple): (yaw, pitch, roll) in degrees
            velocity (tuple): (vx, vy, vz) in m/s
            acceleration (tuple): (ax, ay, az) in m/s^2
        """
        if "position" in kwargs:
            self.position = kwargs["position"]
        if "orientation" in kwargs:
            self.orientation = kwargs["orientation"]
        if "velocity" in kwargs:
            self.velocity = kwargs["velocity"]
        if "acceleration" in kwargs:
            self.acceleration = kwargs["acceleration"]
        
        if self.canvas:
            self._refresh()
    
    def update_from_navigator(self):
        """Update display from the Navigation3D instance."""
        if self.navigator:
            self.position = tuple(self.navigator.pos)
            self.orientation = tuple(self.navigator.orientation)
            self.velocity = tuple(self.navigator.velocity)
            self.acceleration = tuple(self.navigator.acceleration)
            
            if self.canvas:
                self._refresh()
    
    def _refresh(self):
        """Refresh the entire display."""
        self.canvas.delete("all")
        self._draw_grid()
        self._draw_rover()
        # Raise rover to top layer
        self.canvas.tag_raise("rover")
        self._update_info_panel()
    
    def start(self):
        """Initialize the display window (non-blocking setup)."""
        self.root = tk.Tk()
        self.root.title(self.title)
        self.root.geometry(f"{self.width}x{self.height + 100}")
        
        # Create info frame at top
        self.info_frame = tk.Frame(self.root, bg="#F0F0F0", height=100)
        self.info_frame.pack(fill=tk.X, side=tk.TOP)
        self.info_frame.pack_propagate(False)
        
        # Info labels
        font = ("Consolas", 11)
        self.pos_label = tk.Label(self.info_frame, text="Position (x, y, z): (0.0, 0.0, 0.0) m",
                                  font=font, bg="#F0F0F0", anchor="w")
        self.pos_label.pack(fill=tk.X, padx=10, pady=2)
        
        self.orient_label = tk.Label(self.info_frame, text="Orientation (yaw, pitch, roll): (0.0, 0.0, 0.0)°",
                                     font=font, bg="#F0F0F0", anchor="w")
        self.orient_label.pack(fill=tk.X, padx=10, pady=2)
        
        self.vel_label = tk.Label(self.info_frame, text="Velocity (x, y, z): (0.0, 0.0, 0.0) m/s",
                                  font=font, bg="#F0F0F0", anchor="w")
        self.vel_label.pack(fill=tk.X, padx=10, pady=2)
        
        self.accel_label = tk.Label(self.info_frame, text="Acceleration (x, y, z): (0.0, 0.0, 0.0) m/s²",
                                    font=font, bg="#F0F0F0", anchor="w")
        self.accel_label.pack(fill=tk.X, padx=10, pady=2)
        
        # Create canvas (expandable)
        self.canvas = tk.Canvas(
            self.root,
            width=self.width,
            height=self.height,
            bg="white"
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Bind resize event
        self.canvas.bind("<Configure>", self._on_resize)
        
        self._refresh()
        self.running = True
    
    def process_events(self):
        """Process pending GUI events (call in main loop)."""
        if self.root:
            self.root.update()
    
    def close(self):
        """Close the display window."""
        self.running = False
        if self.root:
            self.root.destroy()
            self.root = None
    
    async def run_continuous(self, **kwargs):
        """
        Run continuous display update loop integrated with Navigation3D.
        
        Args:
            update_interval (float): Update interval in seconds (default: 0.1)
        """
        update_interval = kwargs.get("update_interval", 0.1)
        
        self.start()
        
        try:
            while self.running:
                # Update from navigator if available
                self.update_from_navigator()
                
                # Process GUI events
                self.process_events()
                
                await asyncio.sleep(update_interval)
        except tk.TclError:
            # Window was closed
            pass
        finally:
            self.running = False
    
    def run(self):
        """Start the display window (blocking)."""
        self.start()
        self.root.mainloop()


if __name__ == "__main__":
    # Demo: show display with simulated navigation data
    import time
    
    display = NavigationDisplay(
        width=800,
        height=800,
        scale=50.0
    )
    
    display.start()
    
    # Animate a circular path
    try:
        t = 0
        while display.running:
            # Circular motion
            x = 3 * math.cos(t)
            y = 3 * math.sin(t)
            z = 0.5 * math.sin(t * 0.5)
            
            yaw = math.degrees(t) + 90  # Tangent to circle
            pitch = 10 * math.sin(t * 2)
            roll = 5 * math.cos(t * 3)
            
            vx = -3 * math.sin(t)
            vy = 3 * math.cos(t)
            vz = 0.25 * math.cos(t * 0.5)
            
            ax = -3 * math.cos(t)
            ay = -3 * math.sin(t)
            az = -0.125 * math.sin(t * 0.5)
            
            display.update(
                position=(x, y, z),
                orientation=(yaw, pitch, roll),
                velocity=(vx, vy, vz),
                acceleration=(ax, ay, az)
            )
            display.process_events()
            
            t += 0.05
            time.sleep(0.05)
    except tk.TclError:
        # Window was closed
        pass
    except KeyboardInterrupt:
        display.close()
