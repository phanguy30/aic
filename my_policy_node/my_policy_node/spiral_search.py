"""
Archimedean Spiral Search Algorithm for Force-Compliant Cable Insertion.

This module implements a spiral search pattern that moves the robot's end-effector
in an expanding spiral while monitoring Z-axis force. When the force drops (indicating
the plug has found the hole), the search stops and returns the current pose as the
insertion target.

Usage:
    searcher = SpiralSearch(center_x=0.35, center_y=0.0, center_z=1.05)
    
    while not searcher.is_done():
        wrench = get_wrench_reading()  # from F/T sensor
        target_pose = searcher.step(wrench)
        move_robot_to(target_pose)
"""

import math
from dataclasses import dataclass, field
from typing import Optional


# --------------------------------------------------------------------------
# Mock geometry_msgs types (pure Python, no ROS dependency)
# --------------------------------------------------------------------------

@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class Wrench:
    force: Vector3 = field(default_factory=Vector3)
    torque: Vector3 = field(default_factory=Vector3)


@dataclass
class Header:
    frame_id: str = ""
    stamp: float = 0.0


@dataclass
class WrenchStamped:
    header: Header = field(default_factory=Header)
    wrench: Wrench = field(default_factory=Wrench)


# --------------------------------------------------------------------------
# Spiral Search Algorithm
# --------------------------------------------------------------------------

class SpiralSearch:
    """
    Archimedean spiral search for force-compliant insertion.

    The robot moves in an expanding spiral pattern: r(t) = a * t, where
    x(t) = a*t*cos(t) and y(t) = a*t*sin(t).

    The search monitors the Z-axis force:
    - If force exceeds `contact_force_threshold`, the plug is pressing against
      the surface (not in the hole) — continue spiraling.
    - If force drops below `insertion_force_threshold`, the plug has found the
      hole — stop and descend.

    Parameters
    ----------
    center_x, center_y, center_z : float
        The initial contact point (where the plug first touched the surface).
    orientation : Quaternion
        The desired end-effector orientation (kept constant during search).
    spiral_spacing : float
        Distance between successive spiral loops (meters). Default 0.002 (2mm).
    angular_step : float
        How much theta advances per step (radians). Default 0.1.
    max_radius : float
        Maximum spiral radius before giving up (meters). Default 0.02 (2cm).
    contact_force_threshold : float
        Z-force (N) above which we consider the plug is pressing on surface.
    insertion_force_threshold : float
        Z-force (N) below which we consider the plug has found the hole.
    descent_rate : float
        How much to lower Z per step during insertion (meters). Default 0.0005.
    """

    def __init__(
        self,
        center_x: float = 0.0,
        center_y: float = 0.0,
        center_z: float = 0.0,
        orientation: Optional[Quaternion] = None,
        spiral_spacing: float = 0.002,
        angular_step: float = 0.1,
        max_radius: float = 0.02,
        contact_force_threshold: float = 10.0,
        insertion_force_threshold: float = 3.0,
        descent_rate: float = 0.0005,
    ):
        self.center_x = center_x
        self.center_y = center_y
        self.center_z = center_z
        self.orientation = orientation or Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Spiral parameters: r = a * theta
        # spacing = 2*pi*a => a = spacing / (2*pi)
        self.a = spiral_spacing / (2.0 * math.pi)
        self.angular_step = angular_step
        self.max_radius = max_radius

        # Force thresholds
        self.contact_force_threshold = contact_force_threshold
        self.insertion_force_threshold = insertion_force_threshold
        self.descent_rate = descent_rate

        # State
        self.theta = 0.0
        self._done = False
        self._inserting = False
        self._found_hole_position: Optional[Point] = None
        self._step_count = 0

    def is_done(self) -> bool:
        """Returns True when insertion is complete or search exhausted."""
        return self._done

    def is_inserting(self) -> bool:
        """Returns True when the hole has been found and we're descending."""
        return self._inserting

    def current_radius(self) -> float:
        """Current spiral radius."""
        return self.a * self.theta

    def step(self, wrench: WrenchStamped) -> Pose:
        """
        Advance one step of the spiral search.

        Parameters
        ----------
        wrench : WrenchStamped
            Current force/torque reading from the wrist sensor.

        Returns
        -------
        Pose
            The next target pose for the end-effector.
        """
        self._step_count += 1
        fz = abs(wrench.wrench.force.z)

        # --- Phase 2: Inserting (hole found, descending) ---
        if self._inserting:
            self.center_z -= self.descent_rate

            # If force builds up again during insertion, we're fully seated
            if fz > self.contact_force_threshold:
                self._done = True

            return Pose(
                position=Point(
                    x=self._found_hole_position.x,
                    y=self._found_hole_position.y,
                    z=self.center_z,
                ),
                orientation=self.orientation,
            )

        # --- Phase 1: Spiral searching ---

        # Check if plug dropped into hole (force dropped)
        if self._step_count > 5 and fz < self.insertion_force_threshold:
            self._inserting = True
            r = self.a * self.theta
            self._found_hole_position = Point(
                x=self.center_x + r * math.cos(self.theta),
                y=self.center_y + r * math.sin(self.theta),
                z=self.center_z,
            )
            return Pose(
                position=self._found_hole_position,
                orientation=self.orientation,
            )

        # Advance spiral
        self.theta += self.angular_step
        r = self.a * self.theta

        # Check if we've exceeded max search radius
        if r > self.max_radius:
            self._done = True
            return Pose(
                position=Point(x=self.center_x, y=self.center_y, z=self.center_z),
                orientation=self.orientation,
            )

        # Archimedean spiral: x = r*cos(t), y = r*sin(t)
        x = self.center_x + r * math.cos(self.theta)
        y = self.center_y + r * math.sin(self.theta)

        return Pose(
            position=Point(x=x, y=y, z=self.center_z),
            orientation=self.orientation,
        )


# --------------------------------------------------------------------------
# Demo / Self-test
# --------------------------------------------------------------------------

if __name__ == "__main__":
    print("=== Archimedean Spiral Search Demo ===\n")

    searcher = SpiralSearch(
        center_x=0.350,
        center_y=0.000,
        center_z=1.050,
        spiral_spacing=0.003,  # 3mm between loops
        angular_step=0.15,
        max_radius=0.015,      # 15mm max search
        contact_force_threshold=15.0,
        insertion_force_threshold=3.0,
    )

    # Simulate: plug pressing on surface for 40 steps, then finds hole
    step = 0
    while not searcher.is_done():
        step += 1

        # Mock wrench: high force (pressing) for first 40 steps, then drops
        if step < 40:
            mock_wrench = WrenchStamped(
                wrench=Wrench(force=Vector3(x=0.1, y=-0.2, z=-12.0))
            )
        else:
            mock_wrench = WrenchStamped(
                wrench=Wrench(force=Vector3(x=0.0, y=0.0, z=-1.5))
            )

        target = searcher.step(mock_wrench)

        if step <= 5 or step % 10 == 0 or searcher.is_inserting():
            status = "INSERTING" if searcher.is_inserting() else f"r={searcher.current_radius()*1000:.2f}mm"
            print(
                f"  Step {step:3d} | Fz={mock_wrench.wrench.force.z:6.1f}N | "
                f"{status:12s} | "
                f"Target: ({target.position.x:.4f}, {target.position.y:.4f}, {target.position.z:.4f})"
            )

        if step > 200:
            print("  [Safety limit reached]")
            break

    print(f"\n  Search complete after {step} steps.")
    if searcher.is_inserting():
        print(f"  Hole found at: ({searcher._found_hole_position.x:.4f}, {searcher._found_hole_position.y:.4f})")
    else:
        print("  Hole NOT found within search radius.")
