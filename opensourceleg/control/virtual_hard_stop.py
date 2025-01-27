from ..hardware.joints import Joint
from enum import Enum

class Direction(Enum):
    POSITIVE = 1
    NEGATIVE = -1

class VirtualHardStop:
    """
    VirtualHardStop adds a snubbing behavior to an actuator to prevent aggressively hitting a hard stop. 
    Will need to update for latest api. 
    Kevin Best, 1/27/2025
    """

    def __init__(self, joint: Joint, direction:Direction, start_position_rad:float, stiffness:float, damping:float):
        """
        Creates an instance of a virtual hard stop

        Parameters:
        joint (Joint): The joint to which the virtual hard stop is applied
        direction (Direction): The direction of the hard stop (POSITIVE or NEGATIVE)
        start_position_rad (float): The starting position of the hard stop in radians
        stiffness (float): The stiffness coefficient for the hard stop in Nm/rad
        damping (float): The damping coefficient for the hard stop in Nm/(rad/s)
        """
        self.joint = joint
        self.direction = direction
        self.start_position  = start_position_rad
        self.stiffness = stiffness
        self.damping = damping

    def calculate_hard_stop_torque(self):
        """
        Calculates the torque to use to slow the joint from moving in the direction of the hard stop

        Returns:
        float: The calculated torque to slow the joint
        """
        current_pos = self.joint.output_position
        delta_theta = current_pos - self.start_position
        current_vel = self.joint.output_velocity

        # Check the direction of the hard stop and calculate torque accordingly
        if self.direction == Direction.POSITIVE:
            if current_pos > self.start_position:
                torque = -self.stiffness * delta_theta - self.damping * current_vel
            else:
                return 0
        else:
            if current_pos < self.start_position:
                torque = -self.stiffness * delta_theta - self.damping * current_vel
            else:
                return 0

        return torque
    
    def calculate_eq_angle_bias(self, joint_K):
        """
        Calculates the equivalent angle bias based on the hard stop torque and joint stiffness

        Parameters:
        joint_K (float): The stiffness of the joint's impedance controller (Nm/rad)

        Returns:
        float: The equivalent angle bias in radians
        """
        hard_stop_torque = self.calculate_hard_stop_torque()
        return hard_stop_torque / joint_K

