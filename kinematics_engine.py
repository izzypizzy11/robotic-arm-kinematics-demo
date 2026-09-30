"""
Kinematics & Reachability Simulation for Planar Manipulators
Author: Abiodun Israel (izzypizzy11)
Description: Forward Kinematics (FK) and Inverse Kinematics (IK) calculation
             with workspace boundary verification to prevent joint singularity.
"""

import math

class PlanarRoboticArm:
    def __init__(self, l1=10.0, l2=10.0):
        self.l1 = l1
        self.l2 = l2

    def forward_kinematics(self, theta1_deg, theta2_deg):
        """Calculates (x, y) end-effector position given joint angles."""
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        
        x = self.l1 * math.cos(t1) + self.l2 * math.cos(t1 + t2)
        y = self.l1 * math.sin(t1) + self.l2 * math.sin(t1 + t2)
        return round(x, 3), round(y, 3)

    def inverse_kinematics(self, x, y):
        """Calculates required joint angles (theta1, theta2) for target (x, y)."""
        dist_sq = x**2 + y**2
        max_reach = (self.l1 + self.l2)**2
        
        if dist_sq > max_reach:
            raise ValueError(f"Target ({x}, {y}) is outside reachable workspace.")
            
        cos_t2 = (dist_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))  # Numerical clamping
        
        t2 = math.acos(cos_t2)
        t1 = math.atan2(y, x) - math.atan2(self.l2 * math.sin(t2), self.l1 + self.l2 * math.cos(t2))
        
        return round(math.degrees(t1), 2), round(math.degrees(t2), 2)

if __name__ == "__main__":
    arm = PlanarRoboticArm(l1=15.0, l2=12.0)
    target_x, target_y = 18.0, 10.0
    
    j1, j2 = arm.inverse_kinematics(target_x, target_y)
    print(f"Target: ({target_x}, {target_y}) -> Computed Joint Angles: J1={j1}°, J2={j2}°")
    
    verified_x, verified_y = arm.forward_kinematics(j1, j2)
    print(f"Forward Kinematics Verification: ({verified_x}, {verified_y})")
