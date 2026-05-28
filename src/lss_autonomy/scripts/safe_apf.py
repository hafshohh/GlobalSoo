#!/usr/bin/env python3
"""
Safe Artificial Potential Field (Safe APF)
Local obstacle avoidance using attractive and repulsive forces
"""

import numpy as np


class SafeAPF:
    """Safe Artificial Potential Field for local obstacle avoidance"""
    
    def __init__(self, k_att=1.0, k_rep=0.5, d_rep=3.0):
        """
        Initialize Safe APF
        
        Args:
            k_att: Attractive force gain (default: 1.0)
            k_rep: Repulsive force gain (default: 0.5)
            d_rep: Repulsive influence distance in meters (default: 3.0)
        """
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_rep = d_rep
        
    def compute_force(self, position, goal, obstacles):
        """
        Compute total APF force (attractive + repulsive)
        
        Args:
            position: Current position (x, y)
            goal: Target position (x, y)
            obstacles: List of obstacle positions [(x, y), ...]
            
        Returns:
            Total force vector [fx, fy]
        """
        pos = np.array(position, dtype=float)
        goal_arr = np.array(goal, dtype=float)
        
        # Attractive force - pulls toward goal
        f_att = self._compute_attractive_force(pos, goal_arr)
        
        # Repulsive force - pushes away from obstacles
        f_rep = self._compute_repulsive_force(pos, obstacles)
        
        # Total force
        total = f_att + f_rep
        return total
    
    def _compute_attractive_force(self, pos, goal):
        """
        Compute attractive force toward goal
        
        Args:
            pos: Current position array [x, y]
            goal: Goal position array [x, y]
            
        Returns:
            Attractive force vector [fx, fy]
        """
        direction = goal - pos
        dist = np.linalg.norm(direction)
        
        if dist > 1e-6:
            f_att = self.k_att * direction / (dist + 1e-6)
        else:
            f_att = np.array([0.0, 0.0])
        
        return f_att
    
    def _compute_repulsive_force(self, pos, obstacles):
        """
        Compute repulsive force from obstacles
        
        Args:
            pos: Current position array [x, y]
            obstacles: List of obstacle positions [(x, y), ...]
            
        Returns:
            Repulsive force vector [fx, fy]
        """
        f_rep = np.array([0.0, 0.0])
        
        for obs in obstacles:
            obs_arr = np.array(obs, dtype=float)
            direction_from_obs = pos - obs_arr
            dist_obs = np.linalg.norm(direction_from_obs)
            
            # Only repel if within influence distance
            if dist_obs < self.d_rep and dist_obs > 1e-6:
                # Repulsive potential: (1/d - 1/d_rep) / d^2
                magnitude = self.k_rep * (1.0/dist_obs - 1.0/self.d_rep) / (dist_obs ** 2)
                f_rep += magnitude * (direction_from_obs / dist_obs)
        
        return f_rep
    
    def set_gains(self, k_att=None, k_rep=None, d_rep=None):
        """
        Update APF gains
        
        Args:
            k_att: Attractive force gain
            k_rep: Repulsive force gain
            d_rep: Repulsive influence distance
        """
        if k_att is not None:
            self.k_att = k_att
        if k_rep is not None:
            self.k_rep = k_rep
        if d_rep is not None:
            self.d_rep = d_rep
