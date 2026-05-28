#!/usr/bin/env python3
"""
PID Controller
Classic proportional-integral-derivative controller for heading and speed control
"""

import numpy as np


class PIDController:
    """
    PID Controller with anti-windup
    
    Implements: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
    """
    
    def __init__(self, kp=1.0, ki=0.05, kd=0.3, dt=0.1):
        """
        Initialize PID Controller
        
        Args:
            kp: Proportional gain (default: 1.0)
            ki: Integral gain (default: 0.05)
            kd: Derivative gain (default: 0.3)
            dt: Sample time in seconds (default: 0.1)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral   = 0.0
        self.prev_error = 0.0
        self.filt_rate  = 0.0   # untuk update_with_rate()
        
    def update(self, error):
        """
        Compute PID control signal
        
        Args:
            error: Current error (setpoint - measured value)
            
        Returns:
            Control signal output
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0
        d_term = self.kd * derivative
        
        self.prev_error = error
        
        # Total output
        output = p_term + i_term + d_term
        
        return output
    
    def update_with_rate(self, error, measured_rate):
        """
        PID dengan derivative on measurement — cocok untuk heading control.

        Derivative term memakai yaw rate r dari sensor (bukan Δerror/dt),
        sehingga:
          - Bebas dari noise finite-difference
          - Tidak ada derivative kick saat setpoint berubah tiba-tiba
          - Ekuivalen dengan -kd·r karena d(e_ψ)/dt = ψ_d_dot - r ≈ -r
            (saat heading reference berubah lambat)

        Anti-windup zero-crossing: integral dikurangi 50% saat error berganti
        tanda, mencegah overshoot saat kapal melewati heading target.

        Args:
            error:         Heading error e_ψ [rad] (setpoint − measured)
            measured_rate: Yaw rate r [rad/s] dari sensor odom

        Returns:
            Control signal output (normalized, di-clip oleh caller)
        """
        p_term = self.kp * error

        # Anti-windup zero-crossing
        if error * self.prev_error < 0:
            self.integral *= 0.5

        self.integral += error * self.dt
        self.integral  = np.clip(self.integral, -1.0, 1.0)
        i_term = self.ki * self.integral

        # Derivative on measurement: filter yaw rate lalu gunakan sebagai D-term
        # α = 0.8 → filter ringan, bandwidth tinggi, cegah spike sesaat
        self.filt_rate += 0.8 * (measured_rate - self.filt_rate)
        d_term = -self.kd * self.filt_rate

        self.prev_error = error
        return p_term + i_term + d_term

    def reset(self):
        """Reset semua state controller."""
        self.integral   = 0.0
        self.prev_error = 0.0
        self.filt_rate  = 0.0
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """
        Update controller gains
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def get_state(self):
        """Get current controller state."""
        return {
            'integral':   self.integral,
            'prev_error': self.prev_error,
            'filt_rate':  self.filt_rate,
        }

    def set_state(self, state):
        """Set controller state."""
        self.integral   = state.get('integral',   0.0)
        self.prev_error = state.get('prev_error', 0.0)
        self.filt_rate  = state.get('filt_rate',  0.0)
