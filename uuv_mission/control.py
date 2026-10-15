"""Feedback controller for the UUV control loop"""

class PDController:
    """Discrete-time Proportional-derivative (PD) feedback controller.

    Implements:
        u[t] = kp * e[t] + kd *(e[t] - e[t-1])
        e[t] = r[t] - y[t]
        with initial condition: e[-1] = 0
    """
    
    def __init__(self, kp: float = 0.15, kd: float = 0.6) -> None:
        self.kp = kp 
        self.kd = kd 
        self._prev_error = 0.0 

    def reset(self) -> None:
        """Reset the controller to its initial state"""
        self._prev_error = 0.0

    def compute(self, reference: float, observation: float) -> float:
        """ Compute the control action u[t] for one time step.

        Args:
            reference: Target depth r[t]
            observation: Measured depth y[t]

        Returns:
            The control action u[t]
        """
        error = reference - observation
        response = self.kp * error + self.kd * (error - self._prev_error)
        self._prev_error = error 
        return response 

