package segway;

public class PID {
	double P;
	double I;
	double D;
	double setpoint;
	double lastErr = 0;
	double errSum = 0; // integral term
	
	public PID(double P, double I, double D, double setpoint) {
		this.P = P;
		this.I = I;
		this.D = D;
		this.setpoint = setpoint;
	}
	
	public double step(double dt, double input) {
		double error = setpoint - input;
		errSum += error * dt * I;
		
		double out = error * P + errSum + (error - lastErr) / dt * D;
		
		lastErr = error;
		return out;
	}
	
	public void setP(double P) {
		this.P = P;
	}
	
	public void setI(double I) {
		this.I = I;
	}

	public void setD(double D) {
		this.D = D;
	}
	
	public void setSetpoint(double setpoint) {
		
	}
	/*
	 * def __init__(self, P, I, D, setpoint):
        self.P = P
        self.I = I
        self.D = D
        self.setpoint = setpoint
        self.last_err = 0
        self.err_sum = 0  # integral term

    def step(self, dt, input):
        error = self.setpoint - input
        self.err_sum += error * dt * self.I

        out = error * self.P + self.err_sum + (error - self.last_err) / dt * self.D

        self.last_err = error

        return out

    def set_p(self, P):
        self.P = P

    def set_i(self, I):
        self.I = I

    def set_d(self, D):
        self.D = D

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
	 */
}
