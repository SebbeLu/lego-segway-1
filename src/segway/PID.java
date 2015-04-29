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
		this.setpoint = setpoint;
	}
	
	public void reset() {
		lastErr = 0;
		errSum = 0;
	}
}
