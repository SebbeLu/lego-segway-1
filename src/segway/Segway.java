package segway;

import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;
import lejos.nxt.addon.GyroSensor;
import lejos.robotics.EncoderMotor;

public class Segway {
	static final double MOVING_AVERAGE_PERCENTAGE = 0.9;
	static final int AVG_SAMPLES = 20;

	AccelHTSensor as = new AccelHTSensor(SensorPort.S1);
	GyroSensor gs = new GyroSensor(SensorPort.S3);
	EncoderMotor mr = new NXTMotor(MotorPort.A); // + back
	EncoderMotor ml = new NXTMotor(MotorPort.C); // + back

	double gyroOffset;
	double gyroLast;

	public void run() {
		mr.setPower(0);
		ml.setPower(0);
		mr.forward();
		ml.forward();

		// wait for horizontal position
		if (as.getYAccel() > -180)
			System.out.println("Wrong position");
		while (as.getYAccel() > -180) {
			Utils.sleep(50);
		}
		Utils.sleep(500);

		// find gyro offset
		double sum = 0;
		for (int i = 0; i < AVG_SAMPLES; i++) {
			setMotors(0);
			sum += gs.getAngularVelocity();
			Utils.sleep(100);
		}
		gyroOffset = sum / AVG_SAMPLES;
		System.out.println("Offset: " + gyroOffset);
		gyroLast = gyroOffset;
		
		System.out.println("Ready");
		while (as.getXAccel() < 205) { Utils.sleep(50); }

		PID pid = new PID(10, 0, 0, 0);
		long t1 = System.currentTimeMillis();

		int x;
		double vel;
		long t2;
		double change;
		long tUpdate = System.currentTimeMillis();
		while (true) {
			x = as.getXAccel(); // + down
			// int y = as.getYAccel(); // + front
			// int z = as.getZAccel(); // + robot left

			vel = getGyroVal();
			t2 = System.currentTimeMillis();
			change = -pid.step(t2 - t1, vel);
			setMotors((float) change);

			if (System.currentTimeMillis() - tUpdate > 250) {
				LCD.clear();
				LCD.drawString(Integer.toString(x), 1, 1);
				LCD.drawString(Double.toString(vel), 1, 2);
				LCD.drawString(Double.toString(change), 1, 3);
				LCD.drawString("" + System.currentTimeMillis(), 1, 4);
				tUpdate = System.currentTimeMillis();
			}

			t1 = t2;
			Utils.sleep(1);
		}
	}

	public double getGyroVal() {
		gyroLast = gyroLast * MOVING_AVERAGE_PERCENTAGE + gs.getAngularVelocity() * (1 - MOVING_AVERAGE_PERCENTAGE);
		//gyroLast = (100 * gyroLast + gs.getAngularVelocity()) / 101;
		return gyroLast - gyroOffset; // + back
	}

	public void setMotors(double mrls) {
		setMotors(mrls + 5*Math.sin(System.currentTimeMillis()/100),
				mrls + 5*Math.sin(System.currentTimeMillis()/100 + Math.PI));
		//setMotors(mrls, mrls);
	}

	public void setMotors(double mrs, double mls) {
		mr.setPower((int) Math.abs(mrs));
		if (mrs < 0) {
			mr.backward();
		} else if (mrs > 0) {
			mr.forward();
		}

		ml.setPower((int) Math.abs(mls));
		if (mls < 0) {
			ml.backward();
		} else if (mls > 0) {
			ml.forward();
		}
	}
}
