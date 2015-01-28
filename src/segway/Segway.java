package segway;

import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;
import lejos.nxt.addon.GyroSensor;

public class Segway {
	AccelHTSensor as = new AccelHTSensor(SensorPort.S1);
	GyroSensor gs = new GyroSensor(SensorPort.S4);
	NXTRegulatedMotor mr = new NXTRegulatedMotor(MotorPort.B); // + back
	NXTRegulatedMotor ml = new NXTRegulatedMotor(MotorPort.C); // + back

	public void run() {
		if (as.getYAccel() > -180)
			System.out.println("Wrong position");
		while (as.getYAccel() > -180) {
			Utils.sleep(50);
		}
		Utils.sleep(500);

		float sum = 0;
		for (int i = 0; i < 10; i++) {
			sum += gs.getAngularVelocity();
		}
		float gyroOffset = sum / 10;

		System.out.println("Ready");

		while (as.getXAccel() < 180) {
			Utils.sleep(50);
		}

		PID pid = new PID(5, 0, 0, 0);
		long t1 = System.currentTimeMillis();

		mr.setSpeed(0);
		ml.setSpeed(0);
		mr.forward();
		ml.forward();

		while (true) {
			int x = as.getXAccel(); // + down
			// int y = as.getYAccel(); // + front
			// int z = as.getZAccel(); // + robot left

			float vel = gs.getAngularVelocity() - gyroOffset; // + back
			long t2 = System.currentTimeMillis();
			double change = -pid.step(t2 - t1, vel);
			t1 = t2;

			// LCD.clear();
			// LCD.drawString(Integer.toString(x), 1, 1);
			// LCD.drawString(Float.toString(vel), 1, 2);
			// LCD.drawString(Double.toString(change), 1, 3);

			setMotors((float) change);

			Utils.sleep(50);
		}
	}

	public void setMotors(float mrls) {
		setMotors(mrls, mrls);
	}

	public void setMotors(float mrs, float mls) {
		if (mrs < 0) {
			mr.setSpeed(Math.abs(mrs));
			mr.backward();
		} else if (mrs > 0) {
			mr.setSpeed(Math.abs(mrs));
			mr.forward();
		} else
			mr.setSpeed(0);

		if (mls < 0) {
			ml.setSpeed(Math.abs(mls));
			ml.backward();
		} else if (mls > 0) {
			ml.setSpeed(Math.abs(mls));
			ml.forward();
		} else
			ml.setSpeed(0);
	}
}
