package segway;

import java.util.Queue;

import lejos.nxt.BasicMotor;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;
import lejos.nxt.addon.GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.navigation.Segoway;

public class Segway {
	final static double MOVING_AVERAGE_PERCENTAGE = 0.6;
	
	AccelHTSensor as = new AccelHTSensor(SensorPort.S1);
	GyroSensor gs = new GyroSensor(SensorPort.S4);
	EncoderMotor mr = new NXTMotor(MotorPort.A); // + back
	EncoderMotor ml = new NXTMotor(MotorPort.C); // + back

	float gyroOffset;
	//Queue<Float> gyroVals = new Queue<>();
	double gyroLast;
	
	public void run() {
		// wait for horizontal position
		if (as.getYAccel() > -180)
			System.out.println("Wrong position");
		while (as.getYAccel() > -180) {
			Utils.sleep(50);
		}
		Utils.sleep(500);

		// find gyro offset
		float sum = 0;
		for (int i = 0; i < 10; i++) {
			sum += gs.getAngularVelocity();
		}
		gyroOffset = sum / 10;
		gyroLast = gyroOffset;

		System.out.println("Ready");

		while (as.getXAccel() < 180) {
			Utils.sleep(50);
		}

		PID pid = new PID(5, 0, 0, 0);
		long t1 = System.currentTimeMillis();

        mr.setPower(0);
        ml.setPower(0);
        mr.forward();
        ml.forward();

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
				// LCD.clear();
				// LCD.drawString(Integer.toString(x), 1, 1);
				// LCD.drawString(Double.toString(vel), 1, 2);
				LCD.drawString(Double.toString(change), 1, 3);
				LCD.drawString(""+System.currentTimeMillis(), 1, 4);
				tUpdate = System.currentTimeMillis();
			}
			
			t1 = t2;
			Utils.sleep(1);
		}
	}

	public double getGyroVal() {
		gyroLast = gyroLast * MOVING_AVERAGE_PERCENTAGE + gs.getAngularVelocity() * (1 - MOVING_AVERAGE_PERCENTAGE);
		return gyroLast - gyroOffset; // + back
	}

	public void setMotors(float mrls) {
		setMotors(mrls, mrls);
	}

	public void setMotors(float mrs, float mls) {
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
