package segway;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;
import lejos.nxt.addon.GyroSensor;
import lejos.nxt.addon.OpticalDistanceSensor;
import lejos.robotics.EncoderMotor;

public class Segway {
	static final double MOVING_AVERAGE_PERCENTAGE = 0.9;
	static final int AVG_SAMPLES = 20;
	static final double CORRECT_ANGLE = 85 - 90;

	AccelHTSensor as = new AccelHTSensor(SensorPort.S1);
	GyroSensor gs = new GyroSensor(SensorPort.S3);
	EncoderMotor mr = new NXTMotor(MotorPort.A); // + back
	EncoderMotor ml = new NXTMotor(MotorPort.C); // + back

	double gyroOffset;
	double gyroVel;
	double accelX;
	double accelY;
	double anglePrev;

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
		gyroVel = 0;
		
		// start
		System.out.println("Ready");
		//while (as.getXAccel() < 205) { Utils.sleep(50); }
		Button.waitForAnyPress();

		PID pid = new PID(10, 0, 0, 0);
		long t1 = System.currentTimeMillis();

		accelX = as.getXAccel();
		accelY = as.getYAccel();
		anglePrev = Math.toDegrees(Math.atan2(accelX, accelY));
		long t2;
		double change;
		long tUpdate = System.currentTimeMillis();
		while (true) {
			t2 = System.currentTimeMillis();
			// x = as.getXAccel(); // + down
			// y = as.getYAccel(); // + front
			// int z = as.getZAccel(); // + robot left
			// gyroVel - front
			updateAccel();
			updateGyroVel();
			double angleCurr = Math.toDegrees(Math.atan2(accelX, accelY)) - 90; // front: -
			double accelChangeFromOptimal = getChangeFromOptimalAccel(angleCurr) * (t2 - t1) / 1000;

			double toPid = 0;
			double offset = Math.abs(CORRECT_ANGLE - angleCurr);
			
			toPid = angleCurr * offset + gyroVel * (90 - offset);
			
			change = pid.step(t2 - t1, 0);
			setMotors((float) change);

			if (System.currentTimeMillis() - tUpdate > 250) {
				LCD.clear();
				LCD.drawString("x  " + accelX, 1, 1);
				LCD.drawString("v  " + gyroVel, 1, 2);
				LCD.drawString("c  " + change, 1, 3);
				LCD.drawString("an " + angleCurr, 1, 4);
				LCD.drawString("co " + accelChangeFromOptimal, 1, 5);
				//LCD.drawString("d  " + (vel - angleChange), 1, 6);
				LCD.drawString("" + System.currentTimeMillis(), 1, 7);
				tUpdate = System.currentTimeMillis();
			}

			anglePrev = angleCurr;
			t1 = t2;
			Utils.sleep(1);
		}
	}
	
	public double getChangeFromOptimalAccel(double angleCurr) {
		double changeFromOptimal;
		if (angleCurr > CORRECT_ANGLE && CORRECT_ANGLE > anglePrev) {
			changeFromOptimal = CORRECT_ANGLE - angleCurr;
		} else if (angleCurr < CORRECT_ANGLE && CORRECT_ANGLE < anglePrev) {
			changeFromOptimal = angleCurr - CORRECT_ANGLE;
		} else {
			changeFromOptimal = Math.abs(angleCurr - anglePrev);
			double dc = Math.abs(CORRECT_ANGLE - angleCurr);
			double dp = Math.abs(CORRECT_ANGLE - anglePrev);
			if (dc > dp) {
				changeFromOptimal *= -1;
			}
		}
		return changeFromOptimal;
	}

	public void updateGyroVel() {
		gyroVel = gyroVel * MOVING_AVERAGE_PERCENTAGE +
				(gs.getAngularVelocity() - gyroOffset) * (1 - MOVING_AVERAGE_PERCENTAGE);
	}
	
	public void updateAccel() {
		accelX = accelX * MOVING_AVERAGE_PERCENTAGE + as.getXAccel() * (1 - MOVING_AVERAGE_PERCENTAGE);
		accelY = accelY * MOVING_AVERAGE_PERCENTAGE + as.getYAccel() * (1 - MOVING_AVERAGE_PERCENTAGE);
	}

	public void setMotors(double mrls) {
		setMotors(mrls + 18*Math.sin(System.currentTimeMillis()/(double)40),
				mrls + 18*Math.sin(System.currentTimeMillis()/(double)40 + Math.PI));
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
