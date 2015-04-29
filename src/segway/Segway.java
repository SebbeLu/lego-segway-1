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
	static final double MOVING_AVG_PERCENTAGE_GYRO = 0.95;
	static final double MOVING_AVG_PERCENTAGE_ACCEL = 0.5;
	static final int AVG_SAMPLES = 20;
	static final double CORRECT_ANGLE = -5;

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
		setMotors(0, 0);
		
		// start
		System.out.println("Ready");
		//while (as.getXAccel() < 205) { Utils.sleep(50); }
		Button.waitForAnyPress();

		//http://www.raysforexcellence.se/wp-content/uploads/2013/01/Dennis-Jin-Development-of-a-stable-control-system-for-a-segway.pdf
		//http://www.chrismarion.net/index.php?option=com_content&view=article&id=122:the-segway-theory&catid=44:robotics
		
		PID pid = new PID(10, 0.01, 1, 0); // *** P I D ***
		long t1 = System.currentTimeMillis();

		accelX = as.getXAccel();
		accelY = as.getYAccel();
		anglePrev = Math.toDegrees(Math.atan2(accelX, accelY));
		long t2;
		double change;
		long tUpdate = System.currentTimeMillis();
		while (true) {
			if (Button.readButtons() == Button.ID_ENTER) {
				pid.reset();
				setMotors(0, 0);
				Utils.sleep(1000);
				t1 = System.currentTimeMillis();
			}
			t2 = System.currentTimeMillis();
			// set motors: - front
			// x = as.getXAccel(); // + down
			// y = as.getYAccel(); // + front
			// int z = as.getZAccel(); // + robot left
			// gyroVel: - front
			updateAccel();
			updateGyroVel();
			
			// accel is SLOOOOOOW !!!!
			double angleCurr = Math.toDegrees(Math.atan2(accelX, accelY)) - 90; // front: -
			double accelChangeFromOptimal = getChangeFromOptimalAccel(angleCurr) * (t2 - t1) / 1000; // better +
			
			double toPid = 0;
			
			double coef = (Math.cos(Math.toRadians(angleCurr) * 2) + 1) / 2;
			double direction = Math.signum(angleCurr + CORRECT_ANGLE);
			toPid = gyroVel;
			//toPid = /*-angleCurr * (1-coef) +*/ 50 * direction * accelChangeFromOptimal /** coef*/;
			//toPid = 50 * direction * accelChangeFromOptimal;
			
			change = -pid.step(t2 - t1, toPid);
			
			if (Button.readButtons() == Button.ID_ESCAPE) setMotors(0, 0);
			else setMotors((float) change);
			
			if (System.currentTimeMillis() - tUpdate > 250) {
				LCD.clear();
				LCD.drawString("y  " + accelY, 1, 1);
				LCD.drawString("v  " + gyroVel, 1, 2);
				LCD.drawString("an " + angleCurr, 1, 3);
				//LCD.drawString("co " + accelChangeFromOptimal, 1, 3);
				//LCD.drawString("c  " + coef, 1, 4);
				LCD.drawString("to " + toPid, 1, 4);
				LCD.drawString("c  " + change, 1, 5);
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
		gyroVel = gyroVel * MOVING_AVG_PERCENTAGE_GYRO +
				(gs.getAngularVelocity() - gyroOffset) * (1 - MOVING_AVG_PERCENTAGE_GYRO);
	}
	
	public void updateAccel() {
		accelX = accelX * MOVING_AVG_PERCENTAGE_ACCEL + as.getXAccel() * (1 - MOVING_AVG_PERCENTAGE_ACCEL);
		accelY = accelY * MOVING_AVG_PERCENTAGE_ACCEL + as.getYAccel() * (1 - MOVING_AVG_PERCENTAGE_ACCEL);
		/*accelX = as.getXAccel();
		accelY = as.getYAccel();*/
	}

	public void setMotors(double mrls) {
		/*setMotors(mrls + 18*Math.sin(System.currentTimeMillis()/(double)50),
				mrls + 18*Math.sin(System.currentTimeMillis()/(double)50 + Math.PI));*/
		setMotors(mrls, mrls);
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
