package segway;

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
	static final double CORRECT_ANGLE = 85;

	AccelHTSensor as = new AccelHTSensor(SensorPort.S1);
	GyroSensor gs = new GyroSensor(SensorPort.S3);
	EncoderMotor mr = new NXTMotor(MotorPort.A); // + back
	EncoderMotor ml = new NXTMotor(MotorPort.C); // + back

	double gyroOffset;
	double gyroLast;
	double xLast;
	double yLast;
	double avgChange = 0;

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

		PID pid = new PID(10000, 100, 0, 0);
		long t1 = System.currentTimeMillis();

		xLast = as.getXAccel();
		yLast = as.getYAccel();
		double anglePrev = Math.toDegrees(Math.atan2(xLast, yLast));
		long t2;
		double change;
		long tUpdate = System.currentTimeMillis();
		while (true) {
			t2 = System.currentTimeMillis();
			// x = as.getXAccel(); // + down
			// y = as.getYAccel(); // + front
			// int z = as.getZAccel(); // + robot left
			updateAccel();
			double angleCurr = Math.toDegrees(Math.atan2(xLast, yLast));
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
			changeFromOptimal *= (double) (t2 - t1) / 1000;			
			//double angleChange = (angleCurr - anglePrev) / (t2 - t1) * 1000; // - front 

			double vel = getGyroVal(); // - front
			//change = -pid.step(t2 - t1, vel);
			change = pid.step(t2 - t1, changeFromOptimal) * Math.signum(vel);
			setMotors((float) change);

			if (System.currentTimeMillis() - tUpdate > 250) {
				LCD.clear();
				LCD.drawString("x  " + xLast, 1, 1);
				LCD.drawString("v  " + vel, 1, 2);
				LCD.drawString("c  " + change, 1, 3);
				LCD.drawString("a  " + angleCurr, 1, 4);
				LCD.drawString("co " + changeFromOptimal, 1, 5);
				//LCD.drawString("d  " + (vel - angleChange), 1, 6);
				LCD.drawString("" + System.currentTimeMillis(), 1, 7);
				tUpdate = System.currentTimeMillis();
			}

			anglePrev = angleCurr;
			t1 = t2;
			Utils.sleep(1);
		}
	}

	public double getGyroVal() {
		gyroLast = gyroLast * MOVING_AVERAGE_PERCENTAGE + gs.getAngularVelocity() * (1 - MOVING_AVERAGE_PERCENTAGE);
		//gyroLast = (100 * gyroLast + gs.getAngularVelocity()) / 101;
		return gyroLast - gyroOffset; // + back
	}
	
	public void updateAccel() {
		xLast = xLast * MOVING_AVERAGE_PERCENTAGE + as.getXAccel() * (1 - MOVING_AVERAGE_PERCENTAGE);
		yLast = yLast * MOVING_AVERAGE_PERCENTAGE + as.getYAccel() * (1 - MOVING_AVERAGE_PERCENTAGE);
	}

	/*public double getAngleChange(double change) {
		//avgChange = avgChange * MOVING_AVERAGE_PERCENTAGE + change * (1 - MOVING_AVERAGE_PERCENTAGE);
		//avgChange = (avgChange + change) / 2;
		
		System.out.println((int)((avgChange + change) / 2)+" "+change+" "+(avgChange+change));
		return avgChange;
	}*/

	public void setMotors(double mrls) {
		setMotors(mrls + 5*Math.sin(System.currentTimeMillis()/100),
				mrls + 5*Math.sin(System.currentTimeMillis()/100 + Math.PI));
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
