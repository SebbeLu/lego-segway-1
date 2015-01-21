package segway;

import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;
import lejos.nxt.addon.GyroSensor;

public class Segway {

	public static void main(String[] args) {
		AccelHTSensor as = new AccelHTSensor(SensorPort.S1);
		GyroSensor gs = new GyroSensor(SensorPort.S4);
		NXTRegulatedMotor mr = new NXTRegulatedMotor(MotorPort.B);
		NXTRegulatedMotor ml = new NXTRegulatedMotor(MotorPort.C);
		
		Utils.sleep(500);
		
		float sum = 0;
		for (int i = 0; i < 10; i++) {
			sum += gs.getAngularVelocity();
		}
		float gyroOffset = sum / 10;
		
		while(true) {
			if (as.getXAccel() > 180) break;
			Utils.sleep(50);
		}
		
		while (true) {
			int x = as.getXAccel(); // + down
			int y = as.getYAccel(); // + front
			int z = as.getZAccel(); // + robot left
			
			float vel = gs.getAngularVelocity(); // + back
			
			LCD.clear();
			LCD.drawString(Integer.toString(x), 1, 1);
			LCD.drawString(Integer.toString(y), 1, 2);
			LCD.drawString(Integer.toString(z), 1, 3);
			LCD.drawString(Float.toString(vel), 1, 4);
			
			Utils.sleep(50);
		}
	}
}
