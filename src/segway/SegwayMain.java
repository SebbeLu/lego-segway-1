package segway;

public class SegwayMain {

	public static void main(String[] args) {
		Segway segway = new Segway();
		segway.run();
//		SegowayPilot sp = new SegowayPilot(new NXTMotor(MotorPort.C), new NXTMotor(MotorPort.B), new GyroSensor(SensorPort.S4), 5.6, 15);
//		Utils.sleep(100000);
	}

}
