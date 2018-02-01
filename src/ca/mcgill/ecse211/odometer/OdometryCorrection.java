/*
 * OdometryCorrection.java
 */

/* 
 * Name: Yaniv Bronshtein 260618099	Varad Kothari 260631831
 * 
 * */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private static EV3ColorSensor lightSensor;
	private static SampleProvider sensorMode;

	private double TILEDIMENSION = 30.48; //constant measured dimension of tile square in cm
	private double[] odoData; //Store XYT as outlined in OdometerData.java

//	private double OFFSET = 5.2; //distance from wheel to sensor
	private double OFFSET = 4.7; //distance from wheel to sensor
	float[] sensorData; //data from light sensor
	

	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(Odometer odometer,EV3ColorSensor lightSensor) throws OdometerExceptions {

		this.odometer = Odometer.getOdometer(); 
		OdometryCorrection.lightSensor = lightSensor;
		OdometryCorrection.sensorMode = lightSensor.getRedMode();
		this.sensorData = new float[lightSensor.sampleSize()];  
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		int blackLineCountX, blackLineCountY; 
		double x,y,theta;
		

		
		while (true) {
			correctionStart = System.currentTimeMillis();
			

			// read value from light sensor and determine if it is a black line or not
			sensorMode.fetchSample(sensorData, 0); //Store elements of sample in arr.offset 0

			blackLineCountX = 0; //re-initialize to 0 
			blackLineCountY = 0; //with each pass through while loop

			


			odoData = odometer.getXYT();
			
			x = odoData[0]; //before I was getting null pointers because odoData is empty before while loop
			y = odoData[1];
			theta = odoData[2];

			//Hit black line if sensor reading 0.20 +/- 2
			if (sensorData[0] < 0.22) {
				Sound.beep();
				//Going North
				/* Got rid of deadzones */
//				if ((theta > 0 && theta <= 5) || theta >355) {
				if ((theta > 0 && theta <= 45) || theta >315) {
					odometer.setY(blackLineCountY * TILEDIMENSION - OFFSET); 
					blackLineCountY++;
				}
				//Going east
//				if(theta <= 95 && theta > 85 ) {
				if(theta <= 135 && theta > 45 ) {
					odometer.setX(blackLineCountX * TILEDIMENSION - OFFSET);
					blackLineCountX++;
				}
				//Going south
//				if (theta <= 185 && theta > 175) {
				if (theta <= 225 && theta > 135) {
					odometer.setY(blackLineCountY * TILEDIMENSION + OFFSET);
					blackLineCountY--;
				}
				//Going west
//				if (theta <=275 && theta >265) {
				if (theta <=315 && theta >225) {
					odometer.setX(blackLineCountX*TILEDIMENSION + OFFSET);
					blackLineCountX--;
				}


			}

			//odometer.setXYT(0.3, 19.23, 5.0); //We did not use this because we prefered to set them individually

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}