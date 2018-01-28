/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;
import java.awt.Point;

import ca.mcgill.ecse211.lab2.Lab2;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;


public class OdometryCorrection implements Runnable {
	private OdometerData odoData;
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	//	private EV3ColorSensor lightSensor = Lab2.lightSensor;
	private EV3ColorSensor lightSensor;

	private SampleProvider csColor;
	private double xprev = 0; private double yprev = 0; 
	private double xnow,ynow,tnow; 
	private double squareLength= 30.48;
	// stores the length of the vehicle to the actual sensor
	private double wheelToSensorLength = 3.5;

	//stores the direction the vehicle is traveling (north,south,east,west,rotating)
	private String direction;

	//stores what the last line was in its current interval (0,1,2)
	private int lastLine = 0;

	//stores the coordinate of the starting point (relative to the overall board)
	//sets to zero initially but updates as sensor reads line to determine location
	private double startingPointX = 0.0;
	private double startingPointY = 0.0;

	private double currentConstantX = 0.0;
	private double currentConstantY = 0.0;
	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(Odometer odometer, EV3ColorSensor lightSensor) throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();
		this.lightSensor = lightSensor;
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
		odoData.setXYT(0, 0, 0);
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		double [] temp = odoData.getXYT();

		while (true) {
			correctionStart = System.currentTimeMillis();

			// get the current values from our odometer
			xnow = temp[0];
			ynow = temp[1];
			tnow = temp[2];

			//determine direction the vehicle is moving
			determineVehicleDirection();
			//correct values based on odometer readings
			correctOnOdometerReadings();
			//correct values based on sensor readings 
			if(lightSensor.getColorID() == 13){
				Sound.beep();
				correctOnSensorReadings();
			}

			//update the prev value to current values
			xprev=xnow;
			yprev=ynow;

			// TODO Trigger correction (When do I have information to correct?)

			// TODO Calculate new (accurate) robot position

			// TODO Update odometer with new calculated (and more accurate) vales

			odometer.setXYT(0.3, 19.23, 5.0);

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

	private void determineVehicleDirection() {
		if ( (Math.abs(xnow-xprev) < 0.1) && (Math.abs(ynow-yprev) < 0.1 )) {
			direction = "rotating";
		} else {
			direction = "undefined";
			if ( tnow > -10 && tnow < 10 ) {
				direction = "north";
			} 
			if ( tnow > 80 && tnow < 110 ) {
				direction = "east";
			} 
			if ( tnow > 170 && tnow < 190 ) {
				direction = "south";
			} 
			if ( tnow > 260 && tnow < 280 ) {		
				direction = "west";
			} 
		}
	}

	/**
	 * A method to correct theta values when going straight and XY values when turning
	 */
	private void correctOnOdometerReadings() {
		if ( direction.equals("rotating") ) {
			//robot is rotating, values of x and y should stay the same
			xnow=xprev;
			ynow=yprev;
			odometer.setX(xnow);
			odometer.setY(ynow);
		} else {
			// correct it to one of the 4 possible angles for when robot is not rotating 
			if ( direction.equals("north") ) {
				tnow=0;
				odometer.setTheta(0);
			} 
			if ( direction.equals("east") ) {
				tnow=90;
				odometer.setTheta(Math.PI/2);
			} 
			if ( direction.equals("south") ) {
				tnow=180;
				odometer.setTheta(Math.PI);
			}
			if ( direction.equals("west") ) {		
				tnow=270;
				odometer.setTheta(3*Math.PI/2);
			}
		}
	}

	/**
	 * A method to correct our XY values based on sensor readings
	 */
	private void correctOnSensorReadings() {
		//call proper sensor correction method depending on the direction the vehicle is moving
		if (tnow > -10 && tnow < 10) {
			sensorCorrectionNorth();
		} 
		if (tnow > 80 && tnow < 110 ) {
			sensorCorrectionEast();
		}
		if ( tnow > 170 && tnow < 190 ) {
			sensorCorrectionSouth(); 
		} 
		if ( tnow > 260 && tnow < 280 ) {		
			sensorCorrectionWest();
		} 
		odometer.setX(xnow);
		odometer.setY(ynow);
	}

	/**
	 * A method sensor corrections while vehicle is moving north
	 */
	private void sensorCorrectionNorth() {
		if (lastLine == 0) {
			startingPointY = squareLength-ynow-wheelToSensorLength;
		} else if (lastLine == 1) {
			ynow = 2*squareLength - startingPointY - wheelToSensorLength; 
		} else if (lastLine == 2) {
			ynow = 3*squareLength - startingPointY - wheelToSensorLength;
		}
		//update value of last line
		if(lastLine < 2)
			lastLine++;
		else
			lastLine = 0;
	}	

	/**
	 * A method sensor corrections while vehicle is moving east
	 */
	private void sensorCorrectionEast() {
		if (lastLine == 0) {
			currentConstantY = ynow;
			startingPointX = squareLength-xnow-wheelToSensorLength;
		} else if (lastLine == 1) {
			xnow = 2*squareLength - startingPointX - wheelToSensorLength; 
		} else if (lastLine == 2) {
			xnow = 3*squareLength - startingPointX - wheelToSensorLength; 
		}
		//update value of last line
		if(lastLine < 2)
			lastLine++;
		else
			lastLine = 0;
	}


	/**
	 * A method sensor corrections while vehicle is moving south
	 */
	private void sensorCorrectionSouth() {
		if (lastLine == 0) {
			currentConstantX = xnow;
			ynow = 3*squareLength - startingPointY- wheelToSensorLength;
		} else if (lastLine == 1) {
			ynow = 2*squareLength - startingPointY - wheelToSensorLength; 
		} else if (lastLine == 2) {
			ynow = squareLength - startingPointY - wheelToSensorLength;
		}
		//update value of last line
		if(lastLine < 2)
			lastLine++;
		else
			lastLine = 0;
	}


	/**
	 * A method sensor corrections while vehicle is moving west
	 */
	private void sensorCorrectionWest() {
		if (lastLine == 0) {
			currentConstantY = ynow;
			xnow = 3*squareLength - startingPointX - wheelToSensorLength; 
		} else if (lastLine == 1) {
			xnow = 2*squareLength - startingPointX - wheelToSensorLength; 
		} else if (lastLine == 2) {
			xnow = squareLength - startingPointX - wheelToSensorLength;
		}
		//update value of last line

		if(lastLine < 2)
			lastLine++;
		else
			lastLine = 0;
	}
}
