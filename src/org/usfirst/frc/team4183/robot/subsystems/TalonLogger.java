package org.usfirst.frc.team4183.robot.subsystems;

import java.io.PrintWriter;

import com.ctre.CANTalon;

public class TalonLogger extends LoggerBase{
	private static final long DEFAULT_INTERVAL = 10;     // Logging interval msec
	private static final double DEFAULT_DURATION = 10.0; // Seconds

	private CANTalon motor;
	
	// interval: msec
	// duration: seconds
	public TalonLogger( CANTalon _motor, String fileName, long interval, double duration) {
		super( fileName, interval, duration);
		motor = _motor;
	}
	
	// Use default filename, interval, duration
	TalonLogger( CANTalon _motor) {
		this( _motor, "talon.txt", DEFAULT_INTERVAL, DEFAULT_DURATION);
	}


	@Override
	protected void writeLine( PrintWriter writer, long millis) {

		double sp = motor.getSetpoint();       // Setpoint (input)
		double fb = motor.get();               // Feedback value
		double err = motor.getError();         // Error value (native units)
		double ov = motor.getOutputVoltage();  // Drive voltage
		
		writer.format("%6d %9.1f %9.1f %9.1f %9.1f\n", millis, sp, fb, err, ov);
	}

}
