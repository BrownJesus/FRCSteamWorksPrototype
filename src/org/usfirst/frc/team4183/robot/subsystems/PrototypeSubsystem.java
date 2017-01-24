package org.usfirst.frc.team4183.robot.subsystems;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.commands.MotorRunCommand;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *
 */
public class PrototypeSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
private final int ENCODER_CODES_PER_ENCODER_REV = 256;	//Codes encoder is set to
	
	public final double P_VALUE = 0.6;
	public final double I_VALUE = 1.2*0.001;
	public final double D_VALUE = .02*1000.0;
	public final double F_VALUE = 0.13;
	
	public final double MAX_SPEED = 6000;
	
	public TalonLogger logger;
	public CANTalon conveyer = new CANTalon(RobotMap.CONVEYER_MOTOR_ID);
	public CANTalon[] motors = new CANTalon[1];
	
	private int loopcnt = 0;
	
	public double motorVelocity = 0;
	
	public PrototypeSubsystem() {
		motors[0] = new CANTalon(RobotMap.SHOOTER_MOTOR1_ID);
		/** This is where you would add more motors if its required*/
		initSpeedMode();
		logger = new TalonLogger(motors[0]);
	}
	
	public double getCurrent() {
		double current = motors[0].getOutputCurrent();
		SmartDashboard.putNumber("Test Motor Current", current);
		return current;
	}
	
	public void runConveyer(double speed) {
		conveyer.set(speed);
	}
	
	public void startLogger() {
		logger.start();
	}
		
	public void setSpeed(double speed) {
		if(speed > MAX_SPEED)
			speed = MAX_SPEED;
		else if(speed < -MAX_SPEED)
			speed = -MAX_SPEED;
			
		CANTalon m = motors[0];
		m.set(speed);

		if (++loopcnt >= 50) {
			loopcnt = 0;
			System.out.println(
					"Trg:" + speed + " FB:" + m.get() + " Drv:" + m.getOutputVoltage() + " Err:" + m.getError());
			SmartDashboard.putNumber("RPM", m.get());
		}
	}
	
	public void initSpeedMode() {
		System.out.println("Init Speed Mode");
		
		CANTalon m = motors[0];
		m.changeControlMode(CANTalon.TalonControlMode.Speed);
		
		for(CANTalon talon : motors) {
			talon.setInverted(false);
			talon.setVoltageRampRate(0.0);
		}
		
		m.reverseOutput(false);
		
		m.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		m.configEncoderCodesPerRev(ENCODER_CODES_PER_ENCODER_REV);
		
		m.reverseSensor(false);
		
		for(int i = 1; i < motors.length; i++) {
			motors[i].changeControlMode(CANTalon.TalonControlMode.Follower);
			motors[i].set(m.getDeviceID());
		}
		/** Reverse motors here as needed: */
		//motors[1].reverseOutput(true);
		
		m.setPID(P_VALUE, I_VALUE, D_VALUE);
		m.setF(F_VALUE);
		m.setIZone(0);
		m.setCloseLoopRampRate(0.0);  // Works better disabled
		m.setAllowableClosedLoopErr(0);
		m.configNominalOutputVoltage(0.0, 0.0);
		m.configPeakOutputVoltage(+12.0, -12.0);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new MotorRunCommand());
    }
}

