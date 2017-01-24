package org.usfirst.frc.team4183.robot.commands;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class MotorRunCommand extends Command {

    public MotorRunCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.prototypeSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println(getName() + " initializing");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double shooterSpeed = SmartDashboard.getNumber("Motor Velocity");
    	double conveyerSpeed = SmartDashboard.getNumber("Conveyer Speed");
    	Robot.prototypeSubsystem.setSpeed(shooterSpeed);
    	Robot.prototypeSubsystem.runConveyer(conveyerSpeed);
    	Robot.prototypeSubsystem.getCurrent();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
