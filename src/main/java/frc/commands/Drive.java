package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.subsystems.Drivetrain;
import frc.robot.OI;

/**
 * Set the speed of the drivetrain with the values from the controller
 *
 * Default command of the Drivetrain subsystem
 */

public class Drive extends Command {

    private Drivetrain drivetrain;
    private double turnSpeed;
    private double driveSpeed;
    private OI oi;

    public Drive(double d, double z) {
        drivetrain = Robot.drivetrain;
        oi = Robot.oi;
        if(z!=0||d!=0){
        turnSpeed = z;
        driveSpeed = d;
        }
        else{
        turnSpeed = oi.getDriveTurnSpeed();
        driveSpeed = oi.getDriveForwardSpeed();
        }

        requires(drivetrain);
    }

    @Override
    public void execute() {
      
      drivetrain.drive(driveSpeed , turnSpeed);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
