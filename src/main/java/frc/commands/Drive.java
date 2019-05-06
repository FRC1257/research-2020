package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.OI;
import frc.subsystems.Drivetrain;

/**
 * Set the speed of the drivetrain with the values from the controller
 *
 * Default command of the Drivetrain subsystem
 */

public class DriveCommand extends Command {

    private Drivetrain drivetrain;
    private OI oi;

    public DriveCommand() {
        drivetrain = Robot.drivetrain;
        oi = Robot.oi;

        requires(drivetrain);
    }

    @Override
    public void execute() {
        if(drivetrain.getCheeseDrive() == false){
        drivetrain.drive(oi.getDriveForwardSpeed(), oi.getDriveTurnSpeed());
        }
        if(drivetrain.getCheeseDrive == true){
        drivetrain.drive(oi.getDriveForwardSpeed(), oi.getDriveCurvature());
        }
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

