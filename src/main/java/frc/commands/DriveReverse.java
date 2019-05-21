package frc.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.subsystems.DriveTrain;



public class DriveReverse extends InstantCommand {

    private DriveTrain driveTrain;

    public DriveReverse() {
        super();
        
        driveTrain = Robot.driveTrain;
        requires(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.toggleReverse();
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
