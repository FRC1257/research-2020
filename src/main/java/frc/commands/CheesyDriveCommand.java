package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class CheesyDriveCommand extends InstantCommand {

    private Drivetrain drivetrain;

    public CheesyDriveCommand() {
        super();
        
        drivetrain = Robot.drivetrain;
        requires(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.cheesyToggle();
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
