package frc.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;


public class QuickTurnCommand extends InstantCommand {

    private Drivetrain drivetrain;

    public QuickTurnCommand() {
        super();
        
        drivetrain = Robot.drivetrain;
        requires(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.quickTurnToggle();
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
