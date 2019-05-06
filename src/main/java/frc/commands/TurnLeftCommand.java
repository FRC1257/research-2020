package frc.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.subsystems.Drivetrain;

/**
 * Begin turning left with the drivetrain
 * Once this begins, the driver WILL be able to cancel it through controller input
 */

public class TurnLeftCommand extends InstantCommand {

    private Drivetrain drivetrain;

    public TurnLeftCommand() {
        super();
        
        drivetrain = Robot.drivetrain;
        requires(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.turnLeft();
    }

    @Override
    public void end() {

    }

    @Override
    public void interrupted() {
        end();
    }
}
