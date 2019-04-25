package main.java.frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/**
 toggle slowturn option
 */

public class SlowTurnCommand extends InstantCommand {

    private Drivetrain drivetrain;

    public SlowTurnCommand() {
        super();
        
        drivetrain = Robot.drivetrain;
        requires(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.slowTurntoggle();
    }

    @Override
    public void end() {

    }

    @Override
    public void interrupted() {
        end();
    }
}
