package frc.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

/**
 toggle slowturn option
 */

public class SlowTurnCommand extends InstantCommand {

    private DriveTrain driveTrain;

    public SlowTurnCommand() {
        super();
        
        driveTrain = Robot.driveTrain;
        requires(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.slowTurntoggle();
    }

    @Override
    public void end() {

    }

    @Override
    public void interrupted() {
        end();
    }
}
