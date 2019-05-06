package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.subsystems.*;

/**
 * Intake a cargo ball
 */

public class IntakeCargoCommand extends Command {

    private CargoRoller cargoIntake;

    public IntakeCargoCommand() {
        cargoIntake = Robot.cargoRoller;

        requires(cargoIntake);
    }

    @Override
    public void execute() {
        cargoIntake.intake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        cargoIntake.neutral();
    }

    @Override
    public void interrupted() {
        end();
    }
}
