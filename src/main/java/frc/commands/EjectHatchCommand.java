package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.subsystems.HatchIntake;

/**
 * Eject a hatch panel
 */

public class EjectHatchCommand extends Command {

    private HatchIntake hatchIntake;

    public EjectHatchCommand() {
        hatchIntake = Robot.hatchIntake;

        requires(hatchIntake);
    }

    @Override
    public void execute() {
        hatchIntake.eject();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        hatchIntake.neutral();
    }

    @Override
    public void interrupted() {
        end();
    }
}
