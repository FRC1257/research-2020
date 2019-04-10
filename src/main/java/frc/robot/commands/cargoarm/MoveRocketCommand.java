package frc.robot.commands.cargoarm;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.robot.subsystems.CargoArm;;

public class MoveRocketCommand extends InstantCommand {

    private CargoArm cargoArm;

    public MoveRocketCommand() {
        super();

        cargoArm = Robot.cargoArm;
        requires(cargoArm);
    }

    @Override
    public void initialize() {
        cargoArm.moveRocket();
    }

    @Override
    public void end() {

    }

    @Override
    public void interrupted() {
        end();
    }
}
