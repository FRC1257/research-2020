package frc.robot.commands.paths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;

public class RightMiddleHatch extends Command {

    private Drivetrain drivetrain;

    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;

    public RightMiddleHatch() {
        drivetrain = Robot.drivetrain;

        requires(drivetrain);
    }

    @Override
    public void initialize() {
        try {
            // Swap left and right due to bug with PathWeaver
            leftTrajectory = PathfinderFRC.getTrajectory("RightMiddleHatch.right");
            rightTrajectory = PathfinderFRC.getTrajectory("RightMiddleHatch.left");
        }
        catch(IOException e) {
            e.printStackTrace();
        }
        drivetrain.runPath(leftTrajectory, rightTrajectory);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return drivetrain.isPathFinished();
    }

    @Override
    public void end() {
        drivetrain.stopPath();
    }

    @Override
    public void interrupted() {
        end();
    }
}
