package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Gyro;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import java.io.File;
import java.io.IOException;

public class Robot extends TimedRobot {
	
	public static Drivetrain drivetrain;
	
	private Gyro gyro;

    private double lastTimeStamp;

    private EncoderFollower left;
	private EncoderFollower right;

    private Command autonomousCommand;

	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();

		gyro = Gyro.getInstance();
	}

	@Override
	public void autonomousInit() {
		Waypoint[] points = new Waypoint[] {
				new Waypoint(-4, -1, Pathfinder.d2r(-45)),
				new Waypoint(-2, -2, 0),
				new Waypoint(0, 0, 0)
		};
//		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
//				Trajectory.Config.SAMPLES_HIGH, 0.02, 1.7, 2.0, 60.0);
//		Trajectory trajectory = Pathfinder.generate(points, config);
		try {
			File traj = new File(Filesystem.getDeployDirectory().getPath(), "RightMiddleHatch.pf1.csv");
			Trajectory trajectory = Pathfinder.readFromCSV(traj);
			TankModifier modifier = new TankModifier(trajectory).modify(0.5);
			left = new EncoderFollower(modifier.getLeftTrajectory());
			right = new EncoderFollower(modifier.getRightTrajectory());
			left.configureEncoder(drivetrain.getLeftEncoderTicks(1000), 1000, RobotMap.DRIVE_WHEEL_DIAMETER);
			left.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);
			right.configureEncoder(drivetrain.getRightEncoderTicks(1000), 1000, RobotMap.DRIVE_WHEEL_DIAMETER);
			right.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);
		}
		catch(IOException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		double outputL = left.calculate(drivetrain.getLeftEncoderTicks(1000));
		double outputR = right.calculate(drivetrain.getRightEncoderTicks(1000));
		double desiredHeading = Pathfinder.r2d(left.getHeading());
		double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyro.getRobotAngle());
		angleDifference = angleDifference % 360.0;
		if(Math.abs(angleDifference) > 180.0) {
			angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
		}
		double turn = 0.8 * (-1.0/80.0) * angleDifference;
		drivetrain.tankDrive(outputL + turn, outputR - turn);
		updateSubsystems();
	}
	
	@Override
	public void teleopInit() {
		autonomousCommand.cancel();
	}
	
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSubsystems();
	}

	private void updateSubsystems() {
		drivetrain.update(Timer.getFPGATimestamp() - lastTimeStamp);

		lastTimeStamp = Timer.getFPGATimestamp();
	}

	private void getConstantTuning() {
		drivetrain.getConstantTuning();
	}
}
