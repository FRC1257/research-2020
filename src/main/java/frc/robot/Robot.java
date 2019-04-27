package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.commands.paths.RightMiddleHatch;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Gyro;

public class Robot extends TimedRobot {
	
	public static Drivetrain drivetrain;
	
	private Gyro gyro;

    private double lastTimeStamp;

    private Command autonomousCommand;

	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();

		gyro = Gyro.getInstance();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = new RightMiddleHatch();
		autonomousCommand.start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
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
