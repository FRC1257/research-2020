package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.CargoArm;

public class Robot extends TimedRobot {

	public static CargoArm cargoArm;

	public static OI oi;

    private double lastTimeStamp;

	@Override
	public void robotInit() {
		cargoArm = new CargoArm();

		oi = OI.getInstance();

		lastTimeStamp = Timer.getFPGATimestamp();
	}
	
	@Override
	public void teleopInit() {
		
	}
	
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSubsystems();
	}

	private void updateSubsystems() {
		cargoArm.update();

		lastTimeStamp = Timer.getFPGATimestamp();
	}

	private void getConstantTuning() {
		cargoArm.getConstantTuning();
	}
}
