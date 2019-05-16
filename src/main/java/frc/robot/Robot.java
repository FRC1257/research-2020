package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.util.Gyro;

public class Robot extends TimedRobot {
	
	public static Drivetrain drivetrain;
   	public static Climb climb;
    	public static Gyro gyro;
	public static HatchIntake hatchIntake;
	public static OI oi;
	private double lastTimeStamp;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();
      		climb = new Climb();
        	hatchIntake = new HatchIntake();
		oi = OI.getInstance();
		gyro = Gyro.getInstance();
		
		lastTimeStamp = Timer.getFPGATimestamp();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called once each time when the robot enters test mode.
	 */
	@Override
	public void testInit() {

	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		climb.update();
		drivetrain.update(Timer.getFPGATimeStamp - lastTimeStamp);
		hatchIntake.update();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
}
