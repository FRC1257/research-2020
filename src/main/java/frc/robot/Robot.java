package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.subsystems.*;
import frc.robot.OI;
import frc.commands.*;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
	
	public static Drivetrain drivetrain;
    public static Climb climb;
    public static CargoArm cargoArm;
    public static CargoRoller cargoRoller;
    public static HatchIntake hatchIntake;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();
        climb = new Climb();
        cargoArm = new CargoArm();
        cargoRoller = new CargoRoller();
        hatchIntake = new HatchIntake();
		oi = OI.getInstance();
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
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
}
