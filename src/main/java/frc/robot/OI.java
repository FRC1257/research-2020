package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.commands.*;
import frc.util.SnailController;

public class OI {

    /**
     * Contains all of the mappings for controls for our robot
     */
    
    private static OI instance = null;

    public SnailController driveController;
    public SnailController operatorController;

    OI() {
        driveController = new SnailController(RobotMap.DRIVE_CONTROLLER_PORT);
        operatorController = new SnailController(RobotMap.OPERATOR_CONTROLLER_PORT);

        driveController.aButton.whenPressed(new Test());
	driverController.bButton.whenPressed(Nothing.end());
	driverController.xButton.whenPressed(new DriveReverse());



    }

    // Drivetrain
    public double getDriveForwardSpeed() {
        return applyDeadband(driveController.getForwardSpeed());
    }

    public double getDriveTurnSpeed() {
        return applyDeadband(driveController.getTurnSpeed());
    }




    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }
    

    public double applyDeadband(double in) {
        if(Math.abs(in) < RobotMap.NEO_DEADBAND) {
            return 0;
        }
        return in;
    }

	public double getClimbDriveSpeed() {
		return operatorController.getRightStickY(Hand.kLeft);
	}
}
