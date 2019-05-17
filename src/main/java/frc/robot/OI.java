package frc.robot;

import frc.util.SnailController;

public class OI {

    /**
     * Contains all of the mappings for controls for our robot
     */
    
    private static OI instance = null;

    public SnailController driveController;
    public SnailController operatorController;

    private OI() {
        driveController = new SnailController(RobotMap.DRIVE_CONTROLLER_PORT);
        operatorController = new SnailController(RobotMap.OPERATOR_CONTROLLER_PORT);
        
        driveController.abutton.whenPressed(new CheesyDriveCommand());
        driveController.ybutton.whenPressed(new QuickTurnCommand());
        driveController.ybutton.whenPressed(new DriveReverse());
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
}
