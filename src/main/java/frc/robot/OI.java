package frc.robot;

import frc.robot.commands.drivetrain.*;
import frc.robot.util.SnailController;
import static frc.robot.util.SnailController.*;

public class OI {

    /**
     * This class is the glue that binds the controls on the physical operator
     * interface to the commands and command groups that allow control of the robot.
     */

    private static OI instance = null;

    private SnailController driveController;
    private SnailController operatorController;

    private OI() {
        driveController = new SnailController(RobotMap.CONTROLLER_DRIVE_PORT);
        operatorController = new SnailController(RobotMap.CONTROLLER_OPERATOR_PORT);

        // Drive
        
    }

    // Drive
    public double getDriveForwardSpeed() {
        return driveController.getForwardSpeed();
    }

    public double getDriveTurnSpeed() {
        return driveController.getTurnSpeed();
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }
}
