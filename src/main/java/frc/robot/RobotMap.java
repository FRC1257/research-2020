package frc.robot;

public class RobotMap {

    /**
     * The RobotMap is a mapping from the ports sensors and actuators are wired into
     * to a variable name. This provides flexibility changing wiring, makes checking
     * the wiring easier and significantly reduces the number of magic numbers
     * floating around. All constants should be ordered by subsystem/use. 
     * The units of each measurement should be specified in a comment
     */

    // General
    public static final int NEO_CURRENT_LIMIT = 50; // amps
    public static final int PID_SLOT = 0;
    public static final int SM_SLOT = 1;

    // Controllers
    public static final int CONTROLLER_DRIVE_PORT = 0;
    public static final int CONTROLLER_OPERATOR_PORT = 1; 
    public static final double CONTROLLER_DEADBAND = 0.08; // Deadband for the joysticks of the controllers

    // Cargo Arm
    public static final int CARGO_ARM_MOTOR_ID = 6;
    public static final int CARGO_ARM_LIMIT_SWITCH_ID = 1;

    // Cargo Arm: 0 is at bottom, positive means higher
    public static final double CARGO_ARM_PID_ROCKET = 11.0; // Target position for rocket (revolutions)
    public static final double CARGO_ARM_PID_CARGO = 18.5; // Target position for cargo ship (revolutions)
    public static final double CARGO_ARM_PID_RAISED = 28.0; // Initial position of arm (revolutions)
    public static double CARGO_ARM_MAX_SPEED = 1.0;

    public static final double[] CARGO_ARM_PIDF = { 0.1, 0.0, 0.0, 0.0 };
    public static final double CARGO_ARM_ARB_F = 0.0;
    public static final double CARGO_ARM_ANGLE_CONV_FACTOR = 90.0 / CARGO_ARM_PID_RAISED; // conversion factor from motor rev to angle
    public static final double[] CARGO_ARM_PIDF_SM = { 0.1, 0.0, 0.0, 0.0 };
    public static final double CARGO_ARM_MAX_VEL = 2000;
    public static final double CARGO_ARM_MIN_VEL = 0;
    public static final double CARGO_ARM_MAX_ACC = 1500;
    public static final double CARGO_ARM_PID_TOLERANCE = 1.0; // revolutions
    public static final double CARGO_ARM_PID_WAIT = 2.0; // seconds
    public static final double CARGO_ARM_PID_MAX_OUTPUT = 1.0; // percentage
    public static final double CARGO_ARM_PID_MIN_OUTPUT = -1.0; // percentage
}
