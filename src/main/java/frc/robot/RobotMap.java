package frc.robot;

public class RobotMap {

    /**
     * Contains constants for use without the entire robot
     * All constants should be ordered by subsystem/use
     * The units of each measurement should be specified in a comment
     */

    // Controllers
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int PCM_SECONDARY_ID = 1;
    
    // Drivetrain
    public static final int DRIVE_FRONT_LEFT_ID = 4;
    public static final int DRIVE_FRONT_RIGHT_ID = 3;
    public static final int DRIVE_BACK_LEFT_ID = 2;
    public static final int DRIVE_BACK_RIGHT_ID = 1;
    public static final double DRIVE_SLOWTURN_FACTOR = 0.5;

    // Hatch Intake
    public static final int HATCH_INTAKE_MOTOR_ID = 5;

    // Cargo Arm
    public static final int CARGO_ARM_MOTOR_ID = 6;
    public static final int CARGO_ARM_LIMIT_SWITCH_ID = 1;

    // Cargo Roller
    public static final int CARGO_ROLLER_MOTOR_ID = 7;

    // Climb
    public static final int CLIMB_FRONT_MOTOR_ID = 9;
    public static final int CLIMB_BACK_MOTOR_ID = 8;
    public static final int CLIMB_FRONT_SOLENOID_FORWARD_ID = 0;
    public static final int CLIMB_FRONT_SOLENOID_REVERSE_ID = 1;
    public static final int CLIMB_BACK_SOLENOID_FORWARD_ID = 2;
    public static final int CLIMB_BACK_SOLENOID_REVERSE_ID = 3;

    // Drivetrain PID
    public static double[] DRIVE_PIDF_LEFT = {1.0, 0.0, 0.0, 0.0};
    public static double[] DRIVE_PIDF_RIGHT = {1.0, 0.0, 0.0, 0.0}; 
    public static double DRIVE_P_WASHOUT = 1.0; // 1.0 for no washout (Testing) 

    // Drivetrain NEO
    public static final int NEO_STALL_LIMIT = 60;
    public static final int NEO_RUN_CURRENT = 1;
    public static final int NEO_MIN_RPM = 10;
    public static final int NEO_MAX_RPM = 5500;
    public static final double NEO_DEADBAND = 0.08;

    // Drivetrain wheel
    public static final double DRIVE_WHEEL_DIAMETER = 6.0; // inches
    public static final double PI = 3.1415926535897932;

    // For ConsolePrinter
    public static final boolean PRINT_SD_DEBUG_DATA = true;


    /// Controller
	public static final double CONTROLLER_DEADBAND = 0;
	public static final int CONTROLLER_POV = 0;
	public static final int NEO_CURRENT_LIMIT = 0;
}
