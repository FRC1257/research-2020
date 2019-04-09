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

    // Controllers
    public static final int CONTROLLER_DRIVE_PORT = 0;
    public static final int CONTROLLER_OPERATOR_PORT = 1; 
    public static final double CONTROLLER_DEADBAND = 0.08; // Deadband for the joysticks of the controllers

    // Drive
    public static final int DRIVE_FRONT_LEFT_ID = 4;
    public static final int DRIVE_FRONT_RIGHT_ID = 3;
    public static final int DRIVE_BACK_LEFT_ID = 2;
    public static final int DRIVE_BACK_RIGHT_ID = 1;
    public static final double DRIVE_FORWARD_MAX_SPEED = 1.0;
    public static final double DRIVE_TURN_MAX_SPEED = 0.8;

    public static final double DRIVE_WHEEL_DIAMETER = 6.0; // inches
    public static final double DRIVE_CONV_FACTOR = Math.PI * DRIVE_WHEEL_DIAMETER; // rev to inches
    public static final double DRIVE_CONV_FACTOR_TALON = 1.0 / 4096.0 * Math.PI * DRIVE_WHEEL_DIAMETER; // pulses to inches
}
