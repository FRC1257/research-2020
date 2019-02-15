package frc.util;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * <h1>Gyro</h1>
 * The Gyro gets us our Yaw angle, in degrees. Our "0" point is the Yaw at the start of the match.
 * @author Allen Du
 * @since 2019-01-23
 */
public class Gyro {
    public AHRS navx;
    private static Gyro instance = null;
    private String angle = "Yaw angle: ";

    /**
     * Constructs a Gyro object.
     */
    private Gyro() {
        navx = new AHRS(Port.kMXP);
    }

    /**
     * Singleton.
     * @return A Gyro object.
     */
    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro();
        }
        return instance;
    }

    /**
     * Sets the current Yaw angle to "0".
     */
    public void zeroAngle() {
        navx.zeroYaw();
    }

    /**
     * Gets the current Yaw angle.
     * @return The angle in degrees.
     */
    public double getAngle() {
        return navx.getYaw();
    }

    /**
     * Resets the Gyro.
     */
    public void resetAngle() {
        navx.reset();
    }

    /**
     * Displays the Yaw angle on {@code SmartDashboard}.
     */
    public void displayAngle() {
        SmartDashboard.putNumber(angle, getAngle());
    }
}