package frc.util;

import edu.wpi.first.wpilibj.XboxController;

// Xbox controller optimized for our drive team.

public class SnailController extends XboxController {
	double [] Deadbanded;

	public SnailController(int port) {
		super(port);
	}

	@Override
	public double getY(Hand hand) {
		return hand == Hand.kLeft ? -super.getY(hand) : super.getY(hand);
	}

	/*
	 * Controls: If they press A, use single stick arcade with the left joystick
	 * 
	 * If they press the left bumper, use the left joystick for forward and backward
	 * motion and the right joystick for turning
	 * 
	 * If they press the right bumper, use the right joystick for forward and
	 * backward motion and the left joystick for turning
	 */
	
	public double getForwardSpeed() {
		if (getAButton())
			return getY(Hand.kLeft);
		else if (getBumper(Hand.kLeft))
			return getY(Hand.kLeft);
		else if (getBumper(Hand.kRight))
			return getY(Hand.kRight);
		else
			return 0;
	}

	public double getTurnSpeed() {
		if (getAButton())
			return getX(Hand.kLeft);
		else if (getBumper(Hand.kLeft))
			return getX(Hand.kRight);
		else if (getBumper(Hand.kRight))
			return getX(Hand.kLeft);
		else
			return 0;
    }
    @Override
    public double[] applyDeadBand(double x, double y){ // scaled radial 
        z = Math.pow(Math.pow(x, 2)+Math.pow(y, 2),0.5); //magnitude of vector
        if( z < RobotMap.Neo_DeadBand){  
            double[] Deadbanded = new double[]{0,0};
            return Deadbanded;
        }
        else{
            double [] Deadbanded = new double[]{x*(z-Robot._Neo_DeadBand)/z,y*(z-Robot._Neo_DeadBand)/z}; 
            return Deadbanded;
        }

    }
}
