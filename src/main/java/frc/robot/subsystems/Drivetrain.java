package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.*;
import frc.robot.util.*;

public class Drivetrain extends Subsystem {

    private CANSparkMax flDrive;
    private CANSparkMax frDrive;
    private CANSparkMax blDrive;
    private CANSparkMax brDrive;

    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private DifferentialDrive drivetrain;
    private Gyro gyro;

    private double driveSpeed;
    private double turnSpeed;
    private double leftSpeed;
    private double rightSpeed;
    private boolean reversed;

    public enum State {
        DRIVER, TANK_DRIVE
    }
    private State state = State.DRIVER;

    public Drivetrain() {
        flDrive = new CANSparkMax(RobotMap.DRIVE_FRONT_LEFT_ID, MotorType.kBrushless);
        frDrive = new CANSparkMax(RobotMap.DRIVE_FRONT_RIGHT_ID, MotorType.kBrushless);
        blDrive = new CANSparkMax(RobotMap.DRIVE_BACK_LEFT_ID, MotorType.kBrushless);
        brDrive = new CANSparkMax(RobotMap.DRIVE_BACK_RIGHT_ID, MotorType.kBrushless);

        flDrive.restoreFactoryDefaults();
        frDrive.restoreFactoryDefaults();
        blDrive.restoreFactoryDefaults();
        brDrive.restoreFactoryDefaults();

        flDrive.setIdleMode(IdleMode.kBrake);
        frDrive.setIdleMode(IdleMode.kBrake);
        blDrive.setIdleMode(IdleMode.kCoast);
        brDrive.setIdleMode(IdleMode.kCoast);

        flDrive.setSmartCurrentLimit(RobotMap.NEO_CURRENT_LIMIT);
        frDrive.setSmartCurrentLimit(RobotMap.NEO_CURRENT_LIMIT);
        blDrive.setSmartCurrentLimit(RobotMap.NEO_CURRENT_LIMIT);
        brDrive.setSmartCurrentLimit(RobotMap.NEO_CURRENT_LIMIT);

        blDrive.follow(flDrive);
        brDrive.follow(frDrive);

        leftEncoder = flDrive.getEncoder();
        rightEncoder = frDrive.getEncoder();

        leftEncoder.setPositionConversionFactor(RobotMap.DRIVE_CONV_FACTOR);
        rightEncoder.setPositionConversionFactor(RobotMap.DRIVE_CONV_FACTOR);

        drivetrain = new DifferentialDrive(flDrive, frDrive);
        gyro = Gyro.getInstance();

        driveSpeed = 0;
        turnSpeed = 0;
        leftSpeed = 0;
        rightSpeed = 0;
        reversed = false;

        setConstantTuning();
        reset();
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }

    public void reset() {
        flDrive.set(0);
        frDrive.set(0);
        blDrive.set(0);
        brDrive.set(0);

        resetSensors();

        reversed = false;
        state = State.DRIVER;
    }

    private void resetSensors() {
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);

        gyro.zeroRobotAngle();
    }

    public void update(double deltaT) {
        switch(state) {
            case DRIVER:
                drivetrain.arcadeDrive(driveSpeed, turnSpeed);
            break;
            case TANK_DRIVE:
                drivetrain.tankDrive(leftSpeed, rightSpeed);
            break;
        }

        driveSpeed = 0;
        turnSpeed = 0;
        leftSpeed = 0;
        rightSpeed = 0;

        SmartDashboard.putString("Drive State", state.name());
        SmartDashboard.putBoolean("Drive Reversed", reversed);

        SmartDashboard.putNumber("Drive FL Current", flDrive.getOutputCurrent());
        SmartDashboard.putNumber("Drive FR Current", frDrive.getOutputCurrent());
        SmartDashboard.putNumber("Drive BL Current", blDrive.getOutputCurrent());
        SmartDashboard.putNumber("Drive BR Current", brDrive.getOutputCurrent());

        SmartDashboard.putNumber("Drive FL Temperature (C)", flDrive.getMotorTemperature());
        SmartDashboard.putNumber("Drive FR Temperature (C)", frDrive.getMotorTemperature());
        SmartDashboard.putNumber("Drive BL Temperature (C)", blDrive.getMotorTemperature());
        SmartDashboard.putNumber("Drive BR Temperature (C)", brDrive.getMotorTemperature());
    }

    public void drive(double x, double z) {
        if(reversed) {
            driveSpeed = -x * RobotMap.DRIVE_FORWARD_MAX_SPEED;
            turnSpeed = z * RobotMap.DRIVE_TURN_MAX_SPEED;
        }
        else {
            driveSpeed = x * RobotMap.DRIVE_FORWARD_MAX_SPEED;
            turnSpeed = z * RobotMap.DRIVE_TURN_MAX_SPEED;
        }
        if(driveSpeed != 0.0 || turnSpeed != 0.0) {
            state = State.DRIVER;
        }
    }

    public void tankDrive(double left, double right) {
        leftSpeed = left;
        rightSpeed = right;

        state = State.TANK_DRIVE;
    }

    public double getLeftEncoderPosition() {
        return flDrive.getEncoder().getPosition();
    }

    public double getRightEncoderPosition() {
        return frDrive.getEncoder().getPosition();
    }

    public int getLeftEncoderTicks(int ticksPerRev) {
        return (int) Math.round(getLeftEncoderPosition() * ticksPerRev);
    }
    public int getRightEncoderTicks(int ticksPerRev) {
        return (int) Math.round(getRightEncoderPosition() * ticksPerRev);
    }

    private void setConstantTuning() {
        
    }

    public void getConstantTuning() {
        
    }

    public State getState() {
        return state;
    }
}
