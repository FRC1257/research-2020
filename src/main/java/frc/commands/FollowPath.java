/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.util.motionprofile.PathGenerator;
import frc.robot.RobotMap;

/**
 * <h1>FollowPath</h1>
 * Follows a given path. Used in conjunction with motion profiling.
 * 
 * @author Allen Du
 * @since 2019-03-10
 */
public class FollowPath extends Command {
    public double[][] path;
    public PathGenerator profile;
    public double maxVel;
    public double maxAccel;

    /**
     * Constructor.
     * 
     * @param path The path to follow, given in an n*2 array.
     */
  public FollowPath(double[][] path) {
    requires(Robot.driveTrain);

    this.maxVel = RobotMap.NEO_MAX_RPM;
    this.maxAccel = RobotMap.NEO_MAX_ACCEL;
    this.profile = new PathGenerator(this.path, this.maxVel, this.maxAccel, 17.0);

    // We're making the coordinates field-centric, so update the robot position based on the position onfield.
    // this.profile.updatePos(0.0, 0.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      SmartDashboard.putString("Path status", "Following a path");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.showRobotPos();
    // Left, then right. Somehow you're going to forget this.
    Robot.driveTrain.tankDrive(this.profile.velocity(RobotMap.TRACKWIDTH, true), this.profile.velocity(RobotMap.TRACKWIDTH, false));
  }

  /**
   * Is the command finished?
   * 
   * @return true if the robot is close enough to our endpoint; false if not
   */
  @Override
  protected boolean isFinished() {
    // Tolerance of distance
    if(Math.abs(Robot.driveTrain.showRobotPos()[0][0] - this.path[this.path.length - 1][0]) < 0.08 && Math.abs(Robot.driveTrain.showRobotPos()[0][1] - this.path[this.path.length - 1][1]) < 0.08) {
      
      return true;
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Path status", "Path finished");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}