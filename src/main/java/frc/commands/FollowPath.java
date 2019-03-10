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

public class FollowPath extends Command {
    public double[][] path;
    public PathGenerator profile;
    public double maxVel;
    public double maxAccel;

  public FollowPath(double[][] path) {
    requires(Robot.driveTrain);

    this.maxVel = RobotMap.NEO_MAX_RPM;
    this.maxAccel = RobotMap.NEO_MAX_ACCEL;
    this.profile = new PathGenerator(this.path, this.maxVel, this.maxAccel, 17.0);

    this.profile.updatePos(0.0, 0.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      SmartDashboard.putString("Path status", "Following a path");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.tankDrive(this.profile.velocities(RobotMap.TRACKWIDTH, Robot.driveTrain.showRobotPos())[0], this.profile.velocities(RobotMap.TRACKWIDTH, Robot.driveTrain.showRobotPos())[1]);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    int index = (int) Math.round(this.profile.lookaheadPoint(Robot.driveTrain.showRobotPos())[1][1]);
    if(index == this.path.length - 1 && Math.abs(Robot.driveTrain.showRobotPos()[0][0] - this.path[this.path.length - 1][0]) < 0.08 && Math.abs(Robot.driveTrain.showRobotPos()[0][1] - this.path[this.path.length - 1][1]) < 0.08) {
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