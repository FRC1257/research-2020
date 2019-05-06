package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;


public class Nothing extends Command {
    
  // Just a wait command

    public Nothing() {
        super();
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
