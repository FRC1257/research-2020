package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;



public class Progress extends Command {
    private OI oi;
  

    public Progress() {
        super();
        oi = Robot.oi;
        
        
    }

    @Override
    public void execute() {
        if(oi.testProceed()){
            end();
        }
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
