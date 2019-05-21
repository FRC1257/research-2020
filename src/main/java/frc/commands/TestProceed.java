package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;


public class TestProceed extends Command {
    //proceed and wait command
    OI oi;

    public TestProceed() {
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
