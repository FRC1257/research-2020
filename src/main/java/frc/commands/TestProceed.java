package frc.commands;

import edu.wpi.first.wpilibj.command.Command;


public class TestProceed extends Command {
    //proceed and wait command

    public TestProceed() {
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
