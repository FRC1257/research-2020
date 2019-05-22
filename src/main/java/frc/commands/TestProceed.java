package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.subsystems.*;


public class TestProceed extends Command {
    private Testing testing;
    //proceed and wait command

    public TestProceed() {
        super();
        testing = Robot.testing;
        requires(testing);
    }

    @Override
    public void execute() {
        if(proceed){
            end();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        testing.Proceedtoggle;
    }

    @Override
    public void interrupted() {
        end();
    }
}
