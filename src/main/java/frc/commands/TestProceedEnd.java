package frc.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.commands.*;



public class TestProceedEnd extends InstantCommand {

    public TestProceedEnd() {
        super();
        
    }

    @Override
    public void initialize() {
      TestProceed.end();
       
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
