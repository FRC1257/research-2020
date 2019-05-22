package frc.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;

import frc.robot.Robot;
import frc.commands.*;
import frc.subsystems.*;



public class TestProceedEnd extends TimedCommand {
    private Testing testing;

    public TestProceedEnd() {
        super(1);
        testing = Robot.testing;
        requires(testing);
        
    }

    @Override
    public void initialize() {
       
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
