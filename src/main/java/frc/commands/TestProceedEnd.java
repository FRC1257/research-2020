package frc.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import frc.robot.Robot;
import frc.commands.*;
import frc.subsystems.*;



public class TestProceedEnd extends InstantCommand {
    private Testing testing;

    public TestProceedEnd() {
        super();
        testing = Robot.testing;
        requires(testing);
        
    }

    @Override
    public void initialize() {
      testing.Proceedtoggle();
       
    }

    @Override
    public void end() {
        
    }

    @Override
    public void interrupted() {
        end();
    }
}
