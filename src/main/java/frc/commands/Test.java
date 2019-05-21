package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class Test extends CommandGroup{
    
   
    public Test(){
       
        addSequential(new Drive(0.5, 0) , 1);
        addSequential(new TestProceed());
        addSequential(new Drive(-0.5, 0) , 1);
        addSequential(new TestProceed());
        addSequential(new TurnLeftCommand(), 1);
        addSequential(new TestProceed());
        addSequential(new TurnRightCommand(), 1);
        addSequential(new TestProceed());
        addSequential(new EjectCargoCommand(), 1);
        addSequential(new TestProceed());
        addSequential(new IntakeCargoCommand(), 1);
        addSequential(new TestProceed());
        addSequential(new EjectHatchCommand(),1);
        addSequential(new TestProceed());
        addSequential(new IntakeHatchCommand(),1);
        addSequential(new TestProceed());
        addSequential(new AdvanceClimbCommand());
        addSequential(new TestProceed(), 5);
        addSequential(new TestProceed());
        addSequential(new AdvanceClimbCommand()); // when this happens make sure that the robot doesnt die
        addSequential(new TestProceed(), 5);
        addSequential(new TestProceed());
        addSequential(new AdvanceClimbCommand());
    }
    public void end() {
        
    }
    public void initialize() {
    
    }
    public void interrupted() {
        end();

    }
    public boolean isFinished(){
    return false;
    }

}
