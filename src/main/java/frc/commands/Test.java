package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class Test extends CommandGroup{
    
   
    public Test(){
       
        addSequential(new Drive(0.5, 0) , 1);
        addSequential(new Nothing());
        addSequential(new Drive(-0.5, 0) , 1);
        addSequential(new Nothing());
        addSequential(new TurnLeftCommand(), 1);
        addSequential(new Nothing());
        addSequential(new TurnRightCommand(), 1);
        addSequential(new Nothing());
        addSequential(new EjectCargoCommand(), 1);
        addSequential(new Nothing());
        addSequential(new IntakeCargoCommand(), 1);
        addSequential(new Nothing());
        addSequential(new EjectHatchCommand(),1);
        addSequential(new Nothing());
        addSequential(new IntakeHatchCommand,1);
        addSequential(new Nothing());
        addSequential(new AdvanceClimbCommand());
        addSequential(new Nothing(), 5);
        addSequential(new Nothing());
        addSequential(new AdvanceClimbCommand()); // when this happens make sure that the robot doesnt die
        addSequential(new Nothing(), 5);
        addSequential(new Nothing());
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
