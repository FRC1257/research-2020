package frc.commands;
import frc.commands.*;
import frc.subsystems.*;
import frc.robot.Robot;

public class Test extends CommandGroup{
    
   
    public Test(){
       
        AddSequential(new DriveCommand(0.5, 0) , 1);
        AddSequential(new Progress());
        AddSequential(new DriveCommand(-0.5, 0) , 1);
        AddSequential(new Progress());
        AddSequential(new TurnLeftCommand(), 1);
        AddSequential(new Progress());
        AddSequential(new TurnRightCommand(), 1);
        AddSequential(new Progress());
        AddSequential(new EjectCargoCommand(),1);
        AddSequential(new Progress());
        AddSequential(new IntakeCargoCommand(),1);
        AddSequential(new Progress());
        AddSequential(new AdvanceClimbCommand());
        AddSequential(new Nothing(), 5);
        AddSequential(new Progress());
        AddSequential(new AdvanceClimbCommand()); // when this happens make sure that the robot doesnt die
        AddSequential(new Nothing(), 5);
        AddSequential(new Progress());
        AddSequential(new AdvanceClimbCommand());
    }
    public end(){
        
    }
    public initialize(){
    
    }
    public interrupted(){

    }
    public boolean isFinished(){
    return false;
    }

}
