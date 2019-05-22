package frc.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.commands.*;



public class Testing extends Subsystem {
    boolean proceed;

    public Testing(){
    proceed = false;
    
    }

    @Override
    public void initDefaultCommand() {
        //no
    }
    public void Proceedtoggle() {
    proceed = !proceed;
    }
    
    }
