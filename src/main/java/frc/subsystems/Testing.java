package frc.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.commands.*;

//subsystem for test proceeding 

public class Testing extends Subsystem {
    private boolean proceed;

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
    public boolean getProceed(){
        return proceed;
    }
    
    
    }
