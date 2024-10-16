package frc.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climb;

public class Retract extends CommandBase {

    public Retract(){
        addRequirements(Climb.getInstance());
    }

    public void initialize(){
        Climb.getInstance().retractLeft();
        Climb.getInstance().ExtendRight();
    }

    public boolean isFinished(){
        return false;
    }
    
}
