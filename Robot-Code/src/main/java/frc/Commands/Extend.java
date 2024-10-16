package frc.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climb;

public class Extend extends CommandBase{
    
    public Extend(){
        addRequirements(Climb.getInstance());
    }

    @Override
    public void initialize(){
        Climb.getInstance().ExtendRight();
        Climb.getInstance().ExtendLeft();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    
}
