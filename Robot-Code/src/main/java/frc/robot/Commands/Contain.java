package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class Contain extends CommandBase {

    public Contain(){
        addRequirements(Shooter.getInstance());
    }

    public void initialize(){
        Shooter.getInstance().contain();
    }

    public boolean isFinished(){
        return true;
    }
    
}