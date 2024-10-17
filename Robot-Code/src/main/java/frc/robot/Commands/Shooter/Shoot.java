package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class Shoot extends CommandBase {

    public Shoot(){
        addRequirements(Shooter.getInstance());
    }

    public void initialize(){
        Shooter.getInstance().shoot();
    }

    public boolean isFinished(){
        return true;
    }
    
}
