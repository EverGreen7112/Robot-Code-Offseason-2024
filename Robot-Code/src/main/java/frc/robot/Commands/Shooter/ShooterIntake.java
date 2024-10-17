package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class ShooterIntake extends CommandBase{

    public ShooterIntake(){
        addRequirements(Shooter.getInstance());
    }

    public void initialize(){
        Shooter.getInstance().intake();
    }

    public boolean isFinished(){
        return true;
    }
    
}
