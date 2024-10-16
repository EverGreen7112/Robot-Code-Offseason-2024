package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class ShooterTurn extends CommandBase {

    public ShooterTurn(){
        addRequirements(Shooter.getInstance());
    }

    public void initialize(){
        
    }
    
}