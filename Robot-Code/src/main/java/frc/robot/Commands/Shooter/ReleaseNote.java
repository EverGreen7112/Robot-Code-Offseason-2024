package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ReleaseNote extends Command{

    public ReleaseNote(){
        addRequirements(Shooter.getInstance());
    }

    public void initialize(){
        Shooter.getInstance().releaseNote();
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopContain();
    }
    
}