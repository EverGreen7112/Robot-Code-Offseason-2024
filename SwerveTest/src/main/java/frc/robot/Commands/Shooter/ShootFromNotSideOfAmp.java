package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShootFromNotSideOfAmp extends Command {

    public ShootFromNotSideOfAmp(){}

    public void initialize(){
        addRequirements(Shooter.getInstance());
        Shooter.getInstance().turnTo(110); //114
        Shooter.getInstance().shoot(1, 0.5);
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();
    }
    
}
