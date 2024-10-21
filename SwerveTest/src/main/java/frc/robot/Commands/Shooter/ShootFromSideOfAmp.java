package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShootFromSideOfAmp extends Command{

     @Override
    public void initialize() {
        Shooter.getInstance().turnTo(113);
        Shooter.getInstance().shoot(0.5, 1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();;
    }
    
}
