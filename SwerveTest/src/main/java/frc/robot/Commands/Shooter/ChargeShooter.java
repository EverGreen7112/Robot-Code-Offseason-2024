package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class ChargeShooter extends Command {

    public ChargeShooter(){}

    public void initialize(){
        Shooter.getInstance().shoot();
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();
    }
    
}
