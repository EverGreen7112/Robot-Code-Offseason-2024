package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ChargeShooterToAmp extends Command {

    public ChargeShooterToAmp(){}

    public void initialize(){
        Shooter.getInstance().shootToAmp();
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();
    }
    
}
