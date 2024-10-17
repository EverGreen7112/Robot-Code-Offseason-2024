package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;

public class ExtendRight extends Command{
    @Override
    public void execute(){
        Climb.getInstance().extendRight();
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Climb.getInstance().stopRight();
    }

    
}
