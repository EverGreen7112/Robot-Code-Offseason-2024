package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class EmitNote extends Command{
    

    public EmitNote(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        Shooter.getInstance().turnToIntake();
    }

    @Override
    public void execute(){
        if(Shooter.getInstance().readyToIntake())
            Intake.getInstance().emitNote();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stop();
    }



}
