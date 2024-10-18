package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class IntakeNote extends Command{

    public IntakeNote(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        Shooter.getInstance().turnToIntake();
    }

    @Override
    public void execute(){
        if(Shooter.getInstance().readyToIntake()){
            // Intake.getInstance().intakeNote();
            // Shooter.getInstance().intakeNote();
        }
        Intake.getInstance().intakeNote();

        Shooter.getInstance().intakeNote();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stop();
        Shooter.getInstance().stopIntake();
    }

    
}
