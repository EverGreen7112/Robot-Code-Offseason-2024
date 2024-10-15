package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;

public class IntakeC extends CommandBase{

    public IntakeC(){
        addRequirements(Intake.getInstance());
    }


    @Override
    public void initialize(){
        Intake.getInstance.noteIntake();
    }



    
}
