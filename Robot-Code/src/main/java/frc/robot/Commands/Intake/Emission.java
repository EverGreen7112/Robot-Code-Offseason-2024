package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;

public class Emission extends CommandBase{
    

    public Emission(){
        addRequirements(Intake.getInstance());
    }

    public void initialize(){
        Intake.getInstance().noteEmission();
    }

}
