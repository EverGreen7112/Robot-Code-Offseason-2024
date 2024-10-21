package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwitchMode extends Command{
    public enum SpeedMode{
        kNormal,
        kTurbo,
        kSlow
    }    
    private SpeedMode m_mode;
    public SwitchMode(SpeedMode mode){
        m_mode = mode;
    }

    @Override
    public void initialize() {
        if(m_mode == SpeedMode.kTurbo){
            SmartDashboard.putNumber("speed", 3);
        }
        else if(m_mode == SpeedMode.kNormal){
            SmartDashboard.putNumber("speed", 1);
        }
        else if(m_mode == SpeedMode.kSlow){
            SmartDashboard.putNumber("speed", 0.5);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("speed", 1);
    }
}


