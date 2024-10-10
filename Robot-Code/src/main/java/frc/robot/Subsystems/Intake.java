package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private Intake m_instance;
    private CANSparkMax m_sparkMax;
    private final double m_speed = 0.3;

    private Intake(){
        m_sparkMax = new CANSparkMax(0,MotorType.kBrushless);
        m_sparkMax.restoreFactoryDefaults();
    }

    public Intake getInstance(){
        if (m_instance == null)
            return m_instance;
        return m_instance;
    }

    public void noteIntake(){
        m_sparkMax.set(1.0);
    }

    public void stopIntake(){
        m_sparkMax.set(0);
    }

    public void idleMode(IdleMode idleMode){
        m_sparkMax.setIdleMode(idleMode);
    }
    
}