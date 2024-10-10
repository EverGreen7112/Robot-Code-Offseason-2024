package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    private static Climb m_climb;
    private CANSparkMax m_motorL, m_motorR;
    private DigitalInput m_limitSwitchR, m_limitSwitchL;
    private final double m_speed = 0.4;


    private Climb(){
        m_motorL = new CANSparkMax(1, MotorType.kBrushless);
        m_motorR = new CANSparkMax(2, MotorType.kBrushless);
        m_limitSwitchL = new DigitalInput(0);
        m_limitSwitchR = new DigitalInput(1);

    }

    public Climb getInstance(){
        if(m_climb == null)
            m_climb = new Climb();
        return m_climb;
    }


    public boolean getLimitR(){
        return m_limitSwitchR.get();
    }
    

    public boolean getLimitL(){
        return m_limitSwitchL.get();
    }

    public void ExtendkRight(){
        m_motorR.set(m_speed);
    }
    
    public void ExtendLeft(){
        m_motorL.set(m_speed);
    }

    public void retractLeft(){
        m_motorL.set(-m_speed);
    }

    public void retractRight(){
        m_motorR.set(-m_speed);
    }
}
