package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private static Shooter m_instance;
    private CANSparkMax m_pivotMotor, m_leftShoot, m_rightShoot, m_containmentMotor;
    private Supplier <Double> m_pivotAngle;
    private DutyCycleEncoder m_pivotEncoder;

    private Shooter(){
        m_leftShoot = new CANSparkMax(0, MotorType.kBrushless);
        m_rightShoot = new CANSparkMax(0, MotorType.kBrushless);
        m_pivotEncoder = new DutyCycleEncoder(0);
        m_pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        m_containmentMotor = new CANSparkMax(0, MotorType.kBrushless);

        m_pivotAngle = (m_pivotEncoder::get);
    }


}
