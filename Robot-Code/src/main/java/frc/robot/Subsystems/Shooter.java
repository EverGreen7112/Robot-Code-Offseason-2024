package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Math.SwervePoint;

public class Shooter extends SubsystemBase {
    
    private static Shooter m_instance = new Shooter();
    private CANSparkMax m_pivotMotor, m_leftShoot, m_rightShoot, m_containmentMotor;
    private Supplier <Double> m_pivotAngle;
    private DutyCycleEncoder m_pivotEncoder;
    private PIDController m_PidController;
    private final double M_WHEEL_PERIMETER = 0,
                         M_MIN_ANGLE = -49, M_MAX_ANGLE = 180;
    private final double BLUE_SPEAKER_X = 0, BLUE_SPEAKER_Y = 0,
                         RED_SPEAKER_X = 0, RED_SPEAKER_Y = 0,
                         SPEAKER_H = 0;


    private Shooter(){
        m_leftShoot = new CANSparkMax(0, MotorType.kBrushless);
        m_rightShoot = new CANSparkMax(0, MotorType.kBrushless);
        m_pivotEncoder = new DutyCycleEncoder(0);
        m_pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        m_containmentMotor = new CANSparkMax(0, MotorType.kBrushless);

        m_PidController.setP(0);
        m_PidController.setI(0);
        m_PidController.setD(0);

        m_leftShoot.setInverted(true);

        m_pivotAngle = (m_pivotEncoder::get);

        m_rightShoot.getEncoder().setVelocityConversionFactor((double)(1 / 60) * M_WHEEL_PERIMETER);
        m_leftShoot.getEncoder().setVelocityConversionFactor((double)(1 / 60) * M_WHEEL_PERIMETER);

    }

    @Override
    public void periodic(){
        m_pivotMotor.set(m_PidController.calculate(m_pivotAngle.get()));
    }

    public static Shooter getInstance(){
        return m_instance;
    }

    public void contain(){
        m_containmentMotor.set(0.5);
    }

    public void stopContain(){
        m_containmentMotor.stopMotor();
    }

    public void shoot(){
        m_rightShoot.set(1);
        m_rightShoot.set(1);

    }

    public void stopShoot(){
        m_rightShoot.stopMotor();
        m_leftShoot.stopMotor();
    }
    
    public void intake(){
        m_leftShoot.set(0.5);
        m_rightShoot.set(0.5);
        m_containmentMotor.set(0.5);
    }

    public void stopIntake(){
        m_leftShoot.stopMotor();
        m_rightShoot.stopMotor();
        m_containmentMotor.stopMotor();
    }

    public void turnTo(double angle){
        m_PidController.setSetpoint(MathUtil.clamp(angle, M_MIN_ANGLE, M_MAX_ANGLE));        
    }

    public void autoAim(){
        double targetAngle, distance;
        SwervePoint pos = getPos();
        if(isBlue()){
            distance = Math.sqrt(Math.pow(pos.getX() - BLUE_SPEAKER_X,2) + Math.pow(pos.getY() - BLUE_SPEAKER_Y,2));
        }
        else{
            distance = Math.sqrt(Math.pow(RED_SPEAKER_X - pos.getX(),2) + Math.pow(RED_SPEAKER_Y - pos.getY(),2));
        }
        targetAngle = Math.toDegrees(Math.atan2(distance , SPEAKER_H));
        turnTo(targetAngle);
        
    }

    public static boolean isBlue(){
        return true;
    }

    public static SwervePoint getPos(){
        return new SwervePoint(0, 0, 0);
    }
}
