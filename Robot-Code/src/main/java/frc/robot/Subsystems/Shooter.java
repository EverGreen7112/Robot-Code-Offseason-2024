package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Math.SwervePoint;

public class Shooter extends SubsystemBase {
    private final double 

                         INTAKE_SPEED = 0.5, CONTAINMENT_SPEED = 0.5, SHOOT_SPEED = 1, 
                         MIN_ANGLE = -49, MAX_ANGLE = 180,
                         BLUE_SPEAKER_X = 0, BLUE_SPEAKER_Y = 0,
                         RED_SPEAKER_X = 0, RED_SPEAKER_Y = 0,
                         SPEAKER_H = 0;

    private static Shooter m_instance = new Shooter();
    private CANSparkMax m_pivotMotor, m_leftShoot, m_rightShoot, m_containmentMotor;
    private Supplier <Double> m_pivotAngle;
    private DutyCycleEncoder m_pivotEncoder;
    private PIDController m_angleController;
    


    private Shooter(){
        m_leftShoot = new CANSparkMax(1, MotorType.kBrushless);
        m_rightShoot = new CANSparkMax(6, MotorType.kBrushless);
        m_leftShoot.restoreFactoryDefaults();
        m_rightShoot.restoreFactoryDefaults();
        m_leftShoot.setInverted(true);

        m_pivotMotor = new CANSparkMax(9, MotorType.kBrushless);
        m_containmentMotor = new CANSparkMax(7, MotorType.kBrushless);
        m_pivotMotor.restoreFactoryDefaults();
        m_containmentMotor.restoreFactoryDefaults();

        m_pivotEncoder = new DutyCycleEncoder(0);
        m_pivotEncoder.setDistancePerRotation(-1 * 360); //set conversion ratio 
        m_pivotEncoder.setPositionOffset(0.222401380560035);

        m_angleController = new PIDController(0.0111,  0.000001901, 0.00015);

        m_pivotAngle = (m_pivotEncoder::getDistance);

        


    }

    @Override
    public void periodic(){
        m_pivotMotor.set(MathUtil.clamp(m_angleController.calculate(m_pivotAngle.get()), -0.3, 0.3));
    }

    public static Shooter getInstance(){
        return m_instance;
    }

    public void containNote(){
        m_containmentMotor.set(CONTAINMENT_SPEED);
    }

    public void stopContain(){
        m_containmentMotor.stopMotor();
    }

    public void releaseNote(){
        m_containmentMotor.set(-CONTAINMENT_SPEED);
    }

    public void shoot(){
        m_rightShoot.set(SHOOT_SPEED);
        m_leftShoot.set(SHOOT_SPEED);

    }

    public void stopShoot(){
        m_rightShoot.stopMotor();
        m_leftShoot.stopMotor();
    }
    
    public void intakeNote(){
        m_leftShoot.set(INTAKE_SPEED);
        m_rightShoot.set(INTAKE_SPEED);
        m_containmentMotor.set(CONTAINMENT_SPEED);
    }

    public void stopIntake(){
        m_leftShoot.stopMotor();
        m_rightShoot.stopMotor();
        m_containmentMotor.stopMotor();
    }

    public void emitNote(){
        m_leftShoot.set(INTAKE_SPEED);
        m_rightShoot.set(INTAKE_SPEED);
        m_containmentMotor.set(-CONTAINMENT_SPEED);
    }

    public void stopEmission(){
        m_leftShoot.stopMotor();
        m_rightShoot.stopMotor();
        m_containmentMotor.stopMotor();
    }

    public void turnTo(double angle){
        m_angleController.reset();//reset the I
        m_angleController.setSetpoint(MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE));        
    }

    public void turnToIntake(){
        turnTo(MIN_ANGLE);
    }

    public boolean readyToIntake(){
        return MathUtil.isNear(MIN_ANGLE, m_pivotAngle.get(), 1); //is the shooter at the bottom
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
