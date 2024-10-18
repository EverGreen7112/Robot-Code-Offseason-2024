package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class SwerveModule extends SubsystemBase {

    // swerve module motor controllers
    private EverPIDController m_velocityController;
    private EverMotorController m_driveMotor;
    private EverEncoder m_driveEncoder;
    
    private EverPIDController m_angleController;
    private EverMotorController m_steerMotor;
    private EverEncoder m_steerEncoder;

    // absolute encoder of module
    private EverAbsEncoder m_absSteerEncoder;

    /**
     * @param velocityController velocity control units of measure should be configured for meters per second(the speed of the wheel)
     * @param angleController angle control units of measure should be configured for degrees(the angle of the wheel)
    */
    public SwerveModule(EverPIDController velocityController, EverMotorController driveMotor,
            EverEncoder driveEncoder, EverPIDController angleController,
            EverMotorController steerMotor, EverEncoder steerEncoder) {
        m_velocityController = velocityController;
        m_driveMotor = driveMotor;
        m_driveEncoder = driveEncoder;
        m_angleController = angleController;
        m_steerMotor = steerMotor;
        m_steerEncoder = steerEncoder;
    }

    /**
     * @param velocityController velocity control units of measure should be configured for meters per second(the speed of the wheel)
     * @param angleController angle control units of measure should be configured for degrees(the angle of the wheel)
     * @param absSteerEncoder absSteerEncoder units of measure should be configured for degrees(absolute of the wheel)
     */
    public SwerveModule(EverPIDController velocityController, EverMotorController driveMotor,
                        EverEncoder driveEncoder, EverPIDController angleController,
                        EverMotorController steerMotor, EverEncoder steerEncoder,
                        EverAbsEncoder absSteerEncoder){
        this(velocityController, driveMotor, driveEncoder,
             angleController, steerMotor, steerEncoder);
        m_absSteerEncoder = absSteerEncoder;
        m_absSteerEncoder.setPosConversionFactor(1);
        m_steerEncoder.setPos(getAbsAngle());
        
    }

    public double getAbsAngle(){
        return Funcs.convertRotationsToDegrees(m_absSteerEncoder.getAbsPos());
    }

    /**
     * @param speed - in meters per second
     * @param angle - in degrees
     */
    public void setState(double speed, double angle) {
        setState(new Vector2d(speed * Math.cos(Math.toRadians(angle)), speed * Math.sin(Math.toRadians(angle))));
    }

    /**
     * 
     * @param desiredState - desired velocity in meters per second
     */
    public void setState(Vector2d desiredState) {
        
        if(desiredState.mag() < SwerveConsts.MIN_SPEED){
            desiredState = new Vector2d(0, 0);
        }        

        if(desiredState.mag() == 0){
            stopModule();
        }

        double targetSpeed = desiredState.mag();
        double targetAngle = Math.toDegrees(desiredState.theta());

        double currentAngle = getAngle();

        // calculate optimal delta angle
        double optimizedFlippedDeltaTargetAngle = Funcs.getShortestAnglePath(currentAngle, targetAngle - 180);
        double optimizedNormalDeltaTargetAngle = Funcs.getShortestAnglePath(currentAngle, targetAngle);

        double optimizedDeltaTargetAngle = 0;
        if (Math.abs(optimizedNormalDeltaTargetAngle) > Math.abs(optimizedFlippedDeltaTargetAngle)) {
            optimizedDeltaTargetAngle = optimizedFlippedDeltaTargetAngle;
        } else {
            optimizedDeltaTargetAngle = optimizedNormalDeltaTargetAngle;
        }

        // turn module to target angle
        m_angleController.activate(currentAngle + optimizedDeltaTargetAngle, ControlType.kPos);

        // dot product to current state
        targetSpeed *= Math.cos(Math.toRadians(optimizedNormalDeltaTargetAngle));
        
        
        // set speed of module at target speed
        m_velocityController.activate(targetSpeed, ControlType.kVel);
    }

    /**
     * turn module to targetAngle
     * @param targetAngle in degrees
     */
    public void turnToAngle(double targetAngle) {
        m_angleController.activate(targetAngle, ControlType.kPos);
    }

    /**                                                 
     * @return current angle in degrees(0 degrees is forward)             
     */                                                 
    public double getAngle() {
        return m_steerEncoder.getPos();
    }

    public void resetDistance(){
        m_driveEncoder.setPos(0);
    }

    /**
     * @return distance travelled in meters 
     */
    public double getDistance(){
        return m_driveEncoder.getPos();
    }

    /**
     * @return current speed in meters per second
     */
    public double getSpeed() {
        return m_driveEncoder.getVel();
    }

    /**
     * @return module's velocity vector in meters per second
     */
    public Vector2d getVelocity(){
        double mag = getSpeed();
        double theta = Math.toRadians(getAngle());
        Vector2d vec = new Vector2d(mag * Math.cos(theta), mag * Math.sin(theta));
        return vec;
    }

    public void stopModule() {
        m_driveMotor.stop();
        m_steerMotor.stop();
    }

    @Override
    public void periodic() {}

    

}
