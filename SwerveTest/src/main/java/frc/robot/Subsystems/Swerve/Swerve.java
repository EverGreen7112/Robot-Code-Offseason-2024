package frc.robot.Subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

/**
NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
 * */
public class Swerve extends SubsystemBase implements SwerveConsts{

    private static Swerve m_instance = new Swerve();

    private SwerveModule[] m_modules;
    public AHRS m_gyro;    //TODO: change gyro to EverGyro
    
    private Swerve() {
        
        //offset of abs encoder to 0 degrees being forward
        ABS_ENCODERS[0].setOffset(166.552734375/360.0);
        ABS_ENCODERS[1].setOffset(292.939453125/360.0);
        ABS_ENCODERS[2].setOffset(48.867191314697266/360.0);
        ABS_ENCODERS[3].setOffset(307.3785705566406/360.0);

        
        for (EverSparkMax driveMotor : DRIVE_MOTORS) {
             driveMotor.restoreFactoryDefaults();
             driveMotor.setInverted(false);
             driveMotor.setIdleMode(IdleMode.kCoast);
        }
        
        for (EverSparkMax steerMotor : STEER_MOTORS) {
             steerMotor.setIdleMode(IdleMode.kCoast);
        }
        
        for (EverSparkMaxPIDController velocityController : WHEEL_VELOCITY_CONTROLLERS) {
             velocityController.setPIDF(WHEEL_VELOCITY_KP, WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_KF);   
             velocityController.setConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER / 60.0, ControlType.kVel);
             velocityController.setConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER, ControlType.kPos);
        }
        
        for (EverSparkMaxPIDController angleController : WHEEL_ANGLE_CONTROLLERS) {
             angleController.setPID(WHEEL_ANGLE_KP, WHEEL_ANGLE_KI, WHEEL_ANGLE_KD);      
             angleController.setConversionFactor(SwerveConsts.STEER_GEAR_RATIO * 360.0, ControlType.kPos);
        }

        m_modules = new SwerveModule[4];
        m_modules[0] = new SwerveModule(SwerveConsts.TL_VELOCITY_CONTROLLER, SwerveConsts.TL_DRIVE_MOTOR, new EverSparkInternalEncoder(SwerveConsts.TL_DRIVE_MOTOR), SwerveConsts.TL_ANGLE_CONTROLLER, SwerveConsts.TL_STEER_MOTOR, new EverSparkInternalEncoder(SwerveConsts.TL_STEER_MOTOR), SwerveConsts.ABS_ENCODERS[0]);
        m_modules[1] = new SwerveModule(SwerveConsts.TR_VELOCITY_CONTROLLER, SwerveConsts.TR_DRIVE_MOTOR, new EverSparkInternalEncoder(SwerveConsts.TR_DRIVE_MOTOR), SwerveConsts.TR_ANGLE_CONTROLLER, SwerveConsts.TR_STEER_MOTOR, new EverSparkInternalEncoder(SwerveConsts.TR_STEER_MOTOR), SwerveConsts.ABS_ENCODERS[1]);
        m_modules[2] = new SwerveModule(SwerveConsts.DL_VELOCITY_CONTROLLER, SwerveConsts.DL_DRIVE_MOTOR, new EverSparkInternalEncoder(SwerveConsts.DL_DRIVE_MOTOR), SwerveConsts.DL_ANGLE_CONTROLLER, SwerveConsts.DL_STEER_MOTOR, new EverSparkInternalEncoder(SwerveConsts.DL_STEER_MOTOR), SwerveConsts.ABS_ENCODERS[2]);
        m_modules[3] = new SwerveModule(SwerveConsts.DR_VELOCITY_CONTROLLER, SwerveConsts.DR_DRIVE_MOTOR, new EverSparkInternalEncoder(SwerveConsts.DR_DRIVE_MOTOR), SwerveConsts.DR_ANGLE_CONTROLLER, SwerveConsts.DR_STEER_MOTOR, new EverSparkInternalEncoder(SwerveConsts.DR_STEER_MOTOR), SwerveConsts.ABS_ENCODERS[3]);
        
        m_gyro = new AHRS(SerialPort.Port.kMXP);
        m_gyro.reset();
        
    }

    /**
     * @return the only instance of the swerve
     */
    public static Swerve getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        //absolute encoders
        // SmartDashboard.putNumber("TL", m_modules[0].getAngle());
        // SmartDashboard.putNumber("TR", m_modules[1].getAngle());
        // SmartDashboard.putNumber("DL", m_modules[2].getAngle());
        // SmartDashboard.putNumber("DR", m_modules[3].getAngle());

        // SmartDashboard.putString("velocity", getRobotOrientedVelocity().toString());
        // SmartDashboard.putNumber("angular velocity", getAngularVelocity());
       
    }

    public double getGyroOrientedAngle(){
        return m_gyro.getAngle() * SwerveConsts.GYRO_FACTOR;
    }

    public SwerveModule[] getModules(){
        return m_modules;
    }


    /**
     * see math on pdf document for more information
     * NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
     * 
     * @param velocity    - robot's target velocity(m/s)
     * @param isGyroOriented - true for origin of the gyro relative driving
     *                         false for robot relative driving
     * @param angularVelocity = robot's target angular velocity(deg/s) 
     */
    public void drive(Vector2d velocity, boolean isGyroOriented, double angularVelocity) {
        //convert to m/s
        double angularVel = (angularVelocity / 360.0) * SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;

        // if drive values are 0 stop moving
        if (velocity.mag() == 0 && angularVel == 0) {
            for (int i = 0; i < m_modules.length; i++) {
                m_modules[i].stopModule();
            }
        }

        // convert to gyro oriented
        if(isGyroOriented)
            velocity.rotate(Math.toRadians(getGyroOrientedAngle() * SwerveConsts.GYRO_FACTOR));
        
        // calculate rotation vectors
        Vector2d[] rotVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(SwerveConsts.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(90 * SwerveConsts.GYRO_FACTOR));
            // change magnitude of rot vector to rotationSpeed
            rotVecs[i].normalise();
            rotVecs[i].mul(angularVel);
        }

        Vector2d[] sumVectors = new Vector2d[m_modules.length];
        for (int i = 0; i < sumVectors.length; i++) {
            // sum rot and drive vectors
            sumVectors[i] = new Vector2d(velocity);
            sumVectors[i].add(rotVecs[i]);

            // set module state
            m_modules[i].setState(sumVectors[i]);
        }
    }

    /**
     * @return robot's angular velocity in NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
     * degrees/sec 
     */
    public double getAngularVelocity() {
        double angularVelocity = 0;
        
        for (int i = 0; i < m_modules.length; i++) {
            Vector2d moduleRotationVector = new Vector2d(SwerveConsts.physicalMoudulesVector[i]);
            moduleRotationVector.normalise();
            moduleRotationVector.rotate(Math.toRadians(90 * SwerveConsts.GYRO_FACTOR));

            Vector2d moduleVelocity = m_modules[i].getVelocity();

            angularVelocity += moduleVelocity.dot(moduleRotationVector);
        }
        
        // at this point the angular velocity is in m/s
        angularVelocity /= (double)SwerveConsts.physicalMoudulesVector.length;

        // converts angularVelocity to degrees/s
        angularVelocity /= SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;  // rotations / sec
        angularVelocity *= 360;  // degrees / sec

        return angularVelocity;
    }

    /**
     * @return robot's velocity in NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
     */
    public Vector2d getRobotOrientedVelocity(){
        Vector2d vel = new Vector2d();
        for(int i = 0; i < m_modules.length; i++){
            vel.add(m_modules[i].getVelocity());
        }
        vel.mul(1.0 / m_modules.length);
        vel = new Vector2d(vel.x, vel.y);
        return vel;
    }

    public Vector2d getGyroOrientedVelocity(){
        Vector2d vel = getRobotOrientedVelocity();
        vel.rotate(Math.toRadians(getGyroOrientedAngle() * GYRO_FACTOR));
        return vel;
    }
    
    public void resetModulesDistance(){
        for(int i = 0; i < m_modules.length; i++){
            m_modules[i].resetDistance();
        }
    }
}
