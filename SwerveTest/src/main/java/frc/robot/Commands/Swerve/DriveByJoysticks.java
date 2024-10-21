package frc.robot.Commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class DriveByJoysticks extends Command{

    private double JOYSTICK_DEADZONE = 0.2;
    private Supplier<Double> m_speedX, m_speedY, m_rotation;
    private Supplier<Boolean> m_activateAnglePID;
    private Supplier<Boolean> m_activateTurnToSpeaker;
    
    private boolean m_lastActivateAnglePID;
    private PIDController m_angleController;
    
    public DriveByJoysticks(Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotation, Supplier<Boolean> activateAnglePID, Supplier<Boolean> activateTurnToSpeaker){
        addRequirements(Swerve.getInstance());
        m_speedX = speedX;
        m_speedY = speedY;
        m_rotation = rotation;
        m_activateAnglePID = activateAnglePID;
        m_lastActivateAnglePID = activateAnglePID.get();
        m_angleController = new PIDController(0.02, 0, 0.0);
        m_activateTurnToSpeaker = activateTurnToSpeaker;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //get values from suppliers
        double speedX = m_speedX.get();
        double speedY = m_speedY.get();
        double rotation = m_rotation.get();

        //apply deadzone on supplier values
        if(Math.abs(speedX) < JOYSTICK_DEADZONE)
            speedX = 0;
        if(Math.abs(speedY) < JOYSTICK_DEADZONE)
            speedY = 0; 
        if(Math.abs(rotation) < JOYSTICK_DEADZONE)
            rotation = 0;

        //round values
        rotation = Funcs.roundAfterDecimalPoint(rotation, 2);
        speedX = Funcs.roundAfterDecimalPoint(speedX, 2);
        speedY = Funcs.roundAfterDecimalPoint(speedY, 2);


        if(m_activateTurnToSpeaker.get() && !m_activateAnglePID.get()){
            double currentAngle = SwerveLocalizer.getInstance().getFieldOrientedAngle();
            m_angleController.setSetpoint(currentAngle + Funcs.getShortestAnglePath(currentAngle, SwerveLocalizer.getInstance().getAngleToSpeaker()));
            rotation = -MathUtil.clamp(m_angleController.calculate(currentAngle), -1, 1);
        }

        if(m_activateAnglePID.get() && m_lastActivateAnglePID == false && !m_activateTurnToSpeaker.get()){
            m_angleController.setSetpoint(Swerve.getInstance().getGyroOrientedAngle());
        }

        if(m_activateAnglePID.get() && !m_activateTurnToSpeaker.get()){
            rotation = -MathUtil.clamp(m_angleController.calculate(Swerve.getInstance().getGyroOrientedAngle()), -1, 1);
        }

        
        m_lastActivateAnglePID = m_activateAnglePID.get();

        //create drive vector
        Vector2d vec = new Vector2d(-speedX * SwerveConsts.MAX_DRIVE_SPEED.get(), speedY * SwerveConsts.MAX_DRIVE_SPEED.get());
        
        //make sure mag never goes over 1 so driving in all directions will be the same speed
        if(vec.mag() > Swerve.MAX_DRIVE_SPEED.get()){
            vec.normalise();
            vec.mul(Swerve.MAX_DRIVE_SPEED.get());
        }

        //drive
        Swerve.getInstance().drive(Funcs.convertFromStandardAxesToWpilibs(vec), true, -rotation * SwerveConsts.MAX_ANGULAR_SPEED.get());       
    }


}