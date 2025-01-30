package frc.robot;
import edu.wpi.first.math.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.command.CommandBase;
import edu.wpi.first.math.controller.PIDController; 
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightHelpers;

public class AimCmd extends CommandBase {
    private final DriveSubsystem m_turnController;
    private final PIDController m_turnController;
    private final SwerveSubsystem SwerveSubsystem;
    private final SwerveSubsystem m_fixedMaxRotationOutput;
    private final SwerveSubsystem m_odometryPoseYEntry;
    private final SwerveSubsystem m_fixedMaxTranslationOutput;
    private final MathUtil calculate;

    private final XboxController m_controller = new XboxController(0); 
    private final SwerveSubsystem m_swerve = new SwerveSubsystem();

    private final SwerveSubsystem m_fixedMaxTranslationOutput = new SlewRateLimiter(3);
    private final SwerveSubsystem m_fixedMaxTranslationOutput = new SlewRateLimiter(3);
    private final SwerveSubsystem m_fixedMaxRotationOutput = new SlewRateLimiter(3);


    
    public void autonomousCmd(){
        drive(false);
        m_swerve.periodic();
    }

    
    public void teleopCmd(){
        drive(true);
    }

    double limelight_aim_proportional(){

        

     double kp = 0.035;

     double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kp;

     targetingAngularVelocity *= SwerveSubsystem.m_fixedMaxRotationOutput;

     targetingAngularVelocity *= -1.0;

     return targetingAngularVelocity;
    }

    double limelight_range_proportional(){
        double kp = .1;

        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kp;
        targetingForwardSpeed *= Constants.SDC.MAX_ROBOT_SPEED_M_PER_SEC;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
    private void drive(boolean fieldRelative) {

        var xSpeed = 
            -m_xspeedlimiter(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02)) 
            * Constants.SDC.MAX_ROBOT_SPEED_M_PER_SEC;

        var ySpeed = 
           -SwerveSubsystem.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * Constants.SDC.MAX_ROBOT_SPEED_M_PER_SEC;
        var rot = 
           -m_fixedMaxRotationOutput.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * SwerveSubsystem.m_fixedMaxRotationOutput;

        if (m_controller.getAButton()) { 
            

            final var rot_limelight = limelight_aim_proportional();
            rot = rot_limelight;

            final var forward_limelight = limelight_range_proportional();
            xSpeed = forward_limelight;

            fieldRelative = false;
        }
     m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
                
}
