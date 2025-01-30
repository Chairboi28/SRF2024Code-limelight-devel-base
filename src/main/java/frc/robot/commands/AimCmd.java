package frc.robot;
import edu.wpi.first.math.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsytems.VisionSubsystem.limelightRangeProportional;
import frc.robot.subsytems.VisionSubsystem.limelightAimProportional;

import edu.wpi.first.wpilibj.command.CommandBase;
import edu.wpi.first.math.controller.PIDController; 
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robor.subsytems.VisionSubsystem;

public class AimCmd extends CommandBase {
    private final DriveSubsystem m_turnController;
    private final PIDController m_turnController;
    private final SwerveSubsystem SwerveSubsystem;
    private final SwerveSubsystem m_fixedMaxRotationOutput;
    private final SwerveSubsystem m_odometryPoseYEntry;
    private final SwerveSubsystem m_fixedMaxTranslationOutput;
    private final MathUtil calculate;
    private final VisionSubsystem limelightRangeProportional;
    private final VisionSubsystem limelightAimProporional;

    private final XboxController m_controller = new XboxController(0); 
    private final SwerveSubsystem m_swerve = new SwerveSubsystem();

    private final SwerveSubsystem m_fixedMaxTranslationOutput = new SlewRateLimiter(3);
    private final SwerveSubsystem m_fixedMaxTranslationOutput = new SlewRateLimiter(3);
    private final SwerveSubsystem m_fixedMaxRotationOutput = new SlewRateLimiter(3);


    //TODO: fix compile errors 
    public void autonomousCmd(){
        drive(false);
        m_swerve.periodic();
    }

    
    public void teleopCmd(){
        drive(true);
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
            

            final var rot_limelight = limelightAimProportional();
            rot = rot_limelight;

            final var forward_limelight = limelightRangeProportional();
            xSpeed = forward_limelight;

            fieldRelative = false;
        }
     m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
                
}
