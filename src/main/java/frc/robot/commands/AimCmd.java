package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.XboxController;

//  import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AimCmd extends Command {    
    private SwerveSubsystem m_swerveDrive;    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;
    // private SlewRateLimiter m_translateSRLimiter;
    // private SlewRateLimiter m_strafeSRLimiter;
    // private SlewRateLimiter m_rotateSRLimiter;
    private double m_translateVal;
    private double m_strafeVal;
    private double m_rotateVal;

    private double getTY;
    private double getTX;
    private double getArea;

    private final XboxController m_controller = new XboxController(0);

   // private double limelightAimProportional = new limelightAimProportional();
   // private double limelightRangeProportional = new limelightRangeProportional();

    public double limelightAimProportional(){

        

        double kp = 0.035;
    
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kp;
    
        targetingAngularVelocity *= 1.0;
    
        targetingAngularVelocity *= -1.0;
    
        return targetingAngularVelocity;
    }
       
    public double limelightRangeProportional(){
       double kp = .1;
    
       double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kp;
    
       targetingForwardSpeed *= 1.0;
    
       targetingForwardSpeed *= -1.0;
       
       return targetingForwardSpeed;
    
    }

    public AimCmd( SwerveSubsystem swerveDriveSubsys, 
                            DoubleSupplier translationSup, 
                            DoubleSupplier strafeSup,
                            DoubleSupplier rotationSup) {
        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
        m_swerveDrive = swerveDriveSubsys;
        addRequirements(swerveDriveSubsys);

        // m_translateSRLimiter = new SlewRateLimiter(0.5);
        //  m_strafeSRLimiter = new SlewRateLimiter(0.5);
        // m_rotateSRLimiter = new SlewRateLimiter(0.5);
    }

    @Override
    public void execute() {
        // Get Values, apply Deadband 
        m_translateVal = limelightRangeProportional();
        m_strafeVal = 0;
        m_rotateVal = limelightAimProportional();

        // Apply slewRateLimiters
        // m_translateVal = m_translateSRLimiter.calculate(m_translateVal);
        // m_strafeVal = m_strafeSRLimiter.calculate(m_strafeVal);
        // m_rotateVal = m_rotateSRLimiter.calculate(m_rotateVal);

        // Drive
        
        m_swerveDrive.drive(new Translation2d(m_translateVal, m_strafeVal)
                                .times(SDC.MAX_ROBOT_SPEED_M_PER_SEC), 
                                m_rotateVal * SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC, 
                                true);
        
    }
}