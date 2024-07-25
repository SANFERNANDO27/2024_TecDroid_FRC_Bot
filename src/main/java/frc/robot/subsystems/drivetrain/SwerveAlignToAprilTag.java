package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.constants.Constants.ShooterPositionerConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.DriveConstants;
import frc.robot.constants.SwerveConstants.OIConstants;
import frc.robot.subsystems.Limelight;

public class SwerveAlignToAprilTag {

    private SwerveSubsystem swerveSubsystem;
    private Limelight limelight;
    private PIDController rotatePidController;
     private PIDController moveInYPidController;

    public SwerveAlignToAprilTag (SwerveSubsystem swerveSubsystem, Limelight limelight) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight;
        this.rotatePidController = new PIDController(SwerveConstants.RotateConstants.kP, SwerveConstants.RotateConstants.kI, SwerveConstants.RotateConstants.kD);
        this.moveInYPidController = new PIDController(SwerveConstants.RotateConstants.kP, SwerveConstants.RotateConstants.kI, SwerveConstants.RotateConstants.kD);

    }

    public void Rotate(double velocity) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, velocity);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    public void moveInY(double velocity) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velocity, 0.0, 0.0);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    public boolean RotateAtAprilTagDetectionSetPoint(double setPoint) {
        double limelightX = limelight.getX();
        double rotatePercentage = -rotatePidController.calculate(limelightX, setPoint);
        boolean isAtSetPoint = false;
        
        if (setPoint + SwerveConstants.RotateConstants.setpointTolerance > limelightX && limelightX > setPoint - SwerveConstants.RotateConstants.setpointTolerance) {
            Rotate(0.0);
            isAtSetPoint = true;
         } else {
            Rotate(rotatePercentage);
            isAtSetPoint = false;
        }

        return isAtSetPoint;
    }

    public boolean MoveAtAprilTagDetectionSetPoint(double setPoint) {
        double limelightY = limelight.getY();
        double movePercentage = moveInYPidController.calculate(limelightY, setPoint);
        boolean isAtSetPoint = false;
        
        if (setPoint + SwerveConstants.RotateConstants.setpointTolerance > limelightY && limelightY > setPoint - SwerveConstants.RotateConstants.setpointTolerance) {
            moveInY(0.0);
            isAtSetPoint = true;
         } else {
            moveInY(movePercentage);
            isAtSetPoint = false;
        }

        return isAtSetPoint;
    }
}
