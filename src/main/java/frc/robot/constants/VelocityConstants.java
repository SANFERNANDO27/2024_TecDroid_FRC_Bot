package frc.robot.constants;
import frc.robot.constants.SwerveConstants;

public class VelocityConstants {

    public static final class Velocities {
        // Swerve
        public static final double kTeleDriveMaxSpeedMetersPerSecond = SwerveConstants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond * .90;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = SwerveConstants.DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * .10;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.0;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2.0;
        
    }
}
