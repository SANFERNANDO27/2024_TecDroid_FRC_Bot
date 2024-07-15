// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.12;
    public static final double kTurningMotorGearRatio = 1 / 6.12;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
}

  public static class ShooterConstants {
    public static final int BOTTOM_SHOOTER_ID = 3;
    public static final int Top_SHOOTER_ID = 4;

    public static final int MAX_NEO_RPMS = 5600;

    // PID's
    public static final double BOTTOM_P = 0.00005;
    public static final double BOTTOM_I = 0.0;
    public static final double BOTTOM_D = 0.0005;
    public static final double BOTTOM_F = 0.000225;

    public static final double TOP_P = 0.00015;
    public static final double TOP_I = 0.0;
    public static final double TOP_D = 0.0;
    public static final double TOP_F = 0.000225;

    public static final double CONFIGURED_SETPOINT = 2000;
    
  }

  public static class ShooterPositionerConstants {
    // Id's
    public static final int leftPositionerId = 6;
    public static final int rightPositionerId = 5;

    // Shooter positioner motors velocity
    public static final double shooterPositionerVelocity = 700;

    // PID's

    // Motors PID's
    public static final double rightP = 0.00005;
    public static final double rightI = 0.0;
    public static final double rightD = 0.0005;
    public static final double rightF = 0.000225;

    public static final double leftP = 0.00015;
    public static final double leftI = 0.0;
    public static final double leftD = 0.0;
    public static final double leftF = 0.000225;

    // Positioner encoder PID
    public static final double kP = 7.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;


    // Shooter Positions
    public static final double home = 0.0;
    public static final double shootPosition = -0.06;
    public static final double setPointTolerance = 1.0;

  }

  public static class IntakeConstants {
    public static final int motorId = 1;

    public static final int maxVelocity = 5000;

    // PID's
    public static final double motor_P = 0.00005;
    public static final double motor_I = 0.0;
    public static final double motor_D = 0.0;
    public static final double motor_f = 0.000225;

    public static final double CONFIGURED_SETPOINT = 1000;
  }

  public static class IndexerConstants {
    public static final int motorId = 2;

    public static final int maxVelocity = 5000;

    // PID's
    public static final double motor_P = 0.00005;
    public static final double motor_I = 0.0;
    public static final double motor_D = 0.0;
    public static final double motor_f = 0.000225;

    public static final double CONFIGURED_SETPOINT = 1000;
  }

  public static class ClimberConstants {
    public static final int rightMotorId = 8;
    public static final int leftMotorId = 7;

    public static final int MAX_NEO_RPMS = 5600;

    // Motors PID's
    public static final double rightP = 0.00005;
    public static final double rightI = 0.0;
    public static final double rightD = 0.0005;
    public static final double rightF = 0.000225;

    public static final double leftP = 0.00015;
    public static final double leftI = 0.0;
    public static final double leftD = 0.0;
    public static final double leftF = 0.000225;

    public static final double CONFIGURED_SETPOINT = 2000;
    
  }
    // Constants swerve Diego's version
    public static final class Swerve {

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(10.875 * 2);
        public static final double wheelBase = Units.inchesToMeters(10.875 * 2);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.12;

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        
        //Swerve Voltage Compensation
        public static final double voltafeComp = 12.0;

        // Swerve Current Limiting
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        //Deadband
        public static final double stickDeadband = 0.05; 

        // Azimuthal motor PID values
        public static final double azimuthalKP = 0.01;
        public static final double azimuthalKI = 0.0;
        public static final double azimuthalKD = 0.0;
        public static final double azimuthalKFF = 0.0;

        // Propulsion motor PID values
        public static final double propulsionlKP = 0.01;
        public static final double propulsionKI = 0.0;
        public static final double propulsionKD = 0.0;
        public static final double propulsionKFF = 0.0;

        // Drive Motor Conversion Factors
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

        // Swerve Profiling Values
        public static final double maxSpeed = 5; // meters per second
        public static final double maxAngularVelocity = 10;

        // Neutral Modes
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        // Gyroscope Constants
        public static final class Gyroscope {
            public static final AHRS gyro = new AHRS();
            public static final gyroscope_convention convention = gyroscope_convention.RIGHT_IS_POSITIVE;
        } 

        // Gyroscope convention enumeration
        public enum gyroscope_convention {
            LEFT_IS_POSITIVE(1),
            RIGHT_IS_POSITIVE(-1);

            public final int value;

            gyroscope_convention(int value) {
                this.value = value;
            }
        }

        // Modules

        // Front Right Module - Module 0
        public static final class Mod0 {
            public static final int propulsionMotorID = 11;
            public static final int azimuthalMotorID = 12;
            public static final int canCoderID = 13;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }

        // Front Left Module - Module 1
        public static final class Mod1 {
            public static final int propulsionMotorID = 21;
            public static final int azimuthalMotorID = 22;
            public static final int canCoderID = 23;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }

        // Back Left Module - Module 2
        public static final class Mod2 {
            public static final int propulsionMotorID = 31;
            public static final int azimuthalMotorID = 32;
            public static final int canCoderID = 33;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }

        // Back Right Module - Module 3
        public static final class Mod3 {
            public static final int propulsionMotorID = 41;
            public static final int azimuthalMotorID = 42;
            public static final int canCoderID = 43;
            public static final boolean propulsionInvert = false;
            public static final boolean azimuthalInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants()
                .withDriveMotorId(propulsionMotorID)
                .withSteerMotorId(azimuthalMotorID)
                .withCANcoderId(canCoderID)
                .withCANcoderOffset(angleOffset.getDegrees());
        }
    }
}
