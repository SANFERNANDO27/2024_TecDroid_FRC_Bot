package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          }, new Pose2d(0.0, 0.0, new Rotation2d()));

    private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        // Auto 
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(SwerveConstants.AutoConstants.translationController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(SwerveConstants.AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
        //ShuffleboardData();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getGyroYaw() {
        return gyro.getAngle() * (SwerveConstants.DriveConstants.gyroReversed ? -1.0 : 1.0);
    }

    public double getHeading() {
        return Math.IEEEremainder(getGyroYaw(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getInvertedRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
    }

    public double getRobotYawInRad() {
        return gyro.getAngle() * Math.PI/180;
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          },pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
          });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void ShuffleboardData() {
        // Shuffleboard
        ShuffleboardTab shooterTab = Shuffleboard.getTab("CanCoders reads");
        shooterTab.addDouble("FrontRight CanCoderRead", () -> {return frontRight.getAbsoluteEncoderRad();});
        shooterTab.addDouble("FrontLeft CanCoderRead", () -> {return frontLeft.getAbsoluteEncoderRad();});
        shooterTab.addDouble("BackRight CanCoderRead", () -> {return backRight.getAbsoluteEncoderRad();});
        shooterTab.addDouble("BackLeft CanCoderRead", () -> {return backLeft.getAbsoluteEncoderRad();});

        shooterTab.addDouble("FrontRight TurningPosition", () -> {return frontRight.getTurningPosition();});
        shooterTab.addDouble("FrontLeft TurningPosition", () -> {return frontLeft.getTurningPosition();});
        shooterTab.addDouble("BackRight TurningPosition", () -> {return backRight.getTurningPosition();});
        shooterTab.addDouble("BackLeft TurningPosition", () -> {return backLeft.getTurningPosition();});
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                                                               frontLeft.getState(),
                                                               backRight.getState(),
                                                               backLeft.getState());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds (desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(desiredStates);
    }

    // Autonomus while align to a apriltag detection

    public ChassisSpeeds chassisSpeedAdition(ChassisSpeeds chassisSpeeds1, ChassisSpeeds chassisSpeeds2) {
        return new ChassisSpeeds(chassisSpeeds1.vxMetersPerSecond + chassisSpeeds2.vxMetersPerSecond,
                                chassisSpeeds1.vyMetersPerSecond + chassisSpeeds2.vyMetersPerSecond,
                                chassisSpeeds1.omegaRadiansPerSecond + chassisSpeeds2.omegaRadiansPerSecond);
    }
    
}