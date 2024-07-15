package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConfig.ModuleConfig;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final AHRS gyro;
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] mSwerveMods;
    private final ShuffleboardTab fieldTab;

    public Swerve() {
        gyro = new AHRS();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
            getYaw(),
            new SwerveModulePosition[] { // Initialize with zeros 
                new SwerveModulePosition(0, new Rotation2d(0)),
                new SwerveModulePosition(0, new Rotation2d(0)),
                new SwerveModulePosition(0, new Rotation2d(0)),
                new SwerveModulePosition(0, new Rotation2d(0))
            }
        );

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, new ModuleConfig(
                Constants.Swerve.Mod0.propulsionMotorID,
                Constants.Swerve.Mod0.azimuthalMotorID,
                Constants.Swerve.Mod0.canCoderID,
                Constants.Swerve.Mod0.propulsionInvert,
                Constants.Swerve.Mod0.azimuthalInvert,
                Constants.Swerve.Mod0.canCoderInvert,
                Constants.Swerve.Mod0.angleOffset
            )),
            new SwerveModule(1, new ModuleConfig(
                Constants.Swerve.Mod1.propulsionMotorID,
                Constants.Swerve.Mod1.azimuthalMotorID,
                Constants.Swerve.Mod1.canCoderID,
                Constants.Swerve.Mod1.propulsionInvert,
                Constants.Swerve.Mod1.azimuthalInvert,
                Constants.Swerve.Mod1.canCoderInvert,
                Constants.Swerve.Mod1.angleOffset
            )),
            new SwerveModule(2, new ModuleConfig(
                Constants.Swerve.Mod2.propulsionMotorID,
                Constants.Swerve.Mod2.azimuthalMotorID,
                Constants.Swerve.Mod2.canCoderID,
                Constants.Swerve.Mod2.propulsionInvert,
                Constants.Swerve.Mod2.azimuthalInvert,
                Constants.Swerve.Mod2.canCoderInvert,
                Constants.Swerve.Mod2.angleOffset
            )),
            new SwerveModule(3, new ModuleConfig(
                Constants.Swerve.Mod3.propulsionMotorID,
                Constants.Swerve.Mod3.azimuthalMotorID,
                Constants.Swerve.Mod3.canCoderID,
                Constants.Swerve.Mod3.propulsionInvert,
                Constants.Swerve.Mod3.azimuthalInvert,
                Constants.Swerve.Mod3.canCoderInvert,
                Constants.Swerve.Mod3.angleOffset
            ))
        };

        fieldTab = Shuffleboard.getTab("Field");
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = new SwerveModulePosition(
                mSwerveMods[i].getPosition(),
                mSwerveMods[i].getAngle()
            );
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.Gyroscope.convention == Constants.Swerve.gyroscope_convention.RIGHT_IS_POSITIVE)
            ? Rotation2d.fromDegrees(-gyro.getYaw())
            : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        
    // Update Shuffleboard with the current robot pose and gyro angle
    //fieldTab.add("Robot Pose", getPose());
    //fieldTab.add("Gyro Angle", getYaw().getDegrees());

    // Update module-specific data on the Shuffleboard
    /*for (SwerveModule mod : mSwerveMods) {
        fieldTab.add("Mod " + mod.moduleNumber + " CANCoder", mod.getCanCoder().getDegrees());
        fieldTab.add("Mod " + mod.moduleNumber + " Integrated", mod.getAngle().getDegrees());
        fieldTab.add("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }*/
    }
}