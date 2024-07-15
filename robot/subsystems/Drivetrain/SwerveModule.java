package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.CTREConfig;
import frc.lib.config.SwerveModuleConfig.ModuleConfig;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax azimuthalMotor;
    private CANSparkMax propulsionMotor;

    private RelativeEncoder azimuthalEncoder;
    private RelativeEncoder propulsionEncoder;
    private CANcoder absoluteEncoder;

    private final SparkPIDController propulsionController;
    private final SparkPIDController azimuthalController;

    public SwerveModule(int moduleNumber, ModuleConfig moduleConfig){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConfig.angleOffset;

        // CANCoder config
        this.absoluteEncoder = new CANcoder(moduleConfig.canCoderID);
        CTREConfig.configureCANcoder(absoluteEncoder, moduleConfig.canCoderInvert);

        // Azimuthal motor config
        this.azimuthalMotor = new CANSparkMax(moduleConfig.azimuthalMotorID, MotorType.kBrushless);
        this.azimuthalEncoder = azimuthalMotor.getEncoder();
        this.azimuthalController = azimuthalMotor.getPIDController();
        configAzimuthalMotor(moduleConfig.azimuthalInvert);

        // Propulsion Motor Config
        this.propulsionMotor = new CANSparkMax(moduleConfig.propulsionMotorID, MotorType.kBrushless);
        this.propulsionEncoder = propulsionMotor.getEncoder();
        this.propulsionController = propulsionMotor.getPIDController();
        configPropulsionMotor(moduleConfig.propulsionInvert);

        this.lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        azimuthalEncoder.setPosition(absolutePosition);
    }

    private void configAzimuthalMotor(boolean azimuthalInvert) {
        azimuthalMotor.restoreFactoryDefaults();
        azimuthalMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        azimuthalMotor.setInverted(azimuthalInvert);
        azimuthalMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        azimuthalEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        azimuthalController.setP(Constants.Swerve.azimuthalKP);
        azimuthalController.setI(Constants.Swerve.azimuthalKI);
        azimuthalController.setD(Constants.Swerve.azimuthalKD);
        azimuthalController.setFF(Constants.Swerve.azimuthalKFF);
        azimuthalMotor.enableVoltageCompensation(Constants.Swerve.voltafeComp);
        azimuthalMotor.burnFlash();
        resetToAbsolute();
    }

    private void configPropulsionMotor(boolean propulsionInvert) {
        propulsionMotor.restoreFactoryDefaults();
        propulsionMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        propulsionMotor.setInverted(propulsionInvert);
        propulsionMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        propulsionEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        propulsionEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        propulsionController.setP(Constants.Swerve.propulsionlKP);
        propulsionController.setI(Constants.Swerve.propulsionKI);
        propulsionController.setD(Constants.Swerve.propulsionKD);
        propulsionController.setFF(Constants.Swerve.propulsionKFF);
        propulsionMotor.enableVoltageCompensation(Constants.Swerve.voltafeComp);
        propulsionMotor.burnFlash();
        propulsionEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            propulsionMotor.set(percentOutput);
        } else {
            propulsionController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0
            );
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

        azimuthalController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(azimuthalEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().refresh().getValue());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(propulsionEncoder.getVelocity(), getAngle());
    }

    public double getPosition() {
        return propulsionEncoder.getPosition();
    }
}