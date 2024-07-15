package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ShooterPositionerConstants;


public class ShooterPositioner {
    
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;
    private final DutyCycleEncoder positionerEncoder;

    private final SparkPIDController rightPidController;
    private final SparkPIDController leftPidController;
    private final PIDController pidController;

    public ShooterPositioner(final int rightId, final int leftId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterPositioner");
        shooterTab.addDouble("Right Velocity (RPMs)", () -> {return getRightVelocity();});
        shooterTab.addDouble("Left Velocity (RPMs)", () -> {return getLeftVelocity();});
        shooterTab.addDouble("Encoder distance", () -> {return getEncoderDistance();});

        rightMotor = new CANSparkMax(rightId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(leftId, MotorType.kBrushless);

        // Positioner encoder
        positionerEncoder = new DutyCycleEncoder(0);
        positionerEncoder.reset();

        // Invert Bottom motor
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        rightEncoder.setPositionConversionFactor(1);
        leftEncoder.setPositionConversionFactor(1);
        rightEncoder.setVelocityConversionFactor(1);
        leftEncoder.setVelocityConversionFactor(1);

        // PID's
        rightPidController = rightMotor.getPIDController();
        rightPidController.setP(ShooterPositionerConstants.rightP);
        rightPidController.setI(ShooterPositionerConstants.rightI);
        rightPidController.setD(ShooterPositionerConstants.rightD);
        rightPidController.setFF(ShooterPositionerConstants.rightF);

        leftPidController = leftMotor.getPIDController();
        leftPidController.setP(ShooterPositionerConstants.leftP);
        leftPidController.setI(ShooterPositionerConstants.leftI);
        leftPidController.setD(ShooterPositionerConstants.leftD);
        leftPidController.setFF(ShooterPositionerConstants.leftF);

        pidController = new PIDController(ShooterPositionerConstants.kP, ShooterPositionerConstants.kI, ShooterPositionerConstants.kD);
        pidController.setTolerance(5, 10);
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getEncoderDistance() {
        return positionerEncoder.getDistance();
    }

    public void setPercentage(final double leftPercentage, final double rightPercentage) {
        rightMotor.set(rightPercentage);
        leftMotor.set(leftPercentage);

    }

    public void setVelocity(final double leftRpms, final double rightRpms) {
        rightPidController.setReference(rightRpms, ControlType.kVelocity);
        leftPidController.setReference(leftRpms, ControlType.kVelocity);

    }

    public double goToPosition(double setPoint) {
        double percentage = -pidController.calculate(getEncoderDistance(), setPoint);
        rightMotor.set(percentage);
        leftMotor.set(percentage);

        return percentage;
    }

    public void resetEncoder() {
        positionerEncoder.reset();
    }


}
