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
import frc.robot.constants.Constants.ShooterPositionerConstants;


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
        shooterTab.addDouble("Encoder distance", () -> {return getEncoderPosition();});

        rightMotor = new CANSparkMax(rightId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(leftId, MotorType.kBrushless);

        // Positioner encoder
        positionerEncoder = new DutyCycleEncoder(0);

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

    public double getEncoderPosition() {
        double positionEncoder = -positionerEncoder.getAbsolutePosition(); // invert encoder 
        // set 0 position
        positionEncoder += ShooterPositionerConstants.zeroPosition;
        return positionEncoder;
    }

    public void setPercentage(final double leftPercentage, final double rightPercentage) {
        // set limits
        if (getEncoderPosition() < ShooterPositionerConstants.downLimit && leftPercentage < 0 || getEncoderPosition() > ShooterPositionerConstants.upLimit && leftPercentage > 0) {
            rightMotor.set(0.0);
            leftMotor.set(0.0);
        }else {
            rightMotor.set(rightPercentage);
            leftMotor.set(leftPercentage);
        }
    }

    public void setVelocity(final double leftRpms, final double rightRpms) {
        rightPidController.setReference(rightRpms, ControlType.kVelocity);
        leftPidController.setReference(leftRpms, ControlType.kVelocity);

    }

    public boolean isAtSetPoint(double setPoint) {
        if (setPoint + ShooterPositionerConstants.setPointTolerance > getEncoderPosition() && getEncoderPosition() > setPoint - ShooterPositionerConstants.setPointTolerance) {
            return true;
        }else {
            return false;
        }
    }

    public double goToPosition(double setPoint) {
        // Stop if it's at set point
        if (!isAtSetPoint(setPoint)) {

            // PID calculation
            double percentage = pidController.calculate(getEncoderPosition(), setPoint);

            // Positioner limits
            if (getEncoderPosition() < ShooterPositionerConstants.downLimit && percentage < 0 || getEncoderPosition() > ShooterPositionerConstants.upLimit && percentage > 0) {
                rightMotor.set(0.0);
                leftMotor.set(0.0);
            }else {
                rightMotor.set(percentage);
                leftMotor.set(percentage);
            }

        return percentage;
        } else {
            rightMotor.set(0.0);
            leftMotor.set(0.0);
            return 0.0;
        }
    }

    public void resetEncoder() {
        positionerEncoder.reset();
    }


}
