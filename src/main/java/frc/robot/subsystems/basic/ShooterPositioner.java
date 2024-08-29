package frc.robot.subsystems.basic;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.ShooterPositionerConstants;
import frc.robot.constants.ShooterPoseObject;
import frc.robot.subsystems.Sensors.Limelight;


public class ShooterPositioner {
    
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;
    private final DutyCycleEncoder positionerEncoder;

    private final SparkPIDController rightPidController;
    private final SparkPIDController leftPidController;
    private final PIDController pidController;

    private final Limelight limelight = new Limelight();

    List<ShooterPoseObject<Double, Double, Double>> estimatedShooterPoseList;

    public ShooterPositioner(final int rightId, final int leftId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterPositioner");
        shooterTab.addDouble("Right Velocity (RPMs)", () -> {return getRightVelocity();});
        shooterTab.addDouble("Left Velocity (RPMs)", () -> {return getLeftVelocity();});
        shooterTab.addDouble("Encoder distance", () -> {return getEncoderPositionInDegrees();});

        shooterTab.addDouble("Limelight Y: ", () -> {return limelight.getY();});
        shooterTab.addDouble("Estimated Pose: ", () -> {return getEstimatedShooterPositionerAngleWithRect();});
        

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
        pidController.setTolerance(2, 10);

        // Estimated angle acording to the pose range
        estimatedShooterPoseList = new ArrayList<>();
            estimatedShooterPoseList.add(new ShooterPoseObject<>(1.0, 6.0, 30.0));
            estimatedShooterPoseList.add(new ShooterPoseObject<>(6.0, 8.0, 25.0));
            estimatedShooterPoseList.add(new ShooterPoseObject<>(8.0, 10.0, 20.0));
            estimatedShooterPoseList.add(new ShooterPoseObject<>(10.0, 12.0, 17.0));
            estimatedShooterPoseList.add(new ShooterPoseObject<>(17.0, 19.0, 16.0));

    }

    public double getEstimatedShooterPositionerAngle() {
        double position = 40.0;

        // Compare all the posible distances with a structure
        for (ShooterPoseObject<Double, Double, Double> item : estimatedShooterPoseList) {
            // invert the limelight read to compare it
            if (-limelight.getY() < item.getMaxDistance() && -limelight.getY() > item.getMinDistance()) {
                position = item.getEstimatedAngle().doubleValue();
            }
        }

        return position;
    }

    public double getEstimatedShooterPositionerAngleWithRect() {
        return -(14.0/16.0) * (-limelight.getY() - 1.0) + 30.0;
    }

    public void goToEstimatedShooterPositionAngle() {
        goToPosition(getEstimatedShooterPositionerAngleWithRect());

    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getEncoderPositionInRot() {
        double positionEncoder = -positionerEncoder.getAbsolutePosition(); // invert encoder 
        // set 0 position
        positionEncoder += ShooterPositionerConstants.zeroPosition;
        return positionEncoder;
    }

    public double getEncoderPositionInDegrees() {
        return getEncoderPositionInRot() * 360;
    }

    public void setPercentage(final double leftPercentage, final double rightPercentage) {
        // set limits
        if (getEncoderPositionInDegrees() < ShooterPositionerConstants.downLimitInDegrees && leftPercentage < 0 || getEncoderPositionInDegrees() > ShooterPositionerConstants.upLimitInDegrees && leftPercentage > 0) {
            rightMotor.set(0.0);
            leftMotor.set(0.0);
        }else {
            rightMotor.set(rightPercentage);
            leftMotor.set(leftPercentage);
        }
    }

    public void setRawPercentage(final double leftPercentage, final double rightPercentage) {
        rightMotor.set(rightPercentage);
        leftMotor.set(leftPercentage);
    }

    public void setVelocity(final double leftRpms, final double rightRpms) {
        rightPidController.setReference(rightRpms, ControlType.kVelocity);
        leftPidController.setReference(leftRpms, ControlType.kVelocity);

    }

    public boolean isAtSetPoint(double setPoint) {
        if (setPoint + ShooterPositionerConstants.setPointTolerance > getEncoderPositionInDegrees() && getEncoderPositionInDegrees() > setPoint - ShooterPositionerConstants.setPointTolerance) {
            return true;
        }else {
            return false;
        }
    }

    public double goToPosition(double setPoint) {
        // PID calculation
        double percentage = pidController.calculate(getEncoderPositionInDegrees(), setPoint);

        // Positioner limits
         if (getEncoderPositionInDegrees() < ShooterPositionerConstants.downLimitInDegrees && percentage < 0 || getEncoderPositionInDegrees() > ShooterPositionerConstants.upLimitInDegrees && percentage > 0) {
            rightMotor.set(0.0);
            leftMotor.set(0.0);
        }else {
            rightMotor.set(percentage);
            leftMotor.set(percentage);
        }

        return percentage;
    }

    public void resetEncoder() {
        positionerEncoder.reset();
    }


}
