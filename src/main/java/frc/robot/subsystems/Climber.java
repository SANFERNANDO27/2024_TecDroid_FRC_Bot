package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.ClimberConstants;

public class Climber {
    
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkPIDController rightPidController;
    private final SparkPIDController leftPidController;

    public Climber(final int rightId, final int leftId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Climber");
        shooterTab.addDouble("Right Position (RPMs)", () -> {return getRightPosition();});
        shooterTab.addDouble("Left Position (RPMs)", () -> {return getLeftPosition();});

        rightMotor = new CANSparkMax(rightId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(leftId, MotorType.kBrushless);

        rightMotor.setInverted(true);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        // Reset encoders position
        rightEncoder.setPosition(0.0);
        leftEncoder.setPosition(0.0);

        rightEncoder.setPositionConversionFactor(1);
        leftEncoder.setPositionConversionFactor(1);
        rightEncoder.setVelocityConversionFactor(1);
        leftEncoder.setVelocityConversionFactor(1);

        // PID's
        rightPidController = rightMotor.getPIDController();
        rightPidController.setP(ClimberConstants.rightP);
        rightPidController.setI(ClimberConstants.rightI);
        rightPidController.setD(ClimberConstants.rightD);
        rightPidController.setFF(ClimberConstants.rightF);

        leftPidController = leftMotor.getPIDController();
        leftPidController.setP(ClimberConstants.leftP);
        leftPidController.setI(ClimberConstants.leftI);
        leftPidController.setD(ClimberConstants.leftD);
        leftPidController.setFF(ClimberConstants.leftF);

    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getLeftPosition() {
        // invert encoder
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        // invert encoder
        return rightEncoder.getPosition();
    }

    public void setPercentage(final double leftPercentage, final double rightPercentage) {
        // Right limits 
        if (getRightPosition() < ClimberConstants.downLimit && rightPercentage < 0 || getRightPosition() > ClimberConstants.upLimit && rightPercentage > 0) {
            rightMotor.set(0.0);
        }else {
            rightPidController.setReference(rightPercentage, ControlType.kVelocity);
        }
        
        // Left limits 
        if (getLeftPosition() < ClimberConstants.downLimit && leftPercentage < 0 || getLeftPosition() > ClimberConstants.upLimit && leftPercentage > 0) {
            leftMotor.set(0.0);
        }else {
            leftPidController.setReference(leftPercentage, ControlType.kVelocity);
        }

    }

    public void setVelocity(final double leftRpms, final double rightRpms) {
        // Right limits
        if (getRightPosition() < ClimberConstants.downLimit && rightRpms < 0 || getRightPosition() > ClimberConstants.upLimit && rightRpms > 0) {
            rightMotor.set(0.0);
        }else {
            rightPidController.setReference(rightRpms, ControlType.kVelocity);
        }

        // Left limits
        if (getLeftPosition() < ClimberConstants.downLimit && leftRpms < 0 || getLeftPosition() > ClimberConstants.upLimit && leftRpms > 0) {
            leftMotor.set(0.0);
        }else {
            leftPidController.setReference(leftRpms, ControlType.kVelocity);
        }
    }


}
