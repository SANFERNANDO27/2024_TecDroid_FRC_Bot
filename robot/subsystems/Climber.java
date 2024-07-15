package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ClimberConstants;

public class Climber {
    
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkPIDController rightPidController;
    private final SparkPIDController leftPidController;

    public Climber(final int rightId, final int leftId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Climber");
        shooterTab.addDouble("Right Velocity (RPMs)", () -> {return getBottomVelocity();});
        shooterTab.addDouble("Left Velocity (RPMs)", () -> {return getTopVelocity();});

        rightMotor = new CANSparkMax(rightId, MotorType.kBrushless);
        leftMotor = new CANSparkMax(leftId, MotorType.kBrushless);

        // Invert Bottom motor
        rightMotor.setInverted(true);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

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

    public double getBottomVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getTopVelocity() {
        return leftEncoder.getVelocity();
    }

    public void setPercentage(final double leftPercentage, final double rightPercentage) {
        rightMotor.set(rightPercentage);
        leftMotor.set(leftPercentage);

    }

    public void setVelocity(final double leftRpms, final double rightRpms) {
        rightPidController.setReference(rightRpms, ControlType.kVelocity);
        leftPidController.setReference(leftRpms, ControlType.kVelocity);

    }


}
