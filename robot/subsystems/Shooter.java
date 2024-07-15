package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ShooterConstants;


public class Shooter {
    
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private final SparkPIDController bottomPidController;
    private final SparkPIDController topPidController;

    public Shooter(final int bottomId, final int topId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
        shooterTab.addDouble("Bottom Velocity (RPMs)", () -> {return getBottomVelocity();});
        shooterTab.addDouble("Top Velocity (RPMs)", () -> {return getTopVelocity();});

        bottomMotor = new CANSparkMax(bottomId, MotorType.kBrushless);
        topMotor = new CANSparkMax(topId, MotorType.kBrushless);

        // Invert Bottom motor
        bottomMotor.setInverted(true);

        bottomEncoder = bottomMotor.getEncoder();
        topEncoder = topMotor.getEncoder();

        bottomEncoder.setPositionConversionFactor(1);
        topEncoder.setPositionConversionFactor(1);
        bottomEncoder.setVelocityConversionFactor(1);
        topEncoder.setVelocityConversionFactor(1);

        // PID's
        bottomPidController = bottomMotor.getPIDController();
        bottomPidController.setP(ShooterConstants.BOTTOM_P);
        bottomPidController.setI(ShooterConstants.BOTTOM_I);
        bottomPidController.setD(ShooterConstants.BOTTOM_D);
        bottomPidController.setFF(ShooterConstants.BOTTOM_F);

        topPidController = topMotor.getPIDController();
        topPidController.setP(ShooterConstants.TOP_P);
        topPidController.setI(ShooterConstants.TOP_I);
        topPidController.setD(ShooterConstants.TOP_D);
        topPidController.setFF(ShooterConstants.TOP_F);

    }

    public double getBottomVelocity() {
        return bottomEncoder.getVelocity();
    }

    public double getTopVelocity() {
        return topEncoder.getVelocity();
    }

    public void setPercentage(final double topPercentage, final double bottomPercentage) {
        bottomMotor.set(bottomPercentage);
        topMotor.set(topPercentage);

    }

    public void setVelocity(final double topRpms, final double bottomRpms) {
        bottomPidController.setReference(bottomRpms, ControlType.kVelocity);
        topPidController.setReference(topRpms, ControlType.kVelocity);

    }


}
