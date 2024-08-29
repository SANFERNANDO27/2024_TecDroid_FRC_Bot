package frc.robot.subsystems.basic;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.subsystems.Sensors.Limelight;
import frc.robot.constants.ShooterPoseObject;


public class Shooter extends SubsystemBase{
    
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private final SparkPIDController bottomPidController;
    private final SparkPIDController topPidController;

    private final Limelight limelight = new Limelight();
    private final List<ShooterPoseObject<Double, Double, Double>> estimatedShooterVelocityList;

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

        // Estimated velocity acording to the pose range
        estimatedShooterVelocityList = new ArrayList<>();
            estimatedShooterVelocityList.add(new ShooterPoseObject<>(1.0, 6.0, 1500.0));
            estimatedShooterVelocityList.add(new ShooterPoseObject<>(6.0, 12.5, 2000.0));
            estimatedShooterVelocityList.add(new ShooterPoseObject<>(12.5, 20.0, 2000.0));
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

    public boolean isAtSetPoint(double setPointRpms) {
        if (getTopVelocity() > setPointRpms - ShooterConstants.setpointRpmsTolerance
            && getBottomVelocity() > setPointRpms - ShooterConstants.setpointRpmsTolerance) {
            return true;
        }else {
            return false;
        }
    }

    public void setVelocity(final double topRpms, final double bottomRpms) {
        bottomPidController.setReference(bottomRpms, ControlType.kVelocity);
        topPidController.setReference(topRpms, ControlType.kVelocity);

    }

    public double getEstimatedShooterVelocity() {
        double velocity = 2000;

        // Compare all the posible distances with a structure
        for (ShooterPoseObject<Double, Double, Double> item : estimatedShooterVelocityList) {
            // invert the limelight read to compare it
            if (-limelight.getY() < item.getMaxDistance() && -limelight.getY() > item.getMinDistance()) {
                velocity = item.getEstimatedAngle().doubleValue();
            }
        }

        return velocity;
    }

}
