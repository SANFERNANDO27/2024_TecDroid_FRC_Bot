package frc.robot.subsystems.basic;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.IntakeConstants;


public class Indexer {
    
    private final CANSparkMax indexerMotor;

    private final RelativeEncoder motorEncoder;

    private final SparkPIDController motorPidController;

    public Indexer(final int motorId) {
        ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
        indexerTab.addDouble("Velocity (RPMs)", () -> {return getMotorVelocity();});

        indexerMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        indexerMotor.setInverted(true);

        motorEncoder = indexerMotor.getEncoder();

        motorEncoder.setPositionConversionFactor(1);
        motorEncoder.setVelocityConversionFactor(1);

        // PID
        motorPidController = indexerMotor.getPIDController();
        motorPidController.setP(IntakeConstants.motor_P);
        motorPidController.setI(IntakeConstants.motor_I);
        motorPidController.setD(IntakeConstants.motor_D);
        motorPidController.setFF(IntakeConstants.motor_f);

    }

    public double getMotorVelocity() {
        return motorEncoder.getVelocity();
    }

    public void setPercentage(final double motorPercentage) {
        indexerMotor.set(motorPercentage);

    }

    public void setVelocity(final double rpms) {
        motorPidController.setReference(rpms, ControlType.kVelocity);

    }


}
