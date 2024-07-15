package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.IntakeConstants;


public class Intake {
    
    private final CANSparkMax intakeMotor;

    private final RelativeEncoder motorEncoder;

    private final SparkPIDController motorPidController;

    public Intake(final int motorId) {
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Intake");
        shooterTab.addDouble("Velocity (RPMs)", () -> {return getBottomVelocity();});

        intakeMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        intakeMotor.setInverted(true);

        motorEncoder = intakeMotor.getEncoder();

        motorEncoder.setPositionConversionFactor(1);
        motorEncoder.setVelocityConversionFactor(1);

        // PID
        motorPidController = intakeMotor.getPIDController();
        motorPidController.setP(IntakeConstants.motor_P);
        motorPidController.setI(IntakeConstants.motor_I);
        motorPidController.setD(IntakeConstants.motor_D);
        motorPidController.setFF(IntakeConstants.motor_f);

    }

    public double getBottomVelocity() {
        return motorEncoder.getVelocity();
    }

    public void setPercentage(final double motorPercentage) {
        intakeMotor.set(motorPercentage);

    }

    public void setVelocity(final double rpms) {
        motorPidController.setReference(rpms, ControlType.kVelocity);

    }


}
