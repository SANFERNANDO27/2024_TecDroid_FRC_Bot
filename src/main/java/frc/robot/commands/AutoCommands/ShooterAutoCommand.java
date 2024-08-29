package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;
import frc.robot.subsystems.drivetrain.SwerveAlignToAprilTag;

public class ShooterAutoCommand extends Command {

    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private final LimitSwitches limitSwitches;
    private final SwerveAlignToAprilTag swerveAlignToAprilTag;

    public ShooterAutoCommand(Indexer indexer, Intake intake, Shooter shooter, LimitSwitches limitSwitches, SwerveAlignToAprilTag swerveAlignToAprilTag) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.limitSwitches = limitSwitches;
        this.swerveAlignToAprilTag = swerveAlignToAprilTag;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setVelocity(ShooterConstants.shootVelocity, ShooterConstants.shootVelocity);

        // Align to Apriltag
        swerveAlignToAprilTag.RotateAtAprilTagDetectionSetPoint(0);

        if (shooter.isAtSetPoint(ShooterConstants.shootVelocity)) {
            // Intake
            intake.setVelocity(IntakeConstants.CONFIGURED_SETPOINT);

            // Indexer
            indexer.setVelocity(IndexerConstants.CONFIGURED_SETPOINT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercentage(0);
        intake.setPercentage(0);
        shooter.setPercentage(0, 0);
    }

    @Override
    public boolean isFinished() {
        return !limitSwitches.getLimitSwitchesRead();
    }
}