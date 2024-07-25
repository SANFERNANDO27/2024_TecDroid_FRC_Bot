package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeAndIndexerCommand extends Command {

    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private final DigitalInput limitSwitch;

    public IntakeAndIndexerCommand(Indexer indexer, Intake intake, Shooter shooter, DigitalInput limitSwitch) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.limitSwitch = limitSwitch;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!limitSwitch.get()) {
            indexer.setVelocity(IndexerConstants.CONFIGURED_SETPOINT);
            intake.setVelocity(IndexerConstants.CONFIGURED_SETPOINT);
        } else {
            indexer.setVelocity(0);
            intake.setVelocity(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercentage(0);
        intake.setPercentage(0);
    }

    @Override
    public boolean isFinished() {
        return limitSwitch.get();
    }
}