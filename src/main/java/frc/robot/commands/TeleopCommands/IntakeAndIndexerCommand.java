package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;

public class IntakeAndIndexerCommand extends Command {

    private final Indexer indexer;
    private final Intake intake;
    private final LimitSwitches limitSwitches;

    public IntakeAndIndexerCommand(Indexer indexer, Intake intake, LimitSwitches limitSwitches) {
        this.indexer = indexer;
        this.intake = intake;
        this.limitSwitches = limitSwitches;

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    public void moveIntakeAndIndexer(Double velocity) {
        intake.setVelocity(velocity);
        indexer.setVelocity(velocity);

    }

    public void stopIntakeAndIndexer() {
        intake.setPercentage(0);
        indexer.setPercentage(0);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPercentage(0);
        indexer.setPercentage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}