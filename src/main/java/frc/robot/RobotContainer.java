
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoCommands.IntakeAndIndexerCommand;
import frc.robot.commands.AutoCommands.ShooterAutoCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.SwerveConstants.OIConstants;
import frc.robot.subsystems.Sensors.LimitSwitches;
import frc.robot.subsystems.basic.Climber;
import frc.robot.subsystems.basic.Indexer;
import frc.robot.subsystems.basic.Intake;
import frc.robot.subsystems.basic.Shooter;
import frc.robot.subsystems.basic.ShooterPositioner;
import frc.robot.subsystems.drivetrain.SwerveAlignToAprilTag;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

      // Shooter positioner booleans
  public boolean goToShootPosition = false;
  public boolean goToHomePosition = false;

  private final Shooter shooter;
  private final ShooterPositioner shooterPositioner;
  private final Intake intake;
  private final Indexer indexer;
  private final Climber climber;
  private final LimitSwitches limitSwitches;
  private final ShuffleboardTab robotContainerTab;
  private final XboxController controller = new XboxController(0);

  // Auto
  private final SwerveAlignToAprilTag swerveAlignToAprilTag;
  private final IntakeAndIndexerCommand intakeAndIndexerCommand;
  private final ShooterAutoCommand shooterAutoCommand;

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true,
                () -> false /*controller.getRightTriggerAxis() > 0.0*/));

        // Shooter
        shooter = new Shooter(Constants.ShooterConstants.BOTTOM_SHOOTER_ID, Constants.ShooterConstants.Top_SHOOTER_ID);

        // Shooter positioner
        shooterPositioner = new ShooterPositioner(Constants.ShooterPositionerConstants.rightPositionerId, Constants.ShooterPositionerConstants.leftPositionerId);
        // Intake
        intake = new Intake(Constants.IntakeConstants.motorId);

        // Indexer
        indexer = new Indexer(Constants.IndexerConstants.motorId);

        // Climber
        climber = new Climber(Constants.ClimberConstants.rightMotorId, Constants.ClimberConstants.leftMotorId);

        // Limit switches
        limitSwitches = new LimitSwitches();

        robotContainerTab = Shuffleboard.getTab("Robot Container");
        robotContainerTab.addBoolean("Limit Switches Read: ", () -> {return limitSwitches.getLimitSwitchesRead();});
        robotContainerTab.addBoolean("Shooter is at set point: ", () -> {return shooter.isAtSetPoint(Constants.ShooterConstants.shootVelocity);});

        // Auto commands and susbsystems
        swerveAlignToAprilTag = new SwerveAlignToAprilTag(swerveSubsystem);
        intakeAndIndexerCommand = new IntakeAndIndexerCommand(indexer, intake, shooter, limitSwitches);
        shooterAutoCommand = new ShooterAutoCommand(indexer, intake, shooter, limitSwitches, swerveAlignToAprilTag);

        // Pathplanner named commands
        NamedCommands.registerCommand("IntakeAndIndexerCommand", intakeAndIndexerCommand);
        NamedCommands.registerCommand("Shooter", shooterAutoCommand);

        // Robot pose
        robotContainerTab.addDouble("X robot position: ", () -> {return swerveSubsystem.getPose().getX();});
        robotContainerTab.addDouble("Y robot position: ", () -> {return swerveSubsystem.getPose().getY();});
        robotContainerTab.addDouble("Theta robot position: ", () -> {return swerveSubsystem.getPose().getRotation().getDegrees();});

        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 2).onTrue(swerveSubsystem.runOnce(swerveSubsystem::zeroHeading));
        new JoystickButton(driverJoytick, 2).onTrue(swerveSubsystem.runOnce(swerveSubsystem::zeroHeading));
    }

    public Command getAutonomousCommand() {
        /* 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        Trajectory testTrayectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(-1.42, 0.0)),
                new Pose2d(-1.42, 1.22, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                testTrayectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        
       ParallelCommandGroup parallelCommandGroup = new ParallelCommandGroup(swerveControllerCommand, intakeAndIndexerCommand);
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(testTrayectory.getInitialPose())),
                parallelCommandGroup,
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                shooterAutoCommand); */

        /*return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.0, 0.0,  new Rotation2d()))),
        new InstantCommand(() -> swerveSubsystem.stopModules()));*/

        // PathPlanner Auto
          
          return new PathPlannerAuto("NotesAuto");
    }

    // Subsystems

  public double setShooterTargetSetPoint() {
    double shooterPercentage = controller.getRightTriggerAxis();
    double rpms = ShooterConstants.shootVelocity * shooterPercentage;
    return rpms;
  }

  public double setIntakeAndIndexerTargetSetPoint() {
    double percentage = controller.getLeftTriggerAxis();
    double rpms = IndexerConstants.CONFIGURED_SETPOINT * percentage;
    return rpms;
  }

  public void shooterPositioner() {
    // Controller actions
    if (controller.getAButton()) {
      goToHomePosition = true;
      goToShootPosition = false;
    }else if (controller.getBButton()) {
      goToHomePosition = false;
      goToShootPosition = true;
    }else if (controller.getPOV() == 90){
      shooterPositioner.setRawPercentage(0.1, 0.1);
    }else if (controller.getPOV() == 180){
      shooterPositioner.setRawPercentage(-0.1, -0.1);
    }else if (controller.getAButtonReleased() || controller.getBButtonReleased() || controller.getPOV() == -1) {
      shooterPositioner.setPercentage(0.0, 0.0);

      goToShootPosition = false;
      goToHomePosition = false;
    }

    /*if (goToShootPosition) {
      double shootPosition = Constants.ShooterPositionerConstants.shootPosition;
      shooterPositioner.goToPosition(shootPosition);

    }else if (goToHomePosition) {
      double homePosition = Constants.ShooterPositionerConstants.home;
      shooterPositioner.goToPosition(homePosition);

    }*/
  }

  public void Climber() {

    if (controller.getXButton() && controller.getLeftBumper()) {
      climber.setVelocity(-Constants.ClimberConstants.velocity, 0.0);
    } else if (controller.getXButton() && controller.getRightBumper()) {
      climber.setVelocity(Constants.ClimberConstants.velocity, 0.0);
    } else if (controller.getYButton() && controller.getLeftBumper()) {
      climber.setVelocity(0.0, -Constants.ClimberConstants.velocity);
    } else if (controller.getYButton() && controller.getRightBumper()) {
      climber.setVelocity(0.0, Constants.ClimberConstants.velocity);
    } else {
      climber.setVelocity(0.0, 0.0);
    }
  }
  
  public void teleopPeriodic() {
    // Reset gyro
    if (controller.getXButtonPressed()) {
      swerveSubsystem.zeroHeading();
    }
    // Intake and Indexer
    double percentage = setIntakeAndIndexerTargetSetPoint();

    if (Constants.GameStrategiesConstants.limitSwitchIntakeIndexerShooter) {
      if (!limitSwitches.getLimitSwitchesRead()) {
        // Intake
        intake.setVelocity(percentage);

        // Indexer
        indexer.setVelocity(percentage);
      } else {
        intake.setPercentage(0);
        indexer.setPercentage(0);
      }

      // Shooter
      double rpmsShooter = setShooterTargetSetPoint();
      shooter.setVelocity(rpmsShooter, rpmsShooter);
      if (shooter.isAtSetPoint(Constants.ShooterConstants.shootVelocity)) {
        // Move intake and indexer when the shooter it's at set point
        indexer.setVelocity(IndexerConstants.CONFIGURED_SETPOINT - 2000);
        intake.setPercentage(IntakeConstants.CONFIGURED_SETPOINT - 2000);
      }

    } else {
      // Intake
      intake.setVelocity(percentage);

      // Indexer
      indexer.setVelocity(percentage);

      // Shooter
      double rpmsShooter = setShooterTargetSetPoint();
      shooter.setVelocity(rpmsShooter, rpmsShooter);
    }

    // Shooter positioner
    shooterPositioner();

    // Climber 
    Climber();
    
  }
}
