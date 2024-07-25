
package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeAndIndexerCommand;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IndexerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.SwerveConstants.AutoConstants;
import frc.robot.constants.SwerveConstants.DriveConstants;
import frc.robot.constants.SwerveConstants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPositioner;
import frc.robot.subsystems.drivetrain.SwerveAlignToAprilTag;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.choreo.lib.*;

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
  private final DigitalInput limitSwitch;
  private final Limelight limelight;
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
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true));

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

        // Limit switch
        limitSwitch = new DigitalInput(Constants.LimitSwitchConstants.limitSwitchId);

        // Limelight
        limelight = new Limelight();

        robotContainerTab = Shuffleboard.getTab("Robot Container");
        robotContainerTab.addBoolean("Limit Switch Read: ", () -> {return limitSwitch.get();});
        robotContainerTab.addBoolean("Shooter is at set point: ", () -> {return shooter.isAtSetPoint(Constants.ShooterConstants.shootVelocity);});

        // Auto commands and susbsystems
        swerveAlignToAprilTag = new SwerveAlignToAprilTag(swerveSubsystem, limelight);
        intakeAndIndexerCommand = new IntakeAndIndexerCommand(indexer, intake, shooter, limitSwitch);
        shooterAutoCommand = new ShooterAutoCommand(indexer, intake, shooter, limitSwitch);

        // Robot pose
        robotContainerTab.addDouble("X robot position: ", () -> {return swerveSubsystem.getPose().getX();});
        robotContainerTab.addDouble("Y robot position: ", () -> {return swerveSubsystem.getPose().getY();});
        robotContainerTab.addDouble("Theta robot position: ", () -> {return swerveSubsystem.getPose().getRotation().getDegrees();});
        
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 2).onTrue(swerveSubsystem.runOnce(swerveSubsystem::zeroHeading));
    }

    public Command chorioTrajectoryCommand() {

      // Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      PIDController thetaController = new PIDController(
              AutoConstants.kPThetaController, 0, 0);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      // Choreo trayectory
      BooleanSupplier isInRedAlliance = () -> {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == Alliance.Red;
        };

      ChoreoTrajectory traj = Choreo.getTrajectory("TestTrajectory"); 
      
      return Choreo.choreoSwerveCommand(traj, swerveSubsystem::getPose, xController, yController, thetaController, swerveSubsystem::setChassisSpeed, isInRedAlliance, swerveSubsystem);
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

        // Autonomus using choreo

        ParallelCommandGroup parallelCommandGroup = new ParallelCommandGroup(chorioTrajectoryCommand(), intakeAndIndexerCommand);

        return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.0, 0.0,  new Rotation2d()))),
        chorioTrajectoryCommand(), 
        new InstantCommand(() -> swerveSubsystem.stopModules()));
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

  public void IntakeIndexerTimeBasedAutoSequence() {
      if (!limitSwitch.get()) {
        indexer.setVelocity(IndexerConstants.CONFIGURED_SETPOINT);
        intake.setVelocity(IndexerConstants.CONFIGURED_SETPOINT);
    } else {
        indexer.setPercentage(0);
        intake.setPercentage(0);
    }
  }

  public void shooterTimeBasedAutoSequence() {
      // Intake
      intake.setPercentage(0.0);;

      // Indexer
      indexer.setPercentage(0.0);

      shooter.setVelocity(ShooterConstants.shootVelocity, ShooterConstants.shootVelocity);
      if (shooter.isAtSetPoint(ShooterConstants.shootVelocity)) {
        // Intake
        intake.setVelocity(IntakeConstants.CONFIGURED_SETPOINT);

        // Indexer
        indexer.setVelocity(IndexerConstants.CONFIGURED_SETPOINT);
      }
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
      shooterPositioner.setPercentage(0.1, 0.1);
    }else if (controller.getPOV() == 180){
      shooterPositioner.setPercentage(-0.1, -0.1);
    }else if (controller.getAButtonReleased() || controller.getBButtonReleased() || controller.getPOV() == -1) {
      shooterPositioner.setPercentage(0.0, 0.0);

      goToShootPosition = false;
      goToHomePosition = false;
    }

    if (goToShootPosition) {
      double shootPosition = Constants.ShooterPositionerConstants.shootPosition;
      shooterPositioner.goToPosition(shootPosition);

    }else if (goToHomePosition) {
      double homePosition = Constants.ShooterPositionerConstants.home;
      shooterPositioner.goToPosition(homePosition);

    }
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
    // Intake and Indexer
    double percentage = setIntakeAndIndexerTargetSetPoint();

    if (Constants.GameStrategiesConstants.limitSwitchIntakeIndexerShooter) {
      if (!limitSwitch.get()) {
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
        indexer.setVelocity(IndexerConstants.CONFIGURED_SETPOINT);
        intake.setPercentage(IntakeConstants.CONFIGURED_SETPOINT);
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
