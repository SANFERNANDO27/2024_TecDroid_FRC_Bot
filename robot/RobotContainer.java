// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPositioner;
import frc.robot.subsystems.Drivetrain.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Shooter positioner booleans
  public boolean goToShootPosition = false;
  public boolean goToHomePosition = false;
  public boolean xIsPressed = false;
  public boolean yIsPressed = false;

  private final Shooter shooter;
  private final ShooterPositioner shooterPositioner;
  private final Intake intake;
  private final Indexer indexer;
  private final Climber climber;
  private final XboxController controller = new XboxController(0);
  private Trigger startAndAButton;

  // Swerve
  private final Swerve s_Swerve = new Swerve();
  private final TeleopSwerve teleopSwerve = new TeleopSwerve(
        s_Swerve,
        () -> -controller.getLeftY(), // forward/backward
        () -> controller.getLeftX(), // left/right
        () -> controller.getRightX(), // rotation
        controller::getStartButton // Toggle field-oriented mode with Start button
    );;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
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

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Diego's swerve 
    startAndAButton = new Trigger(() ->
            controller.getStartButton() && controller.getAButton()
        );

    startAndAButton.onTrue(new InstantCommand(s_Swerve::zeroGyro, s_Swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  
  public double setShooterTargetSetPoint() {
    double shooterPercentage = controller.getRightTriggerAxis();
    double rpms = ShooterConstants.CONFIGURED_SETPOINT * shooterPercentage;
    return rpms;
  }

  public double setIntakeAndIndexerTargetSetPoint() {
    double percentage = controller.getLeftTriggerAxis();
    double rpms = IndexerConstants.CONFIGURED_SETPOINT * percentage;
    return rpms;
  }

  public Command getTeleopCommand() {
    return teleopSwerve;
}
  
  public void teleopPeriodic() {
    // Shooter
    double rpmsShooter = setShooterTargetSetPoint();
    shooter.setVelocity(rpmsShooter, rpmsShooter);

    // Shooter positioner
    if (controller.getAButton()) {
      goToShootPosition = true;
      goToHomePosition = false;
    }if (controller.getBButtonPressed()) {
      goToHomePosition = true;
      goToShootPosition = false;
    }else if (controller.getAButtonReleased() || controller.getBButtonReleased()) {
      shooterPositioner.setVelocity(0, 0);

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

    // reset positioner encoder
    /*if (controller.getXButtonPressed()) {
      shooterPositioner.resetEncoder();
    }*/

    // Climber 
    if (controller.getXButtonPressed()) {
      xIsPressed = true;
    } else if (controller.getXButtonReleased()) {
      xIsPressed = false;
    }

    if (controller.getYButtonPressed()) {
      yIsPressed = true;
    } else if (controller.getYButtonReleased()) {
      yIsPressed = false;
    }

    if (xIsPressed && controller.getLeftBumperPressed()) {
      climber.setVelocity(200.0, 0.0);
    } else if (xIsPressed && controller.getRightBumperPressed()) {
      climber.setVelocity(-200.0, 0.0);
    } else if (yIsPressed && controller.getLeftBumperPressed()) {
      climber.setVelocity(0.0, -200.0);
    } else if (yIsPressed && controller.getRightBumperPressed()) {
      climber.setVelocity(0.0, 200.0);
    } else {
      climber.setVelocity(0.0, 0.0);
    }

    // Intake and Indexer
    double percentage = setIntakeAndIndexerTargetSetPoint();

    // Intake
    intake.setVelocity(Constants.IntakeConstants.maxVelocity * percentage);

    // Indexer
    indexer.setVelocity(Constants.IndexerConstants.maxVelocity * percentage);
    
  }
}
