// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.ElevatorSubsystemEnc; req subsystem code isn't in github
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveInputStream;


public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // final ElevatorSubsystemEnc m_elevatorEnc = new ElevatorSubsystemEnc(); req subsystem code isn't in github
  private final SendableChooser<Command> autoChooser;

  public RobotContainer()
  {
    // Get our auto commands.
    pathplannerCommands();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the autochooser for the driver station dashboard
    autoChooser = AutoBuilder.buildAutoChooser("New Path");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Forward port 5800 for photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    setDriveMode();

    Constants.driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    Constants.driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    Constants.driverController.rightBumper().onTrue(Commands.none());
    // Constants.driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // Constants.driverController.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));

    // Constants.driverController.x().onTrue(m_elevatorEnc.runElev(Constants.ElevatorSetpoints.L1SETPOINT)); // Keep Commeneted, req subsystem code isn't in github

    // Shooter
    Constants.operatorController.axisGreaterThan(1,0.1).onTrue(m_shooter.runShooter(0.25));
    Constants.operatorController.axisLessThan(1,-0.1).onTrue(m_shooter.runShooter(-0.25));
    // Elevator
    Constants.operatorController.rightBumper().onTrue(m_elevator.runElevBtm()); // Left Bumper for Bottom
    Constants.operatorController.povDown().onTrue(m_elevator.runElevL1()); // Down on the DPad for L1
    Constants.operatorController.povLeft().onTrue(m_elevator.runElevL2()); // Left on the DPad for L2
    Constants.operatorController.povRight().onTrue(m_elevator.runElevL3()); // Right on the DPad for L3
    Constants.operatorController.povUp().onTrue(m_elevator.runElevL4()); // Up on the DPad for L4
    // AprilTags 
    // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf (pg 2 for map)
    Constants.operatorController.b(); // Track close right (17/8) 
    Constants.operatorController.a(); // Track close mid (18/7)
    Constants.operatorController.x(); // Track close left (19/6)
    Constants.operatorController.y(); // Track far right (22/9)
    Constants.operatorController.rightBumper(); // Track far mid (21/10)
    Constants.operatorController.rightTrigger(0.1); // Track far left (20/11)
  }

  /**
   * Register the auto Commands
   */
  public void pathplannerCommands() {
    NamedCommands.registerCommand("Elevator", null);
  }

  public Command getAutonomousCommand()
  {
    // Use the auto selected on the dashboard
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /* 
   * ===========================================================================================================
   *                                    We are getting into the swerve area
   * ===========================================================================================================
   */

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1
    )
    .withControllerRotationAxis(driverXbox::getRightX)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
    .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
    .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
      () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
    .headingWhile(true);

  /**
   * Try this to see if I can clear space in configureBindings
   * 
   * @return drivebase.setDefaultCommand
   */
  public void setDriveMode() {
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }
    
    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }
  }
}