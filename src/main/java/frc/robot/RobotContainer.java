// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.commands.PathPlannerAuto;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo")); // Init drivebase from config
  final CommandXboxController driverXbox = new CommandXboxController(0); // Just because
  final ElevatorSubsystem m_elevator = new ElevatorSubsystem(); // Init elevator in container
  final ShooterSubsystem m_shooter = new ShooterSubsystem(); // Init shooter in container
  private final SendableChooser<Command> autoChooser; // Init auto chooser
  private final double maxStrafeSpeed = 0.2; // maxStrafe Speed for crabwalk

  UsbCamera camera1;
  public RobotContainer()
  {
    // Get our auto commands.
    pathplannerCommands();

    // Nice to have so if testing one thing it doesn't spam DS
    // enable if you are actually driving so that you will know if -
    // something got disconnected
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the autochooser for the driver station dashboard
    autoChooser = AutoBuilder.buildAutoChooser("New Path");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Forward port 5800 for photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Configure the trigger bindings
    configureBindings();

    // Start camera capture
    camera1 = CameraServer.startAutomaticCapture(0);
    // Keep the camera feed open
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  private void configureBindings() {
    // set our drive mode
    setDriveMode();

    /*
     * Driver controls
     */
    Constants.driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    Constants.driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    Constants.driverController.rightBumper().onTrue(Commands.none());
    // Constants.driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // Constants.driverController.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));

    // "Crabwalk" || Left/Right move (Makes lining up easier by 100 times)
    // Do note this is robot-relative
              //------//
    // Make a new Trigger with the boolean supplier of the controllers trigger axis to make sure we actually pressed them
    new Trigger(() -> Constants.driverController.getLeftTriggerAxis() > 0.1 || Constants.driverController.getRightTriggerAxis() > 0.1)
      .whileTrue(new RunCommand(() -> {
        // Get the trigger values
        double leftTrigger = driverXbox.getLeftTriggerAxis();
        double rightTrigger = driverXbox.getRightTriggerAxis();

        // Calculate strafe speed: leftTrigger moves left, rightTrigger moves right
        // (Just determines which way to go by with one is greater)
        // (Right +value and Left -value)
        double strafeSpeed = (rightTrigger - leftTrigger) * maxStrafeSpeed;

        // Robot-relative strafe (no forward/backward movement, no rotation)
        drivebase.drive(new ChassisSpeeds(0, strafeSpeed, 0));
    }, drivebase));

    /*
     * Operator Controls
     */

    // Shooter
    // Constants.operatorController.axisGreaterThan(1,0.25).onTrue(m_shooter.runShooter(0.8)).onFalse(m_shooter.stopShooter()); // Old
    // Constants.operatorController.axisLessThan(1,-0.25).onTrue(m_shooter.runShooter(-0.8)).onFalse(m_shooter.stopShooter()); // Old
    Constants.operatorController.rightTrigger(0.1).onTrue(m_shooter.runShooter(0.8)).onFalse(m_shooter.stopShooter()); // We pressing trigger? If so run shooter until we aren't
    Constants.operatorController.leftTrigger(0.1).onTrue(m_shooter.runShooter(-0.8)).onFalse(m_shooter.stopShooter()); // // We pressing trigger? If so run shooter in rev until we aren't
    
    // Elevator
    Constants.operatorController.leftBumper().onTrue(m_elevator.NewEle("Bottom")); // Left Bumper for Bottom
    Constants.operatorController.povDown().onTrue(m_elevator.NewEle("L1")); // Down on the DPad for L1
    Constants.operatorController.povLeft().onTrue(m_elevator.NewEle("L2")); // Left on the DPad for L2
    Constants.operatorController.povRight().onTrue(m_elevator.NewEle("L3")); // Right on the DPad for L3
    Constants.operatorController.povUp().onTrue(m_elevator.NewEle("L4")); // Up on the DPad for L4
    // Elevator Manual
    Constants.operatorController.y().onTrue(m_elevator.ManualRun(1)).onFalse(m_elevator.Stop()); // Manual up for Elevator (No limit switches)
    Constants.operatorController.a().onTrue(m_elevator.ManualRun(-0.8)).onFalse(m_elevator.Stop()); // Manual down for Elevator (No limit switches) 

    // AprilTags 
    // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf (pg 2 for map)
    /*
     * As much as I liked this code and "had fun" making it we have decided to not use apriltags -
     * but I am going to keep this here for furture reference
    if (Constants.alliance.isPresent() && Constants.alliance.get() == Alliance.Blue) {
      Constants.operatorController.b().whileTrue(drivebase.aimAndDrive(17, 0, .2)); // Track close right (17/8) 
      Constants.operatorController.a().whileTrue(drivebase.aimAndDrive(18, 0, .2));; // Track close mid (18/7)
      Constants.operatorController.x().whileTrue(drivebase.aimAndDrive(19, 0, .2));; // Track close left (19/6)
      Constants.operatorController.y().whileTrue(drivebase.aimAndDrive(22, 0, .2));; // Track far right (22/9)
      Constants.operatorController.rightBumper().whileTrue(drivebase.aimAndDrive(21, 0, .2));; // Track far mid (21/10)
      Constants.operatorController.rightTrigger(0.1).whileTrue(drivebase.aimAndDrive(20, 0, .2));; // Track far left (20/11)
    } else { // Assume Red if not blue or no alliance present
      Constants.operatorController.b().whileTrue(drivebase.aimAndDrive(8, 0, .2)); // Track close right (17/8) 
      Constants.operatorController.a().whileTrue(drivebase.aimAndDrive(7, 0, .2)); // Track close mid (18/7)
      Constants.operatorController.x().whileTrue(drivebase.aimAndDrive(6, 0, .2)); // Track close left (19/6)
      Constants.operatorController.y().whileTrue(drivebase.aimAndDrive(9, 0, .2)); // Track far right (22/9)
      Constants.operatorController.rightBumper().whileTrue(drivebase.aimAndDrive(10, 0, .2)); // Track far mid (21/10)
      Constants.operatorController.rightTrigger(0.1).whileTrue(drivebase.aimAndDrive(11, 0, .2)); // Track far left (20/11)
    }
    */
  }

  /**
   * Register the auto Commands
   */
  public void pathplannerCommands() {
    NamedCommands.registerCommand("ElevatorBottom", m_elevator.runElevBtm());
    NamedCommands.registerCommand("ElevatorL1", m_elevator.runElevL1());
    NamedCommands.registerCommand("ElevatorL2", m_elevator.runElevL2());
    NamedCommands.registerCommand("ElevatorL3", m_elevator.runElevL3());
    NamedCommands.registerCommand("ElevatorL4", m_elevator.runElevL4());
    NamedCommands.registerCommand("ElevatorStop", m_elevator.Stop());
    NamedCommands.registerCommand("RunShooter", m_shooter.runShooter(0.8));
    NamedCommands.registerCommand("StopShooter", m_shooter.stopShooter());
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
    // Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
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