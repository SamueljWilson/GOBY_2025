// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HandlerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralPlacement;
import frc.robot.commands.FaceReef;
import frc.robot.commands.FaceStation;
import frc.robot.commands.GetAlgae;
import frc.robot.commands.GetCoral;
import frc.robot.commands.LevelOnePlacement;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final CameraSubsystem m_cameraSystem =
    CameraConstants.kEnable ? new CameraSubsystem(CameraConstants.kCameraConfigs) : null;
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_cameraSystem);
  private final HandlerSubsystem m_handler = new HandlerSubsystem(
    HandlerConstants.kMotorID, HandlerConstants.kmotorConfig);
  private final Crane m_crane = new Crane();
  private final ClimberSubsystem m_climber = ClimberConstants.kEnable ? new ClimberSubsystem() : null;
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);
  FieldPoseUtil m_fieldPoseUtil = new FieldPoseUtil();
  double m_reverseFactor = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;
  boolean m_fieldRelative = true;
  double m_speedFactor = 1.0;

  private static double joystickTransform(double value) {
    double transformedValue = MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
    if (DriveConstants.kSquareInputs) {
      transformedValue = transformedValue * Math.abs(transformedValue);
    }
    return transformedValue;
  }

  private double getXSpeedInput() {
    double robotRelativeReverseFactor = m_fieldRelative ? 1.0 : -1.0;
    // The Y axis on the controller is inverted! Pushing forward (up) generates a negative value.
    // Negate the input value here.
    return m_reverseFactor
      * joystickTransform(-m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis))
      * OIConstants.kMaxMetersPerSec
      * m_speedFactor
      * robotRelativeReverseFactor;
  }

  private double getYSpeedInput() {
    double robotRelativeReverseFactor = m_fieldRelative ? 1.0 : -1.0;
    // The X axis on the controller behaves as expected (right is positive), but we're using it for
    // Y axis control, where left is positive. Negate the input value here.
    return m_reverseFactor
      * joystickTransform(-m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis))
      * OIConstants.kMaxMetersPerSec
      * m_speedFactor
      * robotRelativeReverseFactor;
  }

  private double getRotationSpeedInput() {
    // Moving the joystick to the right causes positive input, which we negate in order to rotate
    // clockwise.
    return -joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis))
      * OIConstants.kMaxRadPerSec
      * m_speedFactor;
  }

  private double getClimbInput() {
    return joystickTransform(-m_operatorController.getRawAxis(OIConstants.kLeftJoyYAxis));
  }

  public RobotContainer() {
    configureBindings();

    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> {
          m_robotDrive.drive(
            getXSpeedInput(),
            getYSpeedInput(),
            getRotationSpeedInput(),
            m_fieldRelative);
        }, m_robotDrive
      )
    );

    if (ClimberConstants.kEnable) {
      m_climber.setDefaultCommand(
        new RunCommand(
          () -> {
            m_climber.setSpeed(
              getClimbInput() * ClimberConstants.kMaxSpeedRadiansPerSecond
            );
          }, m_climber
        )
      );
    }

    NamedCommands.registerCommand("Place A2",
      new CoralPlacement(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil,
      ReefSubPose.A, CraneConstants.kPositionL2));
    NamedCommands.registerCommand("Place B2",
      new CoralPlacement(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil,
      ReefSubPose.B, CraneConstants.kPositionL2));
    NamedCommands.registerCommand("Place B3",
      new CoralPlacement(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil,
      ReefSubPose.B, CraneConstants.kPositionL3));
    NamedCommands.registerCommand("GetAlgae",
      new GetAlgae(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil));
    NamedCommands.registerCommand("Eject",
      Commands.sequence(
        Commands.runOnce(() -> {m_handler.eject();}, m_handler),
        Commands.waitUntil(() -> m_handler.isEmpty())
      )
    );
    NamedCommands.registerCommand("Intake S5",
      new GetCoral(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil, CoralStationSubPose.FIVE));
    m_chooser.setDefaultOption("Empty Auto", new PathPlannerAuto("Empty Auto"));
    m_chooser.addOption("Right Cross The Line", new PathPlannerAuto("Right Cross The Line"));
    m_chooser.addOption("Middle Cross The Line", new PathPlannerAuto("Middle Cross The Line"));
    m_chooser.addOption("Left Cross The Line", new PathPlannerAuto("Left Cross The Line"));
    m_chooser.addOption("Right 2A2", new PathPlannerAuto("Right 2A2"));
    m_chooser.addOption("Left 10B2", new PathPlannerAuto("Left 10B2"));
    m_chooser.addOption("Left 10B2 8B3", new PathPlannerAuto("Left 10B2 8B3"));
    m_chooser.addOption("Left 10B2 10B3", new PathPlannerAuto("Left 10B2 10B3"));
    SmartDashboard.putData(m_chooser);
  }

  public void initializePreloaded() {
    m_handler.initializePreloaded();
  }

  public void setPIDSlotID(ClosedLoopSlot slotID) {
    m_robotDrive.setPIDSlotID(slotID);
  }

  private boolean getFieldRelative() {
    return m_fieldRelative;
  }

  private void setFieldRelative(boolean fieldRelative) {
    m_fieldRelative = fieldRelative;
  }

  private void setSpeedFactor(double speedFactor) {
    m_speedFactor = speedFactor;
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, OIConstants.kZeroGyro)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> {
        m_robotDrive.zeroGyro();
      }, m_robotDrive
    ));

    // Use m_robotDrive requirement as synchronizer for m_fieldRelative.
    new JoystickButton(m_driverController, OIConstants.kRobotRelativeButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> setFieldRelative(false), m_robotDrive))
      .onFalse(Commands.runOnce(() -> setFieldRelative(true), m_robotDrive));

    // Use m_robotDrive requirement as synchronizer for m_speedFactor.
    new JoystickButton(m_driverController, OIConstants.kSlowModeButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> setSpeedFactor(DriveConstants.kSlowSpeedFactor), m_robotDrive))
      .onFalse(Commands.runOnce(() -> setSpeedFactor(1.0), m_robotDrive));

    // Face reef button
    new JoystickButton(m_driverController, OIConstants.kFaceReefButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new FaceReef(
        m_robotDrive,
        () -> getXSpeedInput(),
        () -> getYSpeedInput(),
        () -> getFieldRelative(),
        m_fieldPoseUtil));

    // Face coral station button
    new JoystickButton(m_driverController, OIConstants.kFaceCoralStationButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new FaceStation(
        m_robotDrive,
        () -> getXSpeedInput(),
        () -> getYSpeedInput(),
        () -> getFieldRelative(),
        m_fieldPoseUtil));

    // Coral station slot two intake button
    new Trigger(() ->
        m_operatorController.getPOV() == OIConstants.kCoralIntake2 &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(
        new GetCoral(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil, CoralStationSubPose.TWO));

    // Coral station slot five intake button
    new Trigger(() ->
        m_operatorController.getPOV() == OIConstants.kCoralIntake5 &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(
        new GetCoral(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil, CoralStationSubPose.FIVE));

    // Coral station slot eight intake button
    new Trigger(() ->
        m_operatorController.getPOV() == OIConstants.kCoralIntake8 &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(
        new GetCoral(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil, CoralStationSubPose.EIGHT));

    // Manual crane to level one
    new JoystickButton(m_operatorController, OIConstants.kReadyClimbButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() ->
        m_crane.moveTo(CraneConstants.kPositionL1a), m_crane));

    // Auto place Level 1
    new Trigger(() -> 
        m_operatorController.getRawButton(OIConstants.kReadyClimbButton) && 
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new LevelOnePlacement(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil, ReefSubPose.ALGAE));

    // Manual intake coral
    new JoystickButton(m_operatorController, OIConstants.kManualIntake)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() ->
        m_handler.intakeCoral(), m_handler))
      .onFalse(Commands.runOnce(() ->
        m_handler.cancelIntake(), m_handler));

    // Manual crane home
    new JoystickButton(m_operatorController, OIConstants.kHomeCraneButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() ->
        m_crane.moveTo(CraneConstants.kPositionHome), m_crane));

    // Automatic level two A placement
    new Trigger(() ->
        m_operatorController.getRawButton(OIConstants.kASideButton) &&
        m_operatorController.getRawButton(OIConstants.kLevel2Button) &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new CoralPlacement(m_robotDrive,
        m_handler, m_crane, m_fieldPoseUtil, ReefSubPose.A, CraneConstants.kPositionL2));

    // Automatic level two B placement
    new Trigger(() ->
        m_operatorController.getRawButton(OIConstants.kBSideButton) &&
        m_operatorController.getRawButton(OIConstants.kLevel2Button) &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new CoralPlacement(m_robotDrive,
        m_handler, m_crane, m_fieldPoseUtil, ReefSubPose.B, CraneConstants.kPositionL2));

    // Automatic level three A placement
    new Trigger(() ->
        m_operatorController.getRawButton(OIConstants.kASideButton) &&
        m_operatorController.getRawButton(OIConstants.kLevel3Button) &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new CoralPlacement(m_robotDrive,
        m_handler, m_crane, m_fieldPoseUtil, ReefSubPose.A, CraneConstants.kPositionL3));

    // Automatic level three B placement
    new Trigger(() ->
        m_operatorController.getRawButton(OIConstants.kBSideButton) &&
        m_operatorController.getRawButton(OIConstants.kLevel3Button) &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new CoralPlacement(m_robotDrive,
        m_handler, m_crane, m_fieldPoseUtil, ReefSubPose.B, CraneConstants.kPositionL3));

    // Auto Algae
    new Trigger(() ->
        m_operatorController.getPOV() == OIConstants.kAlgaePOV &&
        m_driverController.getRawButton(OIConstants.kAutoDriveButton))
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(new GetAlgae(m_robotDrive, m_handler, m_crane, m_fieldPoseUtil));

    // Eject
    new JoystickButton(m_operatorController, OIConstants.kEjectButton)
      .debounce(OIConstants.kDebounceSeconds)
      .whileTrue(Commands.runOnce(() -> {
        m_handler.eject();
      }, m_handler));
  }

  public Command getAutonomousCommand() {
    Command selectedCommand = m_chooser.getSelected();
    return selectedCommand;
  }
}