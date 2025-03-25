// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CameraSubsystem.CameraConfig;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.Segment;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.TrapezoidalConstraint;

public final class Constants {
  // Scheduling quantum in seconds.
  public static final double kDt = 0.02;

  public static final PersistMode kPersistMode = PersistMode.kNoPersistParameters;

  // Tunable constants are disabled unless this is set to true.
  // Intended to remain false in committed code.
  public static final boolean kEnableTuning = false;

  public static final class LightConstants {
    public static final int kBlinkinPWMInput = 0;
  }

  public static final class DriveConstants {
    public static final boolean kSquareInputs = false;
    public static final double kSlowSpeedFactor = 0.0625;

    public static final double kTrackWidth = 0.629; // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.629; // Distance between front and back wheels on robot

    public static final Vector<N3> stateStdDeviations =
      VecBuilder.fill(0.005, 0.005, Math.toRadians(1.0));
    public static final Vector<N3> visionStdDeviations =
      VecBuilder.fill(0.050, 0.050, Math.toRadians(5.0));

    public static final boolean kGyroReversed = false;

    // If the aggregate velocity of the swerve modules is beneath this threshold, the robot is considered to be at rest,
    // in which case housekeeping such as re-synchronizing the turning encoders may take place.
    public static final double kAggregateSpeedThresholdMetersPerSecond = 0.01;

    // Note that SwerveModuleConstants.kMaxSpeedMedersPerSecond may saturate if this is set too high, in combination with
    // SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond.
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxDecelerationMetersPerSecondSquared = 30.0;

    public static final double kMaxAngularSpeedRadiansPerSecond = 3.0 * Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3.0 * Math.PI;
    public static final double kMaxAngularDecelerationRadiansPerSecondSquared = 9.0 * Math.PI;

    public static final boolean kLimitSpeedByElevatorHeight = false;

    public static final TrapezoidalConstraint kAngularVelocityProfile = new TrapezoidalConstraint(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      () -> kMaxAngularAccelerationRadiansPerSecondSquared,
      () -> kMaxAngularDecelerationRadiansPerSecondSquared
    );
  }

  public static final class SwerveModuleConstants {
    public static final ClosedLoopSlot kAutoPIDFSlotID = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot kTeleopPIDFSlotID = ClosedLoopSlot.kSlot1;
    public static final ClosedLoopSlot kDefaultPIDFSlotID = kAutoPIDFSlotID;

    private static final double kUnsigned_0To1 = 1.0;
    public static final double kAbsoluteSensorDiscontinuityPoint = kUnsigned_0To1;

    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond * 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = Math.max(
      DriveConstants.kMaxAccelerationMetersPerSecondSquared,
      DriveConstants.kMaxDecelerationMetersPerSecondSquared
    );

    public static final double kWheelDiameterMeters = 0.09525;

    public static final double kDriveGearRatio = 6.75;
    public static final double kTurningGearRatio = 150.0 / 7.0;

    // The native position units are motor rotations, but we want meters.
    public static final double kDrivePositionConversionFactor =
      (SwerveModuleConstants.kWheelDiameterMeters * Math.PI)
      / SwerveModuleConstants.kDriveGearRatio;
    // The native velocity units are motor rotations [aka revolutions] per minute (RPM),
    // but we want meters per second.
    public static final double kDriveVelocityConversionFactor =
      kDrivePositionConversionFactor
      / 60.0 /* s */;

    // The native position units are motor rotations, but we want radians.
    public static final double kTurningPositionConversionFactor =
      Units.rotationsToRadians(1.0 / SwerveModuleConstants.kTurningGearRatio);
    // The native velocity units are motor rotations [aka revolutions] per minute (RPM), but we want radians per second.
    public static final double kTurningVelocityConversionFactor =
      kTurningPositionConversionFactor
      / 60.0 /* s */;

    // The NEO's relative encoder can return spurious results during initialization, and there is
    // no synchronous API for setting the encoder value. This presents a conundrum regarding how to
    // get valid/stable output early on. Our heuristic strategy is to set the encoder to zero, and
    // keep reading the encoder value until it is acceptably close to zero. This value can be quite
    // low, but there is the hypothetical danger of encoder jitter causing an infinite loop as the
    // tolerance approches zero.
    public static final double kTurningEncoderStabilizeToleranceRadians =
      Units.degreesToRadians(1.0);

    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kTurningMotorCurrentLimit = 20;

    public static final double kDriveMotorRampRate = 0.1;
    public static final double kTurningMotorRampRate = 0.1;

    public static final long kValueCacheTtlMicroseconds = 15;

    // Tune via SwerveModule.tunable*DrivePIDF rather than iteratively modifying these fields.
    public static final SparkUtil.PIDFSlot kAutoDrivePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.375, 0.0, 0.0, 0.25),
      kAutoPIDFSlotID
    );
    public static final SparkUtil.PIDFSlot kTeleopDrivePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.3, 0.0, 0.0, 0.25),
      kTeleopPIDFSlotID
    );
    public static final SparkUtil.Config kFrontLeftDriveMotorConfig = new SparkUtil.Config(
      kDriveMotorCurrentLimit,
      kDriveMotorRampRate,
      true,
      kDriveVelocityConversionFactor,
      kDrivePositionConversionFactor,
      kMaxSpeedMetersPerSecond,
      kMaxAccelerationMetersPerSecondSquared,
      new ArrayList<>() {{
        add(kAutoDrivePIDFSlot);
        add(kTeleopDrivePIDFSlot);
      }}
    );
    public static final SparkUtil.Config kRearLeftDriveMotorConfig =
      kFrontLeftDriveMotorConfig.withInvert(true);
    public static final SparkUtil.Config kRearRightDriveMotorConfig =
      kFrontLeftDriveMotorConfig.withInvert(true);
    public static final SparkUtil.Config kFrontRightDriveMotorConfig =
      kFrontLeftDriveMotorConfig.withInvert(true);

    // Tune via SwerveModule.tunable*TurningPIDF rather than iteratively modifying these fields.
    public static final SparkUtil.PIDFSlot kAutoTurningPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.375, 0.0, 0.0),
      kAutoPIDFSlotID
    );
    public static final SparkUtil.PIDFSlot kTeleopTurningPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.375, 0.0, 0.0),
      kTeleopPIDFSlotID
    );
    public static final SparkUtil.Config kFrontLeftTurningMotorConfig = new SparkUtil.Config(
      kTurningMotorCurrentLimit,
      kTurningMotorRampRate,
      true,
      kTurningVelocityConversionFactor,
      kTurningPositionConversionFactor,
      kMaxSpeedMetersPerSecond,
      kMaxAccelerationMetersPerSecondSquared,
      new ArrayList<>() {{
        add(kAutoTurningPIDFSlot);
        add(kTeleopTurningPIDFSlot);
      }}
    );
    public static final SparkUtil.Config kRearLeftTurningMotorConfig =
      kFrontLeftTurningMotorConfig.withInvert(true);
    public static final SparkUtil.Config kRearRightTurningMotorConfig =
      kFrontLeftTurningMotorConfig.withInvert(true);
    public static final SparkUtil.Config kFrontRightTurningMotorConfig =
      kFrontLeftTurningMotorConfig.withInvert(true);

    public static final SwerveModule.Config kFrontLeftSwerveConfig = new SwerveModule.Config(
      1, 5, 9,
      kFrontLeftDriveMotorConfig, kFrontLeftTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.766357))
    );
    public static final SwerveModule.Config kRearLeftSwerveConfig = new SwerveModule.Config(
      2, 6, 10,
      kRearLeftDriveMotorConfig, kRearLeftTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.781982))
    );
    public static final SwerveModule.Config kRearRightSwerveConfig = new SwerveModule.Config(
      3, 7, 11,
      kRearRightDriveMotorConfig, kRearRightTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.976562))
    );
    public static final SwerveModule.Config kFrontRightSwerveConfig = new SwerveModule.Config(
      4, 8, 12,
      kFrontRightDriveMotorConfig, kFrontRightTurningMotorConfig,
      false, new Rotation2d(Units.rotationsToRadians(0.831055))
    );
  }

  public static final class HandlerConstants {
    public static final int kMotorID = 17;
    public static final double kIntakeSpeedCoral = 0.5;
    public static final double kEjectSpeedCoral = -0.4;
    public static final double kSlowEjectSpeedCoral = -0.2; //TODO: Tune.
    public static final double kIntakeSpeedAlgae = -0.4;
    public static final double kEjectSpeedAlgae = 0.2;
    public static final double kTestSpeed = 0.0;

    public static final double kEjectDelaySeconds = 0.5;
    public static final int kHoldingCurrentLimit = 15;
    public static final int kNormalCurrentLimit = 40;

    public static final int kAlgaeSensorInput = 0;
    public static final int kBackSensorInput = 3;
    public static final int kFrontSensorInput = 2;
    public static final int kDistanceSensorInput = 1;

    public static final double kDebounceTime = kDt;
    public static final double kAlgaeSensorProxThreshold = 0.06;
    public static final double kBackSensorProxThreshold = 0.01;
    public static final double kFrontSensorProxThreshold = 0.01;
    public static final double kDistanceSensorProxThreshold = 0.08;

    public static final ClosedLoopSlot kPIDFSlotVelocity = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot kPIDFSlotPosition = ClosedLoopSlot.kSlot1;
    public static final PIDF kMotorPIDFVelocity = new PIDF(2.5, 0.0, 50.0, 1.55);
    public static final PIDF kMotorPIDFPosition = new PIDF(2.5, 0.0, 50.0, 1.55);

    public static final SparkUtil.Config kmotorConfig = new SparkUtil.Config(
      kNormalCurrentLimit,
      0.0,
      true,
      1.083e-4,
      0.0065, // 4 in. wheel and 2.5^3 gear reduction
      1.0,
      6.0,
      new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(
          kMotorPIDFVelocity,
          kPIDFSlotVelocity
        ));
        add(new SparkUtil.PIDFSlot(
        kMotorPIDFPosition,
        kPIDFSlotPosition
        ));
      }}
    );
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kMaxRadPerSec = DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxMetersPerSec = DriveConstants.kMaxSpeedMetersPerSecond;

    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kBack = 7;
    public static final int kStart = 8;
    public static final int kLeftJoy = 9;
    public static final int kRightJoy = 10;

    public static final int kLeftJoyXAxis = 0;
    public static final int kLeftJoyYAxis = 1;
    public static final int kLeftTriggerAxis = 2;
    public static final int kRightTriggerAxis = 3;
    public static final int kRightJoyXAxis = 4;
    public static final int kRightJoyYAxis = 5;

    public static final int kZeroGyro = kStart;
    public static final int kFaceReefButton = kY;
    public static final int kFaceCoralStationButton = kX;
    public static final int kFaceProcessorButton = kA;
    public static final int kASideButton = kLeftBumper;
    public static final int kBSideButton = kRightBumper;
    public static final int kLevel1Button = kA;
    public static final int kLevel2Button = kBack;
    public static final int kLevel3Button = kStart;
    public static final int kIntakeCoralButton = kX;
    public static final int kIntakeALgaePOV = 0;
    public static final int kEjectButton = kY;
    public static final int kAlgaePOV = 0;
    public static final int kAlgaeProcessorPOV = 270;
    public static final int kExtendClimberAxis = kLeftTriggerAxis;
    public static final int kRetractClimberAxis = kRightTriggerAxis;
    public static final int kCoralIntake2 = 270;
    public static final int kCoralIntake5 = 180;
    public static final int kCoralIntake8 = 90;
    public static final int kManualOperatorMode = kLeftJoy;
    public static final int kSlowModeButton = kRightBumper;
    public static final int kHomeCraneButton = kRightJoy;
    public static final int kRobotRelativeButton = kLeftJoy;
    public static final int kManualIntake = kB;
    public static final int kAutoDriveButton = kA;

    public static final double kTriggerAcuationValue = 0.5;

    public static final double kDebounceSeconds = 0.01;

    public static final double kJoystickDeadband = 0.00;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants kTranslationHolonomicPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationHolonomicPID = new PIDConstants(5.0, 0.0, 0.0);

    public static final double kFrameToRobotCenter = 0.381;

    public static final double kStingerYOffset = -0.008;

    public static final double kReefXOffset = -(0.306 + kFrameToRobotCenter);
    public static final double kReefBranchOffset = 0.164; // Offset from center.

    public static final double kStationXOffset = -(0.415 + kFrameToRobotCenter);
    public static final double kStationSlotSpacing = 0.202; // Distance between slots.

    public static final double kProcessorXOffset = -(0.222 + kFrameToRobotCenter); // TODO: Tune.

    public static final Transform2d kL1ExtraReefOffset = new Transform2d(0.09, 0.0, Rotation2d.kZero);
    public static final Transform2d kAlgaeIntakeMove = new Transform2d(0.15, 0.0, Rotation2d.kZero);
    public static final Transform2d kCoralL1PlacementMove = new Transform2d(-0.1, 0.0, Rotation2d.kZero);
  }

  public static final class CameraConstants {
    public static final boolean kEnable = true;
    public static final double kMaxAmbiguity = 0.20;
    // Use single-tag estimate if any detected tags exceed max distance.
    public static final double kMaxTagDistance = 3.5;
    public static final List<CameraConfig> kCameraConfigs = List.of(
      new CameraConfig(
        "reefCamera",
        new Transform3d(
          0.1379980, -0.219292, 0.315,
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(21.5))),
        true
      ),
      new CameraConfig(
        "coralStationCamera",
        new Transform3d(
          0.1300060, -0.223631, 0.996,
          new Rotation3d(0.0, Units.degreesToRadians(-22.5), Units.degreesToRadians(9.5))),
        true
      )
    );
  }

  public static final class DriveCommandConstants {
    public static final double kDefaultTranslationPositionToleranceMeters = 0.02;
    public static final double kDefaultTranslationVelocityToleranceMetersPerSecond = Double.POSITIVE_INFINITY;
    public static final double kDefaultAnglePositionToleranceRadians =
      Units.degreesToRadians(2.0);
    public static final double kDefaultAngleVelocityToleranceRadiansPerSecond =
      Double.POSITIVE_INFINITY;

    public static final PIDF kTranslatingPIDF = new PIDF(3.625, 0.0, 0.0, 0.0);
    public static final PIDF kTurningPIDF = new PIDF(4.0, 0.0, 0.0, 0.0);
  }

  public static final class FieldConstants {
    private static final AprilTagFieldLayout loadTransformedAprilTagFieldLayout() {
      final AprilTagFields rawLayoutFile = AprilTagFields.k2025ReefscapeWelded;
      final AprilTagFieldLayout rawLayout = AprilTagFieldLayout.loadField(rawLayoutFile);
      Map<Integer, Rotation3d> rotations = Map.of(
        // In order to adjust an AprilTag's rotation, specify non-zero roll/pitch/yaw, as viewed
        // from perspective of the field XYZ axes. For example, if the blue speaker AprilTag is
        // tilted upward by 2 degrees:
        //
        //   7, new Rotation3d(0.0, Units.degreesToRadians(-2.0), 0.0)
        //
        // The same upward tilt for the red speaker AprilTag would be corrected as such:
        //
        //   4, new Rotation3d(0.0, Units.degreesToRadians(2.0), 0.0)
      );
      List<AprilTag> aprilTags =
        rawLayout.getTags().stream()
        .map(aprilTag -> {
          AprilTag rotatedAprilTag;
          if (rotations.containsKey(aprilTag.ID)) {
            rotatedAprilTag =
              new AprilTag(aprilTag.ID, aprilTag.pose.rotateBy(rotations.get(aprilTag.ID)));
          } else {
            rotatedAprilTag = aprilTag;
          }
          return rotatedAprilTag;
        })
        .toList();
      return new AprilTagFieldLayout(
        aprilTags, rawLayout.getFieldLength(), rawLayout.getFieldWidth());
    }
    public static final AprilTagFieldLayout kAprilTagFieldLayout =
      loadTransformedAprilTagFieldLayout();

    public static final double kMaxX = kAprilTagFieldLayout.getFieldLength();

    private static AprilTag getAprilTag(AprilTagFieldLayout fieldLayout, int aprilTagID) {
      for (AprilTag aprilTag : fieldLayout.getTags()) {
        if (aprilTag.ID == aprilTagID) {
          return aprilTag;
        }
      }
      assert(false);
      return new AprilTag(0, null);
    }

    public static final FieldPoseUtil.AprilTags kBlueAprilTags = new FieldPoseUtil.AprilTags(
      getAprilTag(kAprilTagFieldLayout, 21), // reefTwelve
      getAprilTag(kAprilTagFieldLayout, 22), // reefTwo
      getAprilTag(kAprilTagFieldLayout, 17), // reefFour
      getAprilTag(kAprilTagFieldLayout, 18), // reefSix
      getAprilTag(kAprilTagFieldLayout, 19), // reefEight
      getAprilTag(kAprilTagFieldLayout, 20), // reefTen
      getAprilTag(kAprilTagFieldLayout, 13), // coralStationLeft
      getAprilTag(kAprilTagFieldLayout, 12), // coralStationRight
      getAprilTag(kAprilTagFieldLayout, 16), // processor
      getAprilTag(kAprilTagFieldLayout, 14), // bargeLeft
      getAprilTag(kAprilTagFieldLayout, 15)  // bargeRight
    );
    public static final FieldPoseUtil.AprilTags kRedAprilTags = new FieldPoseUtil.AprilTags(
      getAprilTag(kAprilTagFieldLayout, 10), // reefTwelve
      getAprilTag(kAprilTagFieldLayout, 9), // reefTwo
      getAprilTag(kAprilTagFieldLayout, 8), // reefFour
      getAprilTag(kAprilTagFieldLayout, 7), // reefSix
      getAprilTag(kAprilTagFieldLayout, 6), // reefEight
      getAprilTag(kAprilTagFieldLayout, 11), // reefTen
      getAprilTag(kAprilTagFieldLayout, 1), // coralStationLeft
      getAprilTag(kAprilTagFieldLayout, 2), // coralStationRight
      getAprilTag(kAprilTagFieldLayout, 3), // processor
      getAprilTag(kAprilTagFieldLayout, 5), // bargeLeft
      getAprilTag(kAprilTagFieldLayout, 4)  // bargeRight
    );
  }

  public static final class CraneConstants {
    public static final int kLeftElevatorMotorID = 13;
    public static final int kRightElevatorMotorID = 14;
    public static final int kPivotMotorID = 15;
    public static final int kDistanceSensorInput = 4;

    public static final long kValueCacheTtlMicroseconds = 15;

    // Time required to accelerate from stationary to maximum velocity. Acceleration must
    // be proportional between the pivot and elevator in order to achieve "linear" movement,
    // hence the shared constant. Both pivot and elevator must be capable of this acceleration
    // for the crane to move smoothly.
    public static final double kAccelerationSeconds = 0.25;
    public static final double kPivotMaxSpeedRadiansPerSecond = Math.PI;
    public static final double kPivotMaxAccelerationRadiansPerSecondSquared =
      kPivotMaxSpeedRadiansPerSecond / kAccelerationSeconds; // Do not change.
    public static final double kElevatorMaxSpeedMetersPerSecond = 1.0;
    public static final double kElevatorMaxAcccelerationMetersPerSecondSquared =
      kElevatorMaxSpeedMetersPerSecond / kAccelerationSeconds; // Do not change.

    // rot -> radians: 75:1 gearbox.
    public static final double kPivotPositionConversionFactor = (2.0 * Math.PI) / 75.0;
    public static final SparkUtil.PIDFSlot kPivotMotorVelocityPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.1, 0.0, 0.0, 0.25),
      ClosedLoopSlot.kSlot0
    );
    public static final SparkUtil.PIDFSlot kPivotMotorVoltagePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.0, 0.0, 0.0, 0.0),
      ClosedLoopSlot.kSlot1
    );
    public static final SparkUtil.Config kPivotMotorConfig = new SparkUtil.Config(
      40,
      0.0,
      false,
      kPivotPositionConversionFactor / 60.0, // rot/min -> rad/s
      kPivotPositionConversionFactor,
      kPivotMaxSpeedRadiansPerSecond,
      kPivotMaxAccelerationRadiansPerSecondSquared,
      new ArrayList<>() {{
        add(kPivotMotorVelocityPIDFSlot);
        add(kPivotMotorVoltagePIDFSlot);
      }}
    );

    public static final double kDutyCycleInitDelaySeconds = 2.0;

    public static final double kPivotHomingVoltage = 0.5;
    public static final double kPivotMinStalledHomingAmperage = 20.0;
    public static final double kPivotHomingDebounceSeconds = 0.2;

    public static final int kPivotAbsEncoderChannel = 5;
    public static final boolean kPivotAbsEncoderInverted = true;
    public static final double kPivotAbsEncoderOffsetRadians = Units.degreesToRadians(12.29);
    public static final double kPivotEndoderFlexRadians = Units.degreesToRadians(3.97);

    public static final boolean kInvertLeftElevatorMotor = false;
    // rot -> meters: 12:1 gearbox, 22T sprocket, 1/4" chain link, 2-stage elevator.
    public static final double kElevatorPositionConversionFactor = 0.0233;
    public static final SparkUtil.PIDFSlot kElevatorMotorVelocityPIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.3, 0.0, 0.0, 0.0),
      ClosedLoopSlot.kSlot0
    );
    public static final SparkUtil.PIDFSlot kElevatorMotorVoltagePIDFSlot = new SparkUtil.PIDFSlot(
      new PIDF(0.0, 0.0, 0.0, 0.0),
      ClosedLoopSlot.kSlot1
    );
    public static final SparkUtil.Config kElevatorMotorConfig = new SparkUtil.Config(
      40,
      0.0,
      kInvertLeftElevatorMotor,
      kElevatorPositionConversionFactor / 60.0, // rot/min -> m/s
      kElevatorPositionConversionFactor,
      kElevatorMaxSpeedMetersPerSecond,
      kElevatorMaxAcccelerationMetersPerSecondSquared,
      new ArrayList<>() {{
        add(kElevatorMotorVelocityPIDFSlot);
        add(kElevatorMotorVoltagePIDFSlot);
      }}
    );
    public static final double kElevatorHomingVoltage = -0.5;
    public static final double kElevatorMinStalledHomingAmperage = 20.0;
    public static final double kElevatorHomingDebounceSeconds = kDt;

    // Static/gravity/velocity gain for ElevatorFeedForward.
    public static final double kS = 0.2;
    public static final double kG = 0.32;
    public static final double kV = 7.0;

    public static final Crane.Tolerance kDefaultPivotTolerance = new Crane.Tolerance(
      Units.degreesToRadians(0.5),
      Double.POSITIVE_INFINITY
    );
    public static final Crane.Tolerance kDefaultElevatorTolerance = new Crane.Tolerance(
      0.01,
      Double.POSITIVE_INFINITY
    );

    public static final PIDF kPivotPIDF = new PIDF(5.0, 0.0, 0.0, 0.0);
    public static final PIDF kElevatorPIDF = new PIDF(10.0, 0.0, 0.0, 0.0);

    public static final double kPivotHiMin = Units.degreesToRadians(-90.0);
    public static final double kPivotHomeRapid = Units.degreesToRadians(75.0);
    public static final double kPivotHiMax = Units.degreesToRadians(44.0);
    public static final double kPivotLoMin = Units.degreesToRadians(-15.0);
    public static final double kPivotLoMax = Units.degreesToRadians(90.0);
    public static final double kPivotHome = kPivotLoMax;

    public static final double kElevatorMax = 1.635;
    public static final double kElevatorHardMin = 0.326;
    public static final double kElevatorHomeRapid = kElevatorHardMin + 0.05;
    public static final double kElevatorSoftMin = kElevatorHardMin + 0.02;
    public static final double kElevatorHome = kElevatorSoftMin;
    // Actual elevator height for a distance sensor measurement of 0.
    public static final double kDistanceSensorBaseMeasurement = 0.344;

    // Valid configuration space boundaries.
    public static final Segment kPivotLoBoundary = new Segment(
      new Translation2d(kPivotHiMin, kElevatorMax),
      new Translation2d(kPivotLoMin, kElevatorSoftMin)
    );
    public static final Segment kPivotHiBoundary = new Segment(
      new Translation2d(kPivotHiMax, kElevatorMax),
      new Translation2d(kPivotLoMax, kElevatorSoftMin)
    );
    public static final Segment kElevatorLoBoundary = new Segment(
      new Translation2d(kPivotLoMin, kElevatorSoftMin),
      new Translation2d(kPivotLoMax, kElevatorSoftMin)
    );
    public static final Segment kElevatorHiBoundary = new Segment(
      new Translation2d(kPivotHiMin, kElevatorMax),
      new Translation2d(kPivotHiMax, kElevatorMax)
    );
    public static final ArrayList<Segment> kBoundaries = new ArrayList<>() {{
      add(kPivotLoBoundary);
      add(kPivotHiBoundary);
      add(kElevatorLoBoundary);
      add(kElevatorHiBoundary);
    }};

    public static final Translation2d kPositionHome =
      new Translation2d(kPivotHome, kElevatorHome);
    public static final Translation2d kPositionL1a =
      new Translation2d(Units.degreesToRadians(25.0), 0.36);
    public static final Translation2d kPositionL1b =
      new Translation2d(Units.degreesToRadians(15.0), 0.36);
    public static final Translation2d kPositionL2 =
      new Translation2d(Units.degreesToRadians(-35.0), 1.22);
    public static final Translation2d kPositionL3 =
      new Translation2d(Units.degreesToRadians(-35.0), 1.62);
    public static final Translation2d kPositionLoAlgae =
      new Translation2d(Units.degreesToRadians(-20.0), 0.85);
    public static final Translation2d kPositionHiAlgae =
      new Translation2d(Units.degreesToRadians(-20.0), 1.22);
    public static final Translation2d kPositionIntake =
      new Translation2d(Units.degreesToRadians(35.0), 0.68);
  }

  public static final class ClimberConstants {
    public static final boolean kEnable = true;
    public static final int kLeftMotorID = 16;
    public static final int kRightMotorID = 19;
    public static final double kMaxSpeedRadiansPerSecond = Math.PI / 3;
    public static final double kMaxAccelerationRadiansPerSecondSquared = Math.PI;
    public static final double kMinRange = Math.toRadians(-25.0);
    public static final double kMaxRange = Math.toRadians(105.0);
    public static final double kInitialAngle = 0.0;
    public static final double kAngleTolerance = Math.toRadians(0.02);

    public static final double kGearRatio = 9.0 * 4.0 * 4.0;
    public static final double kAngleConversionFactor = 2.0 * Math.PI / kGearRatio;

    // The native velocity units are motor rotations [aka revolutions] per minute (RPM),
    // but we want meters per second.
    public static final double kVelocityConversionFactor = kAngleConversionFactor / 60.0;

    public static final PIDF kVelocityPIDF = new PIDF(0.2, 0.0 , 0.0, 0.175);

    public static final ClosedLoopSlot kVelocitySlot = ClosedLoopSlot.kSlot0;
    public static final SparkUtil.PIDFSlot kMotorVelocityPIDFSlot = new SparkUtil.PIDFSlot(
      kVelocityPIDF,
      kVelocitySlot
    );

    public static final boolean kInvertLeftMotor = false;
    public static final SparkUtil.Config kMotorConfig = new SparkUtil.Config(
      80,
      0.1,
      kInvertLeftMotor,
      kVelocityConversionFactor,
      kAngleConversionFactor,
      kMaxSpeedRadiansPerSecond,
      kMaxAccelerationRadiansPerSecondSquared,
      new ArrayList<>() {{
        add(kMotorVelocityPIDFSlot);
      }}
    );
  }
}