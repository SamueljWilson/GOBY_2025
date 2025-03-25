package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.ReefSubPose;

public class LevelOnePlacement extends SequentialCommandGroup {
  private final HandlerSubsystem m_handler;
  private final DriveSubsystem m_drive;
  private final Crane m_crane;
  private final FieldPoseUtil m_fieldPoseUtil;

  public LevelOnePlacement(DriveSubsystem drive, HandlerSubsystem handler, Crane crane,
    FieldPoseUtil fieldPoseUtil, ReefSubPose subPose) {
    m_handler = handler;
    m_drive = drive;
    m_crane = crane;
    m_fieldPoseUtil = fieldPoseUtil;
    addRequirements(m_drive, m_handler, m_crane);

    addCommands(
      Commands.defer((() -> {
        return new DriveToPose(
          m_fieldPoseUtil.getTargetPoseAtReef(
            m_fieldPoseUtil.closestReefHour(m_drive.getPose()),
            subPose).plus(AutoConstants.kL1ExtraReefOffset),  
          drive);
      }), Set.of(drive)),

      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionL1a)),
      Commands.waitUntil(() -> m_crane.atGoal().isPresent()),
      Commands.runOnce(() -> m_handler.slowEject()),
      Commands.waitUntil(() -> m_handler.isEmpty()),
      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionL1b)),

      Commands.defer((() -> {
        Command driveToPose = new DriveToPose(m_drive.getPose().plus(AutoConstants.kCoralL1PlacementMove), m_drive);
        return driveToPose;
      }), Set.of(drive))
    );
  }
}