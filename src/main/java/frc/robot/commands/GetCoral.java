package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;

public class GetCoral extends SequentialCommandGroup {
  public GetCoral(DriveSubsystem drive, HandlerSubsystem handler, Crane crane,
      FieldPoseUtil fieldPoseUtil, CoralStationSubPose subPose) {
    addRequirements(drive, handler, crane);

    addCommands(
      Commands.parallel(
        Commands.defer((() -> {return new DriveToPose(
          fieldPoseUtil.getTargetPoseAtStation(
            fieldPoseUtil.closestStation(drive.getPose()), subPose),
          drive);}), Set.of(drive)),
        Commands.runOnce(() -> crane.moveTo(CraneConstants.kPositionIntake), crane)
      ),
      Commands.runOnce(() -> handler.intakeCoral(), handler),
      Commands.waitUntil(() -> handler.isLoadedCoral()),
      Commands.runOnce(() -> crane.moveTo(CraneConstants.kPositionHome), crane)   
    );
  }
}