package frc.robot.commands.autonomousPathCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.io.IOException;
import java.nio.file.Path;

public class SwerveAutoPathCommand {
  private SwerveDrive swerveDrive;
  private TrajectoryConfig config;
  private Trajectory trajectory;
  private SwerveControllerCommand swerveCommand;

  public SwerveAutoPathCommand(SwerveDrive swerveDrive, String trajectoryJSON) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    this.swerveDrive = swerveDrive;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  public SwerveControllerCommand getSwerveControllerCommand() {
    Constraints constraints =
        new TrapezoidProfile.Constraints(
            SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            2 * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    return this.swerveCommand =
        new SwerveControllerCommand(
            trajectory,
            this.swerveDrive::getPose2d,
            this.swerveDrive.getKinematics(),
            new PIDController(SwerveConstants.kP, SwerveConstants.kI, SwerveConstants.kD),
            new PIDController(SwerveConstants.kP, SwerveConstants.kI, SwerveConstants.kD),
            new ProfiledPIDController(
                SwerveConstants.ThetaKP,
                SwerveConstants.ThetaKI,
                SwerveConstants.ThetaKD,
                constraints),
            swerveDrive::setDesiredStates,
            this.swerveDrive);
  }

  public void resetOdometryToPathStart() {
    this.swerveDrive.resetOdometry(this.trajectory.getInitialPose());
  }
}
