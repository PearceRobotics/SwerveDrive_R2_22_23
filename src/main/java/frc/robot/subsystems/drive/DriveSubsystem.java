package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;

public class DriveSubsystem extends SwerveDrive {
  public Command trajectoryFollowerCommand(PathPlannerTrajectory trajectory) {
    Constants.SwerveConstants.kThetaController.enableContinuousInput(-Math.PI, Math.PI);
    return trajectoryFollowerCommand(
        trajectory,
        Constants.SwerveConstants.kXController,
        Constants.SwerveConstants.kYController,
        Constants.SwerveConstants.kThetaController);
  }

  static final SwerveModule frontLeft =
      new PSwerveModule(Constants.SwerveConstants.FrontLeftModule.S_MODULE_CONSTANTS);

  static final SwerveModule frontRight =
      new PSwerveModule(Constants.SwerveConstants.FrontRightModule.S_MODULE_CONSTANTS);

  static final SwerveModule backLeft =
      new PSwerveModule(Constants.SwerveConstants.BackLeftModule.S_MODULE_CONSTANTS);

  static final SwerveModule backRight =
      new PSwerveModule(Constants.SwerveConstants.BackRightModule.S_MODULE_CONSTANTS);

  public DriveSubsystem(AHRS gyro) {
    super(
        frontLeft,
        frontRight,
        backLeft,
        backRight,
        Constants.SwerveConstants.S_DRIVE_KINEMATICS,
        gyro,
        Constants.SwerveConstants.kMaxSpeed);
  }
}