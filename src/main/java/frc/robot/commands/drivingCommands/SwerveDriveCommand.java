package frc.robot.commands.drivingCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;

public class SwerveDriveCommand extends CommandBase {
  private final SwerveDrive swerveDrive;

  private final /*DoubleSupplier*/ Double m_translationXSupplier;
  private final /*DoubleSupplier*/ Double m_translationYSupplier;
  private final /*DoubleSupplier*/ Double m_rotationSupplier;

  public SwerveDriveCommand(
      SwerveDrive swerveDrive,
      /*DoubleSupplier*/ Double translationXSupplier,
      /*DoubleSupplier*/ Double translationYSupplier,
      /*DoubleSupplier*/ Double rotationSupplier) {
    this.swerveDrive = swerveDrive;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

    addRequirements(swerveDrive);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented
    // movement
    this.swerveDrive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationXSupplier /*.getAsDouble()*/,
            m_translationYSupplier /*.getAsDouble()*/,
            m_rotationSupplier /*.getAsDouble()*/,
            swerveDrive.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
