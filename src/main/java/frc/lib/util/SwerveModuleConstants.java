package frc.lib.util;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int steerMotorID;
  public final int encoderID;
  public final double steerOffset;

  /**
   * Swerve Module Constants to be used when Creating a Swerve Module.
   *
   * @param driveMotorID
   * @param steerMotorID
   * @param encoderID
   * @param steerOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int steerMotorID, int encoderID, double steerOffset) {
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.encoderID = encoderID;
    this.steerOffset = steerOffset;
  }
}
