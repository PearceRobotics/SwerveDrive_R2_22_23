package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PIDGains;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
  public static final int controllerPort = 1;

  public static final class SwerveConstants {

    public static final SimpleMotorFeedforward kDriveFeedforward =
        new SimpleMotorFeedforward(0.0467, 3.3076, 0.01897);

    public static final SimpleMotorFeedforward kTurningFeedforward =
        new SimpleMotorFeedforward(0.35233, 0.39185, 0.0058658);

    /* Drivetrain Constants */
    // The distance between the center of left side module and right side module in inches converted
    // to meters
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    // The distance between the center of front module and back module in inches converted to meters
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    // The diameter of the swerve module wheels in inches converted to meters
    public static final double kWheelDiameter = Units.inchesToMeters(3.94);
    // The circumference of the swerve module wheels in meters
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;

    // conversion factor for steer motor and drive motor
    public static final double kDrivePositionConversionFactor = 0.03841;
    public static final double kSteerPositionConversionFactor = 0.16641971665841623;

    public static final double kDriveVelocityConversionFactor = 6.401640866520717E-4;
    public static final double kSteerVelocityConversionFactor = 0.002773661944306937;

    // open and close loop ramp constants
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    // The Steer and Drive Modules Gear Ratio
    public static final double kSteerGearRatio = (12.8 / 1.0);
    public static final double kDriveGearRatio = (6.75 / 1.0);

    // Swerve Drive Kinematics
    public static final SwerveDriveKinematics S_DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    // Auton path finding controllers
    public static final PIDController kXController = new PIDController(0.100506, 0.0, 0.0);
    public static final PIDController kYController = new PIDController(0.1, 0.0, 0.0);
    public static final PIDController kThetaController = new PIDController(9.0, 0.0, 0.80);

    /* Swerve Current Limiting */
    // The current limit for the drive motors
    public static final int kDriveCurrentLimit = 50;
    public static final int kDriveContinuousCurrentLimit = 35;
    public static final double kDriveCurrentLimitTime = 0.1;
    public static final boolean kDriveCurrentLimitEnable = true;

    // The current limit for the steer motors
    public static final int kSteerCurrentLimit = 40;
    public static final int kSteerContinuousCurrentLimit = 25;
    public static final double kSteerCurrentLimitTime = 0.1;
    public static final boolean kSteerCurrentLimitEnable = true;

    /* Steering Motor PID Values */
    // values started at defualts needs to be tuned, ignore I  TODO: Tune PID Values
    public static PIDGains kSteerMotorPIDGains = new PIDGains(0.001, 0.0, 0.0);

    /* Drive Motor PID Values */
    // values started at defualts needs to be tuned, ignore I  TODO: Tune PID Values
    public static PIDGains kDriveMotorPIDGains = new PIDGains(0.001, 0.0, 0.0);

    public static final TrapezoidProfile.Constraints kTurningConstraints =
        new TrapezoidProfile.Constraints(20, 200);

    /* Drive Motor Characterization values */
    // TODO: Tune Characterization Values

    // The kS value for the drive motor characterization // divide by 12 to convert from volts to
    // percent output
    public static final double kDrivekS = (0.0 / 12.0);
    // The kV value for the drive motor characterization
    public static final double kDrivekV = (0.0 / 12.0);
    // The kA value for the drive motor characterization
    public static final double kDrivekA = (0.0 / 12.0);

    /* Swerve Profiling Values */

    // The max velocity for the swerve drive in meters per second
    public static final double kMaxSpeed = 4.5;
    // The max acceleration for the swerve drive
    public static final double kMaxAngularVelocity = 11.5;

    /* Motor Inverts */
    public static final boolean kDriveMotorInvert = false;
    public static final boolean kSteerMotorInvert = true;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FrontLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 1;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 2;
      // The CAN ID for the steer encoder
      public static final int kSteerEncoderID = 0;
      // The offset for the steer encoder
      public static final double kSteerEncoderOffset =
          0.2205; // TODO: Tune Encoder Offset // was wrong way needs recaculated greater value
      // creating the swerve module constants for the front left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kSteerEncoderID, kSteerEncoderOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 7;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 8;
      // The CAN ID for the steer encoder
      public static final int kSteerEncoderID = 3;
      // The offset for the steer encoder
      public static final double kSteerEncoderOffset = 0.795;
      // creating the swerve module constants for the front right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kSteerEncoderID, kSteerEncoderOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 3;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 4;
      // The CAN ID for the steer encoder
      public static final int kSteerEncoderID = 1;
      // The offset for the steer encoder
      public static final double kSteerEncoderOffset = 0.634; // TODO: Tune Encoder Offset
      // creating the swerve module constants for the back left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kSteerEncoderID, kSteerEncoderOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class BackRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 5;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 6;
      // The CAN ID for the steer encoder
      public static final int kSteerEncoderID = 2;
      // The offset for the steer encoder
      public static final double kSteerEncoderOffset =
          0.714; // TODO: Tune Encoder Offset // was wrong way needs recaculated greater value
      // creating the swerve module constants for the back right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kSteerEncoderID, kSteerEncoderOffset);
    }
  }
}
