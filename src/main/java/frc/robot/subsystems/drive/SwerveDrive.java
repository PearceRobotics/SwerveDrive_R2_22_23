package frc.robot.subsystems.drive;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
  private Gyroscope gyroscope;
  private Encoder turningEncoder;
  private Encoder driveEncoder;

  /**
   * The maximum voltage that will be delivered to the drive motors.
   *
   * <p>This can be reduced to cap the robot's maximum speed. Typically, this is useful during
   * initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      6380.0
          / 60.0
          * SdsModuleConfigurations.MK4I_L1.getDriveReduction()
          * SdsModuleConfigurations.MK4I_L1.getWheelDiameter()
          * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a
  // measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(
              SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation =
      new Translation2d(
          SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0); // recalculate
  Translation2d m_frontRightLocation =
      new Translation2d(
          SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0); // recalculate
  Translation2d m_backLeftLocation =
      new Translation2d(
          -SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0); // recalculate
  Translation2d m_backRightLocation =
      new Translation2d(
          -SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0); // recalculate

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          // Front left
          m_frontLeftLocation,
          // Front right
          m_frontRightLocation,
          // Back left
          m_backLeftLocation,
          // Back right
          m_backRightLocation);

  // Creating my odometry object from the kinematics object. Here,
  // our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing forward.
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics, this.gyroscope.getRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // Convert to module states
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

  public SwerveDrive() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is
    // for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is
    // for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use NEOS in standard configuration.
    // FIXME Setup motor configuration
    m_frontLeftModule =
        Mk4iSwerveModuleHelper.createNeo(
            // This parameter is optional, but will allow you to see the current state of the module
            // on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
            // This can either be L1, L2, L3 depending on your gear configuration // need to find
            // out with modules
            Mk4iSwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            SwerveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            SwerveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            SwerveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is
            // facing straight forward)
            SwerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
    // We will do the same for the other modules
    m_frontRightModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            SwerveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            SwerveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            SwerveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            SwerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            SwerveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            SwerveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            SwerveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
            SwerveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            SwerveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            SwerveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            SwerveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
            SwerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getPose2d() {
    return m_odometry.update(
        this.gyroscope.getRotation2d(),
        ((SwerveDrive) m_frontLeftModule).getState(),
        ((SwerveDrive) m_frontRightModule).getState(),
        ((SwerveDrive) m_backLeftModule).getState(),
        ((SwerveDrive) m_backRightModule).getState());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently
   * facing to the 'forwards' direction.
   */
  public void zeroGyroscope() {
    this.gyroscope.resetGyro();
    this.gyroscope.zeroYAW();
  }

  public void fieldOrientedDrive() {
    // The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    ChassisSpeeds SwerveSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(SwerveSpeeds);
  }

  public void robotModuleStates() {
    // Example module states
    var frontLeftState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19));
    var frontRightState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81));
    var backLeftState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44));
    var backRightState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56));

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds =
        m_kinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState);

    // Getting individual speeds
    double forward = chassisSpeeds.vxMetersPerSecond;
    double sideways = chassisSpeeds.vyMetersPerSecond;
    double angular = chassisSpeeds.omegaRadiansPerSecond;
  }

  public Rotation2d getGyroscopeRotation() {
    if (gyroscope.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(gyroscope.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
    // the angle increase.
    return Rotation2d.fromDegrees(360.0 - gyroscope.getYaw());
  }

  public void setDesiredStates(SwerveModuleState[] newStates) {
    moduleStates = newStates;
  }

  public void resetOdometry(Pose2d pose2d) {
    this.m_odometry.resetPosition(pose2d, this.gyroscope.getRotation2d());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module. Might need to change driveEncoder to actual encoder
   *     counting the rotations(speed) of wheels
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getRate(), new Rotation2d(this.gyroscope.getGyroAngle()));
  }

  public void periodic() {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(
        moduleStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        moduleStates[0].angle.getRadians());
    m_frontRightModule.set(
        moduleStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        moduleStates[1].angle.getRadians());
    m_backLeftModule.set(
        moduleStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        moduleStates[2].angle.getRadians());
    m_backRightModule.set(
        moduleStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        moduleStates[3].angle.getRadians());
  }
}
