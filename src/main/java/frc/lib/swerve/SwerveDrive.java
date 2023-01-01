package frc.lib.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  public enum ModuleLocation {
    frontLeft(0),
    frontRight(1),
    rearLeft(2),
    rearRight(3);

    public final int value;
    private static final ModuleLocation[] m_mapping =
        new ModuleLocation[] {frontLeft, frontRight, rearLeft, rearRight};

    private ModuleLocation(int v) {
      this.value = v;
    }

    public static ModuleLocation fromInt(int v) {
      return m_mapping[v];
    }
  }
  // Robot Modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_rearRight;

  // Gyro sensor
  private final AHRS m_gyro;

  // Robot kinematics
  private final SwerveDriveKinematics m_kinematics;

  // Odometry for tracking pose
  SwerveDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();

  private final double m_maxSpeed;

  // Position based Odometry updates
  private final Timer m_loopTimer;
  private double[] m_lastDistances;
  private double m_lastLoopTime;

  /**
   * Creates a new Swerve Drive Module
   *
   * @param frontLeft Swerve Module
   * @param frontRight Swerve Module
   * @param rearLeft Swerve Module
   * @param rearRight Swerve Module
   * @param kinematics Swerve Drive Kinematics
   * @param gyro used for odometry and field centric control
   * @param maxSpeed of the wheels used to normilize wheel speeds
   */
  public SwerveDrive(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight,
      SwerveDriveKinematics kinematics,
      AHRS gyro,
      double maxSpeed) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_rearLeft = rearLeft;
    m_rearRight = rearRight;

    m_kinematics = kinematics;

    m_gyro = gyro;

    m_maxSpeed = maxSpeed;
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    m_loopTimer = new Timer();
    m_loopTimer.reset();
    m_loopTimer.start();
    m_lastDistances =
        new double[] {
          m_frontLeft.getDriveDistanceMeters(),
          m_frontRight.getDriveDistanceMeters(),
          m_rearLeft.getDriveDistanceMeters(),
          m_rearRight.getDriveDistanceMeters()
        };

    m_lastLoopTime = 0.0;

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
  }

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

  // init sendable method (not yet implemented)

  @Override
  public void periodic() {
    // run swerve module if they require it
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    m_chassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());

    double[] distances =
        new double[] {
          m_frontLeft.getDriveDistanceMeters(),
          m_frontRight.getDriveDistanceMeters(),
          m_rearLeft.getDriveDistanceMeters(),
          m_rearRight.getDriveDistanceMeters()
        };

    double time = m_loopTimer.get();
    double dt = time - m_lastLoopTime;
    m_lastLoopTime = time;

    if (dt == 0) {
      return;
    }

    // update odometry in periodic block
    m_odometry.updateWithTime(
        time,
        m_gyro.getRotation2d(),
        new SwerveModuleState(
            (distances[0] - m_lastDistances[0]) / dt, m_frontLeft.getState().angle),
        new SwerveModuleState(
            (distances[1] - m_lastDistances[1]) / dt, m_frontRight.getState().angle),
        new SwerveModuleState(
            (distances[2] - m_lastDistances[2]) / dt, m_rearLeft.getState().angle),
        new SwerveModuleState(
            (distances[3] - m_lastDistances[3]) / dt, m_rearRight.getState().angle));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_lastDistances = distances;
  }

  //         // simulationPeriodic method (not yet implemented)

  /**
   * return the current velocity of the chasis as a ChassisSpeeds object
   *
   * @return velocity of the chasis
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeeds;
  }

  /**
   * Predict the motion between the current position and future position
   *
   * @param lookAhead time in seconds to predict ahead
   * @return twist2d representing the change in pose over the lookahead time
   */
  public Twist2d getPredictedMotion(double lookAhead) {
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
    return new Twist2d(
        chassisSpeeds.vxMetersPerSecond * lookAhead,
        chassisSpeeds.vyMetersPerSecond * lookAhead,
        chassisSpeeds.omegaRadiansPerSecond * lookAhead);
  }

  /**
   * Returns the currently-estemated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick inputs
   *
   * @param xSpeed Speed of the robot in the x direction (forward)
   * @param ySpeed Speed of the robot in the y direction (sideways)
   * @param rot Angular rate of the robot
   * @param fieldRelative Whether the provided x and y speeds are relative to the field
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // if nothing is commanded , hold the same position
    if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
      holdAllModulesRotation();
      return;
    }

    ySpeed = ySpeed * m_maxSpeed;
    xSpeed = xSpeed * m_maxSpeed;
    rot = rot * m_maxSpeed;

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private void holdModulesRotation(SwerveModule m) {
    var state = m.getDesiredState();
    state.speedMetersPerSecond = 0;
    m.setDesiredState(state);
  }

  private void holdAllModulesRotation() {
    holdModulesRotation(m_frontLeft);
    holdModulesRotation(m_frontRight);
    holdModulesRotation(m_rearLeft);
    holdModulesRotation(m_rearRight);
  }

  /* Set the swerve drive into an X which is not drivable but should help from being pushed around */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /* Set the swerve drive into 0 position - Method is only for when needing to return all modules back to home for autonomous (testing)*/
  public void setZero() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModuleStates.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Set an individual module state independently of all other modules testing/tuning purposes */
  public void testPeriodic() {
    m_frontLeft.testPeriodic();
    m_frontRight.testPeriodic();
    m_rearLeft.testPeriodic();
    m_rearRight.testPeriodic();
  }

  public SwerveModuleState[] getModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a poition of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** Calibrate the gyro. Requirements for the devices being still depend on the gyro being used */
  public void calibrateGyro() {
    m_gyro.calibrate();
  }

  // TODO: fix this
  public void setHeading(double degreesCCWPositive) {
    m_gyro.setAngleAdjustment(degreesCCWPositive);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading in degrees, from -180 to 180.
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The robot's turn rate in degrees per second.
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  /**
   * Create a trajectory following command. Note that the begginning and end states of the command
   * are not neccesairilly 0 speed.
   *
   * @param trajectory PathPlanner trajectory to follow
   * @param xController PID controller for the x direction (left/right)
   * @param yController PID controller for the y direction (forward/backward)
   * @param thetaController PID controller for the rotation (CCW positive)
   * @return Command to be Scheduled
   */
  public Command trajectoryFollowerCommand(
      PathPlannerTrajectory trajectory,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {
    Command swCommand =
        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            m_kinematics,
            xController,
            yController,
            thetaController,
            (states) -> setModuleStates(states),
            this);
    return new InstantCommand(() -> m_field.getObject("Trajectory").setTrajectory(trajectory))
        .alongWith(swCommand);
  }
}

