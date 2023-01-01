package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class PSwerveModule implements SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final ProfiledPIDController m_steeringController;
  private final SparkMaxPIDController m_driveController;

  private final AnalogEncoder m_steerEncoder;
  private final RelativeEncoder m_steerMotorEncoder;
  private final RelativeEncoder m_driveMotorEncoder;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_turningFeedforward;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private void driveMotorConfig(CANSparkMax driveMotor, RelativeEncoder enc) {
    enc = driveMotor.getEncoder();

    enc.setPositionConversionFactor(Constants.SwerveConstants.kDrivePositionConversionFactor);

    enc.setVelocityConversionFactor(Constants.SwerveConstants.kDriveVelocityConversionFactor);

    driveMotor.setInverted(Constants.SwerveConstants.kDriveMotorInvert);

    driveMotor.getPIDController().setOutputRange(-1, 1);
    driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.kDriveCurrentLimit);

    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
  }

  public void steerMotorConfig(CANSparkMax steerMotor, RelativeEncoder enc) {
    enc = steerMotor.getEncoder();

    enc.setPositionConversionFactor(Constants.SwerveConstants.kSteerPositionConversionFactor);

    enc.setVelocityConversionFactor(Constants.SwerveConstants.kSteerVelocityConversionFactor);

    steerMotor.setInverted(Constants.SwerveConstants.kSteerMotorInvert);

    steerMotor.getPIDController().setOutputRange(-1, 1);

    steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    steerMotor.setSmartCurrentLimit(Constants.SwerveConstants.kSteerCurrentLimit);

    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
  }

  public PSwerveModule(SwerveModuleConstants constants) {
    m_driveMotor = new CANSparkMax(constants.driveMotorID, CANSparkMax.MotorType.kBrushless);

    m_turnMotor = new CANSparkMax(constants.steerMotorID, CANSparkMax.MotorType.kBrushless);

    m_driveMotorEncoder = m_driveMotor.getEncoder();
    m_steerMotorEncoder = m_turnMotor.getEncoder();
    m_driveMotorEncoder.setPosition(0.0);
    m_steerEncoder = new AnalogEncoder(constants.encoderID);

    driveMotorConfig(m_driveMotor, m_driveMotorEncoder);
    steerMotorConfig(m_turnMotor, m_steerMotorEncoder);

    m_driveFeedforward = Constants.SwerveConstants.kDriveFeedforward;
    m_turningFeedforward = Constants.SwerveConstants.kTurningFeedforward;

    m_driveController = m_driveMotor.getPIDController();
    m_driveController.setP(Constants.SwerveConstants.kDriveMotorPIDGains.P);
    m_driveController.setI(Constants.SwerveConstants.kDriveMotorPIDGains.I);
    m_driveController.setD(Constants.SwerveConstants.kDriveMotorPIDGains.D);

    m_steeringController =
        new ProfiledPIDController(
            Constants.SwerveConstants.kSteerMotorPIDGains.P,
            Constants.SwerveConstants.kSteerMotorPIDGains.I,
            Constants.SwerveConstants.kSteerMotorPIDGains.D,
            Constants.SwerveConstants.kTurningConstraints);

    m_steeringController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

    m_desiredState.angle = new Rotation2d(getScaledPosition());
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotorEncoder.getVelocity(), new Rotation2d(getScaledPosition()));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.angle = new Rotation2d(desiredState.angle.getRadians() % (2 * Math.PI));

    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getScaledPosition()));

    desiredState.angle = new Rotation2d(desiredState.angle.getRadians() % (2 * Math.PI));

    m_desiredState = state;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Override
  public void periodic() {
    // calculate the turning motor output from the PID controller.
    m_driveController.setReference(
        m_desiredState.speedMetersPerSecond,
        ControlType.kVelocity,
        0,
        m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond));
    m_steeringController.setGoal(m_desiredState.angle.getRadians());
    double demand = m_steeringController.calculate(getScaledPosition());
    demand += m_turningFeedforward.calculate(m_steeringController.getSetpoint().velocity);
    m_turnMotor.setVoltage(demand);
  }

  @Override
  public void resetEncoders() {
    m_driveMotorEncoder.setPosition(0.0);
    m_steerMotorEncoder.setPosition(getScaledPosition());
  }

  @Override
  public double getDriveDistanceMeters() {
    return m_driveMotorEncoder.getPosition();
  }

  private double getScaledPosition() {
    m_steerEncoder.reset();
    double absolutePosition = m_steerEncoder.getAbsolutePosition();
    double positionOffset = m_steerEncoder.getPositionOffset();

    double position = Math.round((absolutePosition - positionOffset) * 360.0);

    // If the position is negative and the absolute position is positive, add 360 to the position
    if (position < 0) {
      position += 360;
    }

    return position;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.setActuator(true);
  }
  // TODO Auto-generated method stub

}
