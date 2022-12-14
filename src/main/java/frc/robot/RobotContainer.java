// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.util.JoystickUtil;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);

  private final XboxController m_controller = new XboxController(Constants.controllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive.calibrateGyro();
    m_drive.zeroHeading();

    configureButtonBindings();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
  }

  private void configureButtonBindings() {
    double rAxis = -m_controller.getRawAxis(4);
    /*************************************************
     * Driver controls
     *************************************************/
    m_drive.setDefaultCommand(
        new RunCommand(
                () ->
                    m_drive.drive(
                        MathUtil.applyDeadband(-m_controller.getLeftY(), 0.1),
                        MathUtil.applyDeadband(-m_controller.getLeftX(), 0.1),
                        JoystickUtil.squareAxis(
                            m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis()),
                        !m_controller.getLeftStickButton()),
                m_drive)
            .withName("Default Drive"));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
