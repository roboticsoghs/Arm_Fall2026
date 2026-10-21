// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.ArmJoint;

public class RobotContainer {
  // Subsystems
  private final ArmJoint arm = new ArmJoint();

  // Controllers
  public static final Joystick op = new Joystick(0);

  // Joystick buttons
  public static final JoystickButton zeroButton = new JoystickButton(op, 1);
  public static final JoystickButton arm45Button = new JoystickButton(op, 2);
  public static final JoystickButton arm0Button = new JoystickButton(op, 3);
  public static final JoystickButton arm90Button = new JoystickButton(op, 4);
  public static final JoystickButton pullPID = new JoystickButton(op, 4);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    zeroButton.onTrue(arm.reset());

    arm45Button.whileTrue(arm.setArmAngle(-45.0));

    arm0Button.whileTrue(arm.setArmAngle(0.0));

    arm90Button.whileTrue(arm.setArmAngle(-88));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
