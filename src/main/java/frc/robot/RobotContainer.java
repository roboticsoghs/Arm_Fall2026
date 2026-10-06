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
  public static final JoystickButton zeroButton = new JoystickButton(op, 8);
  public static final JoystickButton armUpButton = new JoystickButton(op, 9);
  public static final JoystickButton armDownButton = new JoystickButton(op, 13);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    zeroButton.onTrue(arm.zeroEncoder());

    armUpButton.whileTrue(arm.setArmAngle(45.0));

    armDownButton.whileTrue(arm.setArmAngle(0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
