// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeAngle;
import frc.robot.subsystems.IntakeRollers;

public class RobotContainer {
  private final CommandXboxController driverCtrl = new CommandXboxController(0);
  private final CommandXboxController operatorCtrl = new CommandXboxController(1);

  private final IntakeRollers intakeRollers = new IntakeRollers(Constants.IntakeRollersConstants.intakeRollerID, Constants.IntakeRollersConstants.gearRatio);
  private final IntakeAngle intakeAngle = new IntakeAngle(Constants.IntakePivotConstants.intakePivotID, Constants.IntakePivotConstants.gearRatio);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //Example Intake Roller Bindings
    operatorCtrl.a().whileTrue(intakeRollers.setVoltageCmd(Volts.of(6)));
    operatorCtrl.b().whileTrue(intakeRollers.setVoltageCmd(Volts.of(-6)));
    
    operatorCtrl.x().onTrue(intakeAngle.setPositionCmd(Degrees.of(0)));
    operatorCtrl.y().onTrue(intakeAngle.setPositionCmd(Degrees.of(90)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
