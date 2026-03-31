// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAngle;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.ShooterWheel;

public class RobotContainer {
  @SuppressWarnings("unused")
  private final CommandXboxController driverCtrl = new CommandXboxController(0);
  private final CommandXboxController operatorCtrl = new CommandXboxController(1);

  private final IntakeRollers intakeRollers = new IntakeRollers(Constants.IntakeRollersConstants.intakeRollerID,
      Constants.IntakeRollersConstants.gearRatio);
  private final IntakeAngle intakeAngle = new IntakeAngle(Constants.IntakePivotConstants.intakePivotID,
      Constants.IntakePivotConstants.gearRatio);
  private final Indexer indexer = new Indexer(Constants.IndexerConstants.indexerID,
      Constants.IndexerConstants.gearRatio);
  private final ShooterWheel shooterWheel = new ShooterWheel(Constants.ShooterWheelConstants.flywheelID,
      Constants.ShooterWheelConstants.gearRatio);

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }

  private void configureBindings() {
    // Example Intake Roller Bindings
    operatorCtrl.a().whileTrue(intakeRollers.setVoltageCmd(Volts.of(6)));
    operatorCtrl.b().whileTrue(intakeRollers.setVoltageCmd(Volts.of(-6)));

    operatorCtrl.x().onTrue(intakeAngle.setPositionCmd(Degrees.of(0)));
    operatorCtrl.y().onTrue(intakeAngle.setPositionCmd(Degrees.of(90)));

    operatorCtrl.povUpRight().whileTrue(indexer.setVoltageCmd(Volts.of(6)));// idk sigmasigmasigmasigma
    
    // TOOD: Constant RPM. Use Double Supplier with shootermap at some point
    operatorCtrl.rightBumper().whileTrue(shooterWheel.setVelocityCmd(RPM.of(2000)));

    // Example shooting command
    // Our shoot cmd should look something like this
    // operatorCtrl.rightBumper().whileTrue(
    //     shooterWheel.setVelocityCmd(RPM.of(2000))
    //         .alongWith(
    //             Commands.sequence(
    //                 new WaitUntilCommand(() -> shooterWheel.atSetpoint()),
    //                 new WaitCommand(2.0),
    //                 // Kicker and other stuff
    //                 indexer.setVoltageCmd(Volts.of(6))))
    //         .withName("ShootCmd"));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
