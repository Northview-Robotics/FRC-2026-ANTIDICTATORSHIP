// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAngle;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.ShooterWheel;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;


  private final CommandXboxController driverCtrl = new CommandXboxController(0);
  private final CommandXboxController operatorCtrl = new CommandXboxController(1);

  private final IntakeRollers intakeRollers = new IntakeRollers(Constants.IntakeRollersConstants.intakeRollerID, Constants.IntakeRollersConstants.gearRatio);
  private final IntakeAngle intakeAngle = new IntakeAngle(Constants.IntakePivotConstants.intakePivotID);
  private final Indexer indexer = new Indexer(Constants.IndexerConstants.indexerID, Constants.IndexerConstants.gearRatio);
  private final Feeder feeder = new Feeder(Constants.FeederConstants.feederID, Constants.FeederConstants.gearRatio);
  private final ShooterWheel shooterWheel = new ShooterWheel(Constants.ShooterWheelConstants.shooterMotor1ID,
                                                          Constants.ShooterWheelConstants.shooterMotor2ID, 
                                                          Constants.ShooterWheelConstants.shooterMotor3ID, 
                                                          Constants.ShooterWheelConstants.shooterMotor4ID,
                                                          Constants.ShooterWheelConstants.gearRatio);
  private final Hood hood = new Hood(Constants.HoodConstants.hoodID);

  private File directory = new File(Filesystem.getDeployDirectory(),"swerve2");
  private final Drive drivetrain = new Drive(directory);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
                                                              () -> -driverCtrl.getLeftY(), 
                                                              () -> -driverCtrl.getLeftX())
                                                              .withControllerRotationAxis(driverCtrl::getRightX)
                                                              .deadband(SwerveConstants.deadband)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverCtrl::getRightX,
                                                                                              driverCtrl::getRightY).headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                      .allianceRelativeControl(false);
  
  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

  }

  private void configureBindings() {
    // Command driveFieldOrientedDirectAngle = drivetrain.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity = drivetrain.driveFieldOriented(driveRobotOriented);
    Command driveHubAlign = drivetrain.driveAlignedHubCommand(driveAngularVelocity);
    
    drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverCtrl.a().whileTrue(driveHubAlign);
    
    //Example Intake Roller Bindings
    operatorCtrl.a().whileTrue(intakeRollers.setVoltageCmd(Volts.of(6)));
    operatorCtrl.b().whileTrue(intakeRollers.setVoltageCmd(Volts.of(-6)));

    operatorCtrl.x().onTrue(intakeAngle.setPositionCmd(Degrees.of(0)));
    operatorCtrl.y().onTrue(intakeAngle.setPositionCmd(Degrees.of(90)));

    operatorCtrl.povUpRight().whileTrue(indexer.setVoltageCmd(Volts.of(6)));// idk sigmasigmasigmasigma
    
    // TOOD: Constant RPM. Use Double Supplier with shootermap at some point
    // operatorCtrl.rightBumper().whileTrue(shooterWheel.setVelocityCmd(2000));

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

    //TODO configure other bindings after zain gives button map

    operatorCtrl.rightBumper().whileTrue(Commands.sequence(
        Commands.parallel(shooterWheel.shootCmd(2.0),hood.setShootAngleCmd(2.0)),
        Commands.waitUntil(shooterWheel::isAtTargetSpeed).alongWith(Commands.waitUntil(hood::isAtAngle)),
        Commands.parallel(indexer.setVelocityCmd(Constants.IndexerConstants.defaultAngularVelocity), feeder.setVelocityCmd(Constants.FeederConstants.defaultAngularVelocity))
    ).withName("ShootCmd"));

    operatorCtrl.leftBumper().onTrue(//idk what button to set, just example rn
      //Zero subsystems
      Commands.parallel(hood.setPositionCmd(Degrees.of(0)).until(hood::isAtAngle), intakeAngle.setPositionCmd(Degrees.of(0)).until(intakeAngle::isAtAngle))
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
