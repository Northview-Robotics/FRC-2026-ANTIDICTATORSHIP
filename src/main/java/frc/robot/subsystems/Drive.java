// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.vision.AutoAlign;
import frc.robot.subsystems.vision.Vision;

//Yagsl
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

//import org.littletonrobotics.junction.Logger;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

//Pathplanner
//Pathplanner imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class Drive extends SubsystemBase
{
  private static Drive drive;
  private SwerveDrive swerveDrive;
  private Vision vision;
  private static File directory = new File(Filesystem.getDeployDirectory(),"swerve2");
  private RobotConfig config;
  public boolean isRed;
  private SlewRateLimiter xLim;
  private SlewRateLimiter yLim;
  private SlewRateLimiter rotLim;

  public Drive(File directory)
  { 
    //YAGSL config
    boolean blueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(4),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(12.5),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maxDriveSpeed, startingPose);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(true,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    
    xLim = new SlewRateLimiter(SwerveConstants.maxAcceleration);
    yLim = new SlewRateLimiter(SwerveConstants.maxAcceleration);
    rotLim = new SlewRateLimiter(SwerveConstants.maxAcceleration);

    //Pathplanner config
    try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Configure AutoBuilder last
            AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotVelocity, 
            (speeds, feedforwards) -> drive(speeds), 
            new PPHolonomicDriveController( 
                    SwerveConstants.Translation.PID, // Movement PID constants
                    SwerveConstants.Rotation.PID // Rotation PID constants
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this// Reference to this subsystem to set requirements
        );
  }

  public void setVision(Vision vision) {
    this.vision = vision;
  }

  @Override
  public void periodic()
  {
    swerveDrive.updateOdometry();
    if (vision != null)
    {
      vision.updateVision();
    }
  }

  public Drive(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  SwerveConstants.maxDriveSpeed,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0))
    );
  }

  

  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    double xVelocity = xLim.calculate(MathUtil.applyDeadband(translationX.getAsDouble(), SwerveConstants.deadband));
    double yVelocity = yLim.calculate(MathUtil.applyDeadband(translationY.getAsDouble(), SwerveConstants.deadband));
    double angularVelocity = rotLim.calculate(MathUtil.applyDeadband(angularRotationX.getAsDouble(), SwerveConstants.deadband));
    // Make the robot move
    return run(() -> {
    swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                        Math.pow(xVelocity,3) * swerveDrive.getMaximumChassisVelocity(),
                        Math.pow(yVelocity, 3) * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularVelocity, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public Command driveAlignedHubCommand(DoubleSupplier translationX, DoubleSupplier translationY)
  {
    AutoAlign align = AutoAlign.getInstance();
    double xVelocity = xLim.calculate(MathUtil.applyDeadband(translationX.getAsDouble(), SwerveConstants.deadband));
    double yVelocity = yLim.calculate(MathUtil.applyDeadband(translationY.getAsDouble(), SwerveConstants.deadband));
    Rotation2d hubHeading = align.getHubHeading();
    double angularVelocity = swerveDrive.getSwerveController().headingCalculate(swerveDrive.getOdometryHeading().getRadians(), hubHeading.getRadians());

    return run(() -> {
    swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                        Math.pow(xVelocity,3) * swerveDrive.getMaximumChassisVelocity(),
                        Math.pow(yVelocity, 3) * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularVelocity, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  //idk if this will work
  public Command driveAlignedHubCommand(Supplier<ChassisSpeeds> velocity)
  {
    AutoAlign align = AutoAlign.getInstance();
    Rotation2d hubHeading = align.getHubHeading();
    double angularVel = swerveDrive.getSwerveController().headingCalculate(swerveDrive.getOdometryHeading().getRadians(), hubHeading.getRadians());
    ChassisSpeeds targetSpeeds = velocity.get();
    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
      targetSpeeds.vxMetersPerSecond,
      targetSpeeds.vyMetersPerSecond,
      angularVel
    );
    return run(() -> {
      swerveDrive.driveFieldOriented(adjustedSpeeds);
    });
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  @AutoLogOutput
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  // public void zeroGyroWithAlliance()
  // {
  //   if (isRedAlliance())
  //   {
  //     zeroGyro();
  //     //Set the pose 180 degrees
  //     resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
  //   } else
  //   {
  //     zeroGyro();
  //   }
  // }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  @AutoLogOutput
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        SwerveConstants.maxDriveSpeed);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        SwerveConstants.maxDriveSpeed);
  }

  public double getEncoderOffset(int targetModule){
    SwerveModule[] modules = swerveDrive.getModules();
    SwerveModule module = modules[targetModule];
    return module.getAbsolutePosition();
  }

  public double getMotorVel(int targetModule){
    SwerveModule[] modules = swerveDrive.getModules();
    SwerveModule module = modules[targetModule];
    return module.getDriveMotor().getVelocity();
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  @AutoLogOutput
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  @AutoLogOutput
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  @AutoLogOutput
  public SwerveModuleState[] getModuleStates() {
    return swerveDrive.getStates();
  }

  public static Drive getInstance(){
    if (drive == null){
      drive = new Drive(directory);
    }
    return drive;
  }
}