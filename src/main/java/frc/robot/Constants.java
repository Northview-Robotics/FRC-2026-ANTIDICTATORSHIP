package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;

public final class Constants {

    public static final class VisionConstants {
        //Camera 1 position and rotation relative to the robot center (in meters and degrees)
    public static final double frontCamPosX = -0.213;
    public static final double frontCamPosY = 0.027;
    public static final double frontCamPosZ = 0.509;
    public static final double frontCamRotPitch = Units.degreesToRadians(-15);
    public static final double frontCamRotYaw = Math.PI;

    //Camera 2 position and rotation relative to the robot center (in meters and degrees)
    public static final double backCamPosX = 0.213;
    public static final double backCamPosY = 0.110;
    public static final double backCamPosZ = 0.508;
    public static final double backCamRotPitch = Units.degreesToRadians(-15);
    public static final double backCamRotYaw = 0;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public final static class SwerveConstants{
        public static final double maxDriveSpeed = Units.feetToMeters(14);
        public static final double maxAcceleration = 5;
        public static final double deadband = 0.05;

        public static final class Rotation {
            public static final PIDConstants PID = new PIDConstants(0.4, 0.0, 0);
        }

        public static final class Translation {
            public static final PIDConstants PID = new PIDConstants(5.0, 0.0, 0.0);
        }
    }
    
    public final static class IntakeRollersConstants{
        public static final int intakeRollerID = 51;
        public static final double gearRatio = 3.0/2.0;

        //TODO Adjust Current Limit based on irl intake rollers
        public static final Current supplyCurrentLimit = Amps.of(60);
        public static final Current statorCurrentLimit = Amps.of(100);

        public static final AngularVelocity cruiseVelocity = Rotations.per(Minute).of(3000);
        public static final AngularAcceleration acceleration = Rotations.per(Minute).per(Second).of(6000);
        public static final Velocity<AngularAccelerationUnit> jerk = Rotations.per(Minute).per(Second).per(Second).of(12000);
    }

    public final static class IntakePivotConstants{
        public static final int intakePivotID = 53;
        public static final double positionConversionFactor = 360.0/47.5308642;
        public static final double velocityConversionFactor = 360.0/47.5308642/60.0;

        //TODO PLEASE FIND THE POSITION THAT'S ON THE UPPER HARD STOP. Run a test where you zero the angle at fully delpoyed intake.
        //Honestly, just contact Hangyul when you're about to find this value.
        public static final Angle resetAngle = Degrees.of(90);

        //TODO find actual angles for these values.
        public static final Angle stowedAngle = Degrees.of(0);
        public static final Angle deployedAngle = Degrees.of(90);
        public static final Angle highAgitateAngle = Degrees.of(70);
        public static final Angle lowAgitateAngle = Degrees.of(35);

        //TODO Tune Velocity PID and FeedForward Constants
        public static final PIDController velocityPID = new PIDController(0, 0, 0);
        public static final ArmFeedforward velocityFF = new ArmFeedforward(0, 0, 0,0);

        //TODO Tune Position PID and FeedForawrd Constnats
        public static final PIDController positionPID = new PIDController(0, 0, 0);
        public static final ArmFeedforward positionFF = new ArmFeedforward(0, 0, 0,0);

        //TODO PLEASE TUNE THESE VALUES. They're okay starting values, but tune them. READ THE DOCS ON MAXMOTION
        public static final AngularVelocity cruiseVelocity = RotationsPerSecond.of(3);
        public static final AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(6);
        public static final Angle allowedError = Rotations.of(0.5);

    }

    public final static class IndexerConstants{
        public static final int indexerID = 52;
        public static final double gearRatio = 3.0/1.0;

        //TODO Adjust Current Limit based on irl indexer
        public static final Current supplyCurrentLimit = Amps.of(60);
        public static final Current statorCurrentLimit = Amps.of(100);

        public static final AngularVelocity cruiseVelocity = Rotations.per(Minute).of(3000);
        public static final AngularAcceleration acceleration = Rotations.per(Minute).per(Second).of(6000);
        public static final Velocity<AngularAccelerationUnit> jerk = Rotations.per(Minute).per(Second).per(Second).of(12000);

        public static final AngularVelocity defaultAngularVelocity = Rotations.per(Minute).of(3000);

        public static final double ejectCurrent = 160;
        public static final double ejectTimeout = 0.1;
        public static final double ejectRPM = -10;
    }

    public final static class FeederConstants{
        public static final int feederID = 41;
        public static final double gearRatio = 2.0/1.0;

        //TODO Adjust Current Limit based on irl feeder
        public static final Current supplyCurrentLimit = Amps.of(80);
        public static final Current statorCurrentLimit = Amps.of(120);

        public static final AngularVelocity defaultAngularVelocity = Rotations.per(Minute).of(3000);
    }

    public final static class ShooterWheelConstants {
        public static final int shooterMotor1ID = 42;
        public static final int shooterMotor2ID = 43;
        public static final int shooterMotor3ID = 44;
        public static final int shooterMotor4ID = 45;

        public static final double gearRatio = 1.0/1.0;

        //TODO Adjust Current Limit based on irl shooter wheel
        public static final Current supplyCurrentLimit = Amps.of(80);
        public static final Current statorCurrentLimit = Amps.of(140);

        //TODO Maybe adjust this value based on how consistent the shooter is. This is the acceptable error in RPM for the shooter to be considered at the target speed.
        public static final double acceptableDeltaRPM = 50;

        public static final AngularVelocity cruiseVelocity = Rotations.per(Minute).of(1000);
        // TODO: The old code used RP/s^2 so im using it here. Maybe change this
        public static final AngularAcceleration acceleration = Rotations.per(Second).per(Second).of(2500);
        public static final Velocity<AngularAccelerationUnit> jerk = Rotations.per(Minute).per(Second).per(Second).of(12000);

        public static final AngularVelocity shootingTolerance = Rotations.per(Minute).of(50);

    }

    public final static class HoodConstants{
        public static final int hoodID = 46;
        public static final double positionConversionFactor = 360.0/1250.0;
        public static final double velocityConversionFactor = 360.0/1250.0/60.0;

        //TODO run a test where you zero the angle when hood is fully retracted.
        public static final Angle resetAngle = Degrees.of(0);

        //TODO Tune Velocity PID and FeedForward Constants
        public static final PIDController velocityPID = new PIDController(0, 0, 0);
        public static final ArmFeedforward velocityFF = new ArmFeedforward(0, 0, 0,0);

        //TODO Tune Position PID and FeedForawrd Constants
        public static final PIDController positionPID = new PIDController(0, 0, 0);
        public static final ArmFeedforward positionFF = new ArmFeedforward(0, 0, 0,0);

        //TODO PLEASE TUNE THESE VALUES. They're okay starting values, but tune them. READ THE DOCS ON MAXMOTION
        public static final AngularVelocity cruiseVelocity = RotationsPerSecond.of(1.5);
        public static final AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(3);
        public static final Angle allowedError = Rotations.of(0.5);
    }

    public final static class ShootLUT {
        public record ShooterParams(double flywheelRPM, double hoodAngle, double tof) implements Interpolatable<ShooterParams> {
        public ShooterParams interpolate(ShooterParams endValue, double t) {
            return new ShooterParams(
                MathUtil.interpolate(flywheelRPM(), endValue.flywheelRPM(), t),
                MathUtil.interpolate(hoodAngle(), endValue.hoodAngle(), t),
                MathUtil.interpolate(tof(), endValue.tof(), t));
            }
        }

        public static final InterpolatingTreeMap<Double, ShooterParams> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParams::interpolate);
        static {
            //TODO Fill this LUT with actual values. The distance is in meters, flywheelRPM is in RPM, hoodAngle is in degrees, and tof is in seconds.
            //map.put(distance, new ShooterParams(flywheelRPM, hoodAngle, tof));
            map.put(1.0, new ShooterParams(1000, 10, 1.0));
            map.put(2.0, new ShooterParams(2000, 20, 1.5));
            map.put(3.0, new ShooterParams(3000, 30, 2.0));
        }
    }

    public static final class AutoConstants{
        //Trench Poses
        public static final Pose2d RedTrenchRight = new Pose2d(11.9, 7.4, Rotation2d.fromDegrees(0));
        public static final Pose2d RedTrenchRightI = new Pose2d(12.4, 7.4, Rotation2d.fromDegrees(0));
        public static final Pose2d RedTrenchLeft = new Pose2d(11.9, 0.7, Rotation2d.fromDegrees(0));
        public static final Pose2d RedTrenchLeftI = new Pose2d(12.4, 0.7, Rotation2d.fromDegrees(0));

        //point1: blue trench right is x = 4.6, y = 7.4
        // point2: blue trench right x = 4.6, y = 0.6
        public static final Pose2d BlueTrenchRight = new Pose2d(4.6, 7.4, Rotation2d.fromDegrees(0));
        public static final Pose2d BlueTrenchRightI = new Pose2d(5.1, 7.4, Rotation2d.fromDegrees(0));
        public static final Pose2d BlueTrenchLeft = new Pose2d(4.6, 0.6, Rotation2d.fromDegrees(0));
        public static final Pose2d BlueTrenchLeftI = new Pose2d(5.1, 0.6, Rotation2d.fromDegrees(0));

        //IMPORTANT Note I stands for intermediate pose for pathplanner

        //Hub Pose
        public static final Pose2d HubB = new Pose2d(4.6, 4, Rotation2d.fromDegrees(0)); //blue
        public static final Pose2d HubR = new Pose2d(11.9, 4, Rotation2d.fromDegrees(0)); //red

        //Tower Poses
        public static final Pose2d TowerB = new Pose2d(1, 3.7, Rotation2d.fromDegrees(0)); //blue
        public static final Pose2d TowerBI = new Pose2d(2, 3.7, Rotation2d.fromDegrees(0));

        public static final Pose2d TowerR = new Pose2d(15.5, 3.7, Rotation2d.fromDegrees(0)); //red
        public static final Pose2d TowerRI = new Pose2d(14.5, 3.7, Rotation2d.fromDegrees(0));
    }
}
