package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;

public final class Constants {

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
        public static final double gearRatio = 47.5308642;

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

        public static final AngularVelocity cruiseVelocity = Rotations.per(Minute).of(1000);
        // TODO: The old code used RP/s^2 so im using it here. Maybe change this
        public static final AngularAcceleration acceleration = Rotations.per(Second).per(Second).of(2500);
        public static final Velocity<AngularAccelerationUnit> jerk = Rotations.per(Minute).per(Second).per(Second).of(12000);

        public static final AngularVelocity shootingTolerance = Rotations.per(Minute).of(50);
    }

    public final static class HoodConstants{
        public static final int hoodID = 46;
        public static final double gearRatio = 100.0/1.0;

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

}
