package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

public final class Constants {
    
    public final static class IntakeRollersConstants{
        public static final int intakeRollerID = 51;
        public static final double gearRatio = 3.0/2.0;

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

        public static final AngularVelocity cruiseVelocity = Rotations.per(Minute).of(3000);
        public static final AngularAcceleration acceleration = Rotations.per(Minute).per(Second).of(6000);
        public static final Velocity<AngularAccelerationUnit> jerk = Rotations.per(Minute).per(Second).per(Second).of(12000);
    }

    public final static class ShooterWheelConstants {
        public static final int flywheelID = 42;
        public static final double gearRatio = 1.0/1.0;

        public static final AngularVelocity cruiseVelocity = Rotations.per(Minute).of(3000);
        // TODO: The old code used RP/s^2 so im using it here. Maybe change this
        public static final AngularAcceleration acceleration = Rotations.per(Second).per(Second).of(2500);
        public static final Velocity<AngularAccelerationUnit> jerk = Rotations.per(Minute).per(Second).per(Second).of(12000);

        public static final AngularVelocity shootingTolerance = Rotations.per(Minute).of(50);
    }

}
