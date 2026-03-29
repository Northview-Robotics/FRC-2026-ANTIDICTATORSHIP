package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

public final class Constants {
    
    public final static class IntakeRollers{
        public static final int intakeRollerID = 51;
        public static final double gearRatio = 3.0/2.0;

        public static final AngularVelocity cruiseVelocity = Rotations.per(Minute).of(3000);
        public static final AngularAcceleration acceleration = Rotations.per(Minute).per(Second).of(6000);
        public static final Velocity<AngularAccelerationUnit> jerk = Rotations.per(Minute).per(Second).per(Second).of(12000);
    }

}
