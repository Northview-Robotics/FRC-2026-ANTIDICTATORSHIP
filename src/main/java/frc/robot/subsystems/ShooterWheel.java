// TODO: Finish shootermap one day

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class ShooterWheel extends SubsystemBase {

    // Motor
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX motor3;
    private final TalonFX motor4;

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0)
            .withAcceleration(Constants.ShooterWheelConstants.acceleration);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    Seconds.of(5),
                    (state) -> Logger.recordOutput("Shooter Wheel State", state.toString())),

            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                null,
                    this));

    public ShooterWheel(int leaderID, int follower1ID, int follower2ID, int follower3ID, double gearRatio){
        motor1 = new TalonFX(leaderID);
        motor2 = new TalonFX(follower1ID);
        motor3 = new TalonFX(follower2ID);
        motor4 = new TalonFX(follower3ID);
        motorConfig.CurrentLimits
            .withSupplyCurrentLimit(Constants.ShooterWheelConstants.supplyCurrentLimit)
            .withStatorCurrentLimit(Constants.ShooterWheelConstants.statorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);
        
        motorConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            //TODO Adjust Inverted based on irl shooter wheel
            .withInverted(InvertedValue.CounterClockwise_Positive);
        
        motorConfig.Feedback
            .withSensorToMechanismRatio(gearRatio)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        
        motorConfig.Slot0
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);

        motorConfig.MotionMagic
            .withMotionMagicCruiseVelocity(Constants.ShooterWheelConstants.cruiseVelocity)
            .withMotionMagicAcceleration(Constants.ShooterWheelConstants.acceleration);
            // Let's keep jerk default. It's optional acc to CTRE docs
            // ref: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
            // .withMotionMagicJerk(Constants.ShooterWheelConstants.jerk);

        motor1.getConfigurator().apply(motorConfig);
        motor2.getConfigurator().apply(motorConfig);
        motor3.getConfigurator().apply(motorConfig);
        motor4.getConfigurator().apply(motorConfig);

        motor2.setControl(new Follower(leaderID, MotorAlignmentValue.Aligned));
        motor3.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
        motor4.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
    }

    @AutoLogOutput
    public Voltage getVoltage() {
        return motor1.getMotorVoltage().getValue();
    }

    @AutoLogOutput
    public Current getStatorCurrent() {
        return motor1.getStatorCurrent().getValue();
    }

    @AutoLogOutput
    public Current getSupplyCurrent() {
        return motor1.getSupplyCurrent().getValue();
    }

    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return motor1.getVelocity().getValue();
    }

    @AutoLogOutput
    public Angle getPosition() {
        return motor1.getPosition().getValue();
    }

    public void shoot(){
        //Use LUT here
    }

    public void setVoltage(Voltage volts) {
        motor1.setControl(voltCtrl.withOutput(volts));
    }

    public void setVelocity(AngularVelocity velocity) {
        motor1.setControl(velCtrl.withVelocity(velocity));
    }

    private double getError() {
        return motor1.getClosedLoopError().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return Math.abs(getError()) < 50;
    }

    public Command setSysIdDynamicCmd(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command setSysIdQuasistaticCmd(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command setVoltageCmd(Voltage volts) {
        return this.runEnd(
                () -> setVoltage(volts),
                () -> setVoltage(Volts.zero()));
    }

    public Command setVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
                () -> setVelocity(velocity),
                () -> setVoltage(Volts.zero()));
    }
}
