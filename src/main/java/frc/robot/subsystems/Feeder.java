package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;


public class Feeder extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final VoltageOut voltCtrl = new VoltageOut(0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0);

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, 
            Volts.of(4), 
            Seconds.of(5), 
            (state) -> Logger.recordOutput("Feeder State", state.toString())), 
            
            new SysIdRoutine.Mechanism(this::setVoltage, null, this)
        ); 

    public Feeder(int canid, double gearRatio){
        motor = new TalonFX(canid);

        motorConfig.CurrentLimits
            .withSupplyCurrentLimit(Constants.FeederConstants.supplyCurrentLimit)
            .withStatorCurrentLimit(Constants.FeederConstants.statorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);

        motorConfig.Slot0
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);

        motorConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            //TODO Adjust Inverted based on irl indexer
            .withInverted(InvertedValue.Clockwise_Positive);

        motorConfig.Feedback
            .withSensorToMechanismRatio(gearRatio)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        motor.getConfigurator().apply(motorConfig);
    }

    @AutoLogOutput
    public Voltage getVoltage(){
        return motor.getMotorVoltage().getValue();
    }

    @AutoLogOutput
    public AngularVelocity getVelocity(){
        return motor.getVelocity().getValue();
    }

    @AutoLogOutput
    public Current getStatorCurrent(){
        return motor.getStatorCurrent().getValue();
    }

    @AutoLogOutput
    public Current getSupplyCurrent(){
        return motor.getSupplyCurrent().getValue();
    }

    public void setVoltage(Voltage voltage){
        motor.setControl(voltCtrl.withOutput(voltage));
    }

    public void setVelocity(AngularVelocity velocity){
        motor.setControl(velCtrl.withVelocity(velocity));

    }

    public Command setSysIdDynamicCmd(Direction direction){
        return sysIdRoutine.dynamic(direction);
    }

    public Command setSysIdQuasistaticCmd(Direction direction){
        return sysIdRoutine.quasistatic(direction);
    }

    public Command setVoltageCmd(Voltage voltage){
        return this.runEnd(
            () -> setVoltage(voltage),
            () -> setVoltage(Volts.of(0)));
    }

    public Command setVelocityCmd(AngularVelocity velocity){
        return this.runEnd(
            () -> setVelocity(velocity),
            () -> setVoltage(Volts.of(0)));
    }
}
