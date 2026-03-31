package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class IntakeAngle extends SubsystemBase{
    
    private final SparkMax motor;
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SparkClosedLoopController closedLoopController;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), 
            Volts.of(2), 
            Seconds.of(5), 
            (state) -> Logger.recordOutput("Intake Angle State", state.toString())), 
            
        new SysIdRoutine.Mechanism(this::setVoltage, null, this)
    );

    public IntakeAngle(int canId, double gearRatio){
        motor = new SparkMax(canId, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        
        config.encoder
            .velocityConversionFactor(1.0/gearRatio)
            .positionConversionFactor(1.0/gearRatio);
        
        //Don't use for actual PID Calculations, only for config values.
        PIDController velocityPID = Constants.IntakePivotConstants.velocityPID;
        PIDController positionPID = Constants.IntakePivotConstants.positionPID;
        ArmFeedforward velocityFF = Constants.IntakePivotConstants.velocityFF;
        ArmFeedforward positionFF = Constants.IntakePivotConstants.positionFF;

        //Velocity Control in Slot 0
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(velocityPID.getP(), velocityPID.getI(), velocityPID.getD(), ClosedLoopSlot.kSlot0)
            .feedForward
            .svacr(velocityFF.getKs(), velocityFF.getKv(), velocityFF.getKa(), velocityFF.getKg(), 1, ClosedLoopSlot.kSlot0);

        //Position Control in Slot 1
        config.closedLoop
            .pid(positionPID.getP(), positionPID.getI(), positionPID.getD(), ClosedLoopSlot.kSlot1)
            .feedForward
            .svacr(positionFF.getKs(), positionFF.getKv(), positionFF.getKa(), positionFF.getKg(), 1, ClosedLoopSlot.kSlot1);
        
        config.closedLoop
            .maxMotion
            .cruiseVelocity(Constants.IntakePivotConstants.cruiseVelocity.in(Rotations.per(Minute)), ClosedLoopSlot.kSlot1)
            .maxAcceleration(Constants.IntakePivotConstants.maxAcceleration.in(Rotations.per(Minute).per(Second)), ClosedLoopSlot.kSlot1)
            .allowedProfileError(Constants.IntakePivotConstants.allowedError.in(Rotations), ClosedLoopSlot.kSlot1);

        //TODO When setting current limits, set ResetMode to RestMode.kNoResetSafeParameters
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //TODO FIND THE REAL RESET VALUE AND SET IT IN CONSTANTS
        motor.getEncoder().setPosition(Constants.IntakePivotConstants.resetAngle.in(Rotations));
    }

    @AutoLogOutput
    public Voltage getVoltage(){
        return Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @AutoLogOutput
    public Current getCurrent(){
        return Amps.of(motor.getOutputCurrent());
    }

    @AutoLogOutput
    public AngularVelocity getVelocity(){
        return Rotations.per(Minute).of(motor.getEncoder().getVelocity());
    }

    @AutoLogOutput
    public Angle getPosition(){
        return Rotations.of(motor.getEncoder().getPosition());
    }

    public void setVoltage(Voltage volts){
        motor.setVoltage(volts);
    }

    public void setVelocity(AngularVelocity velocity){
        closedLoopController.setSetpoint(velocity.in(Rotations.per(Minute)), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void setPosition(Angle position){
        closedLoopController.setSetpoint(position.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public Command setSysIdDynamicCmd(Direction direction){
        return sysIdRoutine.dynamic(direction);
    }

    public Command setSysIdQuasistaticCmd(Direction direction){
        return sysIdRoutine.quasistatic(direction);
    }

    public Command setVoltageCmd(Voltage volts){
        return this.runEnd(
            () -> setVoltage(volts), 
            () -> setVoltage(Volts.zero()));
    }

    public Command setVoltageCmd(Supplier<Voltage> volts){
        return this.runEnd(
            () -> setVoltage(volts.get()), 
            () -> setVoltage(Volts.zero()));
    }

    public Command setVelocityCmd(AngularVelocity velocity){
        return this.runEnd(
            () -> setVelocity(velocity), 
            () -> setVoltage(Volts.zero()));
    }

    public Command setVelocityCmd(Supplier<AngularVelocity> velocity){
        return this.runEnd(
            () -> setVelocity(velocity.get()), 
            () -> setVoltage(Volts.zero()));
    }

    public Command setPositionCmd(Angle position){
        return this.runEnd(
            () -> setPosition(position), 
            () -> setVoltage(Volts.zero()));
    }

    public Command setPositionCmd(Supplier<Angle> position){
        return this.runEnd(
            () -> setPosition(position.get()), 
            () -> setVoltage(Volts.zero()));
    }
}
