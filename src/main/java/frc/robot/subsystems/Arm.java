// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ArmConstants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public static enum ArmPosition {
    INTAKE(Degrees.of(170)),
    IDLE(Degrees.of(140)),
    PLACE_L2(Degrees.of(55)),
    PLACE_L34(Degrees.of(37)),
    PLACE_L1(Degrees.of(62)),
    NET(Degrees.of(37));

    private Angle position;
    private ArmPosition(Angle position) {
      this.position = position;
    }

    public Angle getPosition() {return position;}
  }

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  private final RelativeEncoder encoder;

  private final CANcoder absoluteEncoder;

  private ArmPosition setpoint = ArmPosition.IDLE;

  // Mechanism (sim)
  private final Mechanism2d mech;
  private final MechanismRoot2d armMech;
  private final MechanismLigament2d armLigament;

  public Arm() {
    motor = new SparkFlex(kMotorId, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    motorConfig
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(kMotorRampRate)
      .closedLoopRampRate(kMotorRampRate)
      .smartCurrentLimit(kMotorCurrentLimit)
      .voltageCompensation(12);
    motorConfig.encoder
      .positionConversionFactor(kConversionFactor);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = motor.getEncoder();

    absoluteEncoder = new CANcoder(kAbsoluteEncoderId);

    encoder.setPosition(getAbsolutePosition().in(Rotations));
    
    // Mechanism visualization (sim)
    mech = new Mechanism2d(1, 1, new Color8Bit(255, 255, 255));
    armMech = mech.getRoot("Arm", 0, 1);
    armLigament = new MechanismLigament2d("ArmLigament", 1, 90);
    armMech.append(armLigament);

    // Log position setpoints for debugging
    for(ArmPosition pos : ArmPosition.values()) {
      SmartDashboard.putData("Arm/Setpoint/" + pos.name(), setPositionCommand(pos).ignoringDisable(true));
    }

    // Log PID for debugging
    SmartDashboard.putData("Arm/PID", pidController);

  }

  // private void setPosition(Angle position) {
  //   if(position.in(Rotations) > kMax.in(Rotations) || position.in(Rotations) < kMin.in(Rotations)) {
  //     System.out.println("Arm angle overshoot");
  //   };
  //   pid.setReference(position.in(Rotations), ControlType.kMAXMotionPositionControl);
  // }

  public void resetPID() {
    pidController.reset(encoder.getPosition());
  }

  public void setPosition(ArmPosition position) {
    setpoint = position;
  }

  public Angle getAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition().refresh().getValue();
  }


  public boolean atPosition() {
    return Math.abs(encoder.getPosition() - setpoint.getPosition().in(Rotations)) < kRotationEpsilon.in(Rotations);
  }

  public Command setPositionCommand(ArmPosition position) {
    return new InstantCommand(()->setPosition(position), this);

  }
  public Command setPostionWaitCommand(ArmPosition position) {
    return new InstantCommand(()->setPosition(position), this).until(this::atPosition);
  }

  @Override
  public void periodic() {
    // TODO: Try with absolute encoder instead of integrated encoder
    double pidResult = pidController.calculate(encoder.getPosition(), setpoint.getPosition().in(Rotations));
    // ? setpoint should be passed instead of pid value to ff?
    // ? Position should be offset 90deg to take gravity into account properly
    double ffResult = feedforward.calculate(setpoint.getPosition().plus(Degrees.of(-90 - 180)).in(Radians), pidResult);

    // ! DON'T APPLY VOLTAGE TO MOTOR BEFORE TESTING
    motor.setVoltage(pidResult + ffResult);
    // motor.setVoltage(0);

    // Update mechanism2d
    armLigament.setAngle(Rotation2d.fromRotations(encoder.getPosition()));
    SmartDashboard.putData("Arm/Mechanism", mech);

    // Update telemetry
    SmartDashboard.putNumber("Arm/MotEncPositionDeg", encoder.getPosition() * 360);
    SmartDashboard.putNumber("Arm/AbsEncPosition", getAbsolutePosition().in(Degrees));
    SmartDashboard.putNumber("Arm/SetpointPosition", setpoint.getPosition().in(Degrees));
    SmartDashboard.putBoolean("Arm/AtPosition", atPosition());
    SmartDashboard.putNumber("Arm/AppliedOutput", motor.getAppliedOutput());
    SmartDashboard.putNumber("Arm/PIDResult", pidResult);
    SmartDashboard.putNumber("Arm/FFResult", ffResult);
  }
}
