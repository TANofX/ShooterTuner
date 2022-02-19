// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ccsdbraves.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTuner extends SubsystemBase {
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("kF", this::getF, this::setF);
    builder.addDoubleProperty("kP", this::getP, this::setP);
    builder.addDoubleProperty("kI", this::getI, this::setI);
    builder.addDoubleProperty("kD", this::getD, this::setD);
    builder.addDoubleProperty("iZone", this::getIzone, this::setIzone);
    builder.addDoubleProperty("Suggested kF", this::suggested_kF, null);
  }

  private WPI_TalonFX primaryShooterMotor;
  private WPI_TalonFX secondaryShooterMotor;

  private SlotConfiguration motorConfiguration;

  private double targetVelocity;

  /** Creates a new ShooterTuner. */
  public ShooterTuner() {
    primaryShooterMotor = new WPI_TalonFX(10);
    secondaryShooterMotor = new WPI_TalonFX(11);

    configureFalcon(primaryShooterMotor);
    configureFalcon(secondaryShooterMotor);

    secondaryShooterMotor.follow(primaryShooterMotor, FollowerType.PercentOutput);
    secondaryShooterMotor.setInverted(TalonFXInvertType.OpposeMaster);
  }

  private void configureFalcon(WPI_TalonFX motor) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 39, 60, 0.01));
    motor.selectProfileSlot(0, 0);
    motor.config_kF(0, 0, 0);
    motor.config_kP(0, 0, 0);
    motor.config_kI(0, 0, 0);
    motor.config_kD(0, 0, 0);
    motor.configAllowableClosedloopError(0, 0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    primaryShooterMotor.getSlotConfigs(motorConfiguration, 0, 0);

    SmartDashboard.putNumber("Target Velocity", targetVelocity);
    SmartDashboard.putNumber("Actual Velocity", getVelocity());
    SmartDashboard.putNumber("Velocity Error", getVelocityError());
    SmartDashboard.putNumber("Applied Throttle", primaryShooterMotor.get() * 1023);
  }

  public double getF() {
    return motorConfiguration.kF;
  }

  public void setF(double F) {
    primaryShooterMotor.config_kF(0, F);
  }

  public double getP() {
    return motorConfiguration.kP;
  }

  public void setP(double P) {
    primaryShooterMotor.config_kP(0, P);
  }

  public double getI() {
    return motorConfiguration.kI;
  }

  public void setI(double I) {
    primaryShooterMotor.config_kI(0, I);
  }

  public double getD() {
    return motorConfiguration.kD;
  }

  public void setD(double D) {
    primaryShooterMotor.config_kD(0, D);
  }

  public double getIzone() {
    return motorConfiguration.integralZone;
  }

  public void setIzone(double izone) {
    primaryShooterMotor.config_IntegralZone(0, izone);
  }

  public void runMotor(double speed) {
    primaryShooterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void motorVelocity(double velocity) {
    targetVelocity = velocity;
    primaryShooterMotor.set(ControlMode.Velocity, velocity);
  }

  public double getVelocityError() {
    return targetVelocity - primaryShooterMotor.getSelectedSensorVelocity();
  }

  public double getVelocity() {
    return primaryShooterMotor.getSelectedSensorVelocity();
  }

  public double suggested_kF() {
    return (primaryShooterMotor.get() * 1023) / getVelocity();
  }
}
