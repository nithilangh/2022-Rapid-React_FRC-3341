// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallHandlerSubsystem extends SubsystemBase {
  /** Creates a new BallHandler. */

  private WPI_TalonSRX _leftFlywheelMC;
  private WPI_TalonSRX _rightFlywheelMC;
  private WPI_VictorSPX _feederMC;
  private WPI_TalonSRX _pivotMC;

  public BallHandlerSubsystem() {
    _leftFlywheelMC = new WPI_TalonSRX(Constants.CanID.BallHandler.LeftFlywheel);
    _rightFlywheelMC = new WPI_TalonSRX(Constants.CanID.BallHandler.RightFlywheel);
    _feederMC = new WPI_VictorSPX(Constants.CanID.BallHandler.FeederTalon);
    _pivotMC = new WPI_TalonSRX(Constants.CanID.BallHandler.PivotTalon);

    _leftFlywheelMC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _rightFlywheelMC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _pivotMC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    _rightFlywheelMC.setInverted(true);
    _feederMC.setInverted(true);
    _pivotMC.setInverted(true);

    _pivotMC.setSensorPhase(false);    
    _pivotMC.setSelectedSensorPosition(0);

    _pivotMC.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    _pivotMC.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void pivotBallHandler(double pivotSpeed) {
    _pivotMC.set(ControlMode.PercentOutput, pivotSpeed);
  }

  public void actuateFlywheels(double rotateSpeed) {
    _leftFlywheelMC.set(ControlMode.PercentOutput, rotateSpeed);
    _rightFlywheelMC.set(ControlMode.PercentOutput, rotateSpeed);
  }

  public void setFlywheelVelocityRPM(double angularVelocityRPM) {
    double _positionsPer100MS = (angularVelocityRPM * Constants.Values.EncoderTicksPerRotation) / 600;
    _leftFlywheelMC.set(ControlMode.Velocity, _positionsPer100MS);
    _rightFlywheelMC.set(ControlMode.Velocity, _positionsPer100MS);
  }

  public void setFlywheelVelocityMPS (double angularVelocityMPS) {
    double _velocityPer100MS = (angularVelocityMPS * 600) / 4096;
    _leftFlywheelMC.set(ControlMode.Velocity, _velocityPer100MS);
    _rightFlywheelMC.set(ControlMode.Velocity, _velocityPer100MS);
  }

  public void actuateCollector(double rotateSpeed) {
    _feederMC.set(ControlMode.PercentOutput, rotateSpeed);
  }

  public double getPivotEncoderTicks() {
    return _pivotMC.getSelectedSensorPosition();
  }

  public double getPivotAngle() {
    double rotations = _pivotMC.getSelectedSensorPosition() % Constants.Values.EncoderTicksPerRotation;
    return ((Constants.Values.DegreesPerRotation * rotations) / Constants.Values.EncoderTicksPerRotation)
            * Constants.BallHandler.Pivot.GearRatio;
  }

  public double getLeftFlywheelVelocityRPM() {
    return (_leftFlywheelMC.getSelectedSensorVelocity() * 600) / 4096;
  }

  public double getRightFlywheelVelocityRPM () {
    return (_rightFlywheelMC.getSelectedSensorVelocity() * 600) / 4096;
  }

  public double getLeftFlywheelVelocityMPS () {
    return (getLeftFlywheelVelocityRPM() * Constants.BallHandler.Flywheel.FlywheelRadiusMeters);
  }

  public double getRightFlywheelVelocityMPS () {
    return (getRightFlywheelVelocityRPM() * Constants.BallHandler.Flywheel.FlywheelRadiusMeters);
  }

  public double getFlywheelVelocityMPS () {
    return (getLeftFlywheelVelocityMPS() + getRightFlywheelVelocityMPS()) / 2;
  }

  public double getCollectorVelocity() {
    double angularVelocity = _feederMC.getSelectedSensorVelocity();
    double linearVelocity = angularVelocity * Constants.Values.MillisecondsPerSecond * Constants.BallHandler.Collector.CollectorRadiusMeters;
    return linearVelocity;
  }
}
