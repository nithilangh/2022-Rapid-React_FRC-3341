// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_TalonSRX _leftDriveMC;
  private WPI_TalonSRX _rightDriveMC;
  private WPI_VictorSPX _leftFollowerVictor;
  private WPI_VictorSPX _rightFollowerVictor;

  public DriveTrainSubsystem() {
    _leftDriveMC = new WPI_TalonSRX(Constants.CanID.DriveTrain.LeftDriveTalon);
    _rightDriveMC = new WPI_TalonSRX(Constants.CanID.DriveTrain.RightDriveTalon);
    _leftFollowerVictor = new WPI_VictorSPX(Constants.CanID.DriveTrain.LeftFollowerVictor);
    _rightFollowerVictor = new WPI_VictorSPX(Constants.CanID.DriveTrain.RightDriveTalon);

    _leftDriveMC.configFactoryDefault();
    _rightDriveMC.configFactoryDefault();
    _leftFollowerVictor.configFactoryDefault();
    _rightFollowerVictor.configFactoryDefault();

    _leftDriveMC.setInverted(false);
    _rightDriveMC.setInverted(true);
    _leftFollowerVictor.setInverted(false);
    _rightFollowerVictor.setInverted(true);

    _leftDriveMC.setSensorPhase(false);
    _rightDriveMC.setSensorPhase(false);
    _leftFollowerVictor.setSensorPhase(false);
    _rightFollowerVictor.setSensorPhase(false);

    _leftDriveMC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    _rightDriveMC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    _leftFollowerVictor.follow(_leftDriveMC);
    _rightFollowerVictor.follow(_rightDriveMC);
  }

  public void arcadeDrive(double speed, double rotation) {
   _leftDriveMC.set(ControlMode.PercentOutput, speed + rotation);
   _rightDriveMC.set(ControlMode.PercentOutput, speed - rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    _leftDriveMC.set(ControlMode.PercentOutput, leftSpeed);
    _rightDriveMC.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void stopMotors() {
   _leftDriveMC.set(ControlMode.PercentOutput, 0);
   _rightDriveMC.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
