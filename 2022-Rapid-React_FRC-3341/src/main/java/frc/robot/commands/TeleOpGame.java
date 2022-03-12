// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallHandlerSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class TeleOpGame extends CommandBase {
  /** Creates a new TeleOpGame. */
  DriveTrainSubsystem _driveTrain;
  BallHandlerSubsystem _ballHandler;
  LimelightSubsystem _limelight;
  Joystick _driveTrainJoystick;
  Joystick _ballHandlerJoystick;

  private enum Action {
    None,
    PivotStow,
    PivotIntake,
    PivotScan,
    PivotStop,
    ScanStart,
    ScanStop,
    CollectStart,
    CollectStop,
    ShootStart,
    ShootStop
  }
  private Action _ongoingAction = Action.None;
  private Action _newAction = Action.None;

  double _actuateSpeed = 0.7;
  double _direction = Math.signum(0);
  double _joystickPower;
  boolean _isFinished = false;
  
  double _ballHandlerPivotAngle = 0;
  double _heightFromLimelightToHub = 0;
  double _distanceFromLimelightToHub = 0;
  double _heightFromFloorToPeakHeight = Constants.Field.HubHeightMeters + Constants.Field.BufferHeightMeters;
  double _timeOfFlight = 0;
  double _gravity = -9.8;
  double _launchVelocity = 0;
  double _launchAngle = 0;

  public TeleOpGame(DriveTrainSubsystem driveTrain, BallHandlerSubsystem ballHandler, LimelightSubsystem limelight, Joystick driveTrainJoystick, Joystick ballHandlerJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = driveTrain;
    _ballHandler = ballHandler;
    _limelight = limelight;
    _driveTrainJoystick = driveTrainJoystick;
    _ballHandlerJoystick = ballHandlerJoystick;

    addRequirements(_driveTrain, _ballHandler, _limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arcadeDrive();
    pivotBallHandler();

    getNewAction();
    executeAction();
    updateAction();

    if(_driveTrainJoystick.getRawButton(3)) {
      _ballHandler.actuateFlywheels(0.5);
      SmartDashboard.putNumber(" Flywheel Power", 0.5);
    }
    else {
      _ballHandler.actuateFlywheels(0);
    }

    if (_driveTrainJoystick.getRawButton(4)) {
      _ballHandler.actuateCollector(0.5);
    }
    else {
      _ballHandler.actuateCollector(0);
    }
  }

  private void getNewAction() {
    if(_ballHandlerJoystick.getRawButton(4)) {
      _newAction = Action.PivotStop;
    }
    else if(_ballHandlerJoystick.getRawButton(7)) {
      _newAction = Action.ShootStop;
    }
    else if(_ballHandlerJoystick.getRawButton(8)) {
      _newAction = Action.ScanStop;
    }
    else if(_ballHandlerJoystick.getRawButton(3)) {
      _newAction = Action.PivotIntake;
    }
    else if(_ballHandlerJoystick.getRawButton(5)) {
      _newAction = Action.PivotScan;
    }
    else if(_ballHandlerJoystick.getRawButton(6)) {
      _newAction = Action.PivotStow;
    }
    else if(_ballHandlerJoystick.getTop()) {
      _newAction = Action.ScanStart;
    }
    else if(_ballHandlerJoystick.getRawButton(2)) {
      _newAction = Action.CollectStart;
    }
    else if(_ballHandlerJoystick.getRawButton(1)) {
      _newAction = Action.ShootStart;
    }
    else {
      _newAction = Action.None;
    }
  }

  private void executeAction() {
    switch (_ongoingAction) {
      case PivotStow:
        pivotToStowPosition();
        break;
        
      case PivotIntake:
        pivotToIntakePosition();
        break;
      
      case PivotScan:
        pivotToScanPosition();
        break;
      
      case PivotStop:
        stopPivoting();
        break;
      
      case ScanStart:
        scanForTarget();
        break;
      
      case ScanStop:
        stopScanning();
        break;
      
      case CollectStart:
        collectCargo();
        break;
      
      case ShootStart:
        shootCargo();
        break;
      
      case CollectStop:
      case ShootStop:
        stopCollecting();
        break;
      
      case None:
        arcadeDrive();
        break;

      default:
        updateAction();
    }
  }

  private boolean canActionContinue () {
    return (_newAction == Action.None || _ongoingAction == _newAction) ? true : false;
  }

  private void updateAction() {
    _ongoingAction = _newAction;
  }

  private void resetDirection () {
    _direction = Math.signum(0);
  }

  private boolean isSameDirection (double currentDirection) {
    if (_direction == 0) _direction = Math.signum(currentDirection);
    return (Math.signum(_direction) == Math.signum(currentDirection));
  }

  private void arcadeDrive () {  
    _driveTrain.arcadeDrive(-1 * 0.8 * _driveTrainJoystick.getY(), 0.8 * _driveTrainJoystick.getX());
  }

  private void pivotBallHandler () {
    _ballHandler.pivotBallHandler(-1 * _ballHandlerJoystick.getRawAxis(Constants.JoystickAxis.YAxis));
  }

  private void stopPivoting() {
    _ballHandler.pivotBallHandler(0);
    updateAction();
  }

  private void pivotToPosition(double pivotAngle) {
    if (canActionContinue()) {
      double _offset = pivotAngle - _ballHandler.getPivotEncoderTicks();
// TBD TBD TBD TBD TBD
      if (isSameDirection(_offset)) {
        double _power = (Math.abs(_offset) > Constants.BallHandler.Pivot.EncoderDeadband) 
                          ? ((Math.signum(_offset) > 0) 
                                ? Constants.BallHandler.Pivot.FeedForwardPower - Constants.BallHandler.Pivot.PivotPower 
                                : Constants.BallHandler.Pivot.FeedForwardPower + Constants.BallHandler.Pivot.PivotPower)
                          : 0;
        _ballHandler.pivotBallHandler(_power);
      }
    }
    else {
      resetDirection();
      stopPivoting();
    }
  }

  private void pivotToStowPosition() {
    pivotToPosition(Constants.BallHandler.Pivot.StowedEncoderPosition);
  }

  private void pivotToIntakePosition() {
    pivotToPosition(Constants.BallHandler.Pivot.IntakeEncoderPosition);
  }

  private void pivotToScanPosition() {
    pivotToPosition(Constants.BallHandler.Pivot.ScanEncoderPosition);
  }

  private void stopScanning () {
    _driveTrain.arcadeDrive(0, 0);
    updateAction();
  }

  public void calculateShootingParameters() {
    _ballHandlerPivotAngle = 0; //TBD
    _heightFromLimelightToHub = Constants.Field.HubHeightMeters - (Constants.BallHandler.Measurements.PivotpointToFloorMeters 
                                                                    + (Constants.BallHandler.Measurements.LimelightToPivotpointMeters 
                                                                        * Math.sin(Constants.BallHandler.Measurements.LimelightToPivotpointDegrees)));
    _distanceFromLimelightToHub = _heightFromLimelightToHub / Math.tan(_limelight.getVerticalAngleOffset() + _ballHandlerPivotAngle);
    _timeOfFlight = 2 * Math.sqrt(2 * _heightFromFloorToPeakHeight / _gravity);;
    _launchVelocity = Math.sqrt(Math.pow(((2 * _distanceFromLimelightToHub) / _timeOfFlight), 2) + Math.pow((_gravity * _timeOfFlight / 2), 2));
    _launchAngle = Math.atan((_gravity * Math.pow(_timeOfFlight, 2)) / (2 * (2 * _distanceFromLimelightToHub)));
  }
  
  private void scanForTarget() {
    if (canActionContinue()) {
      double _offset = _limelight.getHorizontalAngleOffset();

      if (isSameDirection(_offset)) {
        double _power = (Math.abs(_offset) > 5) ? Constants.DriveTrain.TurnSpeed : 0;
        _driveTrain.arcadeDrive(0, Math.signum(_offset) * _power);
      }
      else {
        _driveTrain.arcadeDrive(0, 0);
        calculateShootingParameters();
      }
    }
    else {
      resetDirection();
      stopScanning();
    }
  }

  private void stopCollecting () {
    _ballHandler.actuateFlywheels(0);
    _ballHandler.actuateCollector(0);
    updateAction();
  }

  private void collectCargo () {
    _ballHandler.actuateFlywheels(Constants.BallHandler.Flywheel.IntakePower);
    _ballHandler.actuateCollector(Constants.BallHandler.Collector.IntakePower);
  }

  private void shootCargo () {
    _ballHandler.actuateFlywheels(1.0);
    SmartDashboard.putNumber("POV", _ballHandlerJoystick.getPOV());

    if (_ballHandlerJoystick.getPOV() == 0) {
      _ballHandler.actuateCollector(1.0);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
