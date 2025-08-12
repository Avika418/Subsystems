// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//comment 1

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Reportable{
  private final TalonFX elevatorMotor;
  private final TalonFX elevatorMotor2;
   
  private double desiredPosition = 0.0;
  private boolean enabled = false;
  private TalonFXConfigurator motorConfigurator;
  private TalonFXConfigurator motorConfigurator2;
  private MotionMagicVoltage motionMagicRequest;
  private final Follower followRequest;
  private final NeutralOut neutralRequest = new NeutralOut();

  private double ff = 0.0;
  private double pivotAngle = 0.0;

  private NeutralModeValue neutralMode = NeutralModeValue.Brake;

  public Elevator(){
    elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorID, "rio");
    elevatorMotor2 = new TalonFX(ElevatorConstants.kElevatorMotorID2, "rio");
    motionMagicRequest = new MotionMagicVoltage(0);

    elevatorMotor.setPosition(0.0);

    motorConfigurator = elevatorMotor.getConfigurator();
    motorConfigurator2 = elevatorMotor2.getConfigurator();

    setMotorConfigs();

    followRequest = new Follower(ElevatorConstants.kElevatorMotorID, true);
    motionMagicRequest.withSlot(0);

    zeroEncoder();

    CommandScheduler.getInstance().registerSubsystem(this);
  }
  /** Creates a new ExampleSubsystem. */
  public void setMotorConfigs() {
    private boolean enabled = false;
    
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigurator.refresh(motorConfigs);
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfigs.Feedback.SensorToMechanismRatio = 16;
    motorConfigs.Feedback.RotorToSensorRatio = 1;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 45;
    motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    motorConfigs.MotorOutput.NeutralMode = neutralMode;
    motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kElevatorCruiseVelocity;
    motorConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorAcceleration;
    motorConfigs.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorJerk;

    motorConfigs.Slot0.kP = ElevatorConstants.kPElevatorMotor;
    motorConfigs.Slot0.kG = 0;
    motorConfigs.Slot0.kS = 0;
  }

  public void periodic(){
    if (!enabled) {
      return;
    }
    motionMagicRequest.Position = desiredPosition;


    ff = ElevatorConstants.kGElevatorMotor * Math.sin(pivotAngle * 2 * Math.PI);
    elevatorMotor.setControl(motionMagicRequest.withFeedForward(ff));
  }
  public void setTargetPosition(double position) {
    desiredPosition = position;
  }
  public double getTargetPosition(){
    return desiredPosition;
  }
  public double getPosition(){
    return elevatorMotor.getPosition().getValueAsDouble();
  }
  
  public class atSpeedHelper {
    public static boolean speedTooHigh(double getPosition, double gettargetPosition) {
      return getPosition > gettargetPosition;
    }

    public static void checkSpeed(double getPosition, double gettargetPosition) {
      if (speedTooHigh(getPosition, gettargetPosition)) {
        System.out.println("The current speed is exceeding the desired.");
      } else {
        System.out.println("The current speed is within the desired speed.");
      }
    }
  }