// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//comment 1

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FLyWheel extends SubsystemBase implements Reportable{
  @Override
  public void reportToSmartDashboard(LOG_LEVEL priority) {
    switch (level){
      case OFF:
        break;
      case ALL:
        SmartDashboard.putNumber("FLyWheel Velocity", getVelocity());
        SmartDashboard.putNumber("FLyWheel Supply", getSupply());
      case MEDIUM:
        SmartDashboard.putNumber("FLyWheel Target Velocity", getTargetVelocity());
        SmartDashboard.putBoolean("FLyWheel Enabled", enabled);
      case MINIMAL:
        SmartDashboard.putNumber("FLyWheel Voltage", MotionMagicVoltage());
        SmartDashboard.putNumber("FLyWheel Temperature", getTemperature());
        break;
    }
  }
  private final TalonFX FLyWheelMotor;
  private final TalonFX FLyWheelMotor2;
   
  private double desiredVelocity = 0.0;
  private boolean enabled = false;
  private TalonFXConfigurator motorConfigurator;
  private TalonFXConfigurator motorConfigurator2; 
  private MotionMagicVoltage motionMagicRequest;
  private final Follower followRequest;
  private final NeutralOut neutralRequest = new NeutralOut();

  private double ff = 0.0;
  private double pivotAngle = 0.0;

  private NeutralModeValue neutralMode = NeutralModeValue.Brake;
}
  public FLyWheel(){
    FLyWheelMotor = new TalonFX(FLyWheelConstants.kFLyWheelMotorID, "rio");
    FLyWheelMotor2 = new TalonFX(FLyWheelConstants.kFLyWheelMotorID2, "rio");
    motionMagicRequest = new MotionMagicVoltage(0);

    FLyWheelMotor.setVelocity(0.0);

    motorConfigurator = FLyWheelMotor.getConfigurator();
    motorConfigurator2 = FLyWheelMotor2.getConfigurator();

    setMotorConfigs();

    followRequest = new Follower(FLyWheelConstants.kFLyWheelID, true);
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
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = FLyWheelConstants.kFLyWheelCruiseVelocity;
    motorConfigs.MotionMagic.MotionMagicAcceleration = FLyWheelConstants.kFLyWheelAcceleration;
    motorConfigs.MotionMagic.MotionMagicJerk = FLyWheelConstants.kFLyWheelJerk;

    motorConfigs.Slot0.kP = FLyWheelConstants.kPFLyWheelMotor;
    motorConfigs.Slot0.kG = 0;
    motorConfigs.Slot0.kS = 0;
  }

  public void periodic(){
    if (!enabled) {
      return;
    }
    motionMagicRequest.Velocity = desiredVelocity;


    ff = FLyWheelConstants.kGFLyWheelMotor * Math.sin(pivotAngle * 2 * Math.PI);
    FLyWheelMotor.setControl(motionMagicRequest.withFeedForward(ff));
  }
  public void setTargetVelocity(double Velocity) {
    desiredVelocity = Velocity;
  }
  public double getTargetVelocity(){
    return desiredVelocity;
  }
  public double getVelocity(){
    return FLyWheelMotor.getVelocity().getValueAsDouble();
  }
  
  public class atSpeedHelper {
    public static boolean speedTooHigh(double getVelocity, double gettargetVelocity) {
      return getVelocity> gettargetVelocity;
    }

    public static void checkSpeed(double getVelocity, double gettargetVelocity) {
      if (speedTooHigh(getVelocity, gettargetVelocity)) {
        System.out.println("The current speed is exceeding the desired.");
      } else {
        System.out.println("The current speed is within the desired speed.");
      }
    }
  }