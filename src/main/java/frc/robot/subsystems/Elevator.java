package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  SparkMax elevatormotor = new SparkMax(10, MotorType.kBrushless);
  SparkMaxConfig configelevator1 = new SparkMaxConfig();

  Encoder relativeEncoder = new Encoder(0, 1) ;

  ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  
  PIDController pidElevator = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  double position = 0;

  public Elevator() {
    configelevator1
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(35);

    elevatormotor.configure(configelevator1, null, null);

    relativeEncoder.setDistancePerPulse((ElevatorConstants.distancePerPulse*100)/2);
    relativeEncoder.setReverseDirection(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Relativeencoder", relativeEncoder.getDistance());

    elevatormotor.set(pidElevator.calculate(relativeEncoder.getDistance(), position));

  }

  public void moveElevator() {
    elevatormotor.set(.9);

  }

  public void goDownElevator() {
    elevatormotor.set(-.375);
  }

  public void elevatorStop(){
    elevatormotor.set(0);
  }

  //PID

  public Command goToPosition(double Position) {
    return runOnce(()-> {position = Position;});
  }
}
