package frc.robot.subsystems;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  SparkMax elevatormotor1 = new SparkMax(10, MotorType.kBrushless);
  SparkMax elevatormotor2 = new SparkMax(11, MotorType.kBrushless);

  SparkMaxConfig configelevator1 = new SparkMaxConfig();
  SparkMaxConfig configelevator2 = new SparkMaxConfig();

  public Elevator() {
  configelevator1
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(35);

  configelevator2
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(35);

  elevatormotor1.configure(configelevator1, null, null);
  elevatormotor2.configure(configelevator1, null, null);


  }

  @Override
  public void periodic() {

  }

  public void moveElevator() {
    elevatormotor1.set(.5);
    elevatormotor2.set(.5);
  }

  public void goDownElevator() {
    elevatormotor1.set(-.5);
    elevatormotor2.set(-.5);
  }

  public void elevatorStop(){
    elevatormotor1.set(0);
    elevatormotor2.set(0);
  }
}
