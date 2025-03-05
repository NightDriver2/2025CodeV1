package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  SparkMax indexMotor = new SparkMax(12, MotorType.kBrushless);
  SparkMaxConfig configIndex = new SparkMaxConfig();

  public Index() {
    configIndex
    .inverted(false)
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(40);

    indexMotor.configure(configIndex, null, null);

  }

  @Override
  public void periodic() {}

  public void indexMove() {
    indexMotor.set(.25);
  }

  public void indexStop() {
    indexMotor.set(0);
  }


}
