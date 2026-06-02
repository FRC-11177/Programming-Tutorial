package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    // 馬達 ID 清單，順序要和實體接線一致。
    public static final List<Integer> MoorIDs = List.of(11,12,13,14); //FL,BL,FR,BR
    // 輪子周長，後面會拿來把轉速/位置換算成實際線速度與距離。
    public static final Distance WheelCirc = Inches.of(6).times(Math.PI);
    public static final double GearRatio = 10.71;
    public static final Distance WheelDistance = Centimeters.of(60);
    // DifferentialDriveKinematics 會把機器人整體速度轉成左右輪速度。
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(WheelCirc);
    // 最高速度與角速度限制，避免指令超出機構可承受範圍。
    public static final LinearVelocity MaxVelocity = WheelCirc.times(5676/GearRatio/60).per(Seconds);
    public static final AngularVelocity MaxOmega = RotationsPerSecond.of(1.5);
    // PID、前饋與動作限制設定，會直接送進 SparkMAX 的閉迴路控制。
    public static final ClosedLoopConfig DrivePID = new ClosedLoopConfig()
        .pid(0, 0, 0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    public static final FeedForwardConfig DriveFF = new FeedForwardConfig()
        .sva(0, 0, 0);
    public static final MAXMotionConfig DriveMotion = new MAXMotionConfig()
        .maxAcceleration(0);
    public static final SparkBaseConfig LeadingMotorConfig = new SparkMaxConfig()
        .apply(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(40))
        .apply(
            new EncoderConfig()
                .positionConversionFactor(1/GearRatio)
                .velocityConversionFactor(1/GearRatio/60)
        ).apply(DrivePID.apply(DriveFF).apply(DriveMotion));

}
