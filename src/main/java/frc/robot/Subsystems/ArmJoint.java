package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Telemetry;

public class ArmJoint extends SubsystemBase {
    private final TalonFX motor = new TalonFX(1);
    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);

    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 90.0;

    public ArmJoint() {
        configureArm();
    }

    private void configureArm() {
        Slot0Configs slot0 = configs.Slot0;
        // use phoenix tuner to tune these values
        slot0.kS = 0.23; // voltage where arm barely moves
        slot0.kV = 0.0; // use telemetry at 10V - V/RPS
        slot0.kA = 0.0; 
        slot0.kG = 0.223; // make arm horizontal, apply voltage till arm doesn't drop
        // tune PID
        slot0.kP = 40; 
        slot0.kI = 5;
        slot0.kD = 0.65;

        Slot1Configs slot1 = configs.Slot1;
        // use phoenix tuner to tune these values
        slot1.kS = 0.23; // voltage where arm barely moves
        slot1.kV = 0.0; // use telemetry at 10V - V/RPS
        slot1.kA = 0.0; 
        slot1.kG = 0.223; // make arm horizontal, apply voltage till arm doesn't drop
        // tune PID
        slot1.kP = 10;
        slot1.kI = 5;
        slot1.kD = 0.65;

        // Telemetry.setupPIDTuning("arm", slot0.kP, slot0.kI, slot0.kD);

        MotionMagicConfigs mmConfigs = configs.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = 1.0;
        mmConfigs.MotionMagicAcceleration = 2.0;
        mmConfigs.MotionMagicJerk = 0.0;

        configs.Feedback.SensorToMechanismRatio = Constants.ARM_MOTOR_RATIO;

        motor.getConfigurator().apply(configs);
    }

    private double degreesToRotations(double targetDegrees) {
        return targetDegrees / 360;
    }

    public Command zeroEncoder() {
        return this.runOnce(() -> motor.setPosition(0.0));
    }

    public Command setArmAngle(double degrees) {
        double target = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, degrees));

        return this.run(() -> motor.setControl(controlRequest.withPosition(degreesToRotations(target)).withSlot(1)));
    }

    public Command pullNewPID() {
        return this.runOnce(() -> {
            double[] pidValues = Telemetry.getPIDValues("arm");

            configs.Slot0.kP = pidValues[0];
            configs.Slot0.kI = pidValues[1];
            configs.Slot0.kD = pidValues[2];

            motor.getConfigurator().apply(configs);

            System.out.println("Successfully pushed new PID values from Dashboard to Motor!");
        });
    }

    @Override
    public void periodic() {
        Telemetry.putNumber("Arm", "Angle", motor.getPosition().getValueAsDouble() * 360);
        Telemetry.putNumber("Arm", "Temperature", motor.getDeviceTemp().getValueAsDouble());
    }
}
