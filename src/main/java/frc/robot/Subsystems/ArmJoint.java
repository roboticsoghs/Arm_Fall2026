package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmJoint extends SubsystemBase {
    private final TalonFX motor = new TalonFX(1);
    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);

    public ArmJoint() {
        configureArm();
    }

    private void configureArm() {
        Slot0Configs slot0 = configs.Slot0;
        // use phoenix tuner to tune these values
        slot0.kS = 0.0; // voltage where arm barely moves
        slot0.kV = 0.0; // use telemetry at 10V - V/RPS
        slot0.kA = 0.0; 
        slot0.kG = 0.0; // make arm horizontal, apply voltage till arm doesn't drop
        // tune PID
        slot0.kP = 0.0; 
        slot0.kI = 0.0;
        slot0.kD = 0.0;

        MotionMagicConfigs mmConfigs = configs.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = 10.0;
        mmConfigs.MotionMagicAcceleration = 20.0;
        mmConfigs.MotionMagicJerk = 0.0;

        motor.getConfigurator().apply(configs);
    }

    private double degreesToRotations(double targetDegrees) {
        return targetDegrees / 360;
    }

    public Command zeroEncoder() {
        return this.runOnce(() -> motor.setPosition(0.0));
    }

    public Command setArmAngle(double degrees) {
        return this.run(() -> motor.setControl(controlRequest.withPosition(degreesToRotations(degrees))));
    }

    @Override
    public void periodic() {

    }
}
