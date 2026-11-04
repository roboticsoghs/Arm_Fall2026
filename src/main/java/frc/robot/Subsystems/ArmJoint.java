package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Telemetry;

public class ArmJoint extends SubsystemBase {
    private final TalonFX motor = new TalonFX(1);
    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);
    private final VoltageOut controlRequestRawVoltage = new VoltageOut(0);

    private DigitalInput limitSwitch = new DigitalInput(0);

    private static final double MIN_ANGLE = -180;
    private static final double MAX_ANGLE = 0;

    public ArmJoint() {
        configureArm();
    }

    private void configureArm() {
        // no weight config
        Slot0Configs slot0 = configs.Slot0;
        slot0.kS = 0.23; // voltage where arm barely moves
        slot0.kV = 0.0; // use telemetry at 10V - V/RPS
        slot0.kA = 0.0; 
        slot0.kG = -0.5; // make arm horizontal, apply voltage till arm doesn't drop
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // tune PID
        slot0.kP = 800; // 1200 
        slot0.kI = 0;
        slot0.kD = 0; // 0.65

        // weighted configs
        // Slot1Configs slot1 = configs.Slot1;
        // slot1.kS = 0.235; // voltage where arm barely moves
        // slot1.kV = 0.0; // use telemetry at 10V - V/RPS
        // slot1.kA = 0.0; 
        // slot1.kG = 0.235; // make arm horizontal, apply voltage till arm doesn't drop
        // // tune PID
        // slot1.kP = 0.0;
        // slot1.kI = 0.0;
        // slot1.kD = 0.0;

        // Telemetry.setupPIDTuning("arm", slot0.kP, slot0.kI, slot0.kD);

        MotionMagicConfigs mmConfigs = configs.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = 10.0;
        mmConfigs.MotionMagicAcceleration = 2.0;
        mmConfigs.MotionMagicJerk = 0.0;

        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configs.Feedback.SensorToMechanismRatio = Constants.ARM_MOTOR_RATIO;

        motor.getConfigurator().apply(configs);
    }

    // Degrees to rotation
    private double degreesToRotations(double targetDegrees) {
        return targetDegrees / 360;
    }

    // Angle getter (degrees)
    public double getAngle() {
        return motor.getPosition().getValueAsDouble() * 360;
    }

    // Zero relative encoder
    public Command zeroEncoder() {
        return this.runOnce(() -> motor.setPosition(0.0));
    }

    // Set arm angle in degrees within max and min thresholds
    public Command setArmAngle(double degrees) {
        double target = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, degrees));

        return this.run(() -> motor.setControl(controlRequest.withPosition(degreesToRotations(target)).withSlot(0)));
    }

    // Initial start up and zeroing
    public Command reset() {
        return this.run(() -> motor.setControl(controlRequestRawVoltage.withOutput(-0.5)))
            .withTimeout(0.5)
            .unless(() -> isAtHardLimit())
            .andThen(() -> motor.setControl(controlRequestRawVoltage.withOutput(0)))
            .andThen(() -> {
                zeroEncoder();
            })
            ;
        // return this.run(() -> motor.setControl(controlRequestRawVoltage.withOutput(0.75)))
        //     .until(() -> isAtZeroPoint())
        //     .andThen(this.run(() -> motor.setControl(controlRequestRawVoltage.withOutput(0.40)))
        //         .until(() -> !isAtZeroPoint())
        //     )
        //     .andThen(this.runOnce(() -> {
        //         motor.setControl(controlRequestRawVoltage.withOutput(0));
        //         motor.setPosition(0);
        //     }))
        //     .andThen(this.run(() -> motor.setControl(controlRequest.withPosition(0).withSlot(0))))
        //     .raceWith(
        //         Commands.run(() -> {
        //             double current = motor.getStatorCurrent().getValueAsDouble();
        //             double velocity = motor.getVelocity().getValueAsDouble();
                    
        //             if (current > 50.0 && Math.abs(velocity) < 0.12) {
        //                 motor.setControl(new NeutralOut());
        //                 Telemetry.flashAlert("Arm", "Stall Warn", true, 255, 0, 0);
        //             }
        //         })
        //     );
    }

    // Detect by limit switch
    public boolean isAtZeroPoint() {
        return !limitSwitch.get();
    }

    public boolean isAtHardLimit() {
        return motor.getStatorCurrent().getValueAsDouble() > 50.0 && Math.abs(motor.getVelocity().getValueAsDouble()) > 0.8;
    }

    @Override
    public void periodic() {
        Telemetry.putNumber("Arm", "Relative Angle", motor.getPosition().getValueAsDouble() * 360);
        Telemetry.putNumber("Arm", "Absolute Angle", (motor.getPosition().getValueAsDouble() * 360) + 90);
        Telemetry.putNumber("Arm", "Temperature", motor.getDeviceTemp().getValueAsDouble());
        Telemetry.putNumber("Arm", "Current", motor.getStatorCurrent().getValueAsDouble());
        Telemetry.putNumber("Arm", "Velocity", motor.getVelocity().getValueAsDouble());

        Telemetry.putBoolean("Arm", "LimitSwitch", isAtZeroPoint());

        Telemetry.putNumber("Arm", "Volts", motor.getMotorVoltage().getValueAsDouble());


        double motor_current = motor.getStatorCurrent().getValueAsDouble();
        double motor_velocity = motor.getVelocity().getValueAsDouble();

        // if (isAtPhysicalLimit()) {
        //     motor.setControl(controlRequest.withPosition(0));
        // }
    }
}
