package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.groups.Groups.parallel;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Launcher implements Mechanism {

    public enum Projectile {
        Pollen,
        Nectar,
    }

    private final NextMotor leftMotor = new NextMotor("launcherLeftMotor");
    private final NextMotor rightMotor = new NextMotor("launcherRightMotor");
    private final NextServo servo = new NextServo("launcherServo");

    // Converts projectile velocity (m/s) to motor velocity (rad / s)
    private final double POLLEN_VELOCITY_MULTIPLIER = 20;
    private final double NECTAR_VELOCITY_MULTIPLIER = 20;

    private final double ANGLE_OFFSET = 0;
    private final double ANGLE_MULTIPLIER = 1;

    // Relative tolerance between target velocity and motor velocity
    private final double TOLERANCE = 0.01;

    private double targetVelocity = 0;
    private double leftVelocity = 0;
    private double rightVelocity = 0;


    public Launcher() {
        // NOTE: This needs to be set to BRAKE if motors should not spin after launching due to
        // reasons such as the gyroscope effect
        leftMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets the velocity of the two motors to prepare for launching projectiles.
     *
     * @param velocity   the velocity of the projectile in m/s
     * @param projectile the type of projectile
     */
    public Command setVelocity(double velocity, Projectile projectile) {
        targetVelocity = getVelocityMultiplier(projectile) * velocity;
        return parallel(
                instant(() ->
                        rightMotor.setVelocitySetpoint(Units.RadiansPerSecond.of(targetVelocity))
                ),
                instant(() ->
                        leftMotor.follow(rightMotor, NextMotor.Direction.REVERSE)
                )
        );
    }

    /**
     * "Relaxes" the launcher by allowing the motors to spin freely.
     */
    public Command relax() {
        // NOTE: This may need to use setVelocitySetpoint (which uses a PID controller) rather than
        // setThrottle if ZeroPowerBehavior is BRAKE to prevent a large braking force
        targetVelocity = 0;
        return parallel(
                instant(() -> leftMotor.setThrottle(0)),
                instant(() -> rightMotor.setThrottle(0))
        );
    }

    /**
     * Sets the angle of the launcher by moving the servo.
     *
     * @param angle the angle of the launcher in radians where 0 is horizontal
     */
    public Command setAngle(double angle) {
        double position = Math.min(1, Math.max(0,
                (angle * ANGLE_MULTIPLIER + ANGLE_OFFSET) / (2 * Math.PI)
        ));
        return instant(() -> servo.setPosition(position));
    }

    /**
     * Checks if the launcher is ready where both motors have reached the target velocity.
     *
     * @return true if the launcher is ready, false otherwise
     */
    public boolean isReady() {
        return Math.abs(leftVelocity / targetVelocity - 1) <= TOLERANCE
                && Math.abs(rightVelocity / targetVelocity - 1) <= TOLERANCE;
    }

    @Override
    public void periodic() {
        leftVelocity = leftMotor.getEncoderVelocity().into(Units.RadiansPerSecond);
        rightVelocity = rightMotor.getEncoderVelocity().into(Units.RadiansPerSecond);
    }

    private double getVelocityMultiplier(@NonNull Projectile projectile) {
        switch (projectile) {
            case Pollen:
                return POLLEN_VELOCITY_MULTIPLIER;
            case Nectar:
                return NECTAR_VELOCITY_MULTIPLIER;
            default:
                throw new IllegalStateException();
        }
    }
}
