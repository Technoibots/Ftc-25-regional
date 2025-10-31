package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    public enum Mode {
        POSITION,
        ANGLE,
        VELOCITY
    }

    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;
    private double output = 0;
    private double setPoint = 0;
    private double positionTolerance = 0;
    private boolean atSetPoint = false;
    private final Mode mode;
    private final int ticksPerRev;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD, double kF, Mode mode, int ticksPerRev) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.mode = mode;
        this.ticksPerRev = ticksPerRev;
    }

    public double calculate(double setPoint, double currentValue) {
        this.setPoint = setPoint;
        double error;

        if (mode == Mode.ANGLE) {
            error = angleWrap(setPoint - currentValue);
        } else {
            error = setPoint - currentValue;
        }

        double dt = timer.seconds();
        timer.reset();

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        output = (kP * error) + (kI * integralSum) + (kD * derivative) + kF;

        if (Math.abs(error) < positionTolerance) {
            atSetPoint = true;
        } else {
            atSetPoint = false;
        }

        return output;
    }

    // Aplica saída PID a um motor
    public void setPowerMotor(DcMotorEx motor) {
        motor.setPower(output);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    // Configurações de constantes
    public void setKP(double kP) { this.kP = kP; }
    public void setKI(double kI) { this.kI = kI; }
    public void setKD(double kD) { this.kD = kD; }
    public void setKF(double kF) { this.kF = kF; }

    public void setTolerance(double tolerance) { this.positionTolerance = tolerance; }

    public boolean atSetPoint() { return atSetPoint; }
}
