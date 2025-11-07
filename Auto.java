package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Auto - Drive + Shoot")
public class Auto extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooterMotor, shooterMotor2;
    private DcMotor bufferMotor;

    private PIDController pidShooter1, pidShooter2;

    private long actionStartTime = 0;
    private boolean inAction = false;
    private int step = 0;

    @Override
    public void init() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterMotor  = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        bufferMotor   = hardwareMap.get(DcMotor.class, "bufferMotor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        pidShooter1 = new PIDController(1.8, 0, 0, 0, PIDController.Mode.VELOCITY, 28);
        pidShooter2 = new PIDController(1.8, 0, 0, 0, PIDController.Mode.VELOCITY, 28);

        stopDrive();
        stopShoot();

        telemetry.addLine("Autônomo pronto");
    }

    @Override
    public void start() {
        drive(0.4, 1900);
        step = 1;
    }

    @Override
    public void loop() {

        if (inAction) {
            long elapsed = System.currentTimeMillis() - actionStartTime;

            if (step == 1) {
                if (elapsed < actionDuration) {
                    setDrivePower(actionPower);
                } else {
                    stopDrive();
                    inAction = false;

                    shootBall(1100, 3000); // Shooters + buffer juntos
                    step = 2;
                }
            }

            // ===== PASSO 2: Shooter + Buffer ao mesmo tempo =====
            else if (step == 2) {
                if (elapsed < actionDuration) {
                    double out1 = pidShooter1.calculate(targetVelocity, shooterMotor.getVelocity());
                    double out2 = pidShooter2.calculate(targetVelocity, shooterMotor2.getVelocity());

                    shooterMotor.setPower(out1);
                    shooterMotor2.setPower(out2);
                    bufferMotor.setPower(1); // Liga juntos
                } else {
                    stopShoot();
                    inAction = false;
                    step = 3;
                }
            }
        }

        telemetry.addData("Step", step);
        telemetry.update();
    }

    // FUNÇÕES DE AÇÃO
    private double actionPower = 0;
    private long actionDuration = 0;

    public void drive(double power, long timeMs) {
        actionPower = power;
        actionDuration = timeMs;
        actionStartTime = System.currentTimeMillis();
        inAction = true;
    }

    private void setDrivePower(double p) {
        frontLeft.setPower(p);
        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);
    }

    private void stopDrive() {
        setDrivePower(0);
    }

    // SHOOTER + BUFFER
    private double targetVelocity;

    public void shootBall(double targetVel, long runTime) {
        targetVelocity = targetVel;
        actionDuration = runTime;
        actionStartTime = System.currentTimeMillis();
        inAction = true;
    }

    private void stopShoot() {
        shooterMotor.setPower(0);
        shooterMotor2.setPower(0);
        bufferMotor.setPower(0);
    }
}
