package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.*;

@TeleOp(name = "Mecanum Drive + Intake")
public class MecanumDrive extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx shooterMotor, shooterMotor2;
    private DcMotor bufferMotor;

    private PIDController pidShooter1, pidShooter2;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        intakeMotor   = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor  = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        bufferMotor   = hardwareMap.get(DcMotor.class, "bufferMotor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
        bufferMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidShooter1 = new PIDController(0.1, 0, 0, 0, PIDController.Mode.VELOCITY, 28);
        pidShooter2 = new PIDController(0.1, 0, 0, 0, PIDController.Mode.VELOCITY, 28);

        telemetry.addData("Status", "Inicializado!");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===== DRIVE =====
        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);

        // ===== INTAKE =====
        if (gamepad2.a) {
            intakeMotor.setPower(0.5);
        } else if (gamepad2.b) {
            intakeMotor.setPower(-0.5);
            bufferMotor.setPower(-1);
            shooterMotor.setPower(-1);
            shooterMotor2.setPower(-1);
        } else {
            intakeMotor.setPower(0);
            if (!gamepad2.right_bumper) {
                bufferMotor.setPower(0);
            }
        }
        
        if (gamepad1.y) {
            bufferMotor.setPower(1);
        }

        // ===== SHOOTER (2 motores, LT=2000, RT=1000) =====
        double targetVelocity = 0;

        if (gamepad2.left_trigger > 0.2) {
            targetVelocity = 1550;
        } else if (gamepad2.right_trigger > 0.2) {
            targetVelocity = 1300;
            if (gamepad2.right_bumper && Math.abs(shooterMotor.getVelocity() - targetVelocity)<50) {
                bufferMotor.setPower(1);
            }
        } else {
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
            targetVelocity = 0;
        }

        if (targetVelocity > 0) {
            double output1 = pidShooter1.calculate(targetVelocity, shooterMotor.getVelocity());
            double output2 = pidShooter2.calculate(targetVelocity, shooterMotor2.getVelocity());

            shooterMotor.setPower(output1);
            shooterMotor2.setPower(output2);
        }

        // ===== BUFFER =====
        if (gamepad2.right_bumper) {
            bufferMotor.setPower(1);
        } else if (!gamepad2.b) {
            bufferMotor.setPower(0);
        }

        // ===== TELEMETRIA =====
        telemetry.addData("Shooter1 Vel", shooterMotor.getVelocity());
        telemetry.addData("Shooter2 Vel", shooterMotor2.getVelocity());
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Buffer", bufferMotor.getPower());
        telemetry.update();
    }
}
