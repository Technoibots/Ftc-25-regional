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
        // ---- Motores de locomoção ----
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // ---- Sub-sistemas ----
        intakeMotor   = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor  = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        bufferMotor   = hardwareMap.get(DcMotor.class, "bufferMotor");

        // ---- Direções ----
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
        bufferMotor.setDirection(DcMotor.Direction.FORWARD);

        // ---- Configurações iniciais ----
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Inicialização dos controladores PID ----
        pidShooter1 = new PIDController(0.4, 0.0, 0.0, 0.0, PIDController.Mode.VELOCITY, 28);
        pidShooter2 = new PIDController(0.4, 0.0, 0.0, 0.0, PIDController.Mode.VELOCITY, 28);

        telemetry.addData("Status", "Inicializado com sucesso!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== CONTROLE DO DRIVE TRAIN (gamepad1) =====
        double y  = -gamepad1.left_stick_y;  // frente (+), trás (-)
        double x  = gamepad1.left_stick_x;   // esquerda (-), direita (+)
        double rx = gamepad1.right_stick_x;  // giro

        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        frontLeft.setPower(frontLeftPower / max);
        backLeft.setPower(backLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backRight.setPower(backRightPower / max);

        // ===== CONTROLE DOS SUBSISTEMAS (gamepad2) =====
        // Intake
        if (gamepad2.a) {
            intakeMotor.setPower(0.5); // coleta
        } else if (gamepad2.b) {
            intakeMotor.setPower(-0.5); // reverso
            bufferMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
            if (!gamepad2.right_bumper) {
                bufferMotor.setPower(0);
            }
        }

        // Shooter com PID
        double targetVelocity = 2500; // ticks por segundo (ajuste conforme necessário)
        double output1 = pidShooter1.calculate(targetVelocity, shooterMotor.getVelocity());
        double output2 = pidShooter2.calculate(targetVelocity, shooterMotor2.getVelocity());

        if (gamepad2.right_trigger > 0.2) {
            shooterMotor.setPower(output1);
            shooterMotor2.setPower(output2);
        } else {
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
        }
        
        double targetVelocity2 = 6000; // ticks por segundo (ajuste conforme necessário)
        double output3 = pidShooter1.calculate(targetVelocity2, shooterMotor.getVelocity());
        double output4 = pidShooter2.calculate(targetVelocity2, shooterMotor2.getVelocity());

        if (gamepad2.left_trigger > 0.2) {
            shooterMotor.setPower(output3);
            shooterMotor2.setPower(output4);
        } else {
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
        }

        // Buffer no RB
        if (gamepad2.right_bumper) {
            bufferMotor.setPower(1);
        } else if (!gamepad2.b) {
            bufferMotor.setPower(0);
        }

        // ===== TELEMETRIA =====
        telemetry.addData("FL", frontLeft.getPower());
        telemetry.addData("FR", frontRight.getPower());
        telemetry.addData("BL", backLeft.getPower());
        telemetry.addData("BR", backRight.getPower());
        telemetry.addData("Intake", intakeMotor.getPower());
        telemetry.addData("Shooter1 vel", shooterMotor.getVelocity());
        telemetry.addData("Shooter2 vel", shooterMotor2.getVelocity());
        telemetry.addData("Shooter1 PID out", output1);
        telemetry.addData("Shooter2 PID out", output2);
        telemetry.addData("Buffer", bufferMotor.getPower());
        telemetry.update();
    }
}
