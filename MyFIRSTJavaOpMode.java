package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Template: CHEESE OpMode", group="Linear Opmode")
public class MyFIRSTJavaOpMode extends LinearOpMode {
        private Gyroscope imu;
        private DcMotor[] motorTest;
        private DigitalChannel digitalTouch;
        private DistanceSensor sensorColorRange;
        private Servo servoTest;


        @Override
        public void runOpMode() {
            //imu = hardwareMap.get(Gyroscope.class, "imu");
            motorTest = new DcMotor[4];
            for (int i = 0; i < 4; i++) {
                motorTest[i] = hardwareMap.get(DcMotor.class, "motor" + i);
            }
            //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
            //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
            //servoTest = hardwareMap.get(Servo.class, "servoTest");

            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                if (this.gamepad1.x) {
                    telemetry.addData("Status", "X");
                } else if (this.gamepad1.y) {
                    telemetry.addData("Status", "Y");
                }
                double power0 = this.gamepad1.left_stick_x;
                motorTest[0].setPower(power0);
                double power1 = this.gamepad1.left_stick_y;
                motorTest[1].setPower(power1);
                double power2 = this.gamepad1.right_stick_x;
                motorTest[2].setPower(power2);
                double power3 = this.gamepad1.right_stick_y;
                motorTest[3].setPower(power3);
                telemetry.update();
            }
        }
    }
