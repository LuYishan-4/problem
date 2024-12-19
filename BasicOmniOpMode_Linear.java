

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;





@TeleOp(name="INTOTOTHEDICK2025",group="Linear OpMode")

public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor arm = null;
    private Servo into = null;

    private CRServo dick = null;
    private Servo claw = null;
    private Servo wrist = null;

    private static int fwp = 0;
    private int u = 0;
    private int d = 0;

    private int intofp = 0;

    private int t = 0;
    private final int fpos = 300;
    private static final int spos = 450;
    private static final int tpos = 1100;
    private static final int fspos = 1700;
    private static final int ftpos = 2100;
    private static final int sipos = 2500;
    private static final int sepos = 2600;


    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        arm = hardwareMap.get(DcMotor.class, "arm");
        into = hardwareMap.get(Servo.class, "into");
        dick = hardwareMap.get(CRServo.class, "dick");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "ww");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            int armud = (u - d) * -10;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double udud = gamepad1.right_trigger;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            double intood = udud;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            if (t < 0){
                t = 0;
            }


            if (gamepad1.a) {
                dick.setPower(100);
                t +=1;
            }else if(gamepad1.a){
                dick.setPower(fpos);//1
                t +=1;
            }else if (gamepad1.a) {
                dick.setPower(spos);//2
                t +=1;
            } else if (gamepad1.a){
                dick.setPower(tpos);//3
                t +=1;
            } else if (gamepad1.a) {
                dick.setPower(fspos);//4
                t +=1;
            } else if (gamepad1.a){
                dick.setPower(ftpos);//5
                t +=1;
            } else if (gamepad1.a){
                dick.setPower(sipos);//6
                t +=1;
            } else if (gamepad1.a){
                dick.setPower(sepos);//7
                t +=1;
            }
            if (gamepad1.b){
                if (t < 0){
                    t = 0;
                }if(t == 1){
                    dick.setPower(-100);
                    t -=1;
                }
                else if (t == 2){
                    dick.setPower(-200);
                    t -=1;
                }else if (t == 3){
                    dick.setPower(-150);
                    t -=1;
                }else if (t == 4){
                    dick.setPower(-650);
                    t -=1;
                }else if (t == 5){
                    dick.setPower(-600);
                    t -=1;
                }else if (t == 6){
                    dick.setPower(-400);
                    t -=1;
                }else if(t == 7){
                    dick.setPower(-400);
                    t -=1;
                }else if (t == 8){
                    dick.setPower(-100);
                    t -=1;
                } else if (t < 0) {
                    t =0;
                }


            }
            if (gamepad1.left_bumper){
                into.setPosition(intofp);
                intofp +=intood;
            }


            if (gamepad1.x) {
                claw.setPosition(0.55);
            } else if (gamepad1.y) {
                claw.setPosition(0.7);
            }

            if (gamepad1.right_bumper) {
                arm.setPower(armud);
                wrist.setPosition(0);
            } else if (gamepad1.left_stick_button) {
                arm.setPower(armud);
            } else if (gamepad1.dpad_up) {
                u += 1;
                arm.setPower(10);
            } else if (gamepad1.dpad_down) {
                d += 1;
                arm.setPower(-10);
            } else if (gamepad1.dpad_right) {
                wrist.setPosition(fwp);
                fwp += 1;
            } else if (gamepad1.dpad_left) {
                wrist.setPosition(fwp);
                fwp -= 1;
            }


            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            into.setPosition(intood);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("up/down", "%4.2f", dick);

            telemetry.update();
        }
    }
}

