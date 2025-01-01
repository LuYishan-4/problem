

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp
        (name="INTOTOTHEDxsxasxIK2025",group="Linear OpMode")

public class drive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfDrive = null;

    private DcMotor rightfDrive = null;

    private DcMotor leftbDrive = null;

    private DcMotor rightbDrive = null;


    private DcMotor arm = null;
    private Servo intake = null;

    private Servo claw = null;
    private DcMotor wrist = null;

    double tick = 2786.2;
    double newt;



    @Override
    public void runOpMode() {

        leftfDrive = hardwareMap.get(DcMotor.class, "lfd");
        rightfDrive = hardwareMap.get(DcMotor.class, "rfd");
        leftbDrive = hardwareMap.get(DcMotor.class, "lbd");
        rightbDrive = hardwareMap.get(DcMotor.class, "rbd");




        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(Servo.class, "into");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "ww");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftfDrive.setDirection(DcMotor.Direction.REVERSE);
        leftbDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfDrive.setDirection(DcMotor.Direction.FORWARD);
        rightbDrive.setDirection(DcMotor.Direction.FORWARD);







        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double ps = 0;




            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftfPower = axial + lateral + yaw;
            double rightfPower = axial - lateral - yaw;
            double leftbPower = axial - lateral + yaw;
            double rightbPower = axial + lateral - yaw;




            max = Math.max(Math.abs(leftfPower), Math.abs(rightfPower));
            max = Math.max(max, Math.abs(leftbPower));
            max = Math.max(max, Math.abs(rightbPower));



            if (max > 1.0) {
                leftfPower /= max;
                rightfPower /= max;
                leftbPower /= max;
                rightbPower /= max;
            }
            if (gamepad1.back){
                newt= tick/2;
                arm.setTargetPosition((int)newt);
                arm.setPower(0.3);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }




            if (gamepad1.square) {
                claw.setPosition(0.35);
            } else if (gamepad1.triangle) {
                claw.setPosition(-0.7);
            }



            if (gamepad1.left_bumper) {
                intake.setPosition(1);
            }else if (gamepad1.right_bumper){
                intake.setPosition(-1);
            }
            if (gamepad1.dpad_up) {

                arm.setPower(-0.6);
            } else if (gamepad1.dpad_down) {

                arm.setPower(0.6);
            }else {
                arm.setPower(0);
            }

            if (gamepad1.dpad_left){

                wrist.setPower(-0.6);
            }else if (gamepad1.dpad_right){

                wrist.setPower(0.6);
            }else {
                wrist.setPower(0);
            }




            leftfDrive.setPower(leftfPower);
            leftbDrive.setPower(leftbPower);
            rightfDrive.setPower(rightfPower);
            rightbDrive.setPower(rightbPower);








            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftfPower, rightfPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftbPower, rightbPower);

            telemetry.update();
        }
    }
}
