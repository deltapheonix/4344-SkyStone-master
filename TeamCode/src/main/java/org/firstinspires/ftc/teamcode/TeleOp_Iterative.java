package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Order 67", group = "TeleOp")

public class TeleOp_Iterative extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDriveFwrd = null; // foward left drive motor
    public DcMotor rightDriveFwrd = null; // forward right drive motor
    public DcMotor leftDriveRear = null; //  Rear left drive motor
    public DcMotor rightDriveRear = null; // Rear right drive motor
    public DcMotor LiftMotor = null;
    public CRServo Grabber = null;
    public CRServo Slide;
    public boolean a;
    public double Release = 1;
    public double rightDpadDrive = 0.25;
    public double leftDpadDrive = 0.25;

    // variables to hold the current stick positions
    public double lStickX;
    public double lStickY;
    public double rStickX;
    public double r_stick_x;
    // variables to hold the respective motor speeds
    public double vRightFront;
    public double vLeftFront;
    public double vRightRear;
    public double vLeftRear;

    public final double dBand = 0.1; // define a deadband value for the stick
    public final double FWD = 1.0; // scaling factor depending on direction of travel
    public final double REV = -1.0;
    public int sector;


    @Override
    public void init() {
        leftDriveFwrd = hardwareMap.get(DcMotor.class, "lDriveFront");
        rightDriveFwrd = hardwareMap.get(DcMotor.class, "rDriveFront");
        leftDriveRear = hardwareMap.get(DcMotor.class, "lDriveRear");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rDriveRear");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        Grabber = hardwareMap.get(CRServo.class, "don't know the name");
        leftDriveFwrd.setDirection(DcMotor.Direction.REVERSE); // jwk
        rightDriveFwrd.setDirection(DcMotor.Direction.FORWARD); // jwk
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE); // jwk
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD); // jwk
        LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void init_loop() {
        //manipulatorClaw.setPosition(Grip);
        // BasketRelease.setPosition(Release);
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {


        // jwk: read the stick positions only once per loop
        lStickX = gamepad1.left_stick_x; // Left stick translates bot
        lStickY = -gamepad1.left_stick_y; // make forward stick positive value
        rStickX = gamepad1.right_stick_y; // Right stick rotates bot
        //r_stick_x = gamepad2.left_sticak_y;


        Double speed = Range.clip(Math.hypot(lStickX, lStickY), -1, 1); // magnitude of the stick input
        Double rotate = Range.clip(rStickX, -1, 1); // make sure right stick is withing +/-1
        Double stickAngle = Math.atan2(lStickY, lStickX); // stick angle in radians, 0 deg to right
        Double stickAngleEdge = stickAngle * 4 / Math.PI; // find the edge value of the sectorF
//was previously eight


        //manipulatorClaw.setPosition(manPosTemp);
        // Now, determine which sector we are in based on the stick angle
        if (stickAngleEdge >= -1 && stickAngleEdge < 1) {
            sector = 1;
        }

        if (stickAngleEdge >= 1 && stickAngleEdge < 3) {
            sector = 2;
        }

        if (stickAngleEdge >= 3 || stickAngleEdge < -3) {
            sector = 3;
        }

        if (stickAngleEdge >= -3 && stickAngleEdge < -1) {
            sector = 4;
        }

        // Now, set motor speeds based on sector stick is in
        switch (sector) {
            case 1: // move right
                vRightFront = FWD * speed;
                vLeftFront = REV * speed;
                vRightRear = REV * speed;
                vLeftRear = FWD * speed;
                break;

            case 2: // move forward, right
                vRightFront = FWD * speed;
                vLeftFront = FWD * speed;
                vRightRear = FWD * speed;
                vLeftRear = FWD * speed;
                //  manPosTemp = Release;
                break;

            case 3: // move left
                vRightFront = REV * speed;
                vLeftFront = FWD * speed;
                vRightRear = FWD * speed;
                vLeftRear = REV * speed;
                break;

            case 4: // move reverse
                vRightFront = REV * speed;
                vLeftFront = REV * speed;
                vRightRear = REV * speed;
                vLeftRear = REV * speed;
                // manPosTemp = Release;
                break;

            default: // should never get here, but don't move if we do!
                vRightFront = 0.0;
                vLeftFront = 0.0;
                vRightRear = 0.0;
                vLeftRear = 0.0;

                break;


        }

        if (speed < dBand && rotate > -dBand) { // if speed too low, irrespective of sector, set motors to zero
            vRightFront = 0;
            vLeftFront = 0;
            vRightRear = 0;
            vLeftRear = 0;
        }

        // superimpose the rotation component onto the command, but never exceed -1/1
        if (rotate > dBand || rotate < -dBand) {
            vRightFront = Range.clip(vRightFront + rotate, -1.0, 1.0);
            vLeftFront = Range.clip(vLeftFront - rotate, -1.0, 1.0);
            vRightRear = Range.clip(vRightRear + rotate, -1.0, 1.0);
            vLeftRear = Range.clip(vLeftRear - rotate, -1.0, 1.0);

        }

        // Now, set the motor power. Setting in a method on a hunch that servicing the
        // .setPower in main loop() causes some wonkiness
        setMotorPower(vRightFront, vLeftFront, vRightRear, vLeftRear);

    }

    // perform .setPower here in method. Ensures that setPower is called only once per loop()
    public void setMotorPower(double vRF, double vLF, double vRR, double vLR) {
        rightDriveFwrd.setPower(vRF);
        leftDriveFwrd.setPower(vLF);
        rightDriveRear.setPower(vRR);
        leftDriveRear.setPower(vLR);
    }
}