/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: Iterative OpMode", group="Autonomous OpMode")
// @Disabled
public class AutoSL_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveFront = null;
    private DcMotor leftDriveRear = null;
    private DcMotor rightDriveFront= null;
    private DcMotor rightDriveRear = null;

    private BNO055IMU imu;

    private Orientation angles;
    private double start_hdg;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // jwk: create motor/drive variables with the correct names for each of the four motors
        leftDriveFront = hardwareMap.get(DcMotor.class, "lDriveFront");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rDriveFront");
        leftDriveRear = hardwareMap.get(DcMotor.class, "lDriveRear");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rDriveRear");


        leftDriveFront.setDirection(DcMotor.Direction.REVERSE); // Port 3
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD); // Port 2
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE); // Port 0
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD); // Port 1

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        start_hdg = angles.firstAngle;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // jwk: nothing to do in init loop just yet; remember, this happens continuoulsy until 'start' is pressed
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double vRightFront;
        double vLeftFront;
        double vRightRear;
        double vLeftRear;
        double curr_hdg;
        final double FWD = 1.0;
        final double REV = -1.0;
        double speed = 0.5; // arbitrary, just don't make it too close to 1.0
        double rotate = 0.0;
        final double Khdg = 0.01;

        final int sector = 1; // move to right for this test case


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        curr_hdg = angles.firstAngle;
        rotate = rotate + Khdg * (curr_hdg - start_hdg);

        // Now, set motor speeds based on sector stick is in
        switch (sector) {
            case 1: // move right
                vRightFront = FWD * speed;
                vLeftFront = REV * speed;
                vRightRear = REV * speed;
                vLeftRear = FWD * speed;
                break;

            case 2: // move forward
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

        vRightFront = Range.clip(vRightFront + rotate, -1.0, 1.0);
        vLeftFront = Range.clip(vLeftFront - rotate, -1.0, 1.0);
        vRightRear = Range.clip(vRightRear + rotate, -1.0, 1.0);
        vLeftRear = Range.clip(vLeftRear - rotate, -1.0, 1.0);

        setMotorPower(vRightFront, vLeftFront, vRightRear, vLeftRear);

        if( runtime.seconds() >= 10 ) {
            setMotorPower(0.0, 0.0, 0.0, 0.0);
            stop();
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    // perform .setPower here in method. Ensures that setPower is called only once per loop()
    public void setMotorPower ( double vRF, double vLF, double vRR, double vLR ){
        rightDriveFront.setPower(vRF);
        leftDriveFront.setPower(vLF);
        rightDriveRear.setPower(vRR);
        leftDriveRear.setPower(vLR);
    }

}
