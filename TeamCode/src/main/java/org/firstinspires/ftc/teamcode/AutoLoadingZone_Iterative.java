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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.code.BasicBlockList;

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
public class AutoLoadingZone_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveFront = null;
    private DcMotor leftDriveRear = null;
    private DcMotor rightDriveFront= null;
    private DcMotor rightDriveRear = null;

    private BNO055IMU imu;
    private Orientation angles;
    private double desiredHdg;

    private DistanceSensor sens2mDist;
    private DigitalChannel digitalTouch;

    private ColorSensor rColorSensor;
    private ColorSensor lColorSensor;

    private DistanceSensor rDistSensor;
    private DistanceSensor lDistSensor;

    private final double FWD = 1.0;
    private final double REV = -1.0;

    private boolean BLUE_START;


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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        leftDriveFront = hardwareMap.get(DcMotor.class, "lDriveFront");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rDriveFront");
        leftDriveRear = hardwareMap.get(DcMotor.class, "lDriveRear");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rDriveRear");

        sens2mDist = hardwareMap.get(DistanceSensor.class, "sens2mDist");

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        rColorSensor = hardwareMap.get(ColorSensor.class, "r_color_dist");
        rDistSensor = hardwareMap.get(DistanceSensor.class, "r_color_dist");

        lColorSensor = hardwareMap.get(ColorSensor.class, "l_color_dist");
        lDistSensor = hardwareMap.get(DistanceSensor.class, "l_color_dist");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE); // Port 3
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD); // Port 2
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE); // Port 0
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD); // Port 1
        desiredHdg = 0.0; // assume imu initialized at 0 deg heading angle

        if( digitalTouch.getState() ) { // if the digital channel returns true, the button is unpressed.
            BLUE_START = false; // if button is unpressed, then starting from Red Alliance side
        }
        else {
            BLUE_START = true; // if button is pressed, then starting from Blue Alliance side
        }

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
        double currHdg;
        double speed = 0.5; // arbitrary, just don't make it too close to 1.0
        double rotate = 0.0;
        final double Khdg = 0.05;
        int driveDir = 5;
        int masterMode = 0;
        double TIME_AT_WALL = 2.0;
        double TIME_TO_PAUSE = 1.0;
        double TIME_TO_BUILD_ZONE = 5; // assume it takes 5s to drive from LZ to BZ
        double LENGTH_OF_ROBOT = 16; // inches from front to back of robot
        double DIST_FROM_START = 48 - LENGTH_OF_ROBOT - 8; // 4ft - bot length - 8in (to Stone)
        double DIST_BETWEEN_SENSORS = 8; // distance between 2m Dist sensor and ColorDist sensors
        double DIST_HALF_BLOCK = 4;
        double distFromAudienceWall = 48 - DIST_BETWEEN_SENSORS - DIST_HALF_BLOCK;
        double persist = 0.0;
        final double PERSISTANCE = 3;
        double targetStone = 1;
        double targetDist;
        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;
        double RED_THRESHHOLD = 100; // need to adjust these for Skystone RGB values
        double GREEN_THRESHHOLD = 100;
        double BLUE_THRESHHOLD = 100;
        boolean skyStoneFound = false;
        double skyStonePos = 0;

        // determine the current heading angle
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currHdg = angles.firstAngle;
        rotate = rotate + Khdg * (currHdg - desiredHdg);

        // Now, set motor speeds based on sector stick is in
        switch (driveDir) {
            case 1: // rotate right
                vRightFront = REV * speed;
                vRightRear = REV * speed;
                vLeftFront = FWD * speed;
                vLeftRear = FWD * speed;
                break;

            case 2: // move forward
                vRightFront = FWD * speed;
                vRightRear = FWD * speed;
                vLeftFront = FWD * speed;
                vLeftRear = FWD * speed;
                vRightFront = Range.clip(vRightFront - rotate, -1.0, 1.0);
                vRightRear = Range.clip(vRightRear - rotate, -1.0, 1.0);
                vLeftFront = Range.clip(vLeftFront + rotate, -1.0, 1.0);
                vLeftRear = Range.clip(vLeftRear + rotate, -1.0, 1.0);
                break;

            case 3: // rotate left
                vRightFront = FWD * speed;
                vRightRear = FWD * speed;
                vLeftFront = REV * speed;
                vLeftRear = REV * speed;
                break;

            case 4: // move reverse
                vRightFront = REV * speed;
                vRightRear = REV * speed;
                vLeftFront = REV * speed;
                vLeftRear = REV * speed;
                vRightFront = Range.clip(vRightFront - rotate, -1.0, 1.0);
                vRightRear = Range.clip(vRightRear - rotate, -1.0, 1.0);
                vLeftFront = Range.clip(vLeftFront + rotate, -1.0, 1.0);
                vLeftRear = Range.clip(vLeftRear + rotate, -1.0, 1.0);
                break;

            case 5: // stop
                vRightFront = 0.0;
                vLeftFront = 0.0;
                vRightRear = 0.0;
                vLeftRear = 0.0;
                break;

            default: // should never get here, but don't move if we do!
                vRightFront = 0.0;
                vLeftFront = 0.0;
                vRightRear = 0.0;
                vLeftRear = 0.0;
                break;
        }

        setMotorPower(vRightFront, vLeftFront, vRightRear, vLeftRear);
        targetDist = distFromAudienceWall - (targetStone - 1)*8; // set target distance for next stone

        switch (masterMode) {
            case 0: // hang out at start for a short period of time
                if( runtime.seconds() > TIME_AT_WALL ) {
                    driveDir = 2; // move FWD
                    masterMode++;
                    runtime.reset();
                }
                break;

            case 1:
                if( sens2mDist.getDistance(DistanceUnit.INCH) > DIST_FROM_START ) {
                    persist++; // need at least 3 in a row
                }
                else {
                    persist = 0.0;
                }
                if( persist >= PERSISTANCE ) {
                    driveDir = 5; // stop moving
                    masterMode++;
                    runtime.reset();
                    persist = 0;
                }
                break;

            case 2:
                if( runtime.seconds() > TIME_TO_PAUSE ) {
                    if( BLUE_START ) {
                        driveDir = 2; // rotate left
                    }
                    else {
                        driveDir = 1; // rotate right
                    }
                    masterMode++;
                    runtime.reset();
                }
                break;

            case 3: // rotate until bot has rotated 90 deg from start
                if( currHdg >= 90 || currHdg <=-90 ) {
                    driveDir = 5; // stop rotating
                    masterMode++;
                    runtime.reset();
                    if( BLUE_START ) { // set new target heading depending on Alliance color
                        desiredHdg = 90;
                    }
                    else {
                        desiredHdg = -90;
                    }
                }
                break;

            case 4: // pause for a bit after rotating
                if( runtime.seconds() > TIME_TO_PAUSE ) {
                    masterMode++;
                    runtime.reset();
                }
                break;

            case 5: // set the direction to move to target stone
                if( sens2mDist.getDistance(DistanceUnit.INCH) < targetDist ) {
                    driveDir = 2; // move forward
                }
                else {
                    driveDir = 4; // move in reverse
                }
                masterMode++;
                break;

            case 6: // stop when at target stone
                if( driveDir == 2 ) {
                    if( sens2mDist.getDistance(DistanceUnit.INCH) >= targetDist ) {
                        driveDir = 5; // stop moving
                    }
                }
                if( driveDir == 4 ) {
                    if( sens2mDist.getDistance(DistanceUnit.INCH) <= targetDist ) {
                        driveDir = 5; // stop moving
                    }
                }
                break;

            case 7: // check to see if it's a Skystone
                if( BLUE_START ) {
                    if( rColorSensor.red() < RED_THRESHHOLD
                            && rColorSensor.green() < GREEN_THRESHHOLD
                            && rColorSensor.blue() < BLUE_THRESHHOLD ) {
                        skyStoneFound = true;
                    }
                }
                else {
                    if( lColorSensor.red() < RED_THRESHHOLD
                            && lColorSensor.green() < GREEN_THRESHHOLD
                            && lColorSensor.blue() < BLUE_THRESHHOLD ) {
                        skyStoneFound = true;
                    }
                }
                if( skyStoneFound ) {
                    skyStonePos = targetStone;
                    targetStone = targetStone + 3; // go to next Skystone
                    masterMode = 50; // pick up Skystone
                }
                else {
                    targetStone++; // go to next Stone
                    masterMode = 5; // move to next Stone
                }
                break;

            case 50: // pick up Skystone
                // need to add code to pick-up Skystone; do nothing until h/w is implemented
                masterMode++;
                break;

            case 51: // check to see if we have reached last stone
                // for now, just continue to next stone
                if( targetStone > 6 ) { // can't go past 6th Stone
                    masterMode = 100; // go to Building Zone and stay
                }
                else {
                    masterMode = 90; // move to Building Zone, drop Stone, then come back
                }
                driveDir = 2; // move forward
                runtime.reset();
                break;

            case 90: // drop Stone in Building Zone
                if( runtime.seconds() > TIME_TO_BUILD_ZONE ) {
                    driveDir = 5; // stop once in building zone
                    masterMode++;
                    runtime.reset();
                }
                break;

            case 91:
                if( runtime.seconds() > TIME_TO_PAUSE ) {
                    driveDir = 4; // move in reverse back to LZ
                    masterMode++;
                    runtime.reset();
                }
                break;

            case 92:
                if( runtime.seconds() > TIME_TO_BUILD_ZONE ) {
                    driveDir = 5; // stop once in building zone
                    masterMode = 5; // move to next Skystone
                    runtime.reset();
                }
                break;

            case 100:
                if( runtime.seconds() > TIME_TO_BUILD_ZONE ) {
                    driveDir = 5; // stop once in building zone
                    masterMode = 999;
                    runtime.reset();
                }
                break;

            case 999: // we're done!
                driveDir = 5; // stop
                break;

            default:
                stop(); // something went woefully wrong
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
