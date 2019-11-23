package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Tank_Drive_Auto", group="Autonomous OpMode")
// @Disabled
public class Tank_Drive_Auto extends OpMode {


  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftDriveFront = null;
  private DcMotor leftDriveRear = null;
  private DcMotor rightDriveFront = null;
  private DcMotor rightDriveRear = null;

  private BNO055IMU imu;
  private Orientation angles;
  private double desiredHdg;

  private double vRightFront, vLeftFront, vRightRear, vLeftRear;

  private final double FWD = 1.0;
  private final double REV = -1.0;


  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);


    leftDriveFront = hardwareMap.get(DcMotor.class, "lDriveFront");
    rightDriveFront = hardwareMap.get(DcMotor.class, "rDriveFront");
    leftDriveRear = hardwareMap.get(DcMotor.class, "lDriveRear");
    rightDriveRear = hardwareMap.get(DcMotor.class, "rDriveRear");
    // set correct orientation of Drive motors
    leftDriveFront.setDirection(DcMotor.Direction.REVERSE); // Port 3
    rightDriveFront.setDirection(DcMotor.Direction.FORWARD); // Port 2
    leftDriveRear.setDirection(DcMotor.Direction.REVERSE); // Port 0
    rightDriveRear.setDirection(DcMotor.Direction.FORWARD); // Port 1
  }

  @Override
  public void loop() {

  }

  public void drive_time(double power,float duration)
  {}
  public void drive_dist(double power,float distance)
  {}
  public void turn(double power,float degrees)
  {}
  public void deploy(double power)
  {}
  public void Stow(double power)
  {}
  public void claw_rotate(float time)
  {}
  public void claw_grab(boolean closed)
  {}
  public void claw_slide(double power,float duration)
  {}
  public void pull_foundation(boolean far)
  {
    if(far){
    drive_time(0.5,1);
    turn(0.25,-90);
    drive_time(0.5,  3);
    turn(0.25,+90);
    drive_time(0.5, 2);
    deploy(0.25);
    drive_time(-0.5,3);
    Stow(0.25);
    turn(0.25,90);
    drive_time(0.5, 1);
    turn(0.15,-90);
    drive_time(0.5, 1);
    turn(0.25, -90);
    drive_time(0.5, 1);
    turn(0.25,-90);
    drive_time(0.5,4);


    }
    else(far){

    }
  }

}