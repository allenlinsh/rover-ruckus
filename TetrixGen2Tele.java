package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "TetrixGen2Tele", group = "Gen2")
public class TetrixGen2Tele extends LinearOpMode {

  private DcMotor LFWHEEL;
  private DcMotor RFWHEEL;
  private DcMotor LBWHEEL;
  private DcMotor RBWHEEL;
  private DcMotor ELEVATOR;
  private DcMotor ARM;
  private CRServo SLIDE;
  private Servo TEAMMARKER;
  
  private double slowDriveSpeed = 0.35;
  private double slowTurnSpeed = 0.25;
  private double slowRotateSpeed = 0.25;
  private double driveAxial = 0; //forward is positive
  private double driveLateral = 0; //right is positive
  private double driveYaw = 0; //CCW is positive
  private double elevPower = 0;
  private double armPower = 0;
  private double slidepower = 0;
  private double markPos = 0;

  private BNO055IMU imu;

  @Override
  public void runOpMode() {

    Orientation Angle;
    BNO055IMU.Parameters IMUParameters;
    Acceleration gravity;

    imu = hardwareMap.get(BNO055IMU.class, "imu");

    LFWHEEL = hardwareMap.dcMotor.get("LFWHEEL");
    RFWHEEL = hardwareMap.dcMotor.get("RFWHEEL");
    RBWHEEL = hardwareMap.dcMotor.get("RBWHEEL");
    LBWHEEL = hardwareMap.dcMotor.get("LBWHEEL");
    ELEVATOR = hardwareMap.dcMotor.get("ELEVATOR");
    ARM = hardwareMap.dcMotor.get("ARM");
    SLIDE = hardwareMap.crservo.get("SLIDE");
    TEAMMARKER = hardwareMap.servo.get("TEAMMARKER");

    LFWHEEL.setDirection(DcMotorSimple.Direction.FORWARD);
    RFWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
    LBWHEEL.setDirection(DcMotorSimple.Direction.FORWARD);
    RBWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
    LFWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RFWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LBWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RBWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    ELEVATOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    IMUParameters = new BNO055IMU.Parameters();
    IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    IMUParameters.loggingEnabled = false;
    imu.initialize(IMUParameters);
    telemetry.update();

    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
          if (gamepad1.dpad_up){
            if (gamepad1.dpad_right){
              driveAxial = - slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_left){
              driveAxial = - slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = - slowDriveSpeed;
              driveLateral = 0;
            }
          }
          else if (gamepad1.dpad_down){
            if (gamepad1.dpad_right){
              driveAxial = slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_left){
              driveAxial = slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = slowDriveSpeed;
              driveLateral = 0;
            }
          }
          else if (gamepad1.dpad_left){
            if (gamepad1.dpad_up){
              driveAxial = - slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else if (gamepad1.dpad_down){
              driveAxial = slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = 0;
              driveLateral = - slowDriveSpeed;
            }
          }
          else if (gamepad1.dpad_right){
            if (gamepad1.dpad_up){
              driveAxial = - slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_down){
              driveAxial = slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else{
              driveAxial = 0;
              driveLateral = slowDriveSpeed;
            }
          }
          driveYaw = 0;
        }
        else if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) || gamepad1.b) {
          driveYaw = slowRotateSpeed;
          if (gamepad1.dpad_up){
            if (gamepad1.dpad_right){
              driveAxial = - slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_left){
              driveAxial = - slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = - slowDriveSpeed;
              driveLateral = 0;
            }
          }
          else if (gamepad1.dpad_down){
            if (gamepad1.dpad_right){
              driveAxial = slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_left){
              driveAxial = slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = slowDriveSpeed;
              driveLateral = 0;
            }
          }
          else if (gamepad1.dpad_left){
            if (gamepad1.dpad_up){
              driveAxial = - slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else if (gamepad1.dpad_down){
              driveAxial = slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = 0;
              driveLateral = - slowDriveSpeed;
            }
          }
          else if (gamepad1.dpad_right){
            if (gamepad1.dpad_up){
              driveAxial = - slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_down){
              driveAxial = slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else{
              driveAxial = 0;
              driveLateral = slowDriveSpeed;
            }
          }
        }
        else if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) || gamepad1.x) {
          driveYaw = - slowRotateSpeed;
          if (gamepad1.dpad_up){
            if (gamepad1.dpad_right){
              driveAxial = - slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_left){
              driveAxial = - slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = - slowDriveSpeed;
              driveLateral = 0;
            }
          }
          else if (gamepad1.dpad_down){
            if (gamepad1.dpad_right){
              driveAxial = slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_left){
              driveAxial = slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = slowDriveSpeed;
              driveLateral = 0;
            }
          }
          else if (gamepad1.dpad_left){
            if (gamepad1.dpad_up){
              driveAxial = - slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else if (gamepad1.dpad_down){
              driveAxial = slowTurnSpeed;
              driveLateral = - slowTurnSpeed;
            }
            else{
              driveAxial = 0;
              driveLateral = - slowDriveSpeed;
            }
          }
          else if (gamepad1.dpad_right){
            if (gamepad1.dpad_up){
              driveAxial = - slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else if (gamepad1.dpad_down){
              driveAxial = slowTurnSpeed;
              driveLateral = slowTurnSpeed;
            }
            else{
              driveAxial = 0;
              driveLateral = slowDriveSpeed;
            }
          }
        }
        
        // drive
        else{
          driveAxial = gamepad1.left_stick_y;
          driveLateral = gamepad1.left_stick_x;
          driveYaw = gamepad1.right_stick_x;
        }

        // elevator
        if (gamepad2.dpad_up){
          elevPower = -1;
        }
        else if (gamepad2.dpad_down){
          elevPower = 1;
        }
        else {
          elevPower = 0;
        }
        
        // arm
        if (gamepad2.x){
          armPower = -0.4;
        }
        else if (gamepad2.b){
          armPower = 0.4;
        }
        else {
          armPower = 0;
        }
        
        // slide
        if (gamepad2.y){
          slidepower = 1;
        }
        else if (gamepad2.a){
          slidepower = -1;
        }
        else {
          slidepower = 0;
        }
        
        if (gamepad1.x && markPos == 0) {
          markPos = 0.5;
          sleep(100);
          markPos = 0.6;
        }
        else if (gamepad1.x && markPos == 0.6) {
          markPos = 0;
        }
        
        LFWHEEL.setPower(driveLateral - driveAxial + driveYaw);
        RFWHEEL.setPower(- driveLateral - driveAxial - driveYaw);
        LBWHEEL.setPower(- driveLateral - driveAxial + driveYaw);
        RBWHEEL.setPower(driveLateral - driveAxial - driveYaw);
        ELEVATOR.setPower(elevPower);
        ARM.setPower(armPower);
        SLIDE.setPower(slidepower);
        TEAMMARKER.setPosition(markPos);
        
        telemetry.addData("IMU Status", imu.getSystemStatus());
        telemetry.addData("Calibration Status", imu.getCalibrationStatus());
        gravity = imu.getGravity();
        Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Robot Z", Angle.firstAngle);
        telemetry.addData("Robot Y", Angle.secondAngle);
        telemetry.addData("Robot X", Angle.thirdAngle);

        telemetry.addData("Drive Axial", -(Math.round(driveAxial * 100.0)) / 100.0);
        telemetry.addData("Drive Lateral", -(Math.round(driveLateral * 100.0)) / 100.0);
        telemetry.addData("Drive Yaw", -(Math.round(driveYaw * 100.0)) / 100.0);
        telemetry.update();
      }
    }
  }
}
