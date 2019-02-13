package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

@Autonomous(name = "TetrixGen3Auto", group = "Gen3")
public class TetrixGen3Auto extends LinearOpMode {

  private DcMotor RFWHEEL;
  private DcMotor RBWHEEL;
  private DcMotor LFWHEEL;
  private DcMotor LBWHEEL;
  private DcMotor ELEVATOR;
  private DcMotor ARM;
  private CRServo SLIDE;
  private Servo TEAMMARKER;

  private VuforiaRoverRuckus vuforiaRoverRuckus;
  private TfodRoverRuckus tfodRoverRuckus;
  
  private double driveAxial = 0; //forward is positive
  private double driveLateral = 0; //right is positive
  private double driveYaw = 0; //Counter-clockwise is positive
  private double fullDrive = 1;//driving in full speed
  private double slowDrive = 0.25; //driving speed for autonomous drive
  private double initialElevator = 0;
  private double targetAngle = 0;
  private double alignAngle = 0;
  private double robotAngle = 0;
  private double elevPower = 0;
  private double armPower = 0;
  private double slidePower = 0;
  private double markPos = 1;
  private double flag = - 1;
  
  private BNO055IMU imu;
  
  VuforiaBase.TrackingResults vuforiaResults;

  @Override
  public void runOpMode() {

    //setup for imu
    Orientation Angle;
    BNO055IMU.Parameters IMUParameters;
    Acceleration gravity;
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    //setup for recognitions
    List<Recognition> recognitions;
    double goldMineralX;
    double silverMineral1X;
    double silverMineral2X;
    double objRec = -1; //0:left; 1:right; 2:center
    vuforiaRoverRuckus = new VuforiaRoverRuckus();
    tfodRoverRuckus = new TfodRoverRuckus();

    //setup for motors
    RFWHEEL = hardwareMap.dcMotor.get("RFWHEEL");
    RBWHEEL = hardwareMap.dcMotor.get("RBWHEEL");
    LFWHEEL = hardwareMap.dcMotor.get("LFWHEEL");
    LBWHEEL = hardwareMap.dcMotor.get("LBWHEEL");
    ELEVATOR = hardwareMap.dcMotor.get("ELEVATOR");
    ARM = hardwareMap.dcMotor.get("ARM");
    SLIDE = hardwareMap.crservo.get("SLIDE");
    TEAMMARKER = hardwareMap.servo.get("TEAMMARKER");

    //configuration
    LFWHEEL.setDirection(DcMotorSimple.Direction.FORWARD);
    RFWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
    LBWHEEL.setDirection(DcMotorSimple.Direction.FORWARD);
    RBWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
    
    LFWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LBWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RBWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ELEVATOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    LFWHEEL.setPower(0);
    RFWHEEL.setPower(0);
    LBWHEEL.setPower(0);
    RBWHEEL.setPower(0);
    TEAMMARKER.setPosition(1);
    SLIDE.setPower(0);
    ARM.setPower(0);
    ELEVATOR.setPower(0);

    // imu
    IMUParameters = new BNO055IMU.Parameters();
    IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    IMUParameters.loggingEnabled = false;
    imu.initialize(IMUParameters);
    gravity = imu.getGravity();
    Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    telemetry.update();
    
    //setup for webcam
    vuforiaRoverRuckus.initialize("", hardwareMap.get(WebcamName.class, "Webcam"), "",
        true, true, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
        0, 0, 0, 0, 0, 0, true);
    tfodRoverRuckus.initialize(vuforiaRoverRuckus, (float)0.4, true, true);
    
    telemetry.update();
    waitForStart();
    
    if (opModeIsActive()) {
      vuforiaRoverRuckus.activate();
      tfodRoverRuckus.activate();

      /**
      // latching off
      **/
      //elevator go up
      ELEVATOR.setPower(-1); 
      sleep(2200);
      //stop 
      ELEVATOR.setPower(0); 

      /**
      // adjusting position
      // quickly move to the right, forward, then left
      **/
      // left quickly
      driveLateral = - slowDrive;
      drivePower();
      sleep(373);

      //stop
      driveLateral = 0;
      drivePower();

      //forward quickly
      driveAxial = slowDrive;
      drivePower();
      sleep(373);

      //stop
      driveAxial = 0;
      drivePower();

      //right quickly
      driveLateral = slowDrive;
      drivePower();
      sleep(373);

      //stop
      driveLateral = 0;
      drivePower();

      /**
      // reset elevator
      **/
      //go down
      ELEVATOR.setPower(1);
      sleep(2200);
      
      //stop
      ELEVATOR.setPower(0);

      /**
      return to postition
      **/
      //backward quickly
      driveAxial = - slowDrive;
      drivePower();
      sleep(373);

      //stop
      driveAxial = 0;
      drivePower();

      /**
      //rotate 180 degrees counter-clockwise
      **/

      /**
      //auto rotation
      targetAngle = 180; //180 degrees
      gravity = imu.getGravity();
      Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.update();
      robotAngle = Angle.firstAngle;
      while(Math.abs(targetAngle - robotAngle) > 5 && opModeIsActive()) {
        gravity = imu.getGravity();
        Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.update();
        robotAngle = Angle.firstAngle;
        robotAngle = Angle.firstAngle;
        driveYaw = - slowDrive; //counter-clockwise
        drivePower();
      }
      driveYaw = slowTurn; //counter-clockwise
      drivePower();
      sleep(200);
      driveYaw = 0;
      drivePower();

      //fixed rotation
      driveYaw = - fullDrive;
      drivePower();
      sleep(2394); //180 degrees

      driveYaw = 0;
      drivePower();
      **/


      /**
      //sampling
      **/
      while ((objRec != 0 || objRec !=1 || objRec != -1) && opModeIsActive()) {
        recognitions = tfodRoverRuckus.getRecognitions();
        telemetry.addData("# Objects Recognized", recognitions.size());
        if (recognitions.size() == 3) {
          goldMineralX = -1;
          silverMineral1X = -1;
          silverMineral2X = -1;
          for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals("Gold Mineral")) {
              goldMineralX = recognition.getLeft();
            } else if (silverMineral1X == -1) {
              silverMineral1X = recognition.getLeft();
            } else {
              silverMineral2X = recognition.getLeft();
            }
          }
          // Make sure we found one gold mineral and two silver minerals.
          if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                telemetry.addData("Gold Mineral Position", "Left");
                objRec = 0;
                break;
            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                telemetry.addData("Gold Mineral Position", "Right");
                objRec = 1;
                break;
            } else{
                telemetry.addData("Gold Mineral Position", "Center");
                objRec = 2;
                break;
            }
          }
        }
        sleep(1000);
        else if (){
          driveYaw = - fullDrive;
          drivePower();
          sleep(236);
        }
      }

      //conditions
      //left
      if (objRec == 0){
        //forward
        driveAxial = fullDrive;
        drivePower();
        sleep(821);

        driveAxial = 0;
        drivePower();

        //left
        driveLateral = - fullDrive;
        drivePower();
        sleep(547);

        driveLateral = 0;
        drivePower();

        //forward
        driveAxial = fullDrive;
        drivePower();
        sleep(547); //16.25 inches

        driveAxial = 0;
        drivePower();

        //reverse
        //backward
        driveAxial = - fullDrive;
        drivePower();
        sleep(821); //16.25 inches

        driveAxial = 0;
        drivePower();

        //right
        driveLateral = fullDrive;
        drivePower();
        sleep(547);

        driveLateral = 0;
        drivePower();
      }

      //right
      if (objRec == 1){
        //forward
        driveAxial = fullDrive;
        drivePower();
        sleep(821);

        driveAxial = 0;
        drivePower();

        //right
        driveLateral = fullDrive;
        drivePower();
        sleep(547);

        driveLateral = 0;
        drivePower();

        //forward
        driveAxial = fullDrive;
        drivePower();
        sleep(547); //16.25 inches

        driveAxial = 0;
        drivePower();

        //reverse
        //backward
        driveAxial = - fullDrive;
        drivePower();
        sleep(821); //16.25 inches

        driveAxial = 0;
        drivePower();

        //left
        driveLateral = - fullDrive;
        drivePower();
        sleep(547);

        driveLateral = 0;
        drivePower();
      }

      //center
      if (objRec == 2){
        //forward
        driveAxial = fullDrive;
        drivePower();
        sleep(821);

        driveAxial = 0;
        drivePower();

        //forward
        driveAxial = fullDrive;
        drivePower();
        sleep(547); //16.25 inches

        driveAxial = 0;
        drivePower();

        //reverse
        //backward
        driveAxial = - fullDrive;
        drivePower();
        sleep(821); //16.25 inches

        driveAxial = 0;
        drivePower();
      }

      //left
      driveLateral = - fullDrive;
      drivePower();
      sleep(1642);

      driveLateral = 0;
      drivePower();

      /**
      rotating 135 degrees counter-clockwise
      **/
      //fixed rotation
      driveYaw = - fullDrive;
      drivePower();
      sleep(236);

      driveYaw = 0;
      drivePower();

      //right
      driveLateral = slowDrive;
      drivePower();
      sleep(547);

      driveLateral = 0;
      drivePower();

      /**
      // image recognition
      // (Note we only process first visible target).
      if (isTargetVisible("BluePerimeter")) { //blue
        processTarget();
      } else if (isTargetVisible("RedPerimeter")) { //red
        processTarget();
      } else if (isTargetVisible("FrontPerimeter")) { //audience
        processTarget();
      } else if (isTargetVisible("BackPerimeter")) { //timer
        processTarget();
      } else {
        telemetry.addData("No Targets Detected", "Targets are not visible.");
      }
      **/



      // Deactivate Vuforia
      vuforiaRoverRuckus.deactivate();
      tfodRoverRuckus.deactivate();
    }
    
    vuforiaRoverRuckus.close();
    tfodRoverRuckus.close();
  }

  public void drivePower(){
    LFWHEEL.setPower(driveLateral + driveAxial + driveYaw);
    RFWHEEL.setPower(- driveLateral + driveAxial - driveYaw);
    LBWHEEL.setPower(- driveLateral + driveAxial + driveYaw);
    RBWHEEL.setPower(driveLateral + driveAxial - driveYaw);
    ELEVATOR.setPower(elevPower);
    ARM.setPower(armPower);
    SLIDE.setPower(slidePower);
    TEAMMARKER.setPosition(markPos);
  }
  
  private boolean isTargetVisible(String trackableName) {
    boolean isVisible;

    // Get vuforia results for target.
    vuforiaResults = vuforiaRoverRuckus.track(trackableName);
    // Is this target visible?
    if (vuforiaResults.isVisible) {
      isVisible = true;
    } else {
      isVisible = false;
    }
    return isVisible;
  }

  /**
   * This function displays location on the field and rotation about the Z
   * axis on the field. It uses results from the isTargetVisible function.
   */
  private void processTarget() {
    // Display the target name.
    telemetry.addData("Target Detected", vuforiaResults.name + " is visible.");
    telemetry.addData("X (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.x, "IN"), 2)));
    telemetry.addData("Y (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "IN"), 2)));
    telemetry.addData("Z (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.z, "IN"), 2)));
    telemetry.addData("Rotation about Z (deg)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2)));
    alignAngle = Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2));
  }

  /**
   * By default, distances are returned in millimeters by Vuforia.
   * Convert to other distance units (CM, M, IN, and FT).
   */
  private double displayValue(float originalValue, String units) {
    double convertedValue;

    // Vuforia returns distances in mm.
    if (units.equals("CM")) {
      convertedValue = originalValue / 10;
    } else if (units.equals("M")) {
      convertedValue = originalValue / 1000;
    } else if (units.equals("IN")) {
      convertedValue = originalValue / 25.4;
    } else if (units.equals("FT")) {
      convertedValue = (originalValue / 25.4) / 12;
    } else {
      convertedValue = originalValue;
    }
    return convertedValue;
  }
}
