
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.LightSensor;
        import com.qualcomm.robotcore.hardware.LegacyModule;
//import com.qualcomm.robotcore.hardware.;
/**
 * Created by james.mcpartland on 2/28/2017.
 */
@Autonomous(name="Pushbot: Auto Drive To And Follow Line", group="Pushbot")
public class detectAndFollowLine extends LinearOpMode{

    /* Declare OpMode members. */
    // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LightSensor             lightSensor;      // Primary LEGO Light sensor,
    LightSensor lightSensor2;
    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor


    DcMotor left, right;
    LegacyModule mod;
    DcMotorController cont;

    static final double     WHITE_THRESHOLD = 0.1695;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;

    @Override
    public void runOpMode() throws InterruptedException{

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        HardwareMap map = hardwareMap;
        ;
        mod = map.legacyModule.get("mod");
        cont = map.dcMotorController.get("cont");


        left = map.dcMotor.get("left");
        right = map.dcMotor.get("right");
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = map.lightSensor.get("sensor_light");                // Primary LEGO Light Sensor
        lightSensor2 = map.lightSensor.get("sensor_light2");
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);
        lightSensor2.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level 1", lightSensor.getLightDetected());
            telemetry.addData("Light Level 2", lightSensor2.getLightDetected());
            telemetry.update();
            idle();
        }

        // Start the robot moving forward, and then begin looking for a white line.
        left.setPower(APPROACH_SPEED);
        right.setPower(APPROACH_SPEED);

        DcMotor leftWheel, rightWheel;
        double leftPower, rightPower, correction, correction1, velocity;
        final double PERFECT_COLOR_VALUE = 0.1695;

        boolean on = false;
        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD) && (lightSensor2.getLightDetected() < WHITE_THRESHOLD));
        left.setPower(0.0000);
        right.setPower(0.0000);
        Thread.sleep(200);

        while (opModeIsActive()) {
            int leftPosition = left.getCurrentPosition();
            int rightPosition = right.getCurrentPosition();
            telemetry.addData("Left Position" ,"|%d|", leftPosition);
            telemetry.addData("Right Position" ,"|%d|", rightPosition);
            telemetry.update();

            // Get a correction
            correction = (PERFECT_COLOR_VALUE - lightSensor.getLightDetected());
            correction1 = correction;
            velocity = correction1-correction;

            // Sets the powers so they are no less than .075 and apply to correction
            double speed = 0.2;
            if (correction <= 0) {
                leftPower = speed - 4 * (correction + velocity);
                rightPower = speed;
            } else {
                leftPower = speed;
                rightPower = speed + 4 * (correction + velocity);
            }

            // Sets the powers to the motors
            left.setPower(leftPower);
            right.setPower(rightPower);
        }
        /*while (opModeIsActive()) {
            if (lightSensor2.getLightDetected() >= WHITE_THRESHOLD) {
                left.setPower(APPROACH_SPEED * 3.0);
                right.setPower(APPROACH_SPEED * 0.0);
            }
            if(lightSensor.getLightDetected() >= WHITE_THRESHOLD){
                right.setPower(APPROACH_SPEED*3.0);
                left.setPower(APPROACH_SPEED*0.0);
            }
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level 1", lightSensor.getLightDetected());
            telemetry.addData("Light Level 2", lightSensor2.getLightDetected());
            telemetry.update();
        }*/
        left.setPower(0.0000);
        right.setPower(0.0000);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Stop all motors

    }
    
}
