/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

// this is were we are going to put the variables set and used by the state
// of the robot, for orginization purposes
class RobotPersistentState {
    public float april_tag_pitch;
    public double aproximate_ball_distance;
};

@TeleOp(name = "Concept: Vision Color-Locator (Circle)", group = "Concept")
public class Magickkk extends LinearOpMode {
    static final String CAMERA_NAME = "Webcam 1";
    static final int CAMERA_RESOLUTION_X = 640;
    static final int CAMERA_RESOLUTION_Y = 480;
    ColorBlobLocatorProcessor purple_blob_processor;
    ColorBlobLocatorProcessor green_blob_processor;
    AprilTagProcessor april_tag_processor;
    Robot_State state;
    RobotPersistentState state_variables;

    public ColorBlobLocatorProcessor create_sphere_recognition(ColorRange color) {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        return colorLocator;
    }

    public List<ColorBlobLocatorProcessor.Blob> detect_sphere(ColorBlobLocatorProcessor locator) {
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);  // filter out very small blobs.

        return blobs;
    }

    /*
     * I am beyond pissed that java has pack expansion.
     * but we dont have time to get into that.
     * you SHUOLD be able to put a bunch of processers into this function and such
     */
    public VisionPortal create_camera_interface(VisionProcessor ...processors) {
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(processors)
                .setCameraResolution(new Size(CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y))
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME)).build();
        return portal;
    }

    float ball_cost_function(Circle circle) {
        return circle.getRadius();
    }

    void orient_twards_ball() {
        telemetry.addLine("Locating Ball");

        // find a ball, make sure its the correct order
        List<ColorBlobLocatorProcessor.Blob> purple_blobs = detect_sphere(purple_blob_processor);
        List<ColorBlobLocatorProcessor.Blob> green_blobs = detect_sphere(green_blob_processor);
        Circle best_purple_circle = purple_blobs.get(0).getCircle();
        int best_purple_cost = 0;

        Circle best_green_circle = green_blobs.get(0).getCircle();
        int best_green_cost = 0;

        // logging the balls
        for (ColorBlobLocatorProcessor.Blob b : purple_blobs) {
            Circle circleFit = b.getCircle();
            telemetry.addLine(String.format("[PURPLE] %5.3f      %3d     (%3d,%3d)  %f",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY(), ball_cost_function(circleFit)));
            if(best_purple_cost <= ball_cost_function(circleFit))
                best_purple_circle = circleFit;
        }

        for (ColorBlobLocatorProcessor.Blob b : green_blobs) {
            Circle circleFit = b.getCircle();
            telemetry.addLine(String.format("[GREEN] %5.3f      %3d     (%3d,%3d)   %f",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY(), ball_cost_function(circleFit)));
            if(best_green_cost <= ball_cost_function(circleFit))
                best_green_circle = circleFit;
        }

        // figure out which ball to use
        // for now we are just going to pick the best ball
        // but this can be extended to fit a particular pattern
        Circle picked = (best_green_cost > best_purple_cost) ? best_green_circle : best_purple_circle;
        float x = picked.getX();

        // this solution is very silly, but it tries to get the ball in the center of the camera
        // by rotating it untill the balls x is the center of the camera within some margin of error
        float xdiff = x - (CAMERA_RESOLUTION_X/2);

        if(Math.abs(xdiff) <= 10) {
            state = Robot_State.Moving_Twards_Ball;
            state_variables.aproximate_ball_distance = picked.getRadius(); // we might do a more sophisticated calculation later.
            return;
        }

        // rotate the robot
    }

    void move_twards_ball() {

    }

    void orient_april_tag() {
        // find the april tag
        List<AprilTagDetection> currentDetections = april_tag_processor.getDetections();
        double pitch = 0;
        double yaw = 0;

        for (AprilTagDetection detections : currentDetections) {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            if (detections.id == 24) {
                telemetry.addLine("FOUND IT LMAOOO");
                pitch = detections.ftcPose.pitch;
                yaw = detections.ftcPose.yaw;
            }

            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            // pose range and bearing will come in clutch
        }

        // same code for orienting twards the ball
        if(Math.abs(yaw) <= 5) {
            state = Robot_State.Shoot_Ball;
            state_variables.april_tag_pitch = (float) pitch;
        }
    }

    void shoot_ball() {
        // at this point the robot should be oriented twards april tag, and we can shoot the ball
    }

    @Override
    public void runOpMode() {

        /*
            Here is my plan
                loop 1 - aproaching the ball
                    - find the ball
                        - some cost function to determine what ball we want
                    - rotate the robot until the ball is in the center of the camera
                    - move the robot forward
                        - we could do linear algebra OR we could bullshit it

                loop 2 - shoot the ball ??????
                    - move to a position were we can make the ball
                    - recognise the april tag and use the cords to orient the robot
                    - shoot the ball
         */

        // Set up the blob recognitions
        purple_blob_processor = create_sphere_recognition(ColorRange.ARTIFACT_PURPLE);
        green_blob_processor = create_sphere_recognition(ColorRange.ARTIFACT_GREEN);
        april_tag_processor = new AprilTagProcessor.Builder().build(); // there are a lot more settings we can do here

        VisionPortal camera = create_camera_interface(  purple_blob_processor,
                                                        green_blob_processor,
                                                        april_tag_processor);

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
        state = Robot_State.Orienting_Twards_Ball;
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            switch (state) {
                case Orienting_Twards_Ball:
                    orient_twards_ball();
                    break;
                case Moving_Twards_Ball:
                    move_twards_ball();
                    break;
                case Orient_April_Tag:
                    orient_april_tag();
                    break;
                case Shoot_Ball:
                    shoot_ball();
                    break;
            }

            telemetry.update();
            sleep(100); // Match the telemetry update interval.
        }
    }
}
