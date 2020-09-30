package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

public class Vision {
    private LinearOpMode opMode;
    private VuforiaLocalizer vuforia;

    private Bitmap bitmap;

    //private final static double WIDTH_CONVERSION_FACTOR;

    public Vision(LinearOpMode opMode) throws InterruptedException{
        this.opMode = opMode;
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = "AQvLCbX/////AAABmTGnnsC2rUXvp1TAuiOSac0ZMvc3GKI93tFoRn4jPzB3uSMiwj75PNfUU6MaVsNZWczJYOep8LvDeM/3hf1+zO/3w31n1qJTtB2VHle8+MHWNVbNzXKLqfGSdvXK/wYAanXG2PBSKpgO1Fv5Yg27eZfIR7QOh7+J1zT1iKW/VmlsVSSaAzUSzYpfLufQDdE2wWQYrs8ObLq2kC37CeUlJ786gywyHts3Mv12fWCSdTH5oclkaEXsVC/8LxD1m+gpbRc2KC0BXnlwqwA2VqPSFU91vD8eCcD6t2WDbn0oJas31PcooBYWM6UgGm9I2plWazlIok72QG/kOYDh4yXOT4YXp1eYh864e8B7mhM3VclQ";
        params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        getBitmap();
    }

    public void getBitmap() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImages = frame.getNumImages();
        Image rgb = null;
        for (int i = 0; i < numImages; i++) {
            Image img = frame.getImage(i);
            int fmt = img.getFormat();
            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        opMode.telemetry.addData("width ", rgb.getWidth());
        opMode.telemetry.addData("height ", rgb.getHeight());
        opMode.telemetry.update();

        bitmap = bm;
    }

    public void displayColor(int x, int y){
        int pixel = bitmap.getPixel(x, y);
        opMode.telemetry.addData("RED: ", red(pixel));
        opMode.telemetry.addData("BLUE: ", blue(pixel));
        opMode.telemetry.addData("GREEN: ", green(pixel));
        opMode.telemetry.update();
    }

    //yellow RGB value is 255, 255, 0
    //checks 1 row if pixels that should be in the area where the top ring is
    //checks 80 pixels, and if 40 are yellow, it returns true
    public boolean yellowInTop() throws InterruptedException {
        final int topY = 350;
        final int topXLeft = 380, topXRight = 610;
        int numYellow = 0;

        for(int x = topXLeft; x < topXRight; x++){
            int pixel = bitmap.getPixel(x, topY);
            if (red(pixel) > 200 && green(pixel) > 200){
                numYellow++;
            }
        }
        return (numYellow > 50);
    }

    //checks 1 row if pixels that should be in the area where the bottom ring is
    //checks 80 pixels, and if 40 are yellow, it returns true
    public boolean yellowInBottom() throws InterruptedException {
        final int bottomY = 430;
        final int bottomXLeft = 380, bottomXRight = 610;
        int numYellow = 0;

        for(int x = bottomXLeft; x < bottomXRight; x++){
            int pixel = bitmap.getPixel(x, bottomY);
            if (red(pixel) > 200 && green(pixel) > 200){
                numYellow++;
            }
        }
        return (numYellow > 50);
    }

    //returns how many rings there must be based off on if
    // there was yellow in the areas where the top and bottom rings are
    public int numRings() throws InterruptedException {
        boolean yellowInTop = yellowInTop();
        boolean yellowInBottom = yellowInBottom();

        opMode.telemetry.addData("yellowInTop: ", yellowInTop);
        opMode.telemetry.addData("yellowInBottom: ", yellowInBottom);
        opMode.telemetry.update();

        if (yellowInTop && yellowInBottom){
            return 3;
        }
        else if(yellowInBottom){
            return 1;
        }
        return 0;
    }

    /*
    ROHIT WAY, AVERAGES VALUES
    public double avgX() {
        double avgX = 0;
        double avgY = 0;
        Bitmap bitmap = null;
        try {
            bitmap = getBitmap();
        } catch (InterruptedException e) {
            opMode.telemetry.addData("ERROR", "BRUGHA THIS IS RART");
            opMode.telemetry.update();
        }
        int skystonePixelCount = 0;
        ArrayList<Integer> xValues = new ArrayList<>();
        ArrayList<Integer> yValues = new ArrayList<>();

        boolean[] yellowYVals = new boolean[bitmap.getHeight()];

        for (int y = 0; y < bitmap.getHeight(); y++) {
            for (int x = 0; x < bitmap.getWidth(); x++) {
                int pixel = bitmap.getPixel(x, y);
                if (red(pixel) >= YELRED_THRESHOLD && blue(pixel) <= YELBLUE_THRESHOLD && green(pixel) <= YELGREEN_THRESHOLD) {
                    yellowYVals[y] = true;
                    break;
                }
            }
        }
        for (int y = 0; y < bitmap.getHeight(); y++) {
            if (yellowYVals[y]) {
                for (int x = 0; x < bitmap.getWidth(); x++) {
                    int pixel = bitmap.getPixel(x, y);
                    if (red(pixel) <= RED_THRESHOLD && blue(pixel) <= BLUE_THRESHOLD && green(pixel) <= GREEN_THRESHOLD) {
                        xValues.add(x);
                        yValues.add(y);
                    }
                }
            }
        }
        for (int xCoor : xValues) {
            avgX += xCoor;
        }
        for (int yCoor : yValues) {
            avgY += yCoor;
        }
        avgX /= xValues.size();
        avgY /= yValues.size();

        return avgX;
    }

    public int skyStonePos() {
        double avg = avgX();
        if(avg <= 260) {
            return 1;
        }
        if(avg <= 400) {
            return 2;
        }
        else{
            return 3;
        }
    }
    */
}