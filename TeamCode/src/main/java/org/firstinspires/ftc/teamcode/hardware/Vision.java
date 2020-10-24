package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
//adb.exe connect 192.168.43.1:5555
public class Vision {
    private LinearOpMode opMode;
    private VuforiaLocalizer vuforia;

    public Bitmap bitmap;

    //bitmap is 864x480
    public final static double WIDTH_CONVERSION_FACTOR = 864.0 / 1920.0;
    public final static double HEIGHT_CONVERSION_FACTOR = 480.0 / 1080.0;

    Telemetry.Item top;
    Telemetry.Item bottom;

    Telemetry.Item topNum;
    Telemetry.Item bottomNum;

    Telemetry.Item numRings;

    public Vision(LinearOpMode opMode) throws InterruptedException{
        this.opMode = opMode;
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "AQvLCbX/////AAABmTGnnsC2rUXvp1TAuiOSac0ZMvc3GKI93tFoRn4jPzB3uSMiwj75PNfUU6MaVsNZWczJYOep8LvDeM/3hf1+zO/3w31n1qJTtB2VHle8+MHWNVbNzXKLqfGSdvXK/wYAanXG2PBSKpgO1Fv5Yg27eZfIR7QOh7+J1zT1iKW/VmlsVSSaAzUSzYpfLufQDdE2wWQYrs8ObLq2kC37CeUlJ786gywyHts3Mv12fWCSdTH5oclkaEXsVC/8LxD1m+gpbRc2KC0BXnlwqwA2VqPSFU91vD8eCcD6t2WDbn0oJas31PcooBYWM6UgGm9I2plWazlIok72QG/kOYDh4yXOT4YXp1eYh864e8B7mhM3VclQ";
        params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();
        getBitmap();

        top = opMode.telemetry.addData("yellowInTop: ", 0);
        bottom = opMode.telemetry.addData("yellowInBottom: ", 0);
        topNum = opMode.telemetry.addData("numTop: ", 0);
        bottomNum = opMode.telemetry.addData("numBottom: ", 0);
        numRings = opMode.telemetry.addData("numRings: ", 0);
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
        bitmap = bm;
    }

    public void displayColor(int x, int y) throws InterruptedException {
        getBitmap();
        int pixel = bitmap.getPixel((int)(x * WIDTH_CONVERSION_FACTOR), (int)(y * HEIGHT_CONVERSION_FACTOR));
        opMode.telemetry.addData("RED: ", red(pixel));
        opMode.telemetry.addData("GREEN: ", green(pixel));
        opMode.telemetry.addData("BLUE: ", blue(pixel));
        opMode.telemetry.update();
    }

    //checks 1 row if pixels that should be in the area where the top ring is
    //checks 200 pixels, and if 60 are yellow, it returns true
    public boolean yellowInTopRightSide() throws InterruptedException {
        final int topY = 440;
        final int topXLeft = 1020, topXRight = 1220; //200 total pixels
        int numYellow = 0;

        for(int x = topXLeft; x < topXRight; x++){
            int pixel = bitmap.getPixel((int)(x * WIDTH_CONVERSION_FACTOR), (int)(topY * HEIGHT_CONVERSION_FACTOR));
            if (isOrange(pixel)){
                numYellow++;
            }
        }
        topNum.setValue(numYellow);
        opMode.telemetry.update();
        return (numYellow > 60);
    }

    //checks 1 row if pixels that should be in the area where the top ring is
    //checks 200 pixels, and if 60 are yellow, it returns true
    public boolean yellowInTopLeftSide() throws InterruptedException {
        final int topY = 440;
        final int topXLeft = 1320, topXRight = 1520; //200 total pixels
        int numYellow = 0;

        for(int x = topXLeft; x < topXRight; x++){
            int pixel = bitmap.getPixel((int)(x * WIDTH_CONVERSION_FACTOR), (int)(topY * HEIGHT_CONVERSION_FACTOR));
            if (isOrange(pixel)){
                numYellow++;
            }
        }
        topNum.setValue(numYellow);
        opMode.telemetry.update();
        return (numYellow > 60);
    }

    //checks 1 row if pixels that should be in the area where the top ring is
    //checks 200 pixels, and if 60 are yellow, it returns true
    public boolean yellowInBottomLeftSide() throws InterruptedException {
        final int topY = 500;
        final int topXLeft = 1320, topXRight = 1520; //200 total pixels
        int numYellow = 0;

        for(int x = topXLeft; x < topXRight; x++){
            int pixel = bitmap.getPixel((int)(x * WIDTH_CONVERSION_FACTOR), (int)(topY * HEIGHT_CONVERSION_FACTOR));
            if (isOrange(pixel)){
                numYellow++;
            }
        }
        bottomNum.setValue(numYellow);
        opMode.telemetry.update();
        return (numYellow > 60);
    }

    //checks 1 row if pixels that should be in the area where the bottom ring is
    //checks 200 pixels, and if 60 are yellow, it returns true
    public boolean yellowInBottomRightSide() throws InterruptedException {
        final int bottomY = 500;
        final int bottomXLeft = 1020, bottomXRight = 1220; //200 total pixels
        int numYellow = 0;

        for(int x = bottomXLeft; x < bottomXRight; x++){
            int pixel = bitmap.getPixel((int)(x * WIDTH_CONVERSION_FACTOR), (int)(bottomY * HEIGHT_CONVERSION_FACTOR));
            if (isOrange(pixel)){
                numYellow++;
            }
        }
        bottomNum.setValue(numYellow);
        opMode.telemetry.update();
        return (numYellow > 60);
    }

    public int numRingsLeftSide() throws InterruptedException {
        getBitmap();
        boolean yellowInTop = yellowInTopLeftSide();
        boolean yellowInBottom = yellowInBottomLeftSide();

        top.setValue(yellowInTop);
        bottom.setValue(yellowInBottom);
        opMode.telemetry.update();

        if (yellowInTop && yellowInBottom){
            numRings.setValue(4);
            return 4;
        }
        else if(yellowInBottom){
            numRings.setValue(1);
            return 1;
        }
        numRings.setValue(0);
        return 0;
    }

    //returns how many rings there must be based on if
    //there was yellow in the areas where the top and bottom rings are
    public int numRingsRightSide() throws InterruptedException {
        getBitmap();
        boolean yellowInTop = yellowInTopRightSide();
        boolean yellowInBottom = yellowInBottomRightSide();

        top.setValue(yellowInTop);
        bottom.setValue(yellowInBottom);
        opMode.telemetry.update();

        if (yellowInTop && yellowInBottom){
            numRings.setValue(4);
            return 4;
        }
        else if(yellowInBottom){
            numRings.setValue(1);
            return 1;
        }
        numRings.setValue(0);
        return 0;
    }

    public boolean isOrange(int pixel){
        return (blue(pixel) < 50);
    }
}