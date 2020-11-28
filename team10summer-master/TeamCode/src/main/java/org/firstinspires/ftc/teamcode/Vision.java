package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;
import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

public class Vision {
    private LinearOpMode opMode;

    private VuforiaLocalizer vuforia;
    private Parameters parameters;
    private CameraDirection CAMERA_CHOICE = CameraDirection.BACK; // This is the camera opposite the screen.
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AcELeNr/////AAABmeg7NUNcDkPigDGNImdu5slLREdKn/q+qfajHBypycR0JUZYbfU0q2yZeSud79LJ2DS9uhr7Gu0xDM0DQZ36GRQDgMRwB8lf9TGZFQcoHq4kVAjAoEByEorXCzQ54ITCextAucpL2njKT/1IJxgREr6/axNEL2evyKSpOKoNOISKR6tkP6H3Ygd+FHm2tF/rsUCJHN5bTXrbRbwt5t65O7oJ6Wm8Foz1npbFI0bsD60cug4CpC/Ovovt2usxIRG8cpoQX49eA2jPRRLGXN8y1Nhh9Flr0poOkYoCExWo2iVunAGOwuCdB/rp/+2rkLBfWPvzQzrN9yBBP0JVJZ4biNQ41qqiuVvlc31O9xEvbKHt";

    public static int skystonePosition = 0;

    private final int RED_THRESHOLD = 35;
    private final int GREEN_THRESHOLD = 35;
    private final int BLUE_THRESHOLD = 35;
    private final int YELRED_THRESHOLD = 160;
    private final int YELGREEN_THRESHOLD = 130;
    private final int YELBLUE_THRESHOLD = 30;


    public Vision(LinearOpMode opMode) {
        this.opMode = opMode;
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraName = opMode.hardwareMap.get(WebcamName.class, "vCard");
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);
        vuforia.enableConvertFrameToBitmap();
    }

    public Bitmap getBitmap() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);
        long numImages = picture.getNumImages();

        for (int i = 0; i < numImages; i++) {
            int format = picture.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;
            }
        }
        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());
        picture.close();
        return imageBitmap;
    }

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

    //yellow RGB value is 255, 255, 0
    //checks 1 row if pixels that should be in the area where the top ring is
    //checks 80 pixels, and if 40 are yellow, it returns true
    public boolean yellowInTop() throws InterruptedException {
        final int topY = 325;
        final int topXLeft = 600, topXRight = 680;
        Bitmap bitmap = getBitmap();
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
        final int bottomY = 425;
        final int bottomXLeft = 600, bottomXRight = 680;
        Bitmap bitmap = getBitmap();
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
}


