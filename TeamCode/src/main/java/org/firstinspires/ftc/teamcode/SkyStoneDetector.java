package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class SkyStoneDetector extends DogeCVDetector {
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter filter = new GrayscaleFilter(0,50); //Black Grayscale

    public int stonesToFind = 1;


    public RatioScorer ratioScorerForShortFace = new RatioScorer(3, 10); // Used to find the short face of the stone
    public RatioScorer ratioScorerForLongFace = new RatioScorer(3.75, 10); // Used to find the long face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer( 5);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value


    // Results of the detector
    private ArrayList<Point> screenPositions = new ArrayList<>(); // Screen positions of the stones
    private ArrayList<Rect> foundRects = new ArrayList<>(); // Found rect

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat hierarchy  = new Mat();

    public List<Point> foundScreenPositions() {
        return screenPositions;
    }

    public List<Rect> foundRectangles() {
        return foundRects;
    }

    public SkyStoneDetector() {
        detectorName = "Stone Detector";
    }

    @Override
    public Mat process(Mat input) {
        screenPositions.clear();
        foundRects.clear();

        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        filter.process(workingMat.clone(), blackMask);

        List<MatOfPoint> contoursBlack = new ArrayList<>();
        Imgproc.findContours(blackMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursBlack,-1,new Scalar(230,70,70),2);

        // Current result
        ArrayList<Rect> bestRects = new ArrayList<>();
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

        Collections.sort(contoursBlack, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                return calculateScore(matOfPoint) > calculateScore(t1) ? 1 : 0;
            }
        });

        List<MatOfPoint> subList = contoursBlack;

        if (contoursBlack.size() > stonesToFind) {
            subList = contoursBlack.subList(0, stonesToFind);
        }

        for (MatOfPoint contour : subList) {
            Rect rect = Imgproc.boundingRect(contour);

            // Show chosen result
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 4);
            Imgproc.putText(displayMat, "Chosen", rect.tl(), 0, 1, new Scalar(255, 255, 255));

            screenPositions.add(new Point(rect.x, rect.y));
            foundRects.add(rect);
        }

        if (foundRects.size() > 0) {
            found = true;
        }
        else {
            found = false;
        }

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorerForShortFace);
        addScorer(ratioScorerForLongFace);

        // Add diffrent scoreres depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
    }
}