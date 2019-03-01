package robot.utils;

import org.opencv.core.*;

public class PointFinder{
    Point[] points = new Point[4];

    public PointFinder(double[] cornX, double[] cornY){
        points[0] = new Point(cornX[0], cornY[0]);
        points[1] = new Point(cornX[1], cornY[1]);
        points[2] = new Point(cornX[2], cornY[2]);
        points[3] = new Point(cornX[3], cornY[3]);

        // sort into order of topleft, topright, bottomleft, bottomright
        // sort to top,top,bottom,bottom
        for(int i=0;i<3;i++){
            if(points[i].y > points[i+1].y){
                Point temp = points[i];
                points[i] = points[i+1];
                points[i+1] = temp;
            }
        }
        for (int i = 0; i < 2; i++) {
            if (points[i].y > points[i + 1].y) {
                Point temp = points[i];
                points[i] = points[i + 1];
                points[i + 1] = temp;
            }
        }
        // compare and swap the points for x-pos
        if(points[0].x > points[1].x){
            Point temp = points[0];
            points[0] = points[1];
            points[1] = temp;
        }
        if (points[2].x > points[3].x) {
            Point temp = points[2];
            points[2] = points[3];
            points[3] = temp;
        }
    }

    public Point getTopLeft(){
        return points[0];
    }
    
    public Point getTopRight() {
        return points[1];
    }
    
    public Point getBottomLeft() {
        return points[2];
    }
    
    public Point getBottomRight() {
        return points[3];
    }
}