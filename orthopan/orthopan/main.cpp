//
//  main.cpp
//  orthopan
//
//  Created by kronick on 7/18/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include <iostream>
#include <math.h>
#include <opencv.hpp>

using namespace cv;

struct RANSACResults {
    std::vector<Vec4i> inliers;
    Point intersection;
};

RANSACResults RANSACLineIntersection(std::vector<Vec4i> lines);
Point nlineIntersection(std::vector<Vec4i> lines);
float pointToLineDistance(Vec4i line, Point p);
float pointDistance(Point p1, Point p2);
Point intersection(Vec4i a, Vec4i b);
float slope(Vec4i s);
inline float segmentLength(Vec4i segment);

static const int USE_CAMERA = true;
static const int UNDISTORT = false;

int main (int argc, const char * argv[]) {
    std::vector<Vec4i> verticals;
    std::vector<Vec4i> horizontals;
    
    namedWindow("Yooyo");
    namedWindow("Original");
    namedWindow("Undistorted");
    namedWindow("Rectified");
    
    VideoCapture cap;
    if(USE_CAMERA) {
        cap.open("/Users/kronick/Documents/N55/EARTH ROVER/orthopan/orthopan/too-big/IMG_0543.MOV");
        //cap.open(0);
        if(!cap.isOpened())
            return -1;
    }
    
    string outputDirectory = "/Users/kronick/Documents/N55/EARTH ROVER/orthopan/orthopan/output/";
    
    Mat originalImage, sourceImage, destImage, rectifiedImage, justSegments;
    
    // Set up camera properties
    cv::Mat cameraIntrinsics = (Mat_<float>(3,3) << 47.744713, 0.000000, 319.500000,
                                                    0.000000, 47.744713, 239.500000,
                                                    0, 0, 1.000000); 
    cv::Mat cameraDistortion = (Mat_<float>(5,1) << -0.000468, -0.000035, 0.005934, 0.000723, -0.000000);
    //cv::Mat cameraDistortion = (Mat_<float>(5,1) << 0,0,0,0,0);
           
    Point lastVanish[10];
    
    int imgIndex = 15;
    for(int frameNumber=0;; frameNumber++) {
        justSegments.create(480, 640, CV_8UC3);
        rectangle(justSegments, Point(0,0), Point(640,480), CV_RGB(0,0,0), CV_FILLED);
        
        Mat _sourceImage;
        if(USE_CAMERA) {
            //Mat rawImage;
            //cap >> rawImage;
            cap >> sourceImage;
            //cap >> originalImage;
            originalImage = sourceImage;
            
            cvtColor(sourceImage, sourceImage, CV_BGR2GRAY);
            resize(sourceImage, sourceImage, Size(640,480));
            
            if(UNDISTORT) {
                _sourceImage = sourceImage.clone();
                undistort(_sourceImage, sourceImage, cameraIntrinsics, cameraDistortion);
            }
            
            GaussianBlur(sourceImage, sourceImage, Size(3,3), 0);
        }
        else {
            std::stringstream filename;
            //imgIndex = 16;
            filename << "/Users/kronick/Documents/N55/EARTH ROVER/orthopan/orthopan/" <<
                        (imgIndex < 10 ? "00" : (imgIndex < 100 ? "0" : "")) << imgIndex << ".JPG";
            if(imgIndex++ > 34)
                imgIndex = 1;
            sourceImage = imread(filename.str(), 0);
            originalImage = imread(filename.str());
        
            GaussianBlur(sourceImage, sourceImage, Size(7,7), 0);
        }

        Canny(sourceImage, destImage, 30, 100, 3);  // Edge detect
        
        // Detect lines
        vector<Vec4i> lines;
        HoughLinesP(destImage, lines, 1, CV_PI/180, 20, 15, 5);
        
        cvtColor(destImage, destImage, CV_GRAY2BGR); // Create RGB output image
        
        //rectangle(destImage, Point(0,0), Point(640,480), CV_RGB(0,0,0), CV_FILLED);
        
        // Categorize the line according to its slope
        for(size_t i=0; i<lines.size(); i++) {
            float s = slope(lines[i]); 
            if(fabs(s) < 2) horizontals.push_back(lines[i]);
            else if(fabs(s) > 20) verticals.push_back(lines[i]);
        
            line(justSegments, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), CV_RGB(255,0,128), 1, CV_AA);
            /*
            if(i > 0 && false) {
                Point cross = intersection(lines[i], lines[i-1]);
                if(cross.x != 0 || cross.y != 0) {
                    line(destImage, Point(lines[i][0], lines[i][1]), cross, CV_RGB(255,255,0), 2, CV_AA);
                    circle(destImage, cross, 3, CV_RGB(0,128,255), -1);
                }
            }
            */
        }
        
        //cvtColor(sourceImage, destImage, CV_GRAY2RGB);
        
        // Draw all verticals
        for(int i=0; i<verticals.size(); i++) {
            //line(destImage, Point(verticals[i][0], verticals[i][1]), Point(verticals[i][2], verticals[i][3]),
            //     CV_RGB(0,128,255), 1, CV_AA);
            float s = slope(verticals[i]);
            if(s == 0) s = FLT_EPSILON;
            Point top(verticals[i][0]-verticals[i][1]/s ,0);
            Point bottom(verticals[i][0]+(destImage.rows-verticals[i][1])/s ,destImage.rows);
            line(destImage, bottom, top,
                 CV_RGB(0,64,128), 1, CV_AA);
        }  
        
        
        // Draw all horizontals
        for(int i=0; i<horizontals.size(); i++) {
            float s = slope(horizontals[i]);
            Point left(0, horizontals[i][1] - horizontals[i][0]*s);
            Point right(destImage.cols, horizontals[i][1] + (destImage.cols-horizontals[i][0])*s);
            line(destImage, Point(horizontals[i][0], horizontals[i][1]), Point(horizontals[i][2], horizontals[i][3]),
                 CV_RGB(255,0,128), 1, CV_AA);
        }        
        
        // Calculate interesection
        RANSACResults vanish = RANSACLineIntersection(horizontals);
        //vanish.intersection = Point((vanish.intersection.x + lastVanish.x) / 2, (vanish.intersection.y + lastVanish.y) / 2);
        for(int i=9; i>0; i--) {
            lastVanish[i] = lastVanish[i-1];
        }
        lastVanish[0] = vanish.intersection;
        float sumX = 0;
        float sumY = 0;
        for(int i=0; i<10; i++) {
            sumX += lastVanish[i].x;
            sumY += lastVanish[i].y;
        }
        vanish.intersection = Point(sumX/10., sumY/10.);
        
        for(int i=0; i<vanish.inliers.size(); i++) {
            float s = slope(vanish.inliers[i]);
            Point left(0, vanish.inliers[i][1] - vanish.inliers[i][0]*s);
            Point right(destImage.cols, vanish.inliers[i][1] + (destImage.cols-vanish.inliers[i][0])*s);
            line(destImage, Point(vanish.inliers[i][0], vanish.inliers[i][1]), Point(vanish.inliers[i][2], vanish.inliers[i][3]),
                 CV_RGB(255,0,128), 1, CV_AA);
            
            line(destImage, Point(vanish.inliers[i][0], vanish.inliers[i][1]), vanish.intersection,
                 CV_RGB(255,255,0), 1, CV_AA);
            
            //line(destImage, Point(vanish.inliers[i][2], vanish.inliers[i][3]), right,
            //     CV_RGB(255,255,0), 1, CV_AA);
        }   
        circle(destImage, vanish.intersection, 5, CV_RGB(0,128,255), CV_FILLED);
        
        resize(destImage, destImage, Size(640,480));
        
        horizontals.clear();
        verticals.clear();
        
        
        // Rectify image
        // -------------------------------
        
        // 1) Synthesize four corners
        Point c0,c1,c2,c3;
        if(vanish.intersection.x < 0) {
            float slopeA = slope(Vec4i(0,0, vanish.intersection.x, vanish.intersection.y));
            float slopeB = slope(Vec4i(0,sourceImage.rows, vanish.intersection.x, vanish.intersection.y));
            c0 = Point(0,0);
            c1 = Point(0,sourceImage.rows);
            c2 = Point(sourceImage.cols, sourceImage.rows + sourceImage.cols*slopeB);
            c3 = Point(sourceImage.cols, sourceImage.cols*slopeA);
        }
        else if(vanish.intersection.x > sourceImage.cols) {
            float slopeA = slope(Vec4i(sourceImage.cols,0, vanish.intersection.x, vanish.intersection.y));
            float slopeB = slope(Vec4i(sourceImage.cols,sourceImage.rows, vanish.intersection.x, vanish.intersection.y));
            c0 = Point(0,-sourceImage.cols*slopeA);
            c1 = Point(0,sourceImage.rows - sourceImage.cols*slopeB);
            c2 = Point(sourceImage.cols, sourceImage.rows);
            c3 = Point(sourceImage.cols, 0);                
            
            std::cout << "Top intersection: " << (-sourceImage.cols*slopeA) << "\n";
            std::cout << "Bottom intersection: " << (sourceImage.rows - sourceImage.cols*slopeB)<< "\n";
        }
        else if(vanish.intersection.x > sourceImage.cols/2.) {
            float slopeA = slope(Vec4i(vanish.intersection.x*.5,0, vanish.intersection.x, vanish.intersection.y));
            float slopeB = slope(Vec4i(vanish.intersection.x*.5,sourceImage.rows, vanish.intersection.x, vanish.intersection.y));
            c0 = Point(0,-vanish.intersection.x*.5*slopeA);
            c1 = Point(0,sourceImage.rows - vanish.intersection.x*.5*slopeB);
            c2 = Point(vanish.intersection.x*.5, sourceImage.rows);
            c3 = Point(vanish.intersection.x*.5, 0);                            
        }
        else {
            float slopeA = slope(Vec4i(vanish.intersection.x*2,0, vanish.intersection.x, vanish.intersection.y));
            float slopeB = slope(Vec4i(vanish.intersection.x*2,sourceImage.rows, vanish.intersection.x, vanish.intersection.y));
            c0 = Point(vanish.intersection.x*2, 0);                
            c1 = Point(vanish.intersection.x*2, sourceImage.rows);
            c2 = Point(sourceImage.cols,sourceImage.rows + (sourceImage.cols-vanish.intersection.x*2)*slopeB);
            c3 = Point(sourceImage.cols,(sourceImage.cols-vanish.intersection.x*2)*slopeA);
        }
        
        circle(destImage, c0, 5, CV_RGB(255,0,128), CV_FILLED);
        circle(destImage, c1, 5, CV_RGB(255,0,128), CV_FILLED);
        circle(destImage, c2, 5, CV_RGB(255,0,128), CV_FILLED);
        circle(destImage, c3, 5, CV_RGB(255,0,128), CV_FILLED);
        
        
        Size imageSize = sourceImage.size();
        cv::Mat userCorners = (cv::Mat_<double>(4,2) <<	(double)c0.x,(double)c0.y,
                                                        (double)c1.x,(double)c1.y,
                                                        (double)c2.x,(double)c2.y,
                                                        (double)c3.x,(double)c3.y);
        
        // 2) Get aspect ratio
        // Using equations from: http://research.microsoft.com/en-us/um/people/zhang/Papers/WhiteboardRectification.pdf
        cv::Mat A = cameraIntrinsics;
        
        float k2, k3;
        float ratio;
        cv::Mat _ratio;
        cv::Mat n2, n3;
        cv::Mat m1 = (cv::Mat_<float>(3,1) << (float)c0.x, (float)c0.y, 1);
        cv::Mat m2 = (cv::Mat_<float>(3,1) << (float)c3.x, (float)c3.y, 1);
        cv::Mat m3 = (cv::Mat_<float>(3,1) << (float)c1.x, (float)c1.y, 1);
        cv::Mat m4 = (cv::Mat_<float>(3,1) << (float)c2.x, (float)c2.y, 1);
        
        k2 = (m1.cross(m4).dot(m3)) / ((m2.cross(m4)).dot(m3));
        k3 = (m1.cross(m4).dot(m2)) / ((m3.cross(m4)).dot(m2));
        n2 = (k2*m2) - m1;
        n3 = (k3*m3) - m1;
        
        _ratio = (n2.t()*(A.inv().t())*(A.inv())*n2) / (n3.t()*(A.inv().t())*(A.inv())*n3);
        ratio = sqrt(_ratio.at<float>(0,0));
        
        //float rescaleFactor = fabs(min((float)sourceImage.rows/(c3.y-c2.y), (float)sourceImage.rows/(c1.y-c0.y)));
        //float rescaleFactor = 1/fabs((float)sourceImage.rows/(c3.y-c2.y));
        float rescaleFactor = 0.25;
        std::cout << "Rescaling by: " << rescaleFactor << "\n";
        //float rescaleFactor = 0.5;
        
        float w = max(sqrtf(powf(c3.x-c0.x,2)+powf(c3.y-c0.y,2)),
                                        sqrtf(powf(c2.x-c1.x,2)+powf(c2.y-c1.y,2))) * rescaleFactor;
        float h = w / ratio;
        
        cv::Mat rectangleCorners =	(cv::Mat_<double>(4,2) << 0,0, 0,h, w,h, w,0);
        
        cv::Mat homography = findHomography(userCorners, rectangleCorners);
        
        
        warpPerspective(originalImage, rectifiedImage, homography, Size(640,480));
        imshow("Rectified", rectifiedImage);
        
        std::stringstream outputFilename;
        outputFilename << outputDirectory << (frameNumber < 10 ? "000" : frameNumber < 100 ? "00" : frameNumber < 1000 ? "0" : "") <<
                        frameNumber << ".png";
        imwrite(outputFilename.str(), destImage);
        std::cout << outputFilename.str();
        
        //if(UNDISTORT)
        //    imshow("Undistorted", _sourceImage);
        //imshow("Original", originalImage);
        imshow("Yooyo", destImage);
        
        if(cvWaitKey(1) == 27)
            break;
    }
    
    
    return 0;
}

float slope(Vec4i s) {
    float rise = s[3] - s[1];
    float run  = s[2] - s[0];
    return (run != 0 ? (rise/run) : FLT_MAX);
}

RANSACResults RANSACLineIntersection(std::vector<Vec4i> lines) {
    RNG randomNumberGen;
    
    int sample_size = 2;
    int minimum_inliers = 12;
    int max_iterations = 1000;
    float inlier_threshold = .006;   // radians
    
    int iterations = 0;
    Point bestPoint(0,0);
    std::vector<Vec4i> bestLines;
    float bestError = FLT_MAX;
    
    Point currentPoint(0,0);
    float currentError = FLT_MAX;
    
    while(iterations < max_iterations) {
        std::vector<int> randomIndices;
        std::vector<Vec4i> randomSample;
        std::vector<Vec4i> consensusLines;
        
        for(int i=0; i<sample_size; i++) {
            int randomIndex = randomNumberGen.uniform(0, lines.size());
            randomSample.push_back(lines[randomIndex]);
            randomIndices.push_back(randomIndex);
            
            consensusLines.push_back(lines[randomIndex]);
        }
        
        currentPoint = nlineIntersection(randomSample); // Working model for this iteration
        currentError = 0;

        // Add lines that fit the model to the consensus set
        for(int i=0; i<lines.size(); i++) {
            // Find out if this is already in the consensus
            bool notInConsensus = true;
            for(int j=0; j<randomIndices.size(); j++) {
                if(randomIndices[j] == i) {
                    notInConsensus = false;
                    break;
                }
            }
            // Find error for this line
            Point p(lines[i][0], lines[i][1]);
            //float errorDist = pointToLineDistance(lines[i], currentPoint)
            //        + 1000/pointDistance(p, currentPoint);
            //float errorDist = pointToLineDistance(lines[i], currentPoint)/(sqrt(pointDistance(p, currentPoint))*10);
            //std::cout << "Line segment angle: " << atan2f(lines[i][1]-lines[i][3], lines[i][0]-lines[i][2]) << "\n";
            //std::cout << "Angle to current point: " << atan2f(lines[i][1]-currentPoint.y, lines[i][0]-currentPoint.x) << "\n";
            float errorDist = fabs(atan2f(lines[i][1]-currentPoint.y, lines[i][0]-currentPoint.x) -
                                  atan2f(lines[i][1]-lines[i][3], lines[i][0]-lines[i][2]));
            while(errorDist > M_PI/2) { errorDist -= M_PI; errorDist = fabs(errorDist); }
            
            if(errorDist < inlier_threshold) {
                currentError += errorDist;// / sqrt(segmentLength(lines[i]));
                if(notInConsensus)
                    consensusLines.push_back(lines[i]);
            }
                
            
        }
        
        // Figure out how good this model fits the data
        if(consensusLines.size() > minimum_inliers) {
            if(currentError / consensusLines.size() < bestError) {
                bestPoint = currentPoint;
                bestError = currentError / consensusLines.size();
                bestLines = consensusLines;
            }
            
            if(bestError < inlier_threshold/2.) break;
        }
        
        iterations++;
    }
    
    /*
    std::cout << "RANSAC INLIERS: " << bestLines.size() << "\n";
    std::cout << "RANSAC ERROR:   " << bestError << "\n";
    std::cout << "RANSAC ITERATIONS: " << iterations << "\n";
    */
    
    RANSACResults result;
    result.inliers = bestLines;
    result.intersection = bestPoint;
    
    return result;
}
Point nlineIntersection(std::vector<Vec4i> lines) {
    Mat linePoints(lines.size(), 2, CV_32F);
    Mat lineNormals(lines.size(), 2, CV_32F);
    float mag, n_x, n_y;
    
    for(int i=0; i<lines.size(); i++) {
        linePoints.at<float>(i, 0) = lines[i][0]; // P_x
        linePoints.at<float>(i, 1) = lines[i][1]; // P_y
        mag = sqrt((lines[i][2]-lines[i][0])*(lines[i][2]-lines[i][0]) + (lines[i][3]-lines[i][1])*(lines[i][3]-lines[i][1]));
        n_x = (lines[i][2]-lines[i][0])/mag;
        n_y = (lines[i][3]-lines[i][1])/mag;
        
        lineNormals.at<float>(i,0) = -n_y;
        lineNormals.at<float>(i,1) = n_x;
    }
    
    Mat sumA(2,2, CV_32F, Scalar(0));
    Mat sumB(2,1, CV_32F, Scalar(0));

    float a, b, c, d;
    for (int i=0; i<lines.size(); i++) {
        a = lineNormals.at<float>(i, 0);
        b = lineNormals.at<float>(i, 1);
        c = linePoints.at<float>(i, 0);
        d = linePoints.at<float>(i, 1);
        sumA += (Mat_<float>(2,2) << a*a, a*b, a*b, b*b);
        sumB += (Mat_<float>(2,1) << a*a*c + a*b*d, a*b*c + b*b*d);
    }
    
    Mat out = sumA.inv()*sumB;
    return Point(out.at<float>(0,0), out.at<float>(1,0));
}

float pointToLineDistance(Vec4i line, Point p) {
    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];
    int x3 = p.x;
    int y3 = p.y;
    
    float u = ((x3-x1)*(x2-x1) + (y3-y1)*(y2-y1)) / ((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));    // percent along line segment of normal intersection
    float _x = x1 + u*(x2-x1);
    float _y = y1 + u*(y2-y1);
    
    return sqrt((_x-x3)*(_x-x3) + (_y-y3)*(_y-y3));
    /*
    Mat P = (Mat_<float>(2,1) << (float)line[0], (float)line[1]);
    //float P_x = line[0];
    //float P_y = line[1];
    float mag = sqrt((line[2]-line[0])*(line[2]-line[0]) + (line[3]-line[1])*(line[3]-line[1]));
    Mat N = (Mat_<float>(2,1) << (line[2]-line[0])/mag, (line[3]-line[1])/mag);
    Mat X = (Mat_<float>(2,1) << p.x, p.y);
    //float n_x = (line[2]-line[0])/mag;
    //float n_y = (line[3]-line[1])/mag;    

    Mat d2 = (X-P).t()*N*N.t()*(X-P);
    return sqrt(d2.at<float>(0,0));
     */
}

inline float segmentLength(Vec4i segment) {
    return sqrt((segment[2]-segment[0])*(segment[2]-segment[0]) + (segment[3]-segment[1])*(segment[3]-segment[1]));
}

float pointDistance(Point p1, Point p2) {
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

Point intersection(Vec4i a, Vec4i b) {
    int x1 = a[0];
    int y1 = a[1];
    int x2 = a[2];
    int y2 = a[3];
    int x3 = b[0];
    int y3 = b[1];
    int x4 = b[2];
    int y4 = b[3];
    float denomX = ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
    float denomY = ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
    if(denomX != 0 && denomY != 0)
        return Point(((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / denomX,
                     ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denomY);
    else return Point(-1,-1);
}

