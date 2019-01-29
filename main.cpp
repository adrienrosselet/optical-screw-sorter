//
//  main.cpp
//  screwSorter
//
//  Created by Adrien Rosselet on 26.10.18.
//  Copyright © 2018 Adrien Rosselet. All rights reserved.
//
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <math.h>

using namespace cv;
using namespace std;

static int posmina=40;
static int posmaxa=190;
static int posminb=45;
static int posmaxb=145;
static int posminz=45;
static int posmaxz=132;


static int correctiona=-1;   //when the belt is shifted, correct the "envoie" function
static int correctionb=0;
//static float angleScaleb=(float)7/(float)9;
static float angleScaleb=0.75;

Point2f plateCenter;
Point2f robotCtr(670,936);
float imageAngle=0;
float plateRadius=0;

int blobSize=200;
class CustomBlob;

void on_trackbar(int blobSize,void*);
void simpleBlob(Mat imgInput, vector<KeyPoint> & vecBloCent);
void M3boltDetection(Mat imgInput, vector<KeyPoint> & vecBloCent);
void M3ringDetection(Mat imgInput, vector<KeyPoint> & vecBloCent);
void M3screwDetection(Mat imgInput, vector<KeyPoint> & vecBloCent);
vector<CustomBlob> customDetector(Mat img);
Mat& ScanImageAndReduceC(Mat& I, const uchar* const table);
int setUpSerial(void);
int serialport_write(int fd, const char* str);
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout);
int serialport_flush(int fd);
Point calculAngles(Point2f pt);
void testBoundary(Point& angle,int& angleZ,int& force,bool& lite);
bool envoie(Point angle,int angleZ,int force,bool lite, int fd);
void calculCenter( Mat& I, Point2f& center, float& radius);
void calculImageRotation(Mat& I, float& imageAngle, float& heightLeft, float& heightRight, int distToCorner);
Point2f pix2mm(Point2f pixelPos);
void collect(vector<KeyPoint>& centerBlobs, Point2f pos, int force, int posZDown, int& fd);

int main(int argc, const char * argv[]) {
    Mat image;
    int methode=1;
    //image = imread("/Users/adrien/Desktop/photoVis/IMG_1393.JPG",IMREAD_GRAYSCALE);
    
    
    int fd = setUpSerial();
    
    
    //envoie(Point(90,90),posminz,0,true, fd);
    
    //Point2f testPT(1280/2,960/2);//image center
    
    
    
    envoie(Point(posmina,posmaxb),posminz,0,true, fd);  //turn on lite
    VideoCapture cap;
    cap.open(CAP_ANY);

    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    cap.read(image);
    envoie(Point(posmina,posmaxb),posminz,0,false, fd);     //turn off lite
    //resize(image, image, Size(image.cols/2, image.rows/2));
    imwrite("essai.jpg", image);
    
    image = imread("/Users/adrien/Documents/applicadri/screwSorter/build/Debug/essai.jpg",IMREAD_GRAYSCALE);
    
    
    float leftPt;
    float rightPt;
    
    const int distToBorder = 150;    //used for angle image rotation, specify where the measurement is done
    calculCenter(image, plateCenter, plateRadius);
    calculImageRotation(image,imageAngle, leftPt, rightPt, distToBorder);
    circle( image,plateCenter, plateRadius, Scalar(255), 3, LINE_8);
    robotCtr = plateCenter + Point2f(0,72);
    
    //envoie(calculAngles(pix2mm(testPT)),posminz,0,false, fd);
    //resize(image, image, Size(image.cols/4, image.rows/4));
    
    
    //circle( image,Point(660,895), 700, Scalar(255), 3, LINE_8);
    circle( image,plateCenter, 150, Scalar(0), FILLED, LINE_8);//modify also "envoie" func
    line(image, Point(distToBorder,leftPt), Point(image.cols-distToBorder,rightPt), Scalar(255),5);
    //circle( image,Point(645,895+432), 30, Scalar(255), FILLED, LINE_8);
    Mat M = getRotationMatrix2D(plateCenter, -imageAngle, 1);
    warpAffine(image, image, M, Size(image.cols,image.rows));
    
    //circle( image,Point(round(testPT.x),round(testPT.y)), 10, Scalar(255), FILLED, LINE_8);
    //circle( image,Point(plateCenter.x,plateCenter.y)-Point(0,346), 10, Scalar(255), FILLED, LINE_8);//for calibration
    
//    Mat resisedImg(image.rows/2,image.cols/2,image.type());
//    resize(image, resisedImg, Size(image.cols/2, image.rows/2));
//    imshow("img", resisedImg);
//    waitKey();
//    return 0;
    
    
    if(methode==1){
        Point angles;
        
        vector<KeyPoint> vectorBlobCenters;
        
        
        //simpleBlob(image, vectorBlobCenters);   //it fills vectorCenters
        M3boltDetection(image, vectorBlobCenters);
        imshow("ima", image);
        waitKey();
        collect(vectorBlobCenters, Point2f(-170,0), 130, 130, fd);//blobCentres, boxPosition, force, Zpos, fd
        
        M3ringDetection(image, vectorBlobCenters);
        collect(vectorBlobCenters, Point2f(-140,0), 130, 130, fd);//blobCentres, boxPosition, force, Zpos, fd

        M3screwDetection(image, vectorBlobCenters);
        collect(vectorBlobCenters, Point2f(-110,0), 250, 130, fd);//blobCentres, boxPosition, force, Zpos, fd

        envoie(Point(90,90),posminz,0,0,fd);
        close(fd);
        //angles = calculAngles();
        
        cout<<"coucou"<<endl;
    }
    else if(methode==2){
        Mat mask(image.rows,image.cols, CV_8UC1 , Scalar(0));
        //Mat imgAffiche(image.rows,image.cols, CV_8UC3 , Scalar(0,0,0));
        circle( mask, Point(mask.cols/2,mask.rows/2), mask.rows/2, Scalar(1), FILLED, LINE_8);
        
        namedWindow("black and white",CV_WINDOW_NORMAL);
        //resizeWindow("black and white", 100, 100);
        //medianBlur(image, image, 3);//nbr has to be odd
        image = image < 110;//black and white with threshold = 110
        image = image.mul(mask);//remove the corners
        
        //int fd = setUpSerial();
        
        
//        serialport_flush(fd);
//        if(serialport_write(fd, "140 140\n") == -1){
//            cout << "probleme" << endl;
//        }
//        serialport_read_until(fd, okarray, 'k', 3, 5000);
//        if(!(okarray[0]=='o' && okarray[1]=='k')){
//            cout << "can t read" << endl;
//        }
//        if(serialport_write(fd, "90 90\n") == -1){
//            cout << "probleme" << endl;
//        }
//        close(fd);
        
        
        //int n = read (fd, buf, sizeof(buf));  // read up to 100 characters if ready to read
        
//        FILE* id = fopen("/dev/tty.usbserial-1420","r");
//        if(id==NULL){
//            cout << "connection error" << endl;
//        }
        
        
        vector<CustomBlob> vecBlob = customDetector(image);
        
//        for(int i=0;i<vecBlob.size();++i){
//            if(){
//
//            }
//        }
        
        //cout << image(Range(image.cols/2-2, image.cols/2+2),Range(image.rows/2-2, image.rows/2+2)) << endl;
        imshow("black and white", image);
        waitKey();
    }

//    Mat element = getStructuringElement(MORPH_RECT, Size(2,2));
//    dilate(image, image, element);
//    erode(image, image, element);
    

    
    return 0;
}

class CustomBlob{
    Point pmin,pmax;
public:
    CustomBlob(Point p) : pmin(p), pmax(p) {}     //constructor
    int area () {return ((pmax.x-pmin.x)*(pmax.y-pmin.y));}
    //Point center() {return Point(height/2,width/2);}
    bool isNear(Point p){
        if(p.x < pmax.x+2 && p.y < pmax.y+2 && p.x > pmin.x-2 && p.y > pmin.y-2){
            return true;
        }
        else return false;
    }
    void include(Point p){
        pmax.x = max(pmax.x,p.x);
        pmax.y = max(pmax.y,p.y);
        pmin.x = min(pmin.x,p.x);
        pmin.y = min(pmin.y,p.y);
    }
    void draw(InputOutputArray img){
        rectangle(img, pmin, pmax, Scalar(150));
    }
};

vector<CustomBlob> customDetector(Mat img){
    vector<CustomBlob> vect;
    int chanels = img.channels();
    int nbrRows = img.rows;
    int nbrCols = img.cols * chanels;
    
    if (img.isContinuous())
    {
        nbrCols *= nbrRows;
        nbrRows = 1;
    }
    int i,j;
    uchar* p;
    for(i=0; i<nbrRows; ++i){
        p = img.ptr<uchar>(i);
        for(j=0; j<nbrCols; ++j ){
            if(p[j]==255){ //if pixel white
                Point pnt((j%img.cols)/chanels,(j/img.cols)/(chanels)); //calculate the x and y position
                bool alone = true;
//                for(vector<CustomBlob>::iterator it=vect.begin(); it!=vect.end(); ++it){
//                    if(it->isNear(pnt)){
//                        alone = false;
//                        it->include(pnt);
//                    }
//                }
                for(int i=0; i<vect.size(); ++i){
                    if(vect[i].isNear(pnt)){
                        alone = false;
                        vect[i].include(pnt);
                    }
                }
                if(alone){
                    CustomBlob newBlob(pnt);
                    vect.push_back(newBlob);
                }
            }
        }
    }
    
//    for(vector<CustomBlob>::iterator it=vect.begin();it!=vect.end();++it){
//        if(it->area() < 1){
//            vect.erase(it);
//        }
//    }

    
//    for(vector<CustomBlob>::iterator it=vect.begin();it!=vect.end();++it){
//        if(it->area()>10){
//            it->draw(img);
//        }
//    }
    for(int i=0; i<vect.size(); ++i){
        if(vect[i].area()>10 && vect[i].area()<10500){
            vect[i].draw(img);
        }
    }
    return vect;
}

void on_trackbar(int blobSize,void*){
//    vector<KeyPoint> blobCenters;
//    SimpleBlobDetector::Params ringDetector;
//    ringDetector.maxThreshold=175;
//    ringDetector.minThreshold=70;
//    ringDetector.filterByColor=true;
//    ringDetector.blobColor=255;
//    ringDetector.filterByArea=true;
//    ringDetector.minArea=170;               //criterion
//    ringDetector.maxArea=blobSize;
//    ringDetector.filterByCircularity=true;
//    ringDetector.minCircularity=0.9;
//    ringDetector.filterByConvexity=false;
//    ringDetector.minConvexity=0.87;
//    ringDetector.filterByInertia=false;
//    ringDetector.minInertiaRatio=0.01;
//    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(ringDetector);
//    detector->detect(image, blobCenters);
//    drawKeypoints(image, blobCenters, image, Scalar(255,0,0));
//    imshow("black and white", image);
}
void simpleBlob(Mat imgInput, vector<KeyPoint> & vecBloCent){
    vector<KeyPoint> blobCenters;
    
    SimpleBlobDetector::Params ringDetector;
    ringDetector.maxThreshold=175;
    ringDetector.minThreshold=70;
    ringDetector.filterByColor=true;
    ringDetector.blobColor=255;            //white
    ringDetector.filterByArea=true;
    ringDetector.minArea=1;               //small
    ringDetector.maxArea=50;
    ringDetector.filterByCircularity=true;  //round
    ringDetector.minCircularity=0.7;
    ringDetector.filterByConvexity=false;
    ringDetector.minConvexity=0.87;
    ringDetector.filterByInertia=false;
    ringDetector.minInertiaRatio=0.01;
    
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(ringDetector);
    detector->detect(imgInput, blobCenters);
    drawKeypoints(imgInput, blobCenters, imgInput, Scalar(255,0,0));
    vecBloCent.insert(vecBloCent.end(), blobCenters.begin(), blobCenters.end());
    
//    ringDetector.maxThreshold=175;
//    ringDetector.minThreshold=70;
//    ringDetector.filterByColor=true;
//    ringDetector.blobColor=255;         //white
//    ringDetector.filterByArea=true;
//    ringDetector.minArea=50;           //middle size
//    ringDetector.maxArea=170;
//    ringDetector.filterByCircularity=true;
//    ringDetector.minCircularity=0.7;        //circular
//    ringDetector.filterByConvexity=false;
//    ringDetector.minConvexity=0.87;
//    ringDetector.filterByInertia=false;
//    ringDetector.minInertiaRatio=0.01;
//
//    detector = SimpleBlobDetector::create(ringDetector);
//    detector->detect(imgInput, blobCenters);
//    drawKeypoints(imgInput, blobCenters, imgInput, Scalar(0,255,0));
//    vecBloCent.insert(vecBloCent.end(), blobCenters.begin(), blobCenters.end());
//
//    ringDetector.maxThreshold=175;
//    ringDetector.minThreshold=70;
//    ringDetector.filterByColor=true;
//    ringDetector.blobColor=255;
//    ringDetector.filterByArea=true;
//    ringDetector.minArea=80;
//    ringDetector.maxArea=130;
//    ringDetector.filterByCircularity=true;
//    ringDetector.minCircularity=0.7;
//    ringDetector.filterByConvexity=false;
//    ringDetector.minConvexity=0.87;
//    ringDetector.filterByInertia=false;
//    ringDetector.minInertiaRatio=0.01;
//
//    detector = SimpleBlobDetector::create(ringDetector);
//    detector->detect(imgInput, blobCenters);
//    drawKeypoints(imgInput, blobCenters, imgInput, Scalar(0,0,255));
//    vecBloCent.insert(vecBloCent.end(), blobCenters.begin(), blobCenters.end());
    
    ringDetector.maxThreshold=175;
    ringDetector.minThreshold=70;
    ringDetector.filterByColor=true;
    ringDetector.blobColor=0;
    ringDetector.filterByArea=true;
    ringDetector.minArea=5;
    ringDetector.maxArea=100;
    ringDetector.filterByCircularity=false;
    //ringDetector.minCircularity=0.7;
    ringDetector.maxCircularity=0.5;
    ringDetector.filterByConvexity=false;
    ringDetector.maxConvexity=0.2;
    ringDetector.filterByInertia=true;
    ringDetector.maxInertiaRatio=0.8;
    
    detector = SimpleBlobDetector::create(ringDetector);
    detector->detect(imgInput, blobCenters);
    drawKeypoints(imgInput, blobCenters, imgInput, Scalar(255,255,0));
    vecBloCent.insert(vecBloCent.end(), blobCenters.begin(), blobCenters.end());
    //imshow("simple blob detector", imgInput);
    //waitKey();
}

void M3boltDetection(Mat imgInput, vector<KeyPoint> & vecBloCent){
    vector<KeyPoint> blobCenters;
    
    SimpleBlobDetector::Params ringDetector;
    ringDetector.maxThreshold=175;
    ringDetector.minThreshold=70;
    ringDetector.filterByColor=true;
    ringDetector.blobColor=255;            //white
    ringDetector.filterByArea=true;
    ringDetector.minArea=150;               //small
    ringDetector.maxArea=300;
    ringDetector.filterByCircularity=true;  //round
    ringDetector.minCircularity=0.7;
    ringDetector.filterByConvexity=false;
    ringDetector.minConvexity=0.87;
    ringDetector.filterByInertia=false;
    ringDetector.minInertiaRatio=0.01;
    
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(ringDetector);
    detector->detect(imgInput, blobCenters);
    drawKeypoints(imgInput, blobCenters, imgInput, Scalar(255,0,0));
    vecBloCent.insert(vecBloCent.end(), blobCenters.begin(), blobCenters.end());
    
    //imshow("simple blob detector", imgInput);
    //waitKey();
}

void M3ringDetection(Mat imgInput, vector<KeyPoint> & vecBloCent){
    vector<KeyPoint> blobCenters;
    
    SimpleBlobDetector::Params ringDetector;
    ringDetector.maxThreshold=175;
    ringDetector.minThreshold=70;
    ringDetector.filterByColor=true;
    ringDetector.blobColor=255;            //white
    ringDetector.filterByArea=true;
    ringDetector.minArea=301;               //small
    ringDetector.maxArea=700;
    ringDetector.filterByCircularity=true;  //round
    ringDetector.minCircularity=0.7;
    ringDetector.filterByConvexity=false;
    ringDetector.minConvexity=0.87;
    ringDetector.filterByInertia=false;
    ringDetector.minInertiaRatio=0.01;
    
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(ringDetector);
    detector->detect(imgInput, blobCenters);
    drawKeypoints(imgInput, blobCenters, imgInput, Scalar(255,0,0));
    vecBloCent.insert(vecBloCent.end(), blobCenters.begin(), blobCenters.end());
    
    //imshow("simple blob detector", imgInput);
    //waitKey();
}

void M3screwDetection(Mat imgInput, vector<KeyPoint> & vecBloCent){
    vector<KeyPoint> blobCenters;
    
    SimpleBlobDetector::Params ringDetector;
    ringDetector.maxThreshold=175;
    ringDetector.minThreshold=70;
    ringDetector.filterByColor=true;
    ringDetector.blobColor=0;            //black
    ringDetector.filterByArea=true;
    ringDetector.minArea=301;               //small
    ringDetector.maxArea=5500;
    ringDetector.filterByCircularity=false;  //round
    ringDetector.minCircularity=0.7;
    ringDetector.filterByConvexity=false;
    ringDetector.minConvexity=0.87;
    ringDetector.filterByInertia=false;
    ringDetector.minInertiaRatio=0.01;
    
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(ringDetector);
    detector->detect(imgInput, blobCenters);
    drawKeypoints(imgInput, blobCenters, imgInput, Scalar(255,0,0));
    vecBloCent.insert(vecBloCent.end(), blobCenters.begin(), blobCenters.end());
    
    //imshow("simple blob detector", imgInput);
    //waitKey();
}

Point calculAngles(Point2f pt){
    const float b=100, c=150;   //arm length in mm
    
    Point angles;
    float gamma = atan2(pt.y, pt.x) * 180 / M_PI;
    float a = norm(pt);
    if(a > b + c){
        cout << "too fare" << endl;
        angles.x = -1;
        angles.y = -1;
    }
    else if (a < c - b){
        cout << "too close" << endl;
        angles.x = -1;
        angles.y = -1;
    }
    else{
        angles.y = round(gamma - acos((a*a-b*b+c*c)/(2*c*a))* 180 * M_1_PI); //Al kashiiiii; y is beta
        angles.x = round(acos((-a*a+b*b+c*c)/(2*b*c))* 180 * M_1_PI);  //x is alpha
    }
    
    return angles;
}

void calculCenter( Mat& I, Point2f& center, float& radius){
    //get the edge
    CV_Assert(I.depth()== CV_8U);
    
    int width = I.cols;
    int height = I.rows;
    
    //detect the disc edge
    vector<int> edge(width);   //the x position is the index, the y position is the value
    uchar thresholdIntensity=50;       //threshold
    if (I.channels()==1){
        for( int i = 0; i < width; ++i)
            for( int j = 0; j < height; ++j )
                if(I.at<uchar>(j,i) > thresholdIntensity){
                    edge[i]=j;
                    //I.at<uchar>(j,i) = 255;   //for debuging
                    break;
                }
    }
    else{
        cout << "error: not grayscale image" << endl;
    }
    
    //detect the center and disc radius
    const int squareSize = 201;
    Mat means(squareSize,squareSize,CV_32FC1,Scalar(0));//matrix containing the average distances
    Mat square(squareSize,squareSize,CV_32FC1,Scalar(0));//matrix containing the summed distance to average
    for(int i = 0; i < squareSize; ++i){//row
        for(int j = 0; j < squareSize; ++j){//col
            //average distance in each pixels
            float valAverage=0;
            for(int k = 0; k < width; ++k){
                valAverage += sqrt(((k-(j-(squareSize-1)/2+width/2))*(k-(j-(squareSize-1)/2+width/2))+(edge[k]-(i+height-squareSize))*(edge[k]-(i+height-squareSize)))) / (float)width;
            }
            means.at<float>(i, j) = valAverage;
            
            //squared distance to the average
            float valDist = 0;
            for(int k = 0; k < edge.size(); ++k){
                valDist += abs(valAverage-sqrt(((k-(j-(squareSize-1)/2+width/2))*(k-(j-(squareSize-1)/2+width/2))+(edge[k]-(i+height-squareSize))*(edge[k]-(i+height-squareSize)))));
            }
            square.at<float>(i, j) = valDist;
        }
    }
    
    
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    
    minMaxLoc( square, &minVal, &maxVal, &minLoc, &maxLoc );
    
    radius = means.at<float>(minLoc);
    center = minLoc+Point((-(squareSize-1)/2+width/2),height-squareSize);
    
//    normalize(square, square,255,0);
//    imshow("essai",square);
//    waitKey();
}

void calculImageRotation(Mat& I, float& imageAngle, float& heightLeft, float& heightRight, int distToCorner){
    CV_Assert(I.depth()== CV_8U);
    
    int width = I.cols;
    int height = I.rows;
    
    //detect the down line edge
    const int nbrPoints = 50;
    //const int distToCorner = 50;
    const int deltaLimit = 15;
    
    heightLeft = 0;
    heightRight = 0;
    //vector<int> edgeLeft(nbrPoints);   //vector containing few points on the image left
    //vector<int> edgeRight(nbrPoints);   //vector containing few points on the image right
    //uchar thresholdIntensity=70;       //threshold
    
    int oldIntensity;
    if (I.channels()==1){
        for( int i = 0; i < nbrPoints; ++i){
            oldIntensity = I.at<uchar>(height-1,i+distToCorner);
            for( int j = height-2; j > 0; --j ){
                if((I.at<uchar>(j,i+distToCorner)-oldIntensity) > deltaLimit){
                    heightLeft += ((float)j/(float)nbrPoints);
                    //edgeLeft[i]=j;
                    I.at<uchar>(j,i+distToCorner) = 255;   //for debuging
                    break;
                }
                oldIntensity = I.at<uchar>(j+1,i);
            }
        }
    }
    else{
        cout << "error: not grayscale image" << endl;
    }
    for( int i = 0; i < nbrPoints; ++i){
        oldIntensity = I.at<uchar>(height-1,i-distToCorner);
        for( int j = height-1; j > 0; --j ){
            if((I.at<uchar>(j,i+width-nbrPoints-distToCorner)-oldIntensity) > deltaLimit){
                heightRight += ((float)j/(float)nbrPoints);
                //edgeRight[i]=j;
                I.at<uchar>(j,i+width-nbrPoints-distToCorner) = 255;   //for debuging
                break;
            }
            
        }
    }
    imageAngle = atan2((float)(-heightRight+heightLeft), (float)(width-nbrPoints-2*distToCorner)) * 180 / M_PI;
    
}

void collect(vector<KeyPoint>& centerBlobs, Point2f posBox, int force , int posZDown, int& fd){
    Point2f pos;
    Point angles;
    bool lite = 0;
    static int forcenull = 0;
    int angleZ = posminz;        //position up
    for(int i=0; i < centerBlobs.size(); ++i){
        pos = pix2mm(centerBlobs[i].pt);  //return the x,y position in arm repair
        angles = calculAngles(pos);
        if(angles.x >= 0 && angles.y >= 0){
            testBoundary(angles, angleZ, forcenull, lite);
            envoie(angles, angleZ, forcenull, lite, fd);
            
        }
        angleZ=posZDown;         //position down
        
        testBoundary(angles, angleZ, force, lite);
        envoie(angles,angleZ,force,lite, fd);
        angleZ=posminz;         //Z up
        testBoundary(angles, angleZ, force, lite);
        envoie(angles,angleZ,force,lite, fd);
        
        angles = calculAngles(posBox);
        if(angles.x >= 0 && angles.y >= 0){
            testBoundary(angles, angleZ, force, lite);
            envoie(angles,angleZ,force,lite, fd);
        }
        
        envoie(angles,angleZ,forcenull,lite, fd);   //let it fall
    }
    centerBlobs.clear();
}

Point2f pix2mm(Point2f pixelPos){
    //Point2f robotCtr(645,895+423);      //robot center in pixel 156 230
    //float imgScale=(float)50/(float)140;//50mm equal 35 pixels
    float imgScale=(float)250/plateRadius;    //  mm/pixl
    //ctr = calculCenter(image);
    //circle( image, ctr, 35, Scalar(0), FILLED, LINE_8);
    //imshow("simple blob detector", image);
    //waitKey();
    
    //Mat M = getRotationMatrix2D(plateCenter, imageAngle, imgScale);
    
    Point2f p((pixelPos.x - robotCtr.x) * imgScale,-(pixelPos.y - robotCtr.y) * imgScale);
    //Point2f p((pixelPos.x - robotCtr.x) ,-(pixelPos.y - robotCtr.y) );
    //p = M * p;
    //Point2f pr(p.x * cos(imageAngle)-p.y * sin(imageAngle), p.x * sin(imageAngle) + p.y * cos(imageAngle));
    return p;
}

bool envoie(Point angle,int angleZ,int force,bool lite, int fd){
    //if no error then send
    //serialport_flush(fd);
    String str;
    
    char okarray[5];
    //serialport_flush(fd);
    
    str = to_string(angle.x + correctiona);
    str += " ";
    str += to_string(round((angle.y - 90) * angleScaleb) + correctionb + 90);
    str += " ";
    str += to_string(angleZ);
    str += " ";
    str += to_string(force);
    str += " ";
    str += to_string(lite);
    str += "\n";
    if(serialport_write(fd, str.c_str()) == -1){
        return false;
    }
    if(serialport_read_until(fd, okarray, '\n', 5, 5000) == 0){   //wait for ok
        //cout << "ok" << endl;
    }
    if(!(okarray[0]=='o' && okarray[1]=='k')){
        cout << "can t read" << endl;
        return false;
    }
    
    return true;
}

void testBoundary(Point& angle,int& angleZ,int& force,bool& lite){
    if(angle.x > posmaxa){
        angle.x=posmaxa;
        cout << "posmaxa trepassed" << endl;
    }
    if(angle.x < posmina){
        angle.x=posmina;
        cout << "posmina trepassed" << endl;
    }
    if(angle.y > posmaxb){
        angle.y=posmaxb;
        cout << "posmaxb trepassed" << endl;
    }
    if(angle.y < posminb){
        angle.y=posminb;
        cout << "posminb trepassed" << endl;
    }
    if(angleZ > posmaxz){
        angleZ=posmaxz;
        cout << "posmaxz trepassed" << endl;
    }
    if(angleZ < posminz){
        angleZ=posminz;
        cout << "posmina trepassed" << endl;
    }
    if(force > 255){
        force=255;
        cout << "force max trepassed" << endl;
    }
    if(force < 0){
        force=0;
        cout << "force min trepassed" << endl;
    }
}

Mat& ScanImageAndReduceC(Mat& I, const uchar* const table)
{
    // accept only char type matrices
    CV_Assert(I.depth() == CV_8U);
    int channels = I.channels();
    int nRows = I.rows;
    int nCols = I.cols * channels;
    if (I.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }
    int i,j;
    uchar* p;
    for( i = 0; i < nRows; ++i)
    {
        p = I.ptr<uchar>(i);
        for ( j = 0; j < nCols; ++j)
        {
            p[j] = table[p[j]];
        }
    }
    return I;
}
int setUpSerial(void){
    //dev/tty.usbserial-1420
    int fd = open("/dev/cu.usbserial-A600ajYH", O_RDWR | O_NONBLOCK );
    if (fd == -1)  {
        perror("serialport_init: Unable to open port ");
        return -1;
    }
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    // no flow control
    options.c_cflag &= ~CRTSCTS;
    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
    options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    options.c_oflag &= ~OPOST; // make raw
    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    tcsetattr(fd, TCSANOW, &options);
    if( tcsetattr(fd, TCSAFLUSH, &options) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }
    return fd;
}

int serialport_write(int fd, const char* str)
{
    size_t len = strlen(str);
    ssize_t n = write(fd, str, len);
    if( n!=len ) {
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do {
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            if( timeout==0 ) return -2;
            continue;
        }
#ifdef SERIALPORTDEBUG
        printf("serialport_read_until: i=%d, n=%d b='%c'\n",i,n,b[0]); // debug
#endif
        buf[i] = b[0];
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );
    
    buf[i] = 0;  // null terminate the string
    return 0;
}

//
int serialport_flush(int fd)
{
    sleep(2); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}
