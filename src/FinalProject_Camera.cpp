/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

// Safe CSV logging helper
static inline void append_ttc_rows(
    const std::string &camPath, const std::string &lidarPath,
    int frame, int prevID, int currID,
    size_t nMatches, double ttcCam,
    size_t nLidarPrev, size_t nLidarCurr, double ttcLidar)
{
    { std::ofstream cam(camPath, std::ios::app);
      cam << frame << ',' << prevID << ',' << currID << ','
          << nMatches << ',' << ttcCam << '\n'; }

    { std::ofstream lid(lidarPath, std::ios::app);
      lid << frame << ',' << prevID << ',' << currID << ','
          << nLidarPrev << ',' << nLidarCurr << ',' << ttcLidar << '\n'; }
}


/* MAIN PROGRAM */
int main(int argc, const char* argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // --- Simple CSV loggers for FP.5/FP.6
    ofstream ttcLogCam("ttc_camera_log.csv");
    ttcLogCam << "frame,prevBoxID,currBoxID,nMatches,TTC_camera\n";
    ofstream ttcLogLidar("ttc_lidar_log.csv");
    ttcLogLidar << "frame,prevBoxID,currBoxID,nLidarPrev,nLidarCurr,TTC_lidar\n";

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file 
        cv::Mat img = cv::imread(imgFullFilename);

        // push image into data frame buffer (ring buffer of size dataBufferSize)
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.push_back(frame);
        if (dataBuffer.size() > (size_t)dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }

        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


        /* DETECT & CLASSIFY OBJECTS */

        float confThreshold = 0.2;
        float nmsThreshold = 0.4;        
        detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                      yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

        cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;


        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file
        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
    
        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

        cout << "#3 : CROP LIDAR POINTS done" << endl;


        /* CLUSTER LIDAR POINT CLOUD */

        // associate Lidar points with camera-based ROI
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

        // Visualize 3D objects
        bVis = false; // set true to view 3D top view
        if(bVis)
        {
            show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
        }
        bVis = false;

        cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;

        // NOTE: removed the 'continue;' here so the rest of the pipeline runs


        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        // Default detector; can be overridden by CLI
        std::string detectorType = "SHITOMASI";
        if (argc >= 2) detectorType = argv[1];

        if (detectorType == "SHITOMASI")
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else
        {
            // (HARRIS, FAST, BRISK, ORB, AKAZE, SIFT)
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType == "SHITOMASI")
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            std::cout << "NOTE: Keypoints have been limited!\n";
        }

        // push keypoints for current frame
        (dataBuffer.end() - 1)->keypoints = keypoints;
        std::cout << "#5 : DETECT KEYPOINTS done\n";

        /* EXTRACT KEYPOINT DESCRIPTORS */

        // Choose descriptor type (script may overwrite this line)
        cv::Mat descriptors;
        std::string descriptorType = "BRISK";
        if (argc >= 3) descriptorType = argv[2];

        // Ensure we use a CV_8U image for binary descriptors
        cv::Mat descImg = imgGray;
        if (descImg.empty() || descImg.type() != CV_8U)
        {
            cv::Mat tmp;
            cv::cvtColor((dataBuffer.end() - 1)->cameraImg, tmp, cv::COLOR_BGR2GRAY);
            descImg = tmp;
        }

        if ((dataBuffer.end() - 1)->keypoints.empty())
        {
            std::cout << "WARN: No keypoints detected; skipping descriptor extraction for this frame.\n";
            descriptors.release();
        }
        else
        {
            // Safety checks
            std::cout << "Using detector=" << detectorType
                      << " descriptor=" << descriptorType << "\n";

            if (descriptorType == "AKAZE" && detectorType != "AKAZE")
            {
                std::cout << "  AKAZE descriptor requires AKAZE detector. Switching descriptor to BRISK.\n";
                descriptorType = "BRISK";
            }

            // (Optional) If BRIEF crashes on build, keep this guard; otherwise delete it.
            // if (descriptorType == "BRIEF") {
            //     std::cout << "  BRIEF is unstable on this build. Switching descriptor to BRISK.\n";
            //     descriptorType = "BRISK";
            // }

            // Compute descriptors
            descKeypoints((dataBuffer.end() - 1)->keypoints, descImg, descriptors, descriptorType);
        }

        // push descriptors for current frame
        (dataBuffer.end() - 1)->descriptors = descriptors;
        std::cout << "#6 : EXTRACT DESCRIPTORS done\n";


        


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

        vector<cv::DMatch> matches;
        string matcherType = "MAT_BF";                                  // MAT_BF, MAT_FLANN
        string descriptorCategory = (descriptorType == "SIFT") ? "DES_HOG" : "DES_BINARY";
        string selectorType = "SEL_NN";                                 // SEL_NN, SEL_KNN


            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorCategory, matcherType, selectorType);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            
            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame
            map<int, int> bbBestMatches;
            // NOTE: this repo's signature is (matches, bbBestMatches, prevFrame, currFrame)
            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1));
            // store matches in current data frame
            (dataBuffer.end()-1)->bbMatches = bbBestMatches;

            cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;


            /* COMPUTE TTC ON OBJECT IN FRONT */

            // loop over all BB match pairs
            for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
            {
                // find bounding boxes associated with current match
                BoundingBox *prevBB = nullptr;
                BoundingBox *currBB = nullptr;

                for (auto &bb : (dataBuffer.end() - 1)->boundingBoxes)
                {
                    if (it1->second == bb.boxID) { currBB = &bb; break; }
                }
                for (auto &bb : (dataBuffer.end() - 2)->boundingBoxes)
                {
                    if (it1->first == bb.boxID) { prevBB = &bb; break; }
                }

                if (prevBB == nullptr || currBB == nullptr) continue;

                // compute TTC for current match
                if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                {
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data
                    double ttcLidar = std::numeric_limits<double>::quiet_NaN();
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);

                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box
                    clusterKptMatchesWithROI(*currBB,
                                             (dataBuffer.end() - 2)->keypoints,
                                             (dataBuffer.end() - 1)->keypoints,
                                             (dataBuffer.end() - 1)->kptMatches);

                    //// TASK FP.4 -> compute time-to-collision based on camera
                    double ttcCamera = std::numeric_limits<double>::quiet_NaN();
                    if (currBB->kptMatches.size() >= 6)
                    {
                        computeTTCCamera((dataBuffer.end() - 2)->keypoints,
                                         (dataBuffer.end() - 1)->keypoints,
                                         currBB->kptMatches,
                                         sensorFrameRate,
                                         ttcCamera);
                    }

                    // Log to CSV for FP.5/FP.6
                    ttcLogLidar << imgIndex << "," << prevBB->boxID << "," << currBB->boxID << ","
                                << prevBB->lidarPoints.size() << "," << currBB->lidarPoints.size() << ","
                                << ttcLidar << "\n";
                    ttcLogCam   << imgIndex << "," << prevBB->boxID << "," << currBB->boxID << ","
                                << currBB->kptMatches.size() << ","
                                << ttcCamera << "\n";

                    // Visualization (optional)
                    bVis = false; // set true to see overlays per frame
                    if (bVis)
                    {
                        cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                        
                        char str[200];
                        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                        string windowName = "Final Results : TTC";
                        cv::namedWindow(windowName, 4);
                        cv::imshow(windowName, visImg);
                        cout << "Press key to continue to next frame" << endl;
                        cv::waitKey(0);
                    }
                    bVis = false;

                } // eof TTC computation
            } // eof loop over all BB matches            

        }

    } // eof loop over all images

    return 0;
}
