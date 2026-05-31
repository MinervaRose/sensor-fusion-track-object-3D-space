
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
static cv::Rect shrinkRect(const cv::Rect &r, float shrink)
{
    int dx = static_cast<int>(r.width * shrink * 0.5f);
    int dy = static_cast<int>(r.height * shrink * 0.5f);
    return cv::Rect(r.x + dx, r.y + dy, r.width - 2*dx, r.height - 2*dy);
}

void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr,
                              std::vector<cv::DMatch> &kptMatches)
{
    boundingBox.kptMatches.clear();

    // 1) collect matches whose CURRENT keypoint is inside a slightly shrunken ROI
    const cv::Rect roi = shrinkRect(boundingBox.roi, 0.10f); // 10% shrink
    std::vector<cv::DMatch> inliers;
    inliers.reserve(kptMatches.size());
    std::vector<double> motions;
    motions.reserve(kptMatches.size());

    for (const auto &m : kptMatches)
    {
        const cv::Point2f &ptCurr = kptsCurr[m.trainIdx].pt;
        const cv::Point2f &ptPrev = kptsPrev[m.queryIdx].pt;
        if (roi.contains(ptCurr))
        {
            inliers.push_back(m);
            motions.push_back(cv::norm(ptCurr - ptPrev));
        }
    }
    if (inliers.size() < 6) { boundingBox.kptMatches = inliers; return; }

    // 2) robust motion threshold using median and MAD
    auto motionsCopy = motions;
    std::nth_element(motionsCopy.begin(), motionsCopy.begin()+motionsCopy.size()/2, motionsCopy.end());
    const double med = motionsCopy[motionsCopy.size()/2];

    std::vector<double> absDev; absDev.reserve(motions.size());
    for (double d : motions) absDev.push_back(std::fabs(d - med));
    auto devCopy = absDev;
    std::nth_element(devCopy.begin(), devCopy.begin()+devCopy.size()/2, devCopy.end());
    const double mad = devCopy[devCopy.size()/2] + 1e-9; // avoid zero

    const double k = 3.0; // ~3*MAD -> ~99% inliers for Laplacian-ish tails
    const double low = med - k * 1.4826 * mad; // 1.4826 scales MAD to stdev for normal
    const double high = med + k * 1.4826 * mad;

    // 3) keep only inliers by motion length
    for (size_t i = 0; i < inliers.size(); ++i)
    {
        if (motions[i] >= low && motions[i] <= high)
            boundingBox.kptMatches.push_back(inliers[i]);
    }
}



// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev,
                      std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches,
                      double frameRate,
                      double &TTC,
                      cv::Mat *visImg)
{
    (void)visImg;
    TTC = std::numeric_limits<double>::quiet_NaN();
    if (kptMatches.size() < 6 || frameRate <= 0.0) return;

    std::vector<double> distRatios;
    distRatios.reserve(kptMatches.size() * 2);

    // Form all pairs of matches
    for (size_t i = 0; i < kptMatches.size(); ++i)
    {
        const cv::KeyPoint &kp1Prev = kptsPrev[kptMatches[i].queryIdx];
        const cv::KeyPoint &kp1Curr = kptsCurr[kptMatches[i].trainIdx];

        for (size_t j = i + 1; j < kptMatches.size(); ++j)
        {
            const cv::KeyPoint &kp2Prev = kptsPrev[kptMatches[j].queryIdx];
            const cv::KeyPoint &kp2Curr = kptsCurr[kptMatches[j].trainIdx];

            const double distPrev = cv::norm(kp1Prev.pt - kp2Prev.pt);
            const double distCurr = cv::norm(kp1Curr.pt - kp2Curr.pt);

            // Reject small/unstable prev distances & avoid divide by zero; also reject too-close pairs
            const double minDist = 1.0; // pixels
            if (distPrev > minDist && distCurr > 1e-6)
            {
                distRatios.push_back(distCurr / distPrev);
            }
        }
    }

    if (distRatios.size() < 6) return;

    // Robust median of distance ratios
    std::nth_element(distRatios.begin(), distRatios.begin() + distRatios.size()/2, distRatios.end());
    const double medRatio = distRatios[distRatios.size()/2];

    // Optionally: reject extreme ratios via IQR
    {
        auto v = distRatios;
        std::sort(v.begin(), v.end());
        auto q = [&](double p){
            double pos = p * (v.size()-1);
            size_t i = static_cast<size_t>(pos);
            double frac = pos - i;
            return v[i] * (1.0 - frac) + v[std::min(i+1, v.size()-1)] * frac;
        };
        double q1 = q(0.25), q3 = q(0.75), iqr = q3 - q1;
        double lo = q1 - 1.5*iqr, hi = q3 + 1.5*iqr;
        if (medRatio < lo || medRatio > hi) {
            // fall back to clipped median
            std::vector<double> clipped;
            clipped.reserve(v.size());
            for (double r : v) if (r >= lo && r <= hi) clipped.push_back(r);
            if (clipped.size() >= 6) {
                std::nth_element(clipped.begin(), clipped.begin()+clipped.size()/2, clipped.end());
                // overwrite medRatio via const_cast pattern avoided; recompute TTC with clipped median
                const double mr = clipped[clipped.size()/2];
                if (std::fabs(1.0 - mr) > 1e-6) {
                    TTC = - (1.0 / frameRate) / (1.0 - mr);
                }
                return;
            }
        }
    }

    if (std::fabs(1.0 - medRatio) > 1e-6)
    {
        TTC = - (1.0 / frameRate) / (1.0 - medRatio);
    }
    // else TTC stays NaN (no apparent scale change)
}



static double robustMedian(std::vector<double> v)
{
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
    return v[v.size()/2];
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr,
                     double frameRate,
                     double &TTC)
{
    TTC = std::numeric_limits<double>::quiet_NaN();
    if (lidarPointsPrev.empty() || lidarPointsCurr.empty() || frameRate <= 0.0) return;

    // Collect longitudinal distances (x) with basic lateral filtering
    std::vector<double> prevX, currX;
    prevX.reserve(lidarPointsPrev.size());
    currX.reserve(lidarPointsCurr.size());

    const double laneHalfWidth = 2.0; // meters: keep points near ego lane centerline in ROI
    for (const auto &p : lidarPointsPrev)
        if (std::fabs(p.y) <= laneHalfWidth) prevX.push_back(p.x);
    for (const auto &p : lidarPointsCurr)
        if (std::fabs(p.y) <= laneHalfWidth) currX.push_back(p.x);

    if (prevX.size() < 3 || currX.size() < 3) return;

    // Robustify with trimmed median (optional small trimming of nearest 510%)
    auto trimFront = [](std::vector<double> &v, double frac){
        if (v.empty()) return;
        std::sort(v.begin(), v.end());              // ascending: small x = closer
        size_t k = static_cast<size_t>(frac * v.size());
        if (k*2 >= v.size()) return;
        v.erase(v.begin(), v.begin()+k);            // drop closest k
        v.erase(v.end()-k, v.end());                // drop farthest k
    };
    trimFront(prevX, 0.05);
    trimFront(currX, 0.05);

    double dPrev = robustMedian(prevX);
    double dCurr = robustMedian(currX);

    const double dt = 1.0 / frameRate;
    const double denom = dPrev - dCurr;
    if (dCurr > 0.0 && denom > 1e-4)
    {
        TTC = dCurr * dt / denom;
    }
    // else TTC remains NaN (ill-conditioned or moving away)
}



void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bbBestMatches,
                        DataFrame &prevFrame,
                        DataFrame &currFrame)
{
    // Build vote matrix: rows = prev boxID, cols = curr boxID
    const size_t nPrev = prevFrame.boundingBoxes.size();
    const size_t nCurr = currFrame.boundingBoxes.size();
    if (nPrev == 0 || nCurr == 0) return;

    std::vector<std::vector<int>> votes(nPrev, std::vector<int>(nCurr, 0));

    // For faster point-in-rect checks
    auto findBoxIdxContaining = [](const std::vector<BoundingBox> &boxes, const cv::KeyPoint &kp) -> int {
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            if (boxes[i].roi.contains(kp.pt)) return static_cast<int>(i);
        }
        return -1;
    };

    for (const auto &m : matches)
    {
        int idxPrev = findBoxIdxContaining(prevFrame.boundingBoxes, prevFrame.keypoints[m.queryIdx]);
        int idxCurr = findBoxIdxContaining(currFrame.boundingBoxes, currFrame.keypoints[m.trainIdx]);
        if (idxPrev >= 0 && idxCurr >= 0)
        {
            votes[idxPrev][idxCurr] += 1;
        }
    }

    // For each prev box, assign curr box with the most votes
    bbBestMatches.clear();
    for (size_t iPrev = 0; iPrev < nPrev; ++iPrev)
    {
        int bestCurr = -1;
        int bestCount = 0;
        for (size_t jCurr = 0; jCurr < nCurr; ++jCurr)
        {
            if (votes[iPrev][jCurr] > bestCount)
            {
                bestCount = votes[iPrev][jCurr];
                bestCurr = static_cast<int>(jCurr);
            }
        }
        if (bestCurr >= 0)
        {
            int prevID = prevFrame.boundingBoxes[iPrev].boxID;
            int currID = currFrame.boundingBoxes[bestCurr].boxID;
            bbBestMatches[prevID] = currID;
        }
    }
}
