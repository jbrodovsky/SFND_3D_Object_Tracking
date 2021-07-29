
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
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    /* // Outlier removal
    double dist = 0.0;
    int matches = 0;
    for(cv::DMatch &match : kptMatches)
    {
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {   // Check to see if the keypoint is in the ROI
            dist += match.distance;
            matches++;
        }
    }
    dist /= matches;
    */
    for(cv::DMatch match : kptMatches)
    { 
        if(boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            boundingBox.kptMatches.push_back(match);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distance_ratios;
    for(auto outer = kptMatches.begin(); outer!=kptMatches.end() - 1; ++outer)
    {
        // Keypoint outer loop
        cv::KeyPoint outer_current = kptsCurr.at(outer->trainIdx);
        cv::KeyPoint outer_previous = kptsCurr.at(outer->queryIdx);
        for(auto inner = kptMatches.begin()+1; inner != kptMatches.end(); ++inner)
        {
            // Keypoint inner loop
            double min_distance = 100.0;
            cv::KeyPoint inner_current = kptsCurr.at(inner->trainIdx);
            cv::KeyPoint inner_previous = kptsPrev.at(inner->queryIdx);
            double current_distance = cv::norm(outer_current.pt - inner_current.pt);
            double previous_distance = cv::norm(inner_current.pt - inner_previous.pt);
            double distance_ratio = 0;
            try
            {
                distance_ratio = current_distance / previous_distance;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            distance_ratios.push_back(distance_ratio);           
        }
    }

    if(distance_ratios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    auto mean_distance_ratio = std::accumulate(distance_ratios.begin(), distance_ratios.end(), 0.0) / distance_ratios.size();
    std::sort(distance_ratios.begin(), distance_ratios.end());
    double median_ratio = 0.0;
    int mid = distance_ratios.size() / 2;
    if (distance_ratios.size() % 2 == 0)
    {
        // is even
        median_ratio = (distance_ratios.at(mid) + distance_ratios.at(mid - 1))/2;
    }
    else
    {
        // is odd
        median_ratio = distance_ratios.at(mid);
    }

    TTC = -1 / ((1 - median_ratio) *frameRate);
    std::cout << "TTC Camera: "<< TTC;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dt = 1/frameRate;
    double prev_x = 0;
    double curr_x = 0;
    // Average the x values for the previous frame
    for(auto itr = lidarPointsPrev.begin(); itr!=lidarPointsPrev.end(); ++itr)
    {
        if(fabs(itr->y) <= 2.0) // Lane width assumed to be ~4m
        { prev_x += (*itr).x; }
    }
    prev_x /= lidarPointsPrev.size();
    // Average the x values for the current frame
    for(auto itr = lidarPointsCurr.begin(); itr!=lidarPointsCurr.end(); ++itr)
    {
        if(fabs(itr->y) <= 2) { curr_x += (*itr).x; }
    }
    curr_x /= lidarPointsCurr.size();
    // Calculate x-axis speed and divide curr_x by it to get TTC
    TTC = curr_x / ((prev_x - curr_x) / dt);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // Initialize our matches matrix
    cv::Mat matcher = cv::Mat::zeros(prevFrame.boundingBoxes.size(), currFrame.boundingBoxes.size(), CV_32S);
    // Loop through all keypoint matches
    for(auto itr = matches.begin(); itr!=matches.end(); ++itr)
    {   
        // Check the bounding boxes
        cv::KeyPoint current = currFrame.keypoints[itr->trainIdx];
        cv::KeyPoint previous = prevFrame.keypoints[itr->queryIdx];
        for(int i=0; i<prevFrame.boundingBoxes.size(); i++)
        {
            for(int j=0; j<currFrame.boundingBoxes.size(); j++)
            {
                if(currFrame.boundingBoxes[j].roi.contains(current.pt) && prevFrame.boundingBoxes[i].roi.contains(current.pt))
                {
                    matcher.at<int>(i,j)++; // Increment the number of keypoints in the region identified by the bounding box
                }
            }
        }
    }
    // Count occurences in the matches matrix and select the one with more keypoints
    int best_value, id;
    // Loop through the rows
    for(size_t row=0; row<matcher.rows; row++)
    {
        best_value = 0;
        id = -1;
        // Loop through the cols
        for(size_t col=0; col<matcher.cols; col++)
        {
            // Check to see if the current entry is a better match than the currently identified one
            if(matcher.at<int>(row, col) > best_value && matcher.at<int>(row, col)>0)
            {
                best_value = matcher.at<int>(row, col);
                id = col;
            }
        }
        if (id != -1) { bbBestMatches.emplace(row, id); }
    }
    
}
