#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        // Add in the catch case for the SIFT, as it only works with NORM_L2 and not Hamming for Brute Force
        int normType = descriptorType.compare("DES_SIFT") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        //int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // take care with the FLANN based data type (video note)
        // https://docs.opencv.org/3.4/dc/de2/classcv_1_1FlannBasedMatcher.html
        matcher = cv::FlannBasedMatcher::create();
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { 
        // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    {
        // k nearest neighbors (k=2) (two best matches)
        // (ref Lesson 4 descriptor_matching)
        // My Code
        vector<vector<cv::DMatch>> knn_matches;
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        // TODO : implement k-nearest-neighbor matching
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            // filter matches using descriptor distance ratio test
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints (BRIEF, ORB, FREAK, AKAZE, SIFT)
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        // https://docs.opencv.org/3.4/d1/d93/classcv_1_1xfeatures2d_1_1BriefDescriptorExtractor.html
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        // https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        // https://docs.opencv.org/3.4/df/db4/classcv_1_1xfeatures2d_1_1FREAK.html
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        // https://docs.opencv.org/3.4/d8/d30/classcv_1_1AKAZE.html
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        // ref (Lesson 4 - Tracking Image Features - describe_keypoints.cpp)
        extractor = cv::xfeatures2d::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using the Harris detector
void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Creae t for time tick
    double t = (double)cv::getTickCount();

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    //string windowName = "Harris Corner Detector Response Matrix";
    //cv::namedWindow(windowName, 4);
    //cv::imshow(windowName, dst_norm_scaled);
    //cv::waitKey(0);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    
    // Create a vector variable for storing the keypoints
    //vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression

    //for loop to loop through the rows of the Harris Matrix image
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        //for loop to loop through the columns of the Harris Matrix image
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            // Create a response variable at each j and i iteration point (8 bit int)
            int response = (int)dst_norm.at<float>(j, i);
            // if statement to capture only point more that the minimum threshold set above
            if (response > minResponse)
            {
                // Create new keypoint variable for storing filtered keypoints
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response; // strength of keypoint

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                // NMS (highest cornerness in the local neigbourhood AND should not overlap too much with other keypoints nearby to avoid clustering)
                // Create boolean variable for the overlap state?
                bool bOverlap = false;
                // for loop to loop through the start and end of the keypoints vector
                for (auto it = keypoints.begin(); it !=keypoints.end(); ++it)
                {
                    // create new keypoint overlap veriable of cv keypoint type, that points for the ith value in the loop
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    // if the keypoint overlap variable is more than the maxOverlap value set above
                    if (kptOverlap > maxOverlap)
                    {
                        // set the boolean overlap variable to true
                        bOverlap = true;
                        // check the newKeyPoint response is higher for the new keypint 
                        if (newKeyPoint.response > (*it).response)
                        {
                            // replace the existing keypoint with this new one
                            *it = newKeyPoint;
                            // quit looping over the keypoints
                            break;
                        }
                    }
                }
                // if bOverlap variable is false
                if (!bOverlap)
                {
                    // add new keypoints where no overlap is found in previous NMS
                    // appennd new keypoint to keypoints list
                    keypoints.push_back(newKeyPoint);
                }
            }
        }
    }

    // Calculate and print time taken for detection
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "My Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

}

// Detect keypoints in image using the modern detector (FAST, BRISK, ORB, AKAZE, SIFT)
void detKeypointsModern(vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    //Create open cv feature detector type variable
    cv::Ptr<cv::FeatureDetector> detector;
    // Create threshold for FAST
    int threshold = 30;
    // Create t for time tick
    double t = (double)cv::getTickCount();

    if (detectorType.compare("FAST") == 0)
    {
        // Creat the Fast feature detector, TYPE_9_16, TYPE_7_12, TYPE_5_8 (ref Lesson 4 detect_keypoints)
       cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
       detector = cv::FastFeatureDetector::create(threshold, true, type);
    }
    // https://docs.opencv.org/3.4/de/dbf/classcv_1_1BRISK.html
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }
    // https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html
    else if (detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
    }
    // https://docs.opencv.org/3.4/d8/d30/classcv_1_1AKAZE.html
    else if (detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
    }
    // ref (Lesson 4 - Tracking Image Features - describe_keypoints.cpp)
    else if (detectorType.compare("SIFT") == 0)
    {
        detector = cv::xfeatures2d::SIFT::create();
    }

    // Instantiate the detector (ref Lesson 4 detect_keypoints)
    detector->detect(img, keypoints);

    // Calculate and print time taken for detection
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detector << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "My Modern Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}