// Lifted from OpenCV examples. Modified to write out a YAML file directly
// usable by the ORB_SLAM2 code (see e.g. optical_trajectories.cc).
// Also uses command line flags instead of an input file in the original
// example.

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(board_side_width, 7, "Calibration board width in elements (e.g. "
                                  "the number of inner corners on a "
                                  "checkerboard.");
DEFINE_int32(board_side_height, 5, "Calibration board height in elements (e.g. "
                                   "the number of inner corners on a "
                                   "checkerboard.");
DEFINE_int32(square_size, -1,
             "Linear element size, in some units (e.g. in mm).");
DEFINE_string(pattern, "CHESSBOARD",
              "CHESSBOARD CIRCLES_GRID ASYMMETRIC_CIRCLES_GRID");
DEFINE_string(
    input, "",
    "USB camera id (between 0-9), XML image list file, or a video file.");
DEFINE_bool(flip_horizontal_axis, false,
            "Whether to flip the image around horizontal axis.");
DEFINE_int32(input_delay, 100, "For live camera input, how long to wait (in "
                               "ms) before starting to capture frames for "
                               "processing.");
DEFINE_int32(skip_frames, 0, "How many frames to skip after successfully "
                             "detecting a calibration pattern frame. Used to "
                             "avoid having calibration frames too close and "
                             "similar to each other.");
DEFINE_int32(frames_to_use, 25, "The number of frames to use for calibration.");
DEFINE_double(fix_aspect_ratio, 1.0, "");
DEFINE_bool(assume_zero_tangential_distortion, true, "");
DEFINE_bool(fix_principal_point_at_center, true, "");
DEFINE_string(out_file, "",
              "File name to write the resulting calibration settings to.");
DEFINE_bool(write_extrinsic_parameters, true, "");
DEFINE_bool(show_undistorted_image, true, "Whether to display the remainder of "
                                          "the  video in an undistorted shape "
                                          "after finishing calibration.");

using namespace cv;
using namespace std;

class Settings {
public:
  Settings() : goodInput(false) {}
  enum Pattern {
    NOT_EXISTING,
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID
  };
  enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

  void write(FileStorage &fs) const // Write serialization for this class
  {
    fs << "{"
       << "BoardSize_Width" << boardSize.width << "BoardSize_Height"
       << boardSize.height << "Square_Size" << squareSize << "Calibrate_Pattern"
       << patternToUse << "Calibrate_NrOfFrameToUse" << nrFrames
       << "Calibrate_FixAspectRatio" << aspectRatio
       << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
       << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

       << "Write_DetectedFeaturePoints" << bwritePoints
       << "Write_extrinsicParameters" << bwriteExtrinsics
       << "Write_outputFileName" << outputFileName

       << "Show_UndistortedImage" << showUndistorsed

       << "Input_FlipAroundHorizontalAxis" << flipVertical << "Input_Delay"
       << delay << "Input" << input << "}";
  }

  void interprate() {
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0) {
      cerr << "Invalid Board size: " << boardSize.width << " "
           << boardSize.height << endl;
      goodInput = false;
    }
    if (squareSize <= 10e-6) {
      cerr << "Invalid square size " << squareSize << endl;
      goodInput = false;
    }
    if (nrFrames <= 0) {
      cerr << "Invalid number of frames " << nrFrames << endl;
      goodInput = false;
    }

    if (input.empty()) // Check for valid input
      inputType = INVALID;
    else {
      if (input[0] >= '0' && input[0] <= '9') {
        stringstream ss(input);
        ss >> cameraID;
        inputType = CAMERA;
      } else {
        if (isListOfImages(input) && readStringList(input, imageList)) {
          inputType = IMAGE_LIST;
          nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames
                                                        : (int)imageList.size();
        } else
          inputType = VIDEO_FILE;
      }
      if (inputType == CAMERA)
        inputCapture.open(cameraID);
      if (inputType == VIDEO_FILE)
        inputCapture.open(input);
      if (inputType != IMAGE_LIST && !inputCapture.isOpened())
        inputType = INVALID;
    }
    if (inputType == INVALID) {
      cerr << " Inexistent input: " << input;
      goodInput = false;
    }

    flag = 0;
    if (calibFixPrincipalPoint)
      flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
    if (calibZeroTangentDist)
      flag |= CV_CALIB_ZERO_TANGENT_DIST;
    if (aspectRatio)
      flag |= CV_CALIB_FIX_ASPECT_RATIO;

    calibrationPattern = NOT_EXISTING;
    if (!patternToUse.compare("CHESSBOARD"))
      calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID"))
      calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID"))
      calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibrationPattern == NOT_EXISTING) {
      cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
      goodInput = false;
    }
    atImageList = 0;
  }
  Mat nextImage() {
    Mat result;
    if (inputCapture.isOpened()) {
      Mat view0;
      inputCapture >> view0;
      view0.copyTo(result);
    } else if (atImageList < (int)imageList.size())
      result = imread(imageList[atImageList++], CV_LOAD_IMAGE_COLOR);

    return result;
  }

  static bool readStringList(const string &filename, vector<string> &l) {
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
      return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
      return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
      l.push_back((string)*it);
    return true;
  }

  static bool isListOfImages(const string &filename) {
    string s(filename);
    // Look for file extension
    if (s.find(".xml") == string::npos && s.find(".yaml") == string::npos &&
        s.find(".yml") == string::npos)
      return false;
    else
      return true;
  }

public:
  Size
      boardSize; // The size of the board -> Number of items by width and height
  Pattern calibrationPattern; // One of the Chessboard, circles, or asymmetric
                              // circle pattern
  float squareSize; // The size of a square in your defined unit (point,
                    // millimeter,etc).
  int nrFrames; // The number of frames to use from the input for calibration
  float aspectRatio;           // The aspect ratio
  int delay;                   // In case of a video input
  bool bwritePoints;           //  Write detected feature points
  bool bwriteExtrinsics;       // Write extrinsic parameters
  bool calibZeroTangentDist;   // Assume zero tangential distortion
  bool calibFixPrincipalPoint; // Fix the principal point at the center
  bool flipVertical;     // Flip the captured images around the horizontal axis
  string outputFileName; // The name of the file where to write
  bool showUndistorsed;  // Show undistorted images after calibration
  string input;          // The input ->

  int cameraID;
  vector<string> imageList;
  int atImageList;
  VideoCapture inputCapture;
  InputType inputType;
  bool goodInput;
  int flag;

  string patternToUse;
};

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix,
                           Mat &distCoeffs,
                           vector<vector<Point2f>> imagePoints);

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  Settings s;
  s.boardSize.width = FLAGS_board_side_width;
  s.boardSize.height = FLAGS_board_side_height;
  CHECK_GT(FLAGS_square_size, 0);
  s.squareSize = FLAGS_square_size;
  s.nrFrames = FLAGS_frames_to_use;
  s.aspectRatio = FLAGS_fix_aspect_ratio;
  s.bwritePoints = false;
  s.bwriteExtrinsics = FLAGS_write_extrinsic_parameters;
  s.outputFileName = FLAGS_out_file;
  s.calibZeroTangentDist = FLAGS_assume_zero_tangential_distortion;
  s.calibFixPrincipalPoint = FLAGS_fix_principal_point_at_center;
  s.flipVertical = FLAGS_flip_horizontal_axis;
  s.showUndistorsed = FLAGS_show_undistorted_image;
  s.delay = FLAGS_input_delay;
  CHECK(!FLAGS_input.empty());
  s.input = FLAGS_input;
  s.patternToUse = FLAGS_pattern;

  s.interprate();

  if (!s.goodInput) {
    cout << "Invalid input detected. Application stopping. " << endl;
    return -1;
  }

  vector<vector<Point2f>> imagePoints;
  Mat cameraMatrix, distCoeffs;
  Size imageSize;
  int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
  clock_t prevTimestamp = 0;
  const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
  const char ESC_KEY = 27;

  int remainingSkip = 0;

  for (int i = 0;; ++i) {
    Mat view;
    bool blinkOutput = false;

    view = s.nextImage();

    if (mode != CALIBRATED && remainingSkip > 0) {
      --remainingSkip;
      continue;
    }

    //-----  If no more image, or got enough, then stop calibration and show
    // result -------------
    if (mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames) {
      if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs,
                                imagePoints)) {
        mode = CALIBRATED;
        return EXIT_SUCCESS;
      } else
        mode = DETECTION;
    }
    if (view.empty()) // If no more images then run calibration, save and stop
                      // loop.
    {
      if (imagePoints.size() > 0)
        runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs,
                              imagePoints);
      break;
    }

    imageSize = view.size(); // Format input image.
    if (s.flipVertical)
      flip(view, view, 0);

    vector<Point2f> pointBuf;

    bool found;
    switch (s.calibrationPattern) // Find feature points on the input format
    {
    case Settings::CHESSBOARD:
      found = findChessboardCorners(view, s.boardSize, pointBuf,
                                    CV_CALIB_CB_ADAPTIVE_THRESH |
                                        CV_CALIB_CB_FAST_CHECK |
                                        CV_CALIB_CB_NORMALIZE_IMAGE);
      break;
    case Settings::CIRCLES_GRID:
      found = findCirclesGrid(view, s.boardSize, pointBuf);
      break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
      found = findCirclesGrid(view, s.boardSize, pointBuf,
                              CALIB_CB_ASYMMETRIC_GRID);
      break;
    default:
      found = false;
      break;
    }

    if (found) // If done with success,
    {
      // improve the found corners' coordinate accuracy for chessboard
      if (s.calibrationPattern == Settings::CHESSBOARD) {
        Mat viewGray;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);
        cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      }

      if (mode ==
              CAPTURING && // For camera only take new samples after delay time
          (!s.inputCapture.isOpened() ||
           clock() - prevTimestamp > s.delay * 1e-3 * CLOCKS_PER_SEC)) {
        imagePoints.push_back(pointBuf);
        prevTimestamp = clock();
        blinkOutput = s.inputCapture.isOpened();
      }

      // Draw the corners.
      drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);

      remainingSkip = FLAGS_skip_frames;
    }

    //----------------------------- Output Text
    //------------------------------------------------
    string msg = (mode == CAPTURING) ? "100/100" : mode == CALIBRATED
                                                       ? "Calibrated"
                                                       : "Press 'g' to start";
    int baseLine = 0;
    Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
    Point textOrigin(view.cols - 2 * textSize.width - 10,
                     view.rows - 2 * baseLine - 10);

    if (mode == CAPTURING) {
      if (s.showUndistorsed)
        msg = format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
      else
        msg = format("%d/%d", (int)imagePoints.size(), s.nrFrames);
    }

    putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

    if (blinkOutput)
      bitwise_not(view, view);

    //------------------------- Video capture  output  undistorted
    //------------------------------
    if (mode == CALIBRATED && s.showUndistorsed) {
      Mat temp = view.clone();
      undistort(temp, view, cameraMatrix, distCoeffs);
    }

    //------------------------------ Show image and check for input commands
    //-------------------
    imshow("Image View", view);
    char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

    if (key == ESC_KEY)
      break;

    if (key == 'u' && mode == CALIBRATED)
      s.showUndistorsed = !s.showUndistorsed;

    if (s.inputCapture.isOpened() && key == 'g') {
      mode = CAPTURING;
      imagePoints.clear();
    }
  }

  // -----------------------Show the undistorted image for the image list
  // ------------------------
  if (s.inputType == Settings::IMAGE_LIST && s.showUndistorsed) {
    Mat view, rview, map1, map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs,
                                                      imageSize, 1, imageSize,
                                                      0),
                            imageSize, CV_16SC2, map1, map2);

    for (int i = 0; i < (int)s.imageList.size(); i++) {
      view = imread(s.imageList[i], 1);
      if (view.empty())
        continue;
      remap(view, rview, map1, map2, INTER_LINEAR);
      imshow("Image View", rview);
      char c = (char)waitKey();
      if (c == ESC_KEY || c == 'q' || c == 'Q')
        break;
    }
  }

  return 0;
}

static double
computeReprojectionErrors(const vector<vector<Point3f>> &objectPoints,
                          const vector<vector<Point2f>> &imagePoints,
                          const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                          const Mat &cameraMatrix, const Mat &distCoeffs,
                          vector<float> &perViewErrors) {
  vector<Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

static void calcBoardCornerPositions(
    Size boardSize, float squareSize, vector<Point3f> &corners,
    Settings::Pattern patternType /*= Settings::CHESSBOARD*/) {
  corners.clear();

  switch (patternType) {
  case Settings::CHESSBOARD:
  case Settings::CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; ++i)
      for (int j = 0; j < boardSize.width; ++j)
        corners.push_back(
            Point3f(float(j * squareSize), float(i * squareSize), 0));
    break;

  case Settings::ASYMMETRIC_CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; i++)
      for (int j = 0; j < boardSize.width; j++)
        corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
                                  float(i * squareSize), 0));
    break;
  default:
    break;
  }
}

static bool runCalibration(Settings &s, Size &imageSize, Mat &cameraMatrix,
                           Mat &distCoeffs, vector<vector<Point2f>> imagePoints,
                           vector<Mat> &rvecs, vector<Mat> &tvecs,
                           vector<float> &reprojErrs, double &totalAvgErr) {

  cameraMatrix = Mat::eye(3, 3, CV_64F);
  if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
    cameraMatrix.at<double>(0, 0) = 1.0;

  distCoeffs = Mat::zeros(8, 1, CV_64F);

  vector<vector<Point3f>> objectPoints(1);
  calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0],
                           s.calibrationPattern);

  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  // Find intrinsic and extrinsic camera parameters
  double rms = calibrateCamera(objectPoints, imagePoints, imageSize,
                               cameraMatrix, distCoeffs, rvecs, tvecs,
                               s.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

  cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  totalAvgErr =
      computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs,
                                cameraMatrix, distCoeffs, reprojErrs);

  return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(Settings &s, Size &imageSize, Mat &cameraMatrix,
                             Mat &distCoeffs) {
  FileStorage fs(s.outputFileName, FileStorage::WRITE);

  fs << "Camera_fx" << cameraMatrix.at<double>(0, 0);
  fs << "Camera_fy" << cameraMatrix.at<double>(1, 1);
  fs << "Camera_cx" << cameraMatrix.at<double>(0, 2);
  fs << "Camera_cy" << cameraMatrix.at<double>(1, 2);

  fs << "Camera_k1" << distCoeffs.at<double>(0, 0);
  fs << "Camera_k2" << distCoeffs.at<double>(1, 0);
  fs << "Camera_p1" << distCoeffs.at<double>(2, 0);
  fs << "Camera_p2" << distCoeffs.at<double>(3, 0);

  // Default camera and feature extractor settings.
  fs << "Camera_fps" << 30.0;
  fs << "Camera_RGB" << 1;

  fs << "ORBextractor_nFeatures" << 2000;
  // ORB Extractor: Scale factor between levels in the scale pyramid
  fs << "ORBextractor_scaleFactor" << 1.2;

  // ORB Extractor: Number of levels in the scale pyramid
  fs << "ORBextractor_nLevels" << 8;

  // ORB Extractor: Fast threshold
  // Image is divided in a grid. At each cell FAST are extracted imposing a
  // minimum response.
  // Firstly we impose iniThFAST. If no corners are detected we impose a lower
  // value minThFAST
  // You can lower these values if your images have low contrast
  fs << "ORBextractor_iniThFAST" << 20;
  fs << "ORBextractor_minThFAST" << 7;

  // Viewer Parameters
  fs << "Viewer_KeyFrameSize" << 0.05;
  fs << "Viewer_KeyFrameLineWidth" << 1;
  fs << "Viewer_GraphLineWidth" << 0.9;
  fs << "Viewer_PointSize" << 2;
  fs << "Viewer_CameraSize" << 0.08;
  fs << "Viewer_CameraLineWidth" << 3;
  fs << "Viewer_ViewpointX" << 0;
  fs << "Viewer_ViewpointY" << -0.7;
  fs << "Viewer_ViewpointZ" << -1.8;
  fs << "Viewer_ViewpointF" << 500;
}

bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix,
                           Mat &distCoeffs,
                           vector<vector<Point2f>> imagePoints) {
  vector<Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr = 0;

  const bool ok =
      runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs,
                     tvecs, reprojErrs, totalAvgErr);
  cout << (ok ? "Calibration succeeded" : "Calibration failed")
       << ". avg re projection error = " << totalAvgErr << "\n";

  if (ok) {
    saveCameraParams(s, imageSize, cameraMatrix, distCoeffs);
  }
  return ok;
}
