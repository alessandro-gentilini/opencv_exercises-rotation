// Author: Alessandro Gentilini, 2013

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <algorithm>
#include <random>

double distance(const cv::Point &a, const cv::Point &b)
{
    cv::Point d(a - b);
    return sqrt(d.x * d.x + d.y * d.y);
}

int main( int argc, char *argv[] )
{
    cv::Mat model_img = cv::imread( argv[1] );
    if ( !model_img.data )
    {
        std::cerr << "Error loading model image: " << argv[1] << "\n";
        return 1;
    }
    cv::imshow("original", model_img);



    std::uniform_int_distribution<int> unif_x(0, model_img.cols);
    std::uniform_int_distribution<int> unif_y(0, model_img.rows);
    std::uniform_int_distribution<int> unif_alpha(0, 359);
    std::random_device rd;
    std::default_random_engine re(rd());

    cv::Point centroid(unif_x(re), unif_y(re));

    int alpha = unif_alpha(re);
    std::cout << "\nCentroid\t" << centroid << "\nangle\t" << alpha << "\n";

    cv::Mat rot( 2, 3, cv::DataType<double>::type );
    rot = cv::getRotationMatrix2D( centroid, alpha, 1 );

    cv::Mat homogeneous_midpoint(3, 1, cv::DataType<double>::type);
    homogeneous_midpoint.at<double>(0, 0) = model_img.cols / 2;
    homogeneous_midpoint.at<double>(1, 0) = model_img.rows / 2;
    homogeneous_midpoint.at<double>(2, 0) = 1;

    cv::Mat top_left(3, 1, cv::DataType<double>::type);
    top_left.at<double>(0, 0) = 0;
    top_left.at<double>(1, 0) = 0;
    top_left.at<double>(2, 0) = 1;

    cv::Mat top_right(3, 1, cv::DataType<double>::type);
    top_right.at<double>(0, 0) = model_img.cols;
    top_right.at<double>(1, 0) = 0;
    top_right.at<double>(2, 0) = 1;

    cv::Mat bottom_right(3, 1, cv::DataType<double>::type);
    bottom_right.at<double>(0, 0) = model_img.cols;
    bottom_right.at<double>(1, 0) = model_img.rows;
    bottom_right.at<double>(2, 0) = 1;

    cv::Mat bottom_left(3, 1, cv::DataType<double>::type);
    bottom_left.at<double>(0, 0) = 0;
    bottom_left.at<double>(1, 0) = model_img.rows;
    bottom_left.at<double>(2, 0) = 1;

    cv::Mat top_left_rotated = rot * top_left;
    cv::Mat top_right_rotated = rot * top_right;
    cv::Mat bottom_right_rotated = rot * bottom_right;
    cv::Mat bottom_left_rotated = rot * bottom_left;
    cv::Mat rotated_midpoint = rot * homogeneous_midpoint;

    std::vector< cv::Point > rotated_corners(4);
    rotated_corners[0] = cv::Point(top_left_rotated.at<double>(0, 0), top_left_rotated.at<double>(1, 0));
    rotated_corners[1] = cv::Point(top_right_rotated.at<double>(0, 0), top_right_rotated.at<double>(1, 0));
    rotated_corners[2] = cv::Point(bottom_right_rotated.at<double>(0, 0), bottom_right_rotated.at<double>(1, 0));
    rotated_corners[3] = cv::Point(bottom_left_rotated.at<double>(0, 0), bottom_left_rotated.at<double>(1, 0));

    cv::Rect bb( cv::boundingRect( rotated_corners ) );

    // mi piacerebbe usare la cv::convertPointsToHomogeneous
    // non c'è un modo semplice per usare cv::Mat come se fosse un opportuno cv::Point?
    // mi sa che c'è un errore nei punti disegnati sui corner, forse per via della conversione implicita da double a int





    cv::Size sz(bb.width, bb.height);

    cv::Point displacement(bb.width / 2 - rotated_midpoint.at<double>(0, 0), bb.height / 2 - rotated_midpoint.at<double>(1, 0));

    rot.at<double>(0, 2) += displacement.x;
    rot.at<double>(1, 2) += displacement.y;



    cv::Mat rotated;
    cv::warpAffine(model_img, rotated, rot, sz, cv::INTER_LINEAR, cv::BORDER_CONSTANT, CV_RGB(0, 0, 0));

    std::transform(rotated_corners.begin(), rotated_corners.end(), rotated_corners.begin(), [&displacement](const cv::Point & p)
    {
        return p + displacement;
    });

    for ( size_t i = 0; i < 4; i++ )
    {
        cv::circle(rotated, rotated_corners[i], 2, CV_RGB(255, 0, 0));
    }

    for ( int r = 0; r < rotated.rows; r += 5 )
    {
        for ( int c = 0; c < rotated.cols; c += 5 )
        {
            cv::Point query(c, r);
            if ( cv::pointPolygonTest(rotated_corners, query, false) == -1 )
            {
                cv::circle(rotated, query, 2, CV_RGB(0, 0, 255));
            }
        }
    }

    cv::imshow("rotated", rotated);
    cv::imwrite("rotated.bmp", rotated);

    cv::waitKey();
    return 0;
}