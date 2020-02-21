#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#include "sc.h"

inline int getMinimum3(int a, int b, int c)
{
	return ((a > b ? b : a) > c ? c : (a > b ? b : a));
}

Mat getGradientImage(Mat in_image)
{
        double scale = 1.0;
        double delta = 0.0;
        int ddepth = CV_16S;
        double alpha = 0.5;
        double beta = 0.5;
        double gamma = 0;
        int dtype = -1;
	Mat gaussianBlur_image,gray_image,gradient_x,gradient_y,abs_grad_x,abs_grad_y,weighted_gradient;
        GaussianBlur(in_image, gaussianBlur_image, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cvtColor(gaussianBlur_image,gray_image,COLOR_BGR2GRAY);
        Sobel(gray_image,gradient_x,ddepth,1,0,3,scale,delta,BORDER_DEFAULT);
        Sobel(gray_image,gradient_y,ddepth,0,1,3,scale,delta,BORDER_DEFAULT);
	convertScaleAbs(gradient_x, abs_grad_x);
	convertScaleAbs(gradient_y, abs_grad_y);
	addWeighted(abs_grad_x, alpha, abs_grad_y, beta, gamma, weighted_gradient,dtype);
        return weighted_gradient;
}

bool seam_carving(Mat& in_image, int new_width, int new_height, Mat& out_image) {
	// some sanity checks
	// Check 1 -> new_width <= in_image.cols
	if (new_width > in_image.cols) {
		cout << "Invalid request!!! new_width has to be smaller than the current size!" << endl;
		return false;
	}
	if (new_height > in_image.rows) {
		cout << "Invalid request!!! ne_height has to be smaller than the current size!" << endl;
		return false;
	}

	if (new_width <= 0) {
		cout << "Invalid request!!! new_width has to be positive!" << endl;
		return false;
	}

	if (new_height <= 0) {
		cout << "Invalid request!!! new_height has to be positive!" << endl;
		return false;
	}
	return seam_carving_trivial(in_image, new_width, new_height, out_image);
}

vector<int> computeHorizontalSeamPathDynamically(Mat wg)
{
	int i, j;
	int rows = wg.rows;
	int columns = wg.cols;
	vector<int> seamPath(columns);
	vector<vector<int > > energy(rows,vector<int>(columns));
	vector<vector<int > > path(rows, vector<int>(columns));
	for (i = 0; i < rows; i++)
	{
		energy[i][0] = (int)wg.at<uchar>(i,0);
		path[i][0] = i;
	}
	int backPixel,backBottomPixel, backTopPixel,minimumEnergyPixel;
	for (j = 1; j < columns; j++)
	{
		for (i = 0; i < rows; i++)
		{
			backPixel = energy[i][j-1];
			if (i == 0)
			{
				backTopPixel = INT_MAX;
				backBottomPixel = energy[i + 1][j - 1];				
			}
			else if (i == rows - 1)
			{
				backBottomPixel = INT_MAX;
				backTopPixel = energy[i-1][j-1];
			}
			else
			{
				backTopPixel = energy[i - 1][j - 1];
				backBottomPixel = energy[i + 1][j - 1];
			}
			minimumEnergyPixel = getMinimum3(backTopPixel, backPixel, backBottomPixel);
			energy[i][j] = wg.at<uchar>(i, j) + minimumEnergyPixel;
			if (minimumEnergyPixel == backTopPixel)
			{
				path[i][j] = i - 1;
			}
			else if (minimumEnergyPixel == backPixel)
			{
				path[i][j] = i;
			}
			else
			{
				path[i][j] = i + 1;
			}
		}
	}
	minimumEnergyPixel = energy[0][columns-1];
	int index = 0;
	for(i=1;i<rows;i++)
		if (minimumEnergyPixel > energy[i][columns-1])
		{
			minimumEnergyPixel = energy[i][columns-1];
			index = i;
		}
	seamPath[columns-1] = index;
	for (j=columns-1; j>0; j--)
	{
		seamPath[j-1] = path[index][j];
		index = path[index][j];
	}
	return seamPath;
}

vector<int> computeVerticalSeamPathDynamically(Mat wg)
{
	int i, j;
	int rows = wg.rows;
	int columns = wg.cols;
        vector<int> seamPath(rows);
	vector<vector<int > > energy(rows, vector<int>(columns));
	vector<vector<int > > path(rows, vector<int>(columns));
        for (j = 0; j < columns; j++)
	{
		energy[0][j] = (int)wg.at<uchar>(0, j);
		path[0][j] = j;
	}
	int topPixel, topLeftPixel, topRightPixel, minimumEnergyPixel;
	for (i = 1; i < rows; i++)
	{
		for (j = 0; j < columns; j++)
		{
			topPixel = energy[i-1][j];
			if (j == 0)
			{
				topLeftPixel = INT_MAX;
				topRightPixel = energy[i-1][j+1];
			}
			else if (j == columns - 1)
			{
				topRightPixel = INT_MAX;
				topLeftPixel = energy[i-1][j-1];
			}
			else
			{
				topLeftPixel = energy[i - 1][j - 1];
				topRightPixel = energy[i - 1][j + 1];
			}
			minimumEnergyPixel = getMinimum3(topLeftPixel,topPixel, topRightPixel);
			energy[i][j] = (int)wg.at<uchar>(i, j) + minimumEnergyPixel;
			if (minimumEnergyPixel == topLeftPixel)
			{
				path[i][j] = j - 1;
			}
			else if (minimumEnergyPixel == topPixel)
			{
				path[i][j] = j;
			}
			else
			{
				path[i][j] = j + 1;
			}
		}
	}
	minimumEnergyPixel = energy[rows-1][0];
	int index = 0;
	for (j = 1; j < columns; j++)
		if (minimumEnergyPixel > energy[rows-1][j])
		{
			minimumEnergyPixel = energy[rows-1][j];
			index = j;
		}
	seamPath[rows-1] = index;
        for (i = rows-1; i > 0; i--)
	{
	     seamPath[i-1] = path[i][index];	
             index = path[i][index];
	}
        return seamPath;
}

// seam carves by removing trivial seams
bool seam_carving_trivial(Mat& in_image, int new_width, int new_height, Mat& out_image) {

	Mat iimage = in_image.clone();
	Mat oimage = in_image.clone();
	while (iimage.rows != new_height || iimage.cols != new_width) {
        //      horizontal seam if needed
		if (iimage.rows > new_height) {
			reduce_horizontal_seam_trivial(iimage, oimage);
			iimage = oimage.clone();
		}

		if (iimage.cols > new_width) {
			reduce_vertical_seam_trivial(iimage, oimage);
			iimage = oimage.clone();
		}
	}
	out_image = oimage.clone();
	return true;
}

// horizontl trivial seam is a seam through the center of the image
bool reduce_horizontal_seam_trivial(Mat& in_image, Mat& out_image) {
	// retrieve the dimensions of the new image
	int rows = in_image.rows - 1;
	int cols = in_image.cols;
	// create an image slighly smaller
	out_image = Mat(rows, cols, CV_8UC3);

	//get the energy of the input image
	Mat weighted_gradient = getGradientImage(in_image);
	vector<int> seamPath = computeHorizontalSeamPathDynamically(weighted_gradient);
	//populate the image
	for (int j=0;j<cols; j++)
	{
		for (int i = 0; i < seamPath[j]; i++)
			out_image.at<Vec3b>(i, j) = in_image.at<Vec3b>(i, j);
		for (int i = seamPath[j]; i < rows; i++)
			out_image.at<Vec3b>(i, j) = in_image.at<Vec3b>(i + 1, j);
	}
	return true;
}

// vertical trivial seam is a seam through the center of the image
bool reduce_vertical_seam_trivial(Mat& in_image, Mat& out_image) {
	// retrieve the dimensions of the new image
	int rows = in_image.rows;
	int cols = in_image.cols - 1;

	// create an image slighly smaller
	out_image = Mat(rows, cols, CV_8UC3);

	//get the energy of input image
	Mat weighted_gradient = getGradientImage(in_image);
	vector<int> seamPath = computeVerticalSeamPathDynamically(weighted_gradient);
	//populate the image
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < seamPath[i]; j++)
			out_image.at<Vec3b>(i, j) = in_image.at<Vec3b>(i, j);
		for (int j = seamPath[i]; j < cols; j++)
			out_image.at<Vec3b>(i, j) = in_image.at<Vec3b>(i, j+1);
	}
	return true;
}
