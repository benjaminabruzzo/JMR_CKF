#include "fileops.hpp"

fileops::fileops(){};

void fileops::init_fileops(std::string mfile)
{
	s_filename = mfile;
	ROS_INFO("fileops::filename: %s", s_filename.c_str());
	filename = std::fopen (s_filename.c_str(), "w");
	fprintf (filename, "%% %s\n", s_filename.c_str());
	fprintf (filename, "  %%clc; \n   %%clear all;\n   %%close all;\n\n");
	s_doubleformat = " % -16.14f";
	init_active = true;
}


void fileops::init_pre_fileops(std::string path, std::string mfile, double wb_max)
{
	s_filename = path + mfile;
	s_pre_filename = path + "prealloc/pre_" + mfile;
	pre_filename = std::fopen (s_pre_filename.c_str(), "w");

	ros::Duration(0.2).sleep();
	ROS_INFO("fileops::init_pre_fileops: %s", s_pre_filename.c_str());
	fprintf (pre_filename, "\n%% --------------- fileops::init_pre_fileops() --------------- \n" );
	fprintf (pre_filename, "  wb = waitbar(0,' Loading %s ...');\n", s_filename.c_str());
	fprintf (pre_filename, "  waitbar_max = %f;\n", wb_max);
	ros::Duration(0.1).sleep();

}


void fileops::writeString(std::string data)
{fprintf (filename, "%s", data.c_str());}

void fileops::writeDouble(double data, std::string vectorName, int count)
{fprintf (filename, "%s(%u,:) = % -6.14f;\n", vectorName.c_str(), count, data);}

void fileops::writeVecInt(std::vector<int> vectorOfInts, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfInts.size();

	fprintf (filename, "%s(%i,:) = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++)
	{
		fprintf (filename, "%i ", vectorOfInts[i]);
	}
	fprintf (filename, "      ];\n");
}

void fileops::writeVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfDoubles.size();

	fprintf (filename, "%s(%i,:) = [", vectorName.c_str(), count);
	// fprintf (filename, "%i ", vectorSize);
	// ROS_INFO("augmentSLAM:: %s .size() = %i ", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++)
	{
		fprintf (filename," %6.8f", vectorOfDoubles[i]);
	}
	fprintf (filename, "      ];\n");
}

void fileops::writeVecDoubles_multiline(std::vector<double> vectorOfDoubles, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfDoubles.size();

	fprintf (filename, "%s(%i,:) = [...\n", vectorName.c_str(), count);
	// fprintf (filename, "%i ", vectorSize);
	// ROS_INFO("augmentSLAM:: %s .size() = %i ", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++)
	{
		fprintf (filename," %6.8f ...\n", vectorOfDoubles[i]);
	}
	fprintf (filename, "      ];\n");
}

void fileops::writeCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;

	for(uint rown = 0; rown != rows; rown++)
	{
		fprintf (filename, "%s(%i,:, %i) = [", matrixName.c_str(), rown+1, count);
		for(uint coln = 0; coln != cols; coln++)
		{
			fprintf (filename, s_doubleformat.c_str(), matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "      ];\n");
	}
}

void fileops::cellVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfDoubles.size();

	fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++)
	{
		fprintf (filename," %6.8f", vectorOfDoubles[i]);
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellVecInts(std::vector<int> vectorOfInts, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfInts.size();

	fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++)
	{
		fprintf (filename," %i", vectorOfInts[i]);
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellVecUints(std::vector<uint> vectorOfUints, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfUints.size();

	fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++)
	{
		fprintf (filename," %u", vectorOfUints[i]);
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;


	fprintf (filename, "%s{%i,1} = [", matrixName.c_str(), count);
	for(uint rown = 0; rown != rows; rown++)
	{
		for(uint coln = 0; coln != cols; coln++)
		{
			fprintf (filename, " %-12.10f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "; ");
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellCvMatDoubles_multline_ndec(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;

	fprintf (filename, "%s{%i,1} = [...\n  ", matrixName.c_str(), count);
	for(uint rown = 0; rown != rows; rown++)
	{
		fprintf (filename, "    ");
		for(uint coln = 0; coln != cols; coln++)
		{
			fprintf (filename, "  %-18.10f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "; ...\n  ");
	}
	fprintf (filename, "  ];\n");
}

void fileops::cellCvMatDoubles_multline_4dec(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;

	fprintf (filename, "%s{%i,1} = [...\n  ", matrixName.c_str(), count);
	for(uint rown = 0; rown != rows; rown++)
	{
		fprintf (filename, "  ");
		for(uint coln = 0; coln != cols; coln++)
		{
			fprintf (filename, "  % -9.4f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "; ...\n  ");
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellCvMatDoubles_multline(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;

	fprintf (filename, "%s{%i,1} = [...\n  ", matrixName.c_str(), count);
	for(uint rown = 0; rown != rows; rown++)
	{
		fprintf (filename, "  ");
		for(uint coln = 0; coln != cols; coln++)
		{
			fprintf (filename, "  % -12.8f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "; ...\n  ");
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellCvMatDbl2int_multline(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;

	fprintf (filename, "%s{%i,1} = [...\n", matrixName.c_str(), count);
	for(uint rown = 0; rown != rows; rown++)
	{
		for(uint coln = 0; coln != cols; coln++)
		{
			fprintf (filename, " %1.0f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "; ...\n");
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellVecAugmentedState_nx4(std::vector<double> vectorOfDoubles, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfDoubles.size();

	fprintf (filename, "%s{1,%i} = [...\n", vectorName.c_str(), count);
	for(uint i = 0; i != (vectorSize/4); i++)
	{
		fprintf (filename, "     % -20.16f; % -20.16f; % -20.16f; % -20.16f; ...\n",
			vectorOfDoubles[4*i+0],
			vectorOfDoubles[4*i+1],
			vectorOfDoubles[4*i+2],
			vectorOfDoubles[4*i+3]);
	}
	fprintf (filename, "      ];\n");
}

void fileops::cellMatAugmentedState_nx4(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;


	fprintf (filename, "%s{%i,1} = [...\n", matrixName.c_str(), count);
	for(uint rown = 0; rown != (rows/4); rown++)
	{
			fprintf (filename, "     % -20.16f; % -20.16f; % -20.16f; % -20.16f; ...\n",
				matrixOfDoubles.at<double>(4*rown+0,0),
				matrixOfDoubles.at<double>(4*rown+1,0),
				matrixOfDoubles.at<double>(4*rown+2,0),
				matrixOfDoubles.at<double>(4*rown+3,0));
	}
	fprintf (filename, "      ];\n");
}
