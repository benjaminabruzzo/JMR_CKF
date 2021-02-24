#ifndef HAST_FILEOPS_H
#define HAST_FILEOPS_H

#include "genheaders.hpp"
class fileops
{
	private:

	public:
		std::FILE * filename;
		std::FILE * pre_filename;
		std::string s_filename, s_pre_filename, s_doubleformat;
		int printlength;
		std::string datastring;
		bool init_active;


		fileops();

		void init_fileops(std::string mfile);
		void init_pre_fileops(std::string path, std::string mfile, double wb_max);
		void writeString(std::string data);
		void writeDouble(double data, std::string vectorName, int count);
		void writeVecInt(std::vector<int> vectorOfInts, std::string vectorName, int count);
		void writeVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count);
		void writeVecDoubles_multiline(std::vector<double> vectorOfDoubles, std::string vectorName, int count);
		void cellVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count);
		void cellVecInts(std::vector<int> vectorOfInts, std::string vectorName, int count);
		void cellVecUints(std::vector<uint> vectorOfUints, std::string vectorName, int count);
		void writeCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count);
		void writeCvMatDoubles_multline(cv::Mat matrixOfDoubles, std::string matrixName, int count);
		void cellCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count);

		void cellCvMatDoubles_multline_ndec(cv::Mat matrixOfDoubles, std::string matrixName, int count);
		void cellCvMatDoubles_multline_4dec(cv::Mat matrixOfDoubles, std::string matrixName, int count);
		void cellCvMatDoubles_multline(cv::Mat matrixOfDoubles, std::string matrixName, int count);
		void cellCvMatDbl2int_multline(cv::Mat matrixOfDoubles, std::string matrixName, int count);

		void cellVecAugmentedState_nx4(std::vector<double> vectorOfDoubles, std::string vectorName, int count);
		void cellMatAugmentedState_nx4(cv::Mat matrixOfDoubles, std::string vectorName, int count);
};

#endif
