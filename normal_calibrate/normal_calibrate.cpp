// fisheye_calibrate.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>

#include <opencv2\opencv.hpp>
#include <iostream>
#include <fstream> 
#include <io.h>  
#include <direct.h>
#include <string>  
#include<vector>

using namespace std;
using namespace cv;

void getFilesName(string &File_Directory, string &FileType, vector<string>&FilesName)
{
	string buffer = File_Directory + "\\*" + FileType;

	__finddata64_t c_file;

	__int64 hFile;
	hFile = _findfirst64(buffer.c_str(), &c_file);

	if (hFile == -1L)
		printf("No %s files in current directory!\n", FileType);
	else
	{
		string fullFilePath;
		do
		{
			fullFilePath.clear();

			//file name
			fullFilePath = c_file.name;// File_Directory + "\\" + c_file.name;

			FilesName.push_back(fullFilePath);

		} while (_findnext64(hFile, &c_file) == 0);
		_findclose(hFile);
	}
}

int main()
{
	char buffer[80];
	_getcwd(buffer, 80);

	string File_Directory = buffer;

	File_Directory = File_Directory + "\\images";

	string File_Directory_Result = File_Directory + "\\undisortimage";
	string File_Directory_ResultCorner = File_Directory + "\\cornerimage";
	_mkdir(File_Directory_Result.c_str());
	_mkdir(File_Directory_ResultCorner.c_str());

	string FileType = ".bmp";

	vector<string>FilesName;

	getFilesName(File_Directory, FileType, FilesName);

	ofstream fout("images\\caliberation_result.txt");  /**    保存定标结果的文件     **/

	/************************************************************************
		   读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
	*************************************************************************/
	cout << "开始提取角点………………" << endl;
	//以下三行为需要手动修改的参数
	int image_count = (int)FilesName.size();                   //图像数量
	Size board_size = Size(6, 8);            //定标板上每行、列的角点数
	int x_expand = 0, y_expand = 0;// 200;		//x,y方向的扩展(x横向，y纵向)，适当增大可以不损失原图像信息

	vector<Point2f> corners;                //缓存每幅图像上检测到的角点
	vector<vector<Point2f>>  corners_Seq;   //保存检测到的所有角点/   
	vector<Mat>  image_Seq;
	int successImageNum = 0;				//成功提取角点的棋盘图数量	
	bool conner_flag = true;				//所有图像角点提取成功为true，其余为false
	for (size_t i = 0; i < FilesName.size(); i++)
	{
		string imageFile = File_Directory + "\\" + FilesName[i];
		cout << imageFile << endl;
		cv::Mat imageSrc = imread(imageFile);
		Mat image;//边界扩展后的图片
		copyMakeBorder(imageSrc, image, (int)(y_expand / 2), (int)(y_expand / 2), (int)(x_expand / 2), (int)(x_expand / 2), BORDER_CONSTANT);
		/* 提取角点 */
		Mat imageGray;
		cvtColor(image, imageGray, CV_RGB2GRAY);
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
		if (!patternfound)
		{
			cout << FilesName[i] << "角点提取失败，请删除该图片，重新排序后再次运行程序！" << endl;
			conner_flag = false;
			//break;
		}
		else
		{
			/* 亚像素精确化 */
			cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			/* 绘制检测到的角点并保存 */
			Mat imageTemp = image.clone();
			for (int j = 0; j < corners.size(); j++)
			{
				circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
			}
			/*
			string imageFileName;
			std::stringstream StrStm;
			StrStm<<i+1;
			StrStm>>imageFileName;
			imageFileName += "_corner.jpg";
			imwrite(imageFileName,imageTemp);
			*/
			string imageFileName = File_Directory_ResultCorner + "\\" + FilesName[i];
			imwrite(imageFileName, imageTemp);

			cout << FilesName[i] << " 角点提取完毕" << endl;

			successImageNum = successImageNum + 1;
			corners_Seq.push_back(corners);
		}
		image_Seq.push_back(image);
	}
	if (!conner_flag)//如果有提取失败的标定图，退出程序
	{
		system("pause");
		return 0;
	}
	cout << "角点提取完成！" << endl;
	/************************************************************************
		   摄像机定标
	*************************************************************************/
	cout << "开始定标………………" << endl;
	Size square_size = Size(28, 28);
	vector<vector<Point3f>>  object_Points;        /****  保存定标板上角点的三维坐标   ****/

	vector<int>  point_counts;
	/* 初始化定标板上角点的三维坐标 */
	for (int t = 0; t < successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < board_size.height; i++)
		{
			for (int j = 0; j < board_size.width; j++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
				Point3f tempPoint;
				tempPoint.x = i * square_size.width;
				tempPoint.y = j * square_size.height;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	for (int i = 0; i < successImageNum; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* 开始定标 */
	Size image_size = image_Seq[0].size();
	Mat intrinsic_matrix;
	Mat distortion_coeffs;
	//Vec4f distCoeffs;
	vector<Mat> rotation_vectors;
	vector<Mat> translation_vectors;

	int flags = 0;
	flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flags |= cv::fisheye::CALIB_CHECK_COND;
	flags |= cv::fisheye::CALIB_FIX_SKEW;
	//fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
	calibrateCamera(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors);
	cout << "定标完成！\n";

	/************************************************************************
		   对定标结果进行评价
	*************************************************************************/
	cout << "开始评价定标结果………………" << endl;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/

	cout << "每幅图像的定标误差：" << endl;
	cout << "每幅图像的定标误差：" << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, (int)tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, (int)image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << FilesName[i] << " 平均误差：" << err << "像素" << endl;
		fout << FilesName[i] << " 平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	cout << "评价完成！" << endl;

	/************************************************************************
		   保存定标结果
	*************************************************************************/
	cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */

	fout << "相机内参数矩阵：" << endl;
	fout << intrinsic_matrix << endl;
	fout << "畸变系数：\n";
	fout << distortion_coeffs << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << FilesName[i] << " 旋转向量：" << endl;
		fout << rotation_vectors[i] << endl;

		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rotation_vectors[i], rotation_matrix);
		fout << FilesName[i] << " 旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << FilesName[i] << " 平移向量：" << endl;
		fout << translation_vectors[i] << endl;
	}
	cout << "完成保存" << endl;
	fout << endl;


	/************************************************************************
		   显示定标结果
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	cout << "保存矫正图像" << endl;
	for (int i = 0; i != image_count; i++)
	{
		initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
		Mat t = image_Seq[i].clone();
		cv::remap(image_Seq[i], t, mapx, mapy, INTER_LINEAR);
		string imageFileName = File_Directory_Result + "\\" + FilesName[i];
		imwrite(imageFileName, t);
		//imshow(imageFileName, t);
		cout << FilesName[i] << "保存" << endl;
	}
	cout << "保存结束" << endl;

	while (1)
	{
		if (cvWaitKey(15) == 27)
		{
			break;
		}
	}

	return 0;
}