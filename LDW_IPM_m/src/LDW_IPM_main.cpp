#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include "DetectLane.h"

//추후 걷어낼 header file
#include <sys/types.h>
#include <dirent.h>

#include <stdio.h>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>
#include <stdlib.h>

#include "cJSON.h"

#include "DetectCross.hpp"
#include "CFW.h"


using namespace cv;


//	영상정보 수신위한 file list get function
char** getFileList(char *dirPath, int n);

/* 결과 확인용 display function만을 함수로 .. */
void displayWorldPoint(Mat TOPView, wpoint* left_world, wpoint* right_world,int  left_marker_num,int right_marker_num ,float X_MIN,float Z_MIN, int X_NUM, int Z_NUM, float GRID);
void displayRegionVector(Mat RoadImgMat, int y_min, int y_max, unsigned short* L_area, unsigned short* R_area, lanevector left_vec, lanevector right_vec);
void displayimgPoint(Mat View, ipoint* left_world, ipoint* right_world,int  left_marker_num, int right_marker_num);
void displayTOPView(Mat TOPView, int left_valid, int right_valid, float *left_coeff, float* right_coeff, float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID);
void display1stTOPView(Mat TOPView, int left_valid, int right_valid, float *left_coeff, float* right_coeff, float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID);
void displayGPP(Mat TOPView, float* gpp,float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID);
void displayGPP2(Mat TOPView, float* gpp, float* tpl, float*tpr, float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID);
using namespace AUTONOMOUS::COMMLIB;

struct Test_GPP : public Header
{
	float_t c_x_min = -10.0;
	float_t c_z_min = 0.0;
	float_t c_grid = 0.1;
	uint32_t c_x_num = 200;
	uint32_t c_z_num = 300;
	int32_t c_left_valid = -1;
	int32_t c_right_valid = -1;
	float_t c_left_coeff[3] = {0,};
	float_t c_right_coeff[3] = {0,};

    float_t GPP[2] = {0,};
    uint8_t confidence[2] = {0,};

	float_t _tracking_plf[2] = {0,};
	float_t _tracking_prf[2] = {0,};

    Test_GPP()
    {
        message_ID = 15;
    }     
            
	void getFrameData(std::vector<uint8_t>& buff)
    {
		serializeData(buff, c_left_valid );
		serializeData(buff, c_right_valid);

 		 std::for_each(confidence, confidence+2, [&](uint8_t d)
        {
            serializeData(buff, d);
         });

		 std::for_each(c_left_coeff, c_left_coeff+3, [&](float_t d)
        {
            serializeData(buff, d);
         });

		 std::for_each(c_right_coeff, c_right_coeff+3, [&](float_t d)
        {
            serializeData(buff, d);
         });

		 std::for_each(GPP, GPP+2, [&](float_t d)
        {
            serializeData(buff, d);
         });

		 std::for_each(_tracking_plf, _tracking_plf+2, [&](float_t d)
        {
            serializeData(buff, d);
         });

		 std::for_each(_tracking_prf, _tracking_prf+2, [&](float_t d)
        {
            serializeData(buff, d);
         });		 
    }

    void setFrameData(uint8_t* buff)
    {
		 buff = deserializeData(buff, c_left_valid );
		 buff = deserializeData(buff, c_right_valid);

		std::for_each(confidence, confidence+2, [&](uint8_t& d)
        {
            buff = deserializeData(buff, d);
        });

		std::for_each(c_left_coeff, c_left_coeff+3, [&](float_t& d)
        {
            buff = deserializeData(buff, d);
        });

		std::for_each(c_right_coeff, c_right_coeff+3, [&](float_t& d)
        {
            buff = deserializeData(buff, d);
        });

		std::for_each(GPP, GPP+2, [&](float_t& d)
        {
            buff = deserializeData(buff, d);
        });

		std::for_each(_tracking_plf, _tracking_plf+2, [&](float_t& d)
        {
            buff = deserializeData(buff, d);
        });

		std::for_each(_tracking_prf, _tracking_prf+2, [&](float_t& d)
        {
            buff = deserializeData(buff, d);
        });
    }
};

int main(int argc, char** argv)
{
	//통신 위한 protocol
	Test_GPP m_gpp;
	cout << "id(argc) : "<< argc << endl;
	int id = argc;
	CFW& obj = CFW::GetInstance();
	std::cout << "CFW::GetInstance()" << std::endl;
	obj.Initialize();
	cout << "Initialize() OK : "<< endl;
	obj.StartCFW();
	cout << "StartCFW() OK : "<< endl;

	CDetectLane DL;
	
	// 파라미터 초기화 
	DL.InitializeParams();
   cout << "InitializaParams"<< endl;
	// 나중에 걷어내야 하는 부분은 main function에 	포함함 
	
	//for display
	Mat RoadImgMat, GrayMat;
	Mat TOPView;
 //  char  DIR[100]= "../../DB2_No1"; //20190829
 	char DIR[100]= "/home/hj/Documents/project/DB2_No1"; //debugging위한 절대경로

	char **dirs;
	dirs = getFileList(DIR, DL.imgnum);
	Mat img, new_img;
	unsigned char *RoadImg;
	RoadImg = (unsigned char*)malloc(DL.img_width*DL.img_height *3*sizeof(unsigned char));
	RoadImgMat = Mat(DL.img_height, DL.img_width, CV_8UC3, RoadImg);
	GrayMat = Mat(DL.img_height, DL.img_width, CV_8UC1, DL.GImage);
	TOPView = Mat::zeros(DL.Z_NUM,DL.X_NUM,CV_8UC3);
// ------------------------------

// 	detectcross::DetectCross p{"file_name.json"};
//    p.readJson();

// --------------------------------------------------------------------------------------
// Draw Map

	// Mat result_img = Mat::zeros(1000, 1000, CV_8UC3);

	// for (int i = 0; i < result_img.rows; i++)
	// {
	// 	for (int j = 0; j < result_img.cols; j++)
	// 	{
	// 			result_img.at<Vec3b>(i, j)[2] = 100; // R
	// 			result_img.at<Vec3b>(i, j)[1] = 100; // G
	// 			result_img.at<Vec3b>(i, j)[0] = 100; // B
	// 	}
	// }

// Draw Node
//   for (int i = 0; i < p.node_size; i++) {
//     circle(result_img, Point(p.lon[i]*1000, p.lat[i]*1000), 5, Scalar(255, 255, 255), 2, 8);
//   }

// Draw Link

//   double lon1, lat1, lon2, lat2;
//   for (int j = 0; j < p.link_size; j++) {
//     for (int i = 0; i < p.node_size; i++) {
//       if(p.nid[i] == p.src[j])
//       {
//         lon1 = p.lon[i];
//         lat1 = p.lat[i];
//         break;
//       }
//     }
//     for (int i = 0; i < p.node_size; i++) {
//       if(p.nid[i] == p.tag[j])
//       {
//         lon2 = p.lon[i];
//         lat2 = p.lat[i];
//         break;
//       }
//     }
//     line(result_img, Point(lon1*1000, lat1*1000), Point(lon2*1000, lat2*1000), Scalar(50, 0, 220), 2, 8);
//   }
  // -------------------------------------------------------

	// for(int img_n=0; img_n<DL.imgnum; img_n+=1){
	for(int img_n=6900; img_n<DL.imgnum; img_n++){

		// 영상 정보 수신 :걷어낼 부분 
		img = imread(dirs[img_n]); //원본 영상 그대로 loading한다.   
		
		for(int i=0; i<DL.img_height*DL.img_width; i++)
		{
			RoadImg[i*3] = img.data[i*3];
			RoadImg[i*3+1] = img.data[i*3+1];
			RoadImg[i*3+2] = img.data[i*3+2];
		}
        printf("%s frames\n", dirs[img_n]);
	
		DL.Process(RoadImg);
		
		//client에게 보내는 정보 :confidence level 값과 tracking point, gpp point를 보내어 신뢰도를 보장한다.
		m_gpp.c_left_valid = DL.left_valid;
    	m_gpp.c_right_valid = DL.right_valid;

		for(int i=0; i<2; i++)
		{
			m_gpp.c_left_coeff[i] = DL.left_coeff[i];
			m_gpp.c_right_coeff[i] = DL.right_coeff[i];
		}	

		m_gpp._tracking_plf[0]= DL.m_Track._tracking_points_l[0];
		m_gpp._tracking_plf[1]= DL.Z_MIN;
		m_gpp._tracking_prf[0]= DL.m_Track._tracking_points_r[0];
		m_gpp._tracking_prf[1]= DL.Z_MIN;

	// std::cout << "m_gpp.left_coeff : " << m_gpp.c_left_coeff[0]<<", "<<m_gpp.c_left_coeff[1]<<", "<<m_gpp.c_left_coeff[2]<< std::endl;
	// std::cout << "m_gpp.right_coeff : " << m_gpp.c_right_coeff[0]<<", "<<m_gpp.c_right_coeff[1]<<", "<<m_gpp.c_right_coeff[2]<< std::endl;

	std::cout << "m_gpp.left_coeff : " << m_gpp.c_left_coeff[0]<<", "<<m_gpp.c_left_coeff[1]<< std::endl;
	std::cout << "m_gpp.right_coeff : " << m_gpp.c_right_coeff[0]<<", "<<m_gpp.c_right_coeff[1]<< std::endl;


		displayimgPoint(RoadImgMat, DL.left_lane_marker, DL.right_lane_marker, DL.left_imarker_num, DL.right_imarker_num);
		displayRegionVector(RoadImgMat, DL.yy_min, DL.yy_max, DL.im_area_L, DL.im_area_R,DL.left_init_vec, DL.right_init_vec);

		//displayWorldPoint(TOPView, DL.left_world, DL.right_world, DL.left_marker_num, DL.right_marker_num ,DL.X_MIN, DL.Z_MIN, DL.X_NUM, DL.Z_NUM, DL.GRID);
		TOPView = Scalar(0);
		display1stTOPView(TOPView, DL.left_valid, DL.right_valid,DL.left_coeff, DL.right_coeff, DL.X_MIN, DL.Z_MIN, DL.X_NUM, DL.Z_NUM, DL.GRID);
		
		//displayGPP(TOPView, DL.m_Track.GPP, DL.X_MIN, DL.Z_MIN, DL.X_NUM, DL.Z_NUM, DL.GRID);
		displayGPP2(TOPView, DL.m_Track.GPP, DL.m_Track._tracking_points_l, DL.m_Track._tracking_points_r, DL.X_MIN, DL.Z_MIN, DL.X_NUM, DL.Z_NUM, DL.GRID);

		imshow("Input ", RoadImgMat); //Original image
		imshow("Filtered Image", GrayMat);		
		imshow("TOP VIEW", TOPView);
		
   	//    namedWindow("Result Map", WINDOW_NORMAL); // Create a window for display.
 	//    resizeWindow("Result Map", 1000, 1000);
	//    imshow("Result Map", result_img);
	if(img_n%2000 == 0)
	{
		waitKey(0);
	}
	else
	{
		waitKey(1);
	}

    
	obj.Send(m_gpp);
	std::cout <<"obj.Send(m_gpp);" <<std::endl;		
	}
	free(dirs);
	return 0;
}

char** getFileList(char *dirPath, int n)
{
	char path[256];
	int i, j;
	//File List 2차원 동적 배열
	char **dirs = (char **)malloc(sizeof(char*)*n);
	for(i = 0; i < n; i++) {
		dirs[i] = (char *)malloc(sizeof(char)*256);
	}

	memset(path, 0, sizeof(char)*256);
	strcat(path, dirPath);
	DIR *dir_ptr = NULL;
	struct dirent *file = NULL;
	
	// 목록을 읽을 디렉토리명으로 DIR *을 return 받기
	if((dir_ptr = opendir(dirPath)) == NULL)
	{
		printf("%s directory 정보를 읽을 수 없습니다.\n", dirPath);
	}
	//printf("%s directory 정보를 읽었습니다.\n", dirPath);
	
	//디렉토리 처음부터 파일 또는 디렉토리명을 순서대로 한개씩 읽어본다. 
	int cnt = 0;
	while((file = readdir(dir_ptr)) != NULL)
	{
		if(!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")){
			continue;
		}
		memset(dirs[cnt], 0, sizeof(char)*256);
		sprintf(dirs[cnt], "%s/%s", dirPath, file->d_name);
	//	printf("%s\n", dirs[cnt]);
		cnt ++;
	}
	printf("%d 개의 파일 또는 디렉토리가 있습니다.\n", cnt);
	
	// open된 directory 정보를 close
	closedir(dir_ptr);

	return dirs;
}

void displayWorldPoint(Mat TOPView, wpoint* left_world, wpoint* right_world,int  left_marker_num,int right_marker_num ,float X_MIN,float Z_MIN, int X_NUM, int Z_NUM, float GRID)
{
	float x, z;
	int nx, nz;

	for(int i=0; i<left_marker_num; i++){
		x=left_world[i].x;
		z=left_world[i].z;
		nx = (int)((x-X_MIN + GRID*0.5)/GRID);
		nz = (int)((z-Z_MIN + GRID*0.5)/GRID);
		nz = Z_NUM-1-nz;
		if(nx<0 || nx>X_NUM-1 || nz<0 || nz>Z_NUM-1) continue;
		circle(TOPView, Point(nx, nz), 3, Scalar(255,0,0), 1);
	}

	for(int i=0; i<right_marker_num; i++){
		x=right_world[i].x;
		z=right_world[i].z;
		nx = (int)((x-X_MIN + GRID*0.5)/GRID);
		nz = (int)((z-Z_MIN + GRID*0.5)/GRID);
		nz = Z_NUM-1-nz;

		if(nx<0 || nx>X_NUM-1 || nz<0 || nz>Z_NUM-1) continue;
		circle(TOPView, Point(nx, nz), 3, Scalar(255,0,0), 1);
	}
}

void displayRegionVector(Mat RoadImgMat, int y_min, int y_max, unsigned short* L_area, unsigned short* R_area, lanevector left_vec, lanevector right_vec)
{
	int idx = 0;
	int nx1, ny1, nx2, ny2;
	for(int yy=y_min; yy<y_max; yy++){
		nx1 = (int)L_area[idx*3+1];
		ny1 = (int)L_area[idx*3] ;
		nx2 = (int)L_area[(idx+1)*3+1];
		ny2 = (int)L_area[(idx+1)*3] ;		
		//Left line
		line(RoadImgMat, Point(nx1, ny1), Point(nx2, ny2), Scalar(255,0,0), 1);
		nx1 = (int)L_area[idx*3+2];
		nx2 = (int)L_area[(idx+1)*3+2];
		//Center line
		line(RoadImgMat, Point(nx1, ny1), Point(nx2, ny2), Scalar(255,0,0), 1);
		nx1 = (int)R_area[idx*3+2];
		nx2 = (int)R_area[(idx+1)*3+2];		
		//Right line		
		line(RoadImgMat, Point(nx1, ny1), Point(nx2, ny2), Scalar(255,0,0), 1);
		idx++;
	}
	//first line
	nx1 = (int)L_area[1];
	ny1 = y_min;
	nx2 = (int)R_area[2];
	ny2 = y_min;
	line(RoadImgMat, Point(nx1, ny1), Point(nx2, ny2), Scalar(255,0,0), 1);
	//last line	
	nx1 = (int)L_area[idx*3+1];
	ny1 = y_max;
	nx2 = (int)R_area[idx*3+2];
	ny2 = y_max;
	line(RoadImgMat, Point(nx1, ny1), Point(nx2, ny2), Scalar(255,0,0), 1);

	//left init vector
	line(RoadImgMat, Point(left_vec.x, left_vec.y), Point(left_vec.x+left_vec.gx, left_vec.y + left_vec.gy), Scalar(0,0,0), 3);
	//right init vector
	line(RoadImgMat, Point(right_vec.x, right_vec.y), Point(right_vec.x+right_vec.gx, right_vec.y + right_vec.gy), Scalar(0,0,0), 3);

}

void displayimgPoint(Mat View, ipoint* left_world, ipoint* right_world,int  left_marker_num, int right_marker_num)
{
    int x, z;

    for(int i=0; i<left_marker_num; i++){
        x=(int)left_world[i].x;
        z=(int)left_world[i].y;
        circle(View, Point(x, z), 5, Scalar(255,0,0), 2,8);
    }

    for(int i=0; i<right_marker_num; i++){
        x=(int)right_world[i].x;
        z=(int)right_world[i].y;
        circle(View, Point(x, z), 5, Scalar(0,0,255), 2,8);
    }
}

void displayTOPView(Mat TOPView, int left_valid, int right_valid, float *left_coeff, float* right_coeff, float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID)
{
	float x1, x2;
	float z1, z2;

	int nx1, nx2;
	int nz1, nz2;

	
	if(left_valid != -1){

		 x1= left_coeff[0];
		 nx1=(int)((x1-X_MIN + GRID*0.5)/GRID);
		 nx1= max(nx1, 0);
		 nx1= min(nx1, X_NUM-1);

		 z1=0;
		 nz1=(int)((z1-Z_MIN + GRID*0.5)/GRID);
		 nz1= max(nz1, 0);
		 nz1= min(nz1, Z_NUM-1);
		 nz1 = Z_NUM-1-nz1;
		
		int cnt = 1;
		for(float z=0;z<=10; z=z+1)
		{
			 x2 = left_coeff[2]*z*z+left_coeff[1]*z + left_coeff[0];
			 nx2=(int)((x2-X_MIN + GRID*0.5)/GRID);
			 
			 if(nx2<0 || nx2>X_NUM-1) break;
			 

			 z2=z;
			 nz2=(int)((z2-Z_MIN + GRID*0.5)/GRID);
			 nz2= max(nz2, 0);
			 nz2= min(nz2, Z_NUM-1);
			 nz2 = Z_NUM-1-nz2;

			 line(TOPView, Point(nx1, nz1), Point(nx2, nz2), Scalar(255,0,0), 1);
			 //printf("cnt = %d", cnt);
			 cnt++;
		     //printf("Left :(nx1,nz1)=(%d, %d), (nx2,nz2)=(%d,%d)",nx1,nz1,nx2,nz2);
			 nx1=nx2;
			 nz1=nz2;
			 
		}

	}


	if(right_valid != -1){

		 x1= right_coeff[0];
		 nx1=(int)((x1-X_MIN + GRID*0.5)/GRID);
		 nx1= max(nx1, 0);
		 nx1= min(nx1, X_NUM-1);

		 z1=0;
		 nz1=(int)((z1-Z_MIN + GRID*0.5)/GRID);
		 nz1 = Z_NUM-1-nz1;

		for(float z=0;z<=10; z=z+1)
		{
			 x2 = right_coeff[2]*z*z+right_coeff[1]*z + right_coeff[0];
			 nx2=(int)((x2-X_MIN + GRID*0.5)/GRID);
			 
			 if(nx2<0 || nx2>X_NUM-1) break;
			 
			 z2=z;
			 nz2=(int)((z2-Z_MIN + GRID*0.5)/GRID);
			 nz2 = Z_NUM-1-nz2;

			 line(TOPView, Point(nx1, nz1), Point(nx2, nz2), Scalar(0,0,255), 1);
			//printf("Right :(nx1,nz1)=(%d,%d), (nx2,nz2)=(%d,%d)",nx1,nz1,nx2,nz2);	 
			 nx1=nx2;
			 nz1=nz2;
			 
		}

	}

}

void display1stTOPView(Mat TOPView, int left_valid, int right_valid, float *left_coeff, float* right_coeff, float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID)
{
	float x1, x2;
	float z1, z2;

	int nx1, nx2;
	int nz1, nz2;

	
	if(left_valid != -1){

		 x1= left_coeff[0];
		 nx1=(int)((x1-X_MIN + GRID*0.5)/GRID);
		 nx1= max(nx1, 0);
		 nx1= min(nx1, X_NUM-1);

		 z1=0;
		 nz1=(int)((z1-Z_MIN + GRID*0.5)/GRID);
		 nz1= max(nz1, 0);
		 nz1= min(nz1, Z_NUM-1);
		 nz1 = Z_NUM-1-nz1;
		
		int cnt = 1;
		for(float z=0;z<=10; z=z+1)
		{
			 x2 = left_coeff[1]*z + left_coeff[0];
			 nx2=(int)((x2-X_MIN + GRID*0.5)/GRID);
			 
			 if(nx2<0 || nx2>X_NUM-1) break;
			 

			 z2=z;
			 nz2=(int)((z2-Z_MIN + GRID*0.5)/GRID);
			 nz2= max(nz2, 0);
			 nz2= min(nz2, Z_NUM-1);
			 nz2 = Z_NUM-1-nz2;

			 line(TOPView, Point(nx1, nz1), Point(nx2, nz2), Scalar(255,0,0), 1);
			 //printf("cnt = %d", cnt);
			 cnt++;
		     //printf("Left :(nx1,nz1)=(%d, %d), (nx2,nz2)=(%d,%d)",nx1,nz1,nx2,nz2);
			 nx1=nx2;
			 nz1=nz2;
			 
		}

	}


	if(right_valid != -1){

		 x1= right_coeff[0];
		 nx1=(int)((x1-X_MIN + GRID*0.5)/GRID);
		 nx1= max(nx1, 0);
		 nx1= min(nx1, X_NUM-1);

		 z1=0;
		 nz1=(int)((z1-Z_MIN + GRID*0.5)/GRID);
		 nz1 = Z_NUM-1-nz1;

		for(float z=0;z<=10; z=z+1)
		{
			 x2 = right_coeff[1]*z + right_coeff[0];
			 nx2=(int)((x2-X_MIN + GRID*0.5)/GRID);
			 
			 if(nx2<0 || nx2>X_NUM-1) break;
			 
			 z2=z;
			 nz2=(int)((z2-Z_MIN + GRID*0.5)/GRID);
			 nz2 = Z_NUM-1-nz2;

			 line(TOPView, Point(nx1, nz1), Point(nx2, nz2), Scalar(0,0,255), 1);
			//printf("Right :(nx1,nz1)=(%d,%d), (nx2,nz2)=(%d,%d)",nx1,nz1,nx2,nz2);	 
			 nx1=nx2;
			 nz1=nz2;
			 
		}

	}

}

void displayGPP(Mat TOPView, float* gpp,float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID)
{
	float x, z;
	int nx, nz;

	x = gpp[0];
	z = gpp[1];
	nx = (int)((x-X_MIN + GRID*0.5)/GRID);
	nz = (int)((z-Z_MIN + GRID*0.5)/GRID);
	nz = Z_NUM-1-nz;
	circle(TOPView, Point(nx, nz), 3, Scalar(0,255,0), 1);

}
void displayGPP2(Mat TOPView, float* gpp, float* tpl, float*tpr, float X_MIN, float Z_MIN, int X_NUM, int Z_NUM, float GRID)
{
	float x, z;
	int nx, nz, nx1, nx2, nz1, nz2;

	x = gpp[0];
	z = gpp[1];
	nx = (int)((x-X_MIN + GRID*0.5)/GRID);
	nz = (int)((z-Z_MIN + GRID*0.5)/GRID);
	nz = Z_NUM-1-nz;
	circle(TOPView, Point(nx, nz), 3, Scalar(0,255,0), 1);

	x = tpl[0];
	z = Z_MIN ;
	nx1 = (int)((x-X_MIN + GRID*0.5)/GRID);
	nz1 = (int)((z-Z_MIN + GRID*0.5)/GRID);
	nz1= Z_NUM-1-nz1;
	circle(TOPView, Point(nx1, nz1), 3, Scalar(255,255,255), 1);

	x = tpl[1];
	z = gpp[1];
	nx2 = (int)((x-X_MIN + GRID*0.5)/GRID);
	nz2 = (int)((z-Z_MIN + GRID*0.5)/GRID);
	nz2 = Z_NUM-1-nz2;
	circle(TOPView, Point(nx2, nz2), 3, Scalar(255,255,255), 1);

	//Add line
	line(TOPView, Point(nx1, nz1), Point(nx2, nz2), Scalar(255,255,255), 1);

	x = tpr[0];
	z = Z_MIN;
	nx1 = (int)((x-X_MIN + GRID*0.5)/GRID);
	nz1 = (int)((z-Z_MIN + GRID*0.5)/GRID);
	nz1 = Z_NUM-1-nz1;
	circle(TOPView, Point(nx1, nz1), 3, Scalar(255,255,255), 1);

	x = tpr[1];
	z = gpp[1];
	nx2 = (int)((x-X_MIN + GRID*0.5)/GRID);
	nz2 = (int)((z-Z_MIN + GRID*0.5)/GRID);
	nz2 = Z_NUM-1-nz2;
	circle(TOPView, Point(nx2, nz2), 3, Scalar(255,255,255), 1);

	//Add line
	 line(TOPView, Point(nx1, nz1), Point(nx2, nz2), Scalar(255,255,255), 1);
}


