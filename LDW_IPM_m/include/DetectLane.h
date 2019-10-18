#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <stdlib.h>


#include <vector>
//#include <io.h>

using namespace cv;

#define PI   3.14159265358

//#define min(X,Y) ((X) < (Y) ? (X) : (Y))   
//#define max(X,Y) ((X) > (Y) ? (X) : (Y))  

typedef struct lanevector{
	int x;
	int y;
	int gx;
	int gy;
}_LANEVECTOR;

typedef struct camera{
	float h;  // 높이
	float dx, dz; // x, y 좌표 (월드좌표)
	int m, n;  // 해상도
	float alphaTot; // 대각선 FOV(radian)
	float theta0;  // tilt angle

}_CAMERA;

typedef struct hough{

	int theta;
	int rough;
	int hmax;
	
}_HOUGH;

typedef struct ipoint{
	int x;
	int y;
}_IPOINT;

typedef struct wpoint{
	float x;
	float z;
}_WPOINT;

class TrackingLane
{
private:
	int _nDetect[2];
	int _nMiss[2]; 

	 float _error_th_0;  // near 0.3m
	 float _error_th_1 ; //far 0.5m
	 float _z_max ;
	 float _z_min ;

	 int _confidence_th[5];
	

public:

	float _tracking_points_l[2];
	float _tracking_points_r[2];

	int m_confidence[2];
	float GPP[2]; //

	TrackingLane();
	~TrackingLane();
	void updateLane(int lanevalid0, int lanevalid1,float *lanecoeff0, float *lanecoeff1);
	void plupdate(int lanevalid0, int lanevalid1,float *lanecoeff0, float *lanecoeff1);
	void coeffupdate1st(int lanevalid0, int lanevalid1,float *lanecoeff0, float *lanecoeff1);	
	void coeffupdate(int lanevalid0, int lanevalid1,float *lanecoeff0, float *lanecoeff1);	
	void updateConfidence();
	
};

class CDetectLane
{
	public:
	//variables
    unsigned char *GImage;

	int MAX_MARKER_NUM, X_NUM, Z_NUM; 
	float X_MIN, Z_MIN, GRID;
	int img_width, img_height;
	int imgnum;

	//filter param;
	int filter_width, filter_height;
	float filter_sigma;
	float *logfilter;  // filter value
    
	int rHorizon; // height 0~335번 중 하나
	int hRow;  // 허프영역의 row수
	unsigned short im_area_L[320*3]; // 허프영역 Left, row, col_1, col_2 
	unsigned short im_area_R[320*3]; // 허프영역 Right,  row, col_1, col_2 
	camera camParams;

	float x_min,x_max,z_min, z_max;
	
	wpoint p1[4];
	ipoint p2[4];
	int yy_min, yy_max ;  // 처리 영역의 row number

	lanevector left_init_vec;
	lanevector right_init_vec ;

	ipoint *left_lane_marker;
	ipoint *right_lane_marker;

	wpoint *left_world;
	wpoint *right_world;

	int left_imarker_num, right_imarker_num;
	int left_marker_num, right_marker_num;
	int final_marker_num;
	
	int acc_p;
	wpoint *buf_left, *buf_right;
	int* buf_left_num;
	int* buf_right_num;

	float *left_coeff;
	float *right_coeff;
	float *left_fpnx, *left_fpnz;
	float *right_fpnx, *right_fpnz;

	int left_valid;
	int right_valid;

	TrackingLane m_Track;
	
	CDetectLane(void);
	~CDetectLane(void);
	void InitializeParams(void); //파라미터 초기화
	float* get_logfilter( int fw, int fh, float sigma);
	void setup_cam( camera &camParams);
	int cal_rHorizon(camera camParams);
	int calculation_region(unsigned short* L_area, unsigned short* R_area, float x_min, float x_max, float z_min, float z_max, camera camParams, float deltax); //processing area 계산
	void convert_to_image( ipoint *p2, wpoint *p1, int pnum, camera cam);

	// Process function
	void Process(unsigned char *Img);
	void RGB2Gray(unsigned char* Img, unsigned char* GImage);
	void lane_filter( unsigned char *RoadImg,  float *logfilter, int img_width,int img_height,int filter_width, int filter_height,int rHorizon);
	float conv2(float* output, unsigned char* input, float* w, int inw, int inh, int outw, int outh, int fsize);

	void FilterLane(unsigned char *RoadImg); //노이즈 제거, 영상에서 차선 성분 추출(LOG filter)
	void SetROI(void); //영상 내 허프 변환을 위한 ROI 설정
	
	void FindLaneVector(unsigned char *RoadImg); //허프변환, 좌차선 벡터/우차선 벡터, 정지선 중앙점 
	lanevector find_InitialVector(unsigned char *img, unsigned short* im_area, bool left_lane, int rows);
	hough hough_window(unsigned char *img, unsigned short *area,int rows, int theta1, int theta2);

	void findLanePoints(unsigned char *RoadImg); //슬라이딩 윈도우 설정, 차선 마커의 무게 중심점
	int find_lane_marker(ipoint *lane_marker, unsigned char *img, lanevector vec);
	void find_center_of_mass(ipoint* mass_pt, unsigned char* img, ipoint pt, int winx, int winy);

	void ConvertToWorld(void); //이미지 점들의 월드 좌표계 이동
	void convert_to_world( wpoint *p2, ipoint *p1, int pnum, camera cam);

	void AccLanePoints(int i); //차선 후보점 누적
	void accumul_wp(wpoint *world, int *marker_num, int i, wpoint *buf_m, int *buf_m_num);

    void AccLanePoints(void); //차선 후보점 누적 v2
	
	void FitLane(void); //RANSAC, 월드좌표 기준 좌우 차선 피팅
	int polyfit(wpoint *points,
            int        countOfElements,
            int        order,
            float*             coefficients);
	int parabolic_fit(wpoint *points,  int  N,  float *coeff);
	int ransac_1stfit(wpoint *points,  int  markerNum,  float *coeff);
	int ransac_fit(wpoint *points,  int  markerNum,  float *coeff);

	void TrackLane(void); //Kalman filter

	void UpdateConfidence(void);//Confidence level update
};

