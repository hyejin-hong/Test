#include "DetectLane.h"

CDetectLane::CDetectLane(void)
{
}

CDetectLane::~CDetectLane(void)
{
    delete[] GImage;
	delete[] left_coeff;
	delete[] right_coeff;
	delete[] left_lane_marker;
	delete[] right_lane_marker;
	delete[] left_world;
	delete[] right_world;
	delete[] buf_left;
	delete[] buf_right;
	delete[] buf_left_num;
	delete[] buf_right_num;
}

void CDetectLane::InitializeParams(void)
{
	MAX_MARKER_NUM = 220;
	X_NUM = 200;
	Z_NUM = 300; //300
	X_MIN = -5.0;
	Z_MIN = 0.0; //5.0
 	GRID = 0.05;

	img_width  = 640;
	img_height = 480;
	imgnum = 10783;
	
    GImage = new unsigned char[img_width*img_height];
	left_coeff = new float[2];
	right_coeff = new float[2];

	left_valid=-1;
	right_valid=-1;
	
	//set up log filter
	filter_width =15;
	filter_height=15;
	filter_sigma=5;
	logfilter= get_logfilter( filter_width, filter_height, filter_sigma);

	//setup camera param
	setup_cam(camParams);
	rHorizon= cal_rHorizon(camParams);
	printf("rHorizon = %d\n", rHorizon);
   
	left_lane_marker = new ipoint[sizeof(ipoint)*MAX_MARKER_NUM];
	right_lane_marker= new ipoint[sizeof(ipoint)*MAX_MARKER_NUM];	
	left_world= new wpoint[sizeof(wpoint)*MAX_MARKER_NUM];
	right_world=new wpoint[sizeof(wpoint)*MAX_MARKER_NUM];

	//accumulation 시 필요한  param
	acc_p = 5;
	buf_left= new wpoint[sizeof(wpoint)*MAX_MARKER_NUM];
	buf_right=new wpoint[sizeof(wpoint)*MAX_MARKER_NUM];
	buf_left_num = new int[sizeof(int)*acc_p];
	buf_right_num = new int[sizeof(int)*acc_p];
}

float* CDetectLane::get_logfilter( int fw, int fh, float sigma)
{
#define eps 1.0e-16

	int siz_w = (fw-1)/2;
	int siz_h = (fh-1)/2;
	float std2 = sigma*sigma;

	float *filter = (float *)malloc(fw*fh*sizeof(float));
	
	float sum=0;
	float sum2=0;

	int x, y, indx, indy;
	float val, val2;

	for( y=-siz_h,  indy=0; y<=siz_h; y++, indy++)
		for( x=-siz_w, indx=0 ; x<=siz_w; x++, indx ++)
		{
			val = -(x*x + y*y)/(2*std2);
			val =(float) exp(val);
			if(val<eps) val=0;
			
			*(filter + indy*fw + indx) = val;
			sum = sum + val;

		}
	
	for( y=-siz_h,  indy=0; y<=siz_h; y++, indy++)
		for( x=-siz_w, indx=0 ; x<=siz_w; x++, indx ++)
		{
			
			val = *(filter + indy*fw + indx) ;
			val = val /sum;
			val2 = val*(x*x + y*y -2*std2)/(std2*std2);
			*(filter + indy*fw + indx) = val2;
			sum2 = sum2 + val2;
			
		}

		
	for( y=-siz_h,  indy=0; y<=siz_h; y++, indy++)
		for( x=-siz_w, indx=0 ; x<=siz_w; x++, indx ++)
		{
			
			val = *(filter + indy*fw + indx) ;
			val2= val - sum2/(fw*fh);
			*(filter + indy*fw + indx)= val2 ;
		
			
		}
		return(filter);
}

void CDetectLane::setup_cam( camera &camParams)
{
	camParams.h = 1.421;
	camParams.dx=0;
	camParams.dz=0;
	camParams.m = img_height; // Rows (height) 
	camParams.n = img_width; // Columns (width) 
	camParams.alphaTot =  (95/2)*PI/180; //radian
	camParams.theta0 = 0.2245; //radian
}

int CDetectLane::cal_rHorizon(camera camParams)
{		
	float den = sqrt(float((camParams.m-1)*(camParams.m-1)+(camParams.n-1)*(camParams.n-1))); 
	float alpha_v = atan( (camParams.m-1)/den * tan(camParams.alphaTot) ) ; 
	int rHorizon = ceil( (camParams.m-1)/2*(1 - tan(camParams.theta0)/tan(alpha_v))  )   + ceil(camParams.m*0.01);

	return(rHorizon);
}

void CDetectLane::SetROI(void)
{
	#if 1
	//setup processing area
    x_min= -3.0; //-3.5
    x_max= 3.0; //3.5
	z_min= 2.3;
    z_max= 12.0;	//20
#else
    x_min=-3.5; //-3.5
    x_max= 2.5; //3.5
    z_min= 2.5;
    z_max= 15.0;	//20
#endif
	
	//hough region calculation
	hRow=calculation_region( im_area_L,im_area_R, x_min, x_max, z_min, z_max, camParams, 0.0);
	//hRow=calculation_region( im_area_L,im_area_R, x_min, x_max, z_min, z_max, camParams, m_Track.GPP[0]);
}

int CDetectLane::calculation_region(unsigned short* L_area, unsigned short* R_area, float x_min, float x_max, float z_min, float z_max, camera camParams, float deltax)
{
	int idx=0;

	// 사다리꼴 경계선 직선 파라미터
	double a1, b1, a2, b2;

	//4개의 월드좌표계의 경계점 
	p1[0].x = x_min + deltax;
	p1[0].z = z_max;

	p1[1].x = x_min;
	p1[1].z = z_min;

	p1[2].x = x_max + deltax;
	p1[2].z = z_max;

	p1[3].x = x_max;
	p1[3].z = z_min;

	// 월드 경계점 -> 이미지 경계점
	convert_to_image(p2, p1, 4, camParams);

	a1 =(double)(p2[0].x - p2[1].x)/(p2[0].y-p2[1].y);
	b1 = (double)(-a1*p2[0].y +p2[0].x);
  
	a2= (double)(p2[2].x - p2[3].x)/(p2[2].y-p2[3].y);
	b2 = (double)(-a2*p2[2].y + p2[2].x);

	yy_min = max(p2[0].y, 0);
	yy_max = min(p2[1].y, img_height-1);

	
	for(int yy=yy_min; yy<=yy_max; yy++){
		int x1 = (int)(a1*yy + b1 +0.5); // 반올림 구현 +0.5
	    int x2 = (int)(a2*yy + b2 +0.5);
		int center = (int)((x1+x2)/2);
		
		L_area[idx*3] =yy;
		L_area[idx*3+1] =max(x1,0);
		L_area[idx*3+2]= center;


		R_area[idx*3] =yy;
		R_area[idx*3+1] =center;
		R_area[idx*3+2]= min(x2,img_width-1);
		idx++;
	}

	return(idx);
}

void CDetectLane::convert_to_image( ipoint *p2, wpoint *p1, int pnum, camera cam)
{
	double den = sqrt(double((cam.m-1)*(cam.m-1)+(cam.n-1)*(cam.n-1))); 

	double alpha_u = atan( (cam.n-1)/den * tan(cam.alphaTot) ) ; 
	double alpha_v = atan( (cam.m-1)/den * tan(cam.alphaTot) ) ; 

	for(int i=0; i<pnum; i++)
	{
		
		double u= ((cam.n-1)/2)*(1-(-p1[i].x *(1/tan(alpha_u))/(cam.h*sin(cam.theta0)+p1[i].z*cos(cam.theta0)))  );
		double v = ((cam.m-1)/2)*(1+(cam.h-p1[i].z*tan(cam.theta0))/(cam.h*tan(cam.theta0)+p1[i].z)*(1/tan(alpha_v)));
		p2[i].x = u+1;
		p2[i].y  =v+1;
	}
}

void CDetectLane::Process(unsigned char *Img)
{
	    //RGB2Gray
        RGB2Gray(Img, GImage);
		//printf("GImage[0],[1],[2] = %d %d %d \n", GImage[0], GImage[1], GImage[2]);

		// 노이즈 제거 
		FilterLane(GImage);

		// 영상 내 허프 변환을 위한 ROI 설정
		SetROI();
	
		// 허프기반으로 초기 벡터 찾기 : 허프 변환, 좌차선 벡터/우차선 벡터, 정지선 중앙점 
		FindLaneVector(GImage);

		// 벡터를 기준으로 차선 마커의 무게중심점
		//슬라이딩 윈도우 설정, 차선 마커의 무게중심점
		findLanePoints(GImage);

		// 무게 중심점들을 월드 좌표계로 변환함
		//이미지 점들의 월드 좌표계 이동
		ConvertToWorld();

		// 차선 후보점 누적
		AccLanePoints();

		// RANSAC, 월드좌표 기준 좌우 차선 피팅 
		FitLane();

		//Kalman filter
		TrackLane();

		//Confidence level update
		UpdateConfidence();


}

void CDetectLane::RGB2Gray(unsigned char* Img, unsigned char* GImage)
{
    int R,G,B;
    for(int i=0; i<img_height*img_width; i++)
    {
        R = Img[i*3];
        G = Img[i*3 + 1];
        B = Img[i*3 + 2];
            //GImage[i*img_width + j]= (unsigned char)((double)0.299*(double)Img[i*img_width + j]+ (double)0.587*(double)Img[(i+1)*img_width + j] + (double)0.144*(double)Img[(i+2)*img_width + j]);

        GImage[i]= (unsigned char)((double)0.299*(double)R+ (double)0.587*(double)G + (double)0.144*(double)B);

    }
}
void CDetectLane::FilterLane(unsigned char *RoadImg)
{
	// LOG filter
	lane_filter( RoadImg,  logfilter, img_width, img_height, filter_width, filter_height, rHorizon);
}

void CDetectLane::lane_filter( unsigned char *RoadImg,  float *logfilter, int img_width,int img_height,int filter_width, int filter_height,int rHorizon)
{
	//rHorizon(0~335중, 215번째) 부터 마지막 컬럼까지 모두 처리
   int inputw = img_width;
   int inputh = img_height-rHorizon;

   int outputw = inputw;
   int outputh = inputh-100;

   float *output = (float*) malloc( outputw*outputh*sizeof(float));

   float max = conv2( output , RoadImg+ (rHorizon-1)*img_width, logfilter, inputw, inputh, outputw, outputh, filter_width);
   float thres = max*0.12;

   int half = (filter_width-1)/2;
  
   half=7;

   for(int i=half; i<outputh-half; i++)
   {
	   for(int j=half; j<outputw-half; j++){
		   float v= *(output+i*outputw + j);
			if( v > thres) {
				RoadImg[(i+rHorizon-1)*img_width +j]= 255;
			}else
			{
				RoadImg[(i+rHorizon-1)*img_width +j]= 0;
			}
	   }
   }

   free(output);
}

float CDetectLane::conv2(float* output, unsigned char* input, float* w, int inw, int inh, int outw, int outh, int fsize)
{
    register float acc;
    register int i; 
    register int j;
    register int k1, k2;
    register int l1, l2;
    register int t1, t2;

	float max=0;
	int hsize=fsize/2;

    for(i = 0; i <= outh-fsize; i++) 
    {
        t1 = (i+hsize) * outw; // loop invariants
        for(j = 0; j <= outw-fsize; j++) 
        {   
            acc = 0.0;
            for(k1 = fsize - 1, k2 = 0; k1 >= 0; k1--, k2++)
            {
                t2 = k1 * fsize;  // loop invariants
                for(l1 = fsize - 1, l2 = 0; l1 >= 0; l1--, l2++)
                {
                    acc += w[t2 + l1] * input[(i + k2) * inw + (j + l2)];
                }
            }
			//차선 필터링인경우, 음수값만을 취함
			if(acc<0)  // 차선의 중심은 큰 음수값으로 나타난다
			  output[t1 + j+ hsize] = -1*acc;
			else
			  output[t1 + j+hsize] =0;

			// 가장 큰 값을 찾는다
			//if(max<acc) max = acc;
				if(max<output[t1 + j+ hsize] ) max = output[t1 + j+ hsize] ;
        }
    }

    return max;
}

void CDetectLane::FindLaneVector(unsigned char *RoadImg)
{
	left_init_vec =find_InitialVector(RoadImg, im_area_L,1, hRow);
	right_init_vec =find_InitialVector(RoadImg, im_area_R,0, hRow);
}

lanevector CDetectLane::find_InitialVector(unsigned char *img, unsigned short* im_area, bool left_lane, int rows)
{

	lanevector vec;
	
	int angle1, angle2;

	if(left_lane ==1){
		angle1=35;//start angle
		angle2=60; // end angle
	}else{
		angle1=-60;
		angle2=-35;
	}

	//hough 수행
	hough h=hough_window(img, im_area, rows,angle1, angle2);

	if(h.hmax != -1){

		int x1=(h.rough- im_area[0]* sin(h.theta * (PI/180)))/cos(h.theta* (PI/180));		
		int x2=(h.rough- im_area[(rows-1)*3]* sin(h.theta * (PI/180)))/cos(h.theta* (PI/180));

		vec.x= x2;
		vec.y = im_area[(rows-1)*3];
		vec.gx = x1-x2;
		vec.gy =  im_area[0] - vec.y;
	   
	}else{
		vec.x=-1;
		vec.y=-1;
		vec.gx=-1;
		vec.gy=-1;
	}


	return vec;

}

hough CDetectLane::hough_window(unsigned char *img, unsigned short *area,int rows, int theta1, int theta2)
{
	int thetaResolution =1;
	int rhoResolution =4;

	int rho_min=0;
	int rho_max= sqrt(img_width*img_width + img_height*img_height); // 상수값 sqrt(336^2+640^2)
	
	int ntheta = ceil(float(theta2-(theta1-1))/thetaResolution);
	int nrho = ceil(float(rho_max-(rho_min-1))/rhoResolution);

	int *theta = (int *)malloc(ntheta*sizeof(int));
	int *houghtable = (int *)malloc(ntheta*nrho*sizeof(int));

	int max_h=-1;
	int max_theta=-1;
	int max_rho=-1;
	int total_n=0;

	for(int i=0; i<ntheta*nrho; i++) houghtable[i]=0;

	theta[0]= theta1;
	for(int i=1; i<ntheta; i++)
		theta[i]=theta[i-1]+thetaResolution;

	for(int rown=0; rown<rows; rown++){

		int r= area[rown*3];
		int c1=area[rown*3+1];
		int c2=area[rown*3+2];

		for(int c=c1; c<=c2; c++){
			if( img[ r* img_width +c ] !=255 ) continue;

			int x = c;
			int y = r;

		    total_n =total_n+1;

			for(int t=0; t< (ntheta); t++){
				float theta_rad = theta[t]*PI/180;
				float tmp_rho=x*cos(theta_rad)+y*sin(theta_rad);

				if(tmp_rho >= rho_min && tmp_rho <= rho_max){
					int r_index = floor((tmp_rho-rho_min)/rhoResolution);
					houghtable[ r_index * (ntheta) + t]++;  // 세로는 rho, 가로는 theta

					if(houghtable[ r_index * (ntheta) + t] > max_h)
					{  max_h = houghtable[ r_index * (ntheta) + t];
					   max_theta = t;
					   max_rho = r_index;
					}
				}
			}

		}
	}
	hough result;
	if(max_h > total_n*0.1)
	{
		result.hmax=max_h;
		result.rough= rho_min+max_rho*rhoResolution;
		result.theta= theta[max_theta];
	}else{
		result.hmax=-1;
		result.rough=-1;
		result.theta=-1;
	}

	free(houghtable);
	free(theta);
	return result;
}

void CDetectLane::findLanePoints(unsigned char *RoadImg)
{
	left_imarker_num = find_lane_marker(left_lane_marker, RoadImg, left_init_vec);
	right_imarker_num= find_lane_marker(right_lane_marker, RoadImg, right_init_vec);

	left_marker_num = left_imarker_num;
	right_marker_num = right_imarker_num;

}

int CDetectLane::find_lane_marker(ipoint *lane_marker, unsigned char *img, lanevector vec)
{	
	int init_x=vec.x;
	int init_y=vec.y;
	int init_gx=vec.gx;
	int init_gy=vec.gy;

	int marker_num=0;
	//int winx=15;
	int winx=10;
	
	int winy=2;
	int stepy =winy*2;
	int m=img_height;
	int n=img_width;
	int pre_x;
	int pre_y;

	if(init_x ==-1){
		marker_num=0;
		lane_marker=NULL;
		return(marker_num);
	}
	pre_x =init_x;
	pre_y= init_y;

	int gx =  init_gx*stepy/abs(init_gy);  //gx의 크기를 stepy 크기로 normalize
	int gy =  -1*stepy;
	int idx=0;
	
	
	ipoint est_pt;
	est_pt.x=pre_x;
	est_pt.y=pre_y;

	while(est_pt.y>init_y+init_gy){

		ipoint mass_pt;
		find_center_of_mass(&mass_pt,img, est_pt, winx, winy);
		 //rectangle('Position',[est_x-winx, est_y-winy, 2*winx, 2*winy],'EdgeColor', 'g');
		if(mass_pt.x !=-1){
			
			lane_marker[idx]=mass_pt;
			idx++;
			//plot(mass_x, mass_y, 'p', 'MarkerFaceColor','b');
			//pre_x=est_pt.x;
			pre_x=mass_pt.x;
			//pre_y=est_pt.y;
			pre_y=mass_pt.y;
		}else{
			pre_x=est_pt.x;
			pre_y=est_pt.y;
		}
		est_pt.x = pre_x+gx;
		est_pt.y = pre_y+gy;
	}
	 marker_num = idx;
	 return(marker_num);
}

void CDetectLane::find_center_of_mass(ipoint* mass_pt, unsigned char* img, ipoint pt, int winx, int winy)
{
	float xsum=0;
	float ysum=0;
	int length =0;
	

	if(pt.x<0 || pt.x>img_width-1 || pt.y<0 ||pt.y>img_height-1){
		
		mass_pt->x =-1;
		mass_pt->y=-1;
		
	}
	for(int yy=max(pt.y-winy,0);yy<=min(pt.y+winy,img_height-1);yy++){
		for(int xx=max(pt.x-winx,0); xx<=min(pt.x+winx,img_width-1); xx++){
			if(img[yy*img_width+xx] ==255){
				xsum=xsum+xx;
				ysum=ysum+yy;
				length++;
			}
		}
	}

	if(length>winy){
		mass_pt->x=xsum/length;
		mass_pt->y=ysum/length;
	}else{
		mass_pt->x=-1;
		mass_pt->y=-1;

	}

}

void CDetectLane::ConvertToWorld(void)
{
	convert_to_world(left_world, left_lane_marker, left_marker_num, camParams);
	convert_to_world(right_world, right_lane_marker, right_marker_num, camParams);
}

void CDetectLane::convert_to_world( wpoint *p2, ipoint *p1, int pnum, camera cam)
{
	
	float d = sqrt(float((cam.m-1)*(cam.m-1)+(cam.n-1)*(cam.n-1))); 
	float alpha_u = atan( (cam.n-1)/d * tan(cam.alphaTot) ) ; 
	float alpha_v = atan( (cam.m-1)/d * tan(cam.alphaTot) ) ; 

	for(int i=0; i<pnum; i++)
	{
		float v= p1[i].y;
		float u =p1[i].x;

		float rFactor = (1-2*(v-1)/(cam.m-1))*tan(alpha_v); 
		float num = 1 + rFactor*tan(cam.theta0); 
		float den = tan(cam.theta0) - rFactor; 
		p2[i].z = cam.h*(num/den)+cam.dz; 

		num = (1-2*(u-1)/(cam.n-1))*tan(alpha_u); 
		den = sin(cam.theta0) - rFactor*cos(cam.theta0); 
	    p2[i].x = -cam.h*(num/den)+cam.dx; 

	}
}

void CDetectLane::AccLanePoints(int i)
{
	if(acc_p >1)
	{
		accumul_wp(left_world, &left_marker_num, i, buf_left, buf_left_num);
		accumul_wp(right_world, &right_marker_num, i, buf_right, buf_right_num);
	}
}

void CDetectLane::AccLanePoints(void)
{
	static int acc_i = 0;
	printf("left_marker = %d, right_marker = %d\n", left_marker_num,right_marker_num);
	if(acc_p >1)
	{
		accumul_wp(left_world, &left_marker_num, acc_i, buf_left, buf_left_num);
		accumul_wp(right_world, &right_marker_num, acc_i, buf_right, buf_right_num);
	}
	printf("left_accumul_marker = %d, right_accumul_marker = %d\n", left_marker_num,right_marker_num);		
	acc_i++;
}

void CDetectLane::accumul_wp(wpoint *world, int *marker_num, int i, wpoint *buf_m, int *buf_m_num)
{
	int num = *marker_num;
	if (i<acc_p-1 || acc_p == 1)
	{
		for(int iter = 0; iter<num; iter++)
		{
			buf_m[i*MAX_MARKER_NUM + iter].x = world[iter].x;
			buf_m[i*MAX_MARKER_NUM + iter].z = world[iter].z;
		}
		buf_m_num[i] = num;
	}
	else
	{
		buf_m_num[acc_p-1] = 0;
		// acc_p 만큼의 data merging
		for(int jter = 0; jter<acc_p-1; jter++)
		{
			for(int iter = 0; iter<buf_m_num[jter]; iter++)
			{
				buf_m[(acc_p-1)*MAX_MARKER_NUM + buf_m_num[acc_p-1] + iter].x = buf_m[jter*MAX_MARKER_NUM + iter].x;
				buf_m[(acc_p-1)*MAX_MARKER_NUM + buf_m_num[acc_p-1] + iter].z = buf_m[jter*MAX_MARKER_NUM + iter].z;
			}
			buf_m_num[acc_p-1] = buf_m_num[acc_p-1] + buf_m_num[jter];
		}

		for(int iter = 0; iter<num; iter++)
		{
			buf_m[(acc_p-1)*MAX_MARKER_NUM + buf_m_num[acc_p-1] + iter].x = world[iter].x;
			buf_m[(acc_p-1)*MAX_MARKER_NUM + buf_m_num[acc_p-1] + iter].z = world[iter].z;
		}
		buf_m_num[acc_p-1] = buf_m_num[acc_p-1] + num;

		//Buf Data shift
		for(int jter = 0; jter<acc_p-2; jter++)
		{
			buf_m_num[jter] = buf_m_num[jter+1];
			for(int iter = 0; iter<buf_m_num[jter]; iter++)
			{
				buf_m[jter*MAX_MARKER_NUM + iter].x = buf_m[(jter+1)*MAX_MARKER_NUM + iter].x;
				buf_m[jter*MAX_MARKER_NUM + iter].z = buf_m[(jter+1)*MAX_MARKER_NUM + iter].z;
			}			
		}
		for(int iter = 0; iter<num; iter++)
		{
			buf_m[(acc_p-2)*MAX_MARKER_NUM + iter].x = world[iter].x;
			buf_m[(acc_p-2)*MAX_MARKER_NUM + iter].z = world[iter].z;
		}
		buf_m_num[acc_p-2] = num;
		
		//accumulation data padding
		for(int iter = 0; iter<buf_m_num[acc_p-1]; iter++)
		{
			world[iter].x = buf_m[(acc_p-1)*MAX_MARKER_NUM + iter].x;
			world[iter].z = buf_m[(acc_p-1)*MAX_MARKER_NUM + iter].z;
		}
		*marker_num = buf_m_num[acc_p-1];
	}
	
}

void CDetectLane::FitLane(void)
{
	/* ransac test를 위한 value 고정  */
	// left_marker_num = 3;
	// left_world[0].x = -1.7236;
	// left_world[1].x = -1.7167;
	// left_world[2].x = -1.7266;

	// left_world[0].z = 10.9652;
	// left_world[1].z = 11.5561;
	// left_world[2].z = 12.2149;
    if(left_imarker_num <=10 && left_marker_num<=10*acc_p){
		left_valid = -1;
	}
	else
	{
		left_valid=ransac_1stfit(left_world, left_marker_num,  left_coeff);
		//left_valid=polyfit(left_world, left_marker_num,  2, left_coeff);
		//left_valid=parabolic_fit(left_world, left_marker_num,  left_coeff);		
	}

	if(right_imarker_num <=10 && right_marker_num<=10*acc_p){
		right_valid = -1;
	}
	else
	{
		right_valid=ransac_1stfit(right_world, right_marker_num,  right_coeff);
		//right_valid=polyfit(right_world, right_marker_num,  2, right_coeff); 
		//right_valid=parabolic_fit(right_world, right_marker_num,  right_coeff);
	}

}

// polyfit 
//dependetValues=>y
//independentValues=>x
// y=ax^2+bx+c

int CDetectLane::polyfit(wpoint *points,
            int        countOfElements,
            int        order,
            float*             coefficients)

/*int polyfit(const float* const  dependentValues,
            const float* const independentValues,
            unsigned int        countOfElements,
            unsigned int        order,
            float*             coefficients)*/
{
    // Declarations...
    // ----------------------------------
    enum {maxOrder = 5};
    
    float B[maxOrder+1] = {0.0f};
    float P[((maxOrder+1) * 2)+1] = {0.0f};
    float A[(maxOrder + 1)*2*(maxOrder + 1)] = {0.0f};

    float x, y, powx;

    unsigned int ii, jj, kk;

    // Verify initial conditions....
    // ----------------------------------

    // This method requires that the countOfElements > 
    // (order+1) 
    if (countOfElements <= order)
        return -1;

    // This method has imposed an arbitrary bound of
    // order <= maxOrder.  Increase maxOrder if necessary.
    if (order > maxOrder)
        return -1;

    // Begin Code...
    // ----------------------------------

    // Identify the column vector
    for (ii = 0; ii < countOfElements; ii++)
    {
       // x    = dependentValues[ii];
       // y    = independentValues[ii];
		x = points[ii].z;
		y = points[ii].x;

        powx = 1;

        for (jj = 0; jj < (order + 1); jj++)
        {
            B[jj] = B[jj] + (y * powx);
            powx  = powx * x;
        }
    }

    // Initialize the PowX array
    P[0] = countOfElements;

    // Compute the sum of the Powers of X
    for (ii = 0; ii < countOfElements; ii++)
    {
       // x    = dependentValues[ii];
       // powx = dependentValues[ii];

		x = points[ii].z;
		powx=points[ii].z;

        for (jj = 1; jj < ((2 * (order + 1)) + 1); jj++)
        {
            P[jj] = P[jj] + powx;
            powx  = powx * x;
        }
    }

    // Initialize the reduction matrix
    //
    for (ii = 0; ii < (order + 1); ii++)
    {
        for (jj = 0; jj < (order + 1); jj++)
        {
            A[(ii * (2 * (order + 1))) + jj] = P[ii+jj];
        }

        A[(ii*(2 * (order + 1))) + (ii + (order + 1))] = 1;
    }

    // Move the Identity matrix portion of the redux matrix
    // to the left side (find the inverse of the left side
    // of the redux matrix
    for (ii = 0; ii < (order + 1); ii++)
    {
        x = A[(ii * (2 * (order + 1))) + ii];
        if (x != 0)
        {
            for (kk = 0; kk < (2 * (order + 1)); kk++)
            {
                A[(ii * (2 * (order + 1))) + kk] = 
                    A[(ii * (2 * (order + 1))) + kk] / x;
            }

            for (jj = 0; jj < (order + 1); jj++)
            {
                if ((jj - ii) != 0)
                {
                    y = A[(jj * (2 * (order + 1))) + ii];
                    for (kk = 0; kk < (2 * (order + 1)); kk++)
                    {
                        A[(jj * (2 * (order + 1))) + kk] = 
                            A[(jj * (2 * (order + 1))) + kk] -
                            y * A[(ii * (2 * (order + 1))) + kk];
                    }
                }
            }
        }
        else
        {
            // Cannot work with singular matrices
            return -1;
        }
    }

    // Calculate and Identify the coefficients
    for (ii = 0; ii < (order + 1); ii++)
    {
        for (jj = 0; jj < (order + 1); jj++)
        {
            x = 0;
            for (kk = 0; kk < (order + 1); kk++)
            {
                x = x + (A[(ii * (2 * (order + 1))) + (kk + (order + 1))] *
                    B[kk]);
            }
            coefficients[ii] = x;
        }
    }

    return 0;
}

int CDetectLane::parabolic_fit(wpoint *points,  int  N,  float *coeff)
{


#define n 2
  
	int i, j, k;
	int m;
	
	if(N<n+1) return -1;
   // cout<<"\nWhat degree of Polynomial do you want to use for the fit?\n";
                                  // n is the degree of Polynomial 
    double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	
    for (i=0;i<2*n+1;i++)
    {
        X[i]=0;
        for (j=0;j<N;j++)
            X[i]=X[i]+pow(points[j].z,i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }

	//조정 ax^2+ b = y  형태로
    double B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	
    for (i=0;i<=n;i++)
        for (j=0;j<=n;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    
	float Y[n+1];                      //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i=0;i<=n;i++)
    {    
        Y[i]=0;
        for (j=0;j<N;j++)
           Y[i]=Y[i]+pow(points[j].z,i)*points[j].x;        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }

    for (i=0;i<=n;i++)
        B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
                 
	for(i=0; i<n; i++)
		 B[i][1]=0;
	
   
     m=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
  
    for (i=0;i<m;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<m;k++)
            if (B[i][i]<B[k][i])
                for (j=0;j<=m;j++)
                {
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }
    
    for (i=0;i<m-1;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<m;k++)
            {
                double t=B[k][i]/B[i][i];
                for (j=0;j<=m;j++)
                    B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
    for (i=m-1;i>=0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][m];                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<m;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i]=a[i]-B[i][j]*a[j];
        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
		
    }

	coeff[0]=a[0];
	coeff[1]=a[1];
	//coeff[1] = 0.0;
	coeff[2]=a[2];

	return(0);
}

int CDetectLane::ransac_1stfit(wpoint *points,  int  markerNum,  float *coeff)
{
	int N = 10; //Number of Iteration
	double T = 0.3;//unit: meter
	int n_sample = 2, max_cnt = 0;
	int itr, i, j, k, k1, k2, cnt;
	int *in_k = new int[markerNum]; //최대 markerNum을 넘지 않으므로 
	double*A = new double[markerNum*2];
	double *B = new double[markerNum];
	double *residual = new double[markerNum];
	double X[2][1]={0,}, best_model[2][1]={0,};
	double AA[2][2]={0,}, BB[2][1]={0,};
	double invAA[2][2]={0,}, AX[2][1]={0,};
	double determinant, sum, mat[2][2]={0,},invmat[2][2]={0,};

	//build matrix
	for(itr = 0; itr<markerNum;itr++)
	{
		B[itr] = points[itr].x;
		A[itr*2] = points[itr].z;
		A[itr*2+1] = 1;	  
	}
		
	//random sampling -> 가능한 비복원 추출하도록... 
	srand(static_cast<unsigned int>(time((time_t*)NULL)));

	//Iteration
	for(itr = 0; itr<N; itr++)
	{
		k1 = rand() % static_cast<int>(markerNum-1) + 0;		
		k2 = rand() % static_cast<int>(markerNum-1) + 0;		

		if(k1 == k2)
		{
			k2 = rand() % static_cast<int>(markerNum-1) + 0;	
		}
		// //test 용
		// k1 = 0;
		// k2 = 2;
		
		//model estimation				
		AA[0][0]=points[k1].z;
		AA[0][1]=1;
		AA[1][0]=points[k2].z;
		AA[1][1]=1;
		BB[0][0] = points[k1].x;
		BB[1][0] = points[k2].x;
		
		//Inverse matrix of AA
		determinant=*(*(AA + 0) + 0)* *(*(AA + 1) + 1) - *(*(AA + 0) + 1)* *(*(AA + 1) + 0);
        invAA[0][0] = (1 / determinant)* *(*(AA + 1) + 1);
        invAA[0][1] = -(1 / determinant)* *(*(AA + 0) + 1);
        invAA[1][0] = -(1 / determinant)* *(*(AA + 1) + 0);
        invAA[1][1] = (1 / determinant)* *(*(AA + 0) + 0);

		//AA*X = BB -> X = invAA*BB		
		 for(i=0; i<2;i++){
			 for(j=0; j<1; j++){
				 sum = 0.0;
					 for(k=0; k<2; k++){
						sum +=invAA[i][k]*BB[k][j];	 	
					 }
				 X[i][j] = sum;
			 }
		 }

		//evaluation
		cnt=0;
		for(i = 0; i<markerNum;i++)
		{
			residual[i] = abs(B[i]- (A[i*2]*X[0][0]+A[i*2+1]*X[1][0]));
			if (residual[i]<T){
					cnt++;
			}
		}
		
		if(cnt>max_cnt){        
			best_model[0][0] = X[0][0];
            best_model[1][0] = X[1][0];
            max_cnt = cnt;
		}
	}
	coeff[0] = (float)best_model[0][0];
	coeff[1] = (float)best_model[1][0];

	/*  optional LS(Least Square) fitting			*/		
	
	cnt=0;	
	for(i = 0; i<markerNum;i++)
	{
		residual[i] = abs((A[i*2]*best_model[0][0] + A[i*2+1]*best_model[1][0]) - B[i]);
		if (residual[i]<T){
			in_k[cnt] = i; //index k의 값을 집어넣는다. 
			cnt++;
		}
	}
	//A2, B2의 동적 할당 및 값 지정
	double *A2 = new double[cnt*2];
	double *B2 = new double[cnt];
	double **pinvA2= new double*[2];
	double **A2T = new double*[2];
	for (itr = 0; itr<2; itr++){
		A2T[itr] = new double[cnt];
		pinvA2[itr] = new double[cnt];
		memset(A2T[itr], 0, sizeof(double)*cnt);
		memset(pinvA2[itr], 0, sizeof(double)*cnt);
	}

	int ptr;
	for (itr = 0; itr<cnt; itr++)
	{
		ptr = in_k[itr];
		B2[itr] = points[ptr].x;
		A2[itr*2] = points[ptr].z;
		A2[itr*2+1] = 1;	  
	}
	//A2의 pseudo inverse matrix
	//A2T 구하기
	for (itr = 0; itr<cnt; itr++)
	{
		A2T[0][itr] = A2[itr*2];
		A2T[1][itr] = A2[itr*2+1];
	}

	//mat = A2T*A2;
	for(i=0; i<2;i++){
		 for(j=0; j<2; j++){
			 sum = 0.0;
				 for(k=0; k<cnt; k++){
					sum +=A2T[i][k]*A2[k*2+j];	 	
				 }
			 mat[i][j] = sum;
		 }
	 }

	//inv mat
	determinant=*(*(mat + 0) + 0)* *(*(mat + 1) + 1) - *(*(mat + 0) + 1)* *(*(mat + 1) + 0);
    invmat[0][0] = (1 / determinant)* *(*(mat + 1) + 1);
    invmat[0][1] = -(1 / determinant)* *(*(mat + 0) + 1);
    invmat[1][0] = -(1 / determinant)* *(*(mat + 1) + 0);
    invmat[1][1] = (1 / determinant)* *(*(mat + 0) + 0);
	//invmat*A2T = pinvA2
	for(i=0; i<2;i++){
		for(j=0; j<cnt; j++){
			 sum = 0.0;
			 for(k=0; k<2; k++){
				sum +=invmat[i][k]*A2T[k][j];	 	
			 }
			 pinvA2[i][j] = sum;
		 }
	 }
	//pinvA2*B2 = Best_model
	 for(i=0; i<2;i++){
		 for(j=0; j<1; j++){
			 sum = 0.0;
			 for(k=0; k<cnt; k++){
				sum +=pinvA2[i][k]*B2[k];	 	
			 }
			best_model[i][j] = sum;
		 }
	 }

	coeff[1] = (float)best_model[0][0];
	coeff[0] = (float)best_model[1][0];

	for (itr = 0; itr<2; itr++){
		delete [] A2T[itr];
		delete [] pinvA2[itr];
	}
	delete [] A2T;
	delete [] pinvA2;

	delete []A;
	delete []B;
	delete []residual;
	delete []in_k;
	delete []A2;
	delete []B2;

	return(0);
}

int CDetectLane::ransac_fit(wpoint *points,  int  markerNum,  float *coeff)
{
	int N = 10; //Number of Iteration
	float T = 0.2;//unit: meter
	int n_sample = 3, max_cnt = 0;
	int itr, i, j, k1, k2, k3, cnt;
	int *in_k = new int[markerNum]; //최대 markerNum을 넘지 않으므로 
	float *A = new float[markerNum*3];
	float *B = new float[markerNum];
	float *residual = new float[markerNum];
	float X[3]={0,0,0}, best_model[3]={0,0,0}, AA[9]={0,}, BB[3]={0,0,0};
	float invAA[9]={0,};
	float determinant, sum, mat[9]={0,},mat_1[9]={0,},*invA;

	//build matrix
	for(itr = 0; itr<markerNum;itr++)
	{
		B[itr] = points[itr].x;
		A[itr*3] = points[itr].z*points[itr].z;
		A[itr*3+1] = points[itr].z;
		A[itr*3+2] = 1;	  
	}
		
	//random sampling -> 가능한 비복원 추출하도록... 
	srand(static_cast<unsigned int>(time((time_t*)NULL)));

	//Iteration
	for(itr = 0; itr<N; itr++)
	{
		k1 = rand() % static_cast<int>(markerNum-1) + 0;		
		k2 = rand() % static_cast<int>(markerNum-1) + 0;		
		k3 = rand() % static_cast<int>(markerNum-1) + 0;

		if(k1 == k2 || k1 == k3)
		{
			k1 = rand() % static_cast<int>(markerNum-1) + 0;	
		}
		else if(k2 == k3)
		{
			k2 = rand() % static_cast<int>(markerNum-1) + 0;	
		}
		//test 용
		//k1 = 12;
		//k2 = 6;
		//k3 = 5;

		//model estimation				
		AA[0]=points[k1].z*points[k1].z;
		AA[1]=points[k1].z;
		AA[2]=1;
		AA[3]=points[k2].z*points[k2].z;
		AA[4]=points[k2].z;
		AA[5]=1;
		AA[6]=points[k3].z*points[k3].z;
		AA[7]=points[k3].z;
		AA[8]=1;	
		BB[0] = points[k1].x;
		BB[1] = points[k2].x;
		BB[2] = points[k3].x;

		//Inverse matrix of AA
		//determinant=(((AA[0]*AA[4]*AA[8]+AA[1]*AA[5]*AA[6]+AA[2]*AA[3]*AA[7])-(AA[0]*AA[5]*AA[7]+AA[1]*AA[3]*AA[8]+AA[2]*AA[4]*AA[6])));
	    determinant=(double)((((double)AA[0]*(double)AA[4]*(double)AA[8]+(double)AA[1]*(double)AA[5]*(double)AA[6]+(double)AA[2]*(double)AA[3]*(double)AA[7])-((double)AA[0]*(double)AA[5]*(double)AA[7]+(double)AA[1]*(double)AA[3]*(double)AA[8]+(double)AA[2]*(double)AA[4]*(double)AA[6])));
		invAA[0]=(AA[4]*AA[8]-AA[5]*AA[7])/determinant;
		invAA[1]=(AA[2]*AA[7]-AA[1]*AA[8])/determinant;
		invAA[2]=(AA[1]*AA[5]-AA[2]*AA[4])/determinant;
		invAA[3]=(AA[5]*AA[6]-AA[3]*AA[8])/determinant;
		invAA[4]=(AA[0]*AA[8]-AA[2]*AA[6])/determinant;
		invAA[5]=(AA[2]*AA[3]-AA[0]*AA[5])/determinant;
		invAA[6]=(AA[3]*AA[7]-AA[4]*AA[6])/determinant;
		invAA[7]=(AA[1]*AA[6]-AA[0]*AA[7])/determinant;
		invAA[8]=(AA[0]*AA[4]-AA[1]*AA[3])/determinant;

		//AA*X = BB		
		X[0]=invAA[0]*BB[0]+invAA[1]*BB[1]+invAA[2]*BB[2];
		X[1]=invAA[3]*BB[0]+invAA[4]*BB[1]+invAA[5]*BB[2];
		X[2]=invAA[6]*BB[0]+invAA[7]*BB[1]+invAA[8]*BB[2];

		//evaluation
		cnt=0;
		for(i = 0; i<markerNum;i++)
		{
			residual[i] = abs(B[i]- (A[i*3]*X[0]+A[i*3+1]*X[1]+A[i*3+2]*X[2]));
			if (residual[i]<T){
					cnt++;
			}
		}
		
		if(cnt>max_cnt){        
			best_model[0] = X[0];
            best_model[1] = X[1];
            best_model[2] = X[2];
            max_cnt = cnt;
		}
	}
	coeff[0] = best_model[0];
	coeff[1] = best_model[1];
	coeff[2] = best_model[2];

	//  optional LS(Least Square) fitting				
	/*
	cnt=0;	
	for(i = 0; i<markerNum;i++)
	{
		residual[i] = abs((A[i*3]*best_model[0]+A[i*3+1]*best_model[1]+A[i*3+2]*best_model[2])-B[i]);
		if (residual[i]<T){
			in_k[cnt] = i; //index k의 값을 집어넣는다. 
			cnt++;
		}
	}
	//A2, B2의 동적 할당 및 값 지정
	float *A2 = new float[cnt*3];
	float *B2 = new float[cnt];
	float *pinvA2= new float[3*cnt];
	int ptr;
	for (itr = 0; itr<cnt; itr++)
	{
		ptr = in_k[itr];
		B2[itr] = points[ptr].x;
		A2[itr*3] = points[ptr].z*points[ptr].z;
		A2[itr*3+1] = points[ptr].z;
		A2[itr*3+2] = 1;	  
	}
	//A2의 pseudo inverse matrix
	for(i=0; i<3; i++){
		for(j=0;j<3; j++){
			sum=0;
			
			for(int count=0; count<cnt; count++){
				sum+=(float)A2[3*count+i]*(float)A2[3*count+j];
			}
			mat[i*3+j]=sum;

		}
	}
		
	determinant=(((mat[0]*mat[4]*mat[8]+mat[1]*mat[5]*mat[6]+mat[2]*mat[3]*mat[7])-(mat[0]*mat[5]*mat[7]+mat[1]*mat[3]*mat[8]+mat[2]*mat[4]*mat[6])));
	mat_1[0]=(mat[4]*mat[8]-mat[5]*mat[7])/determinant;
	mat_1[1]=(mat[2]*mat[7]-mat[1]*mat[8])/determinant;
	mat_1[2]=(mat[1]*mat[5]-mat[2]*mat[4])/determinant;
	mat_1[3]=(mat[5]*mat[6]-mat[3]*mat[8])/determinant;
	mat_1[4]=(mat[0]*mat[8]-mat[2]*mat[6])/determinant;
	mat_1[5]=(mat[2]*mat[3]-mat[0]*mat[5])/determinant;
	mat_1[6]=(mat[3]*mat[7]-mat[4]*mat[6])/determinant;
	mat_1[7]=(mat[1]*mat[6]-mat[0]*mat[7])/determinant;
	mat_1[8]=(mat[0]*mat[4]-mat[1]*mat[3])/determinant;

	for(i=0; i<3; i++){
		for(j=0; j<cnt; j++){
			sum=0;
			for(int count=0; count<3; count++){
				sum+=mat_1[i*3+count]*(float)A2[3*j+count];
			}			   
			pinvA2[i*cnt+j]=sum;
		}
	}
	
	//Best coefficient 값을 구함.
	for(i=0; i<3; i++){
		sum=0;
		for(int count=0; count<cnt; count++){
			sum += pinvA2[i*cnt+count]*(float)B2[count];
		}
			coeff[i]=sum;
	}
	*/

	delete []A;
	delete []B;
	delete []residual;
	delete []in_k;
	//delete []A2;
	//delete []B2;
	//delete []pinvA2;
	return(0);
}

void CDetectLane::TrackLane(void)
{
	//각 track point를 업데이트 하도록 한다. 
	//first, 마지막 포인트의 left, right value update
	//m_Track.plupdate(left_valid, right_valid, left_coeff,right_coeff);

	//second, 계수를 kalman filtering한다.
	m_Track.coeffupdate1st(left_valid, right_valid, left_coeff,right_coeff);
	//m_Track.coeffupdate(left_valid, right_valid, left_coeff,right_coeff);
	
}

void CDetectLane::UpdateConfidence(void)
{
	if(left_valid !=-1 && m_Track.m_confidence[0]>0){
		   printf("Left  = %.2f, Confidence:%d ",left_coeff[0], m_Track.m_confidence[0]);
	   }else{
			printf("Left  = 10000, Confidence:0 ");
			m_Track.m_confidence[0] = 0;
	   		m_Track._tracking_points_l[0] = m_Track._tracking_points_r[0]-3.5;
			m_Track._tracking_points_l[1]= m_Track._tracking_points_r[1]-3.5;;
			m_Track.GPP[0]= (m_Track._tracking_points_l[1] +m_Track._tracking_points_r[1] )/2;
	   }

	   if(right_valid !=-1 && m_Track.m_confidence[1]>0){
		   printf("Right  = %.2f, Confidence:%d ",right_coeff[0], m_Track.m_confidence[1]);
		}else{
			printf("Right  = 10000, Confidence:0 ");
			m_Track.m_confidence[1]= 0;
			m_Track._tracking_points_r[0] = m_Track._tracking_points_l[0]+3.5;
			m_Track._tracking_points_r[1]= m_Track._tracking_points_l[1]+3.5;;
			m_Track.GPP[0]= (m_Track._tracking_points_l[1] +m_Track._tracking_points_r[1] )/2;
	   }

	    if(left_valid !=-1 && m_Track.m_confidence[0]>0 &&right_valid !=-1 && m_Track.m_confidence[1]>0 ){
			 printf(" GPP =(%.2f, %.2f ) \n", m_Track.GPP[0], m_Track.GPP[1]);
		}else{
 			 printf(" GPP =( 0, 10 ) \n");
			 m_Track.GPP[0] = 0;
			 m_Track.GPP[1] = 10;
			 m_Track._tracking_points_l[0]=-1.7;
			 m_Track._tracking_points_l[1]=-1.7;
			 m_Track._tracking_points_r[0]=1.7;
			 m_Track._tracking_points_r[1]=1.7;
			 
		}
}

TrackingLane::TrackingLane()
{
	GPP[0] = 0;
	GPP[1] = 10;
	
	_nDetect[0]=0;
	_nDetect[1]=0;

	 m_confidence[0]=0;
	 m_confidence[1]=0;

	 _error_th_0 = 4.0; // near 0.3m
	_error_th_1 = 4.0; //far 0.5m
	 _z_max = 10;
	 _z_min = 0;

	 _confidence_th[0] =3;
	 _confidence_th[1] =6;
	 _confidence_th[2]= 9;
	 _confidence_th[3]= 12;
	 _confidence_th[4]= 15;
}
TrackingLane::~TrackingLane(){

}
void TrackingLane::updateLane(int lanevalid0, int lanevalid1, float *lanecoeff0, float *lanecoeff1)
{ 
	float x1, x2;
	float error0, error1;
	float alpha=0.7;
	
	if(lanevalid0 != -1){
		 x1=  lanecoeff0[2]*_z_min*_z_min+lanecoeff0[1]*_z_min + lanecoeff0[0];
		 x2 = lanecoeff0[2]*_z_max*_z_max+lanecoeff0[1]*_z_max + lanecoeff0[0];

		 if(_nDetect[0] ==0){
			 _tracking_points_l[0]=x1;
			 _tracking_points_l[1]=x2;
			 _nDetect[0]++;
		 }else{
			 error0 = abs(_tracking_points_l[0]-x1);
			 error1=  abs(_tracking_points_l[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				 _tracking_points_l[0] = alpha*_tracking_points_l[0] +(1-alpha) *x1;
				 _tracking_points_l[1] = alpha*_tracking_points_l[1] +(1-alpha) *x2;
				 _nDetect[0]++;

			 }else{
				  _nDetect[0]--;
			 }
		 }
	}

	if(lanevalid1 != -1){
		 x1=  lanecoeff1[2]*_z_min*_z_min+lanecoeff1[1]*_z_min + lanecoeff1[0];
		 x2 = lanecoeff1[2]*_z_max*_z_max+lanecoeff1[1]*_z_max + lanecoeff1[0];

		 if(_nDetect[1] ==0){
			 _tracking_points_r[0]=x1;
			 _tracking_points_r[1]=x2;
			 _nDetect[1]++;
			 _nDetect[1] = min(_nDetect[1], 15);
		 }else{
			 error0 = abs(_tracking_points_r[0]-x1);
			 error1=  abs(_tracking_points_r[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				 _tracking_points_r[0] = alpha*_tracking_points_r[0] +(1-alpha)* x1;
				 _tracking_points_r[1] = alpha*_tracking_points_r[1] +(1-alpha)* x2;
				 _nDetect[1]++;

			 }else{
				  _nDetect[1]--;
				  _nDetect[1] = max(_nDetect[1],0);
			 }
		 }
	}
	
	updateConfidence();

	if(lanevalid0 != -1 && lanevalid1 != -1){
		GPP[1]=_z_max; //10m

		 x1=  lanecoeff0[2]*GPP[1]*GPP[1]+lanecoeff0[1]*GPP[1] + lanecoeff0[0];
		 x2 = lanecoeff1[2]*GPP[1]*GPP[1]+lanecoeff1[1]*GPP[1] + lanecoeff1[0];
        
		//printf("%.2f, %.2f\n", lanecoeff0[0],lanecoeff1[0] );
		
		GPP[0]= (x1+x2)/2;
	}

}

void TrackingLane::plupdate(int lanevalid0, int lanevalid1, float *lanecoeff0, float *lanecoeff1)
{ 
	//재귀함수여서 static 변수가 많다.
	static float A=1, H=1, Q=0.7, R=4;
	float xp, Pp, K;
	static float xlf = -1.6, xll = -1.6, xrf=1.8, xrl=1.8; // x point의 초기값
	static float Plf = 6, Pll =6, Prf=6, Prl=6;
	//초기 값에 대한 정보가 전혀 없을 경우, 오차 공분산을 크게 잡는 것이 좋다.
	float x1, x2;
	float error0, error1;
	float alpha=0.7;
	
	//left lane points
	if(lanevalid0 != -1){
		 x1=  lanecoeff0[2]*_z_min*_z_min+lanecoeff0[1]*_z_min + lanecoeff0[0];
		 x2 = lanecoeff0[2]*_z_max*_z_max+lanecoeff0[1]*_z_max + lanecoeff0[0];

		 if(_nDetect[0] ==0){
			 _tracking_points_l[0]=x1;
			 _tracking_points_l[1]=x2;
			 _nDetect[0]++;
		 }else{
			 error0 = abs(_tracking_points_l[0]-x1);
			 error1=  abs(_tracking_points_l[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				 //_tracking_points_l[0] = alpha*_tracking_points_l[0] +(1-alpha) *x1;
				 //_tracking_points_l[1] = alpha*_tracking_points_l[1] +(1-alpha) *x2;
				 //left first point
				 xp = A*xlf;
				 Pp = A*Plf*A + Q;
				 K = (Pp*H)/(H*Pp*H + R);
				 xlf = xp + K*(x1-H*xp);
				 Plf = Pp - K*H*Pp;
				 _tracking_points_l[0] = xlf;
				 //left last point 
 				 xp = A*xll;
				 Pp = A*Pll*A + Q;
				 K = (Pp*H)/(H*Pp*H + R);
				 xll = xp + K*(x2-H*xp);
				 Pll = Pp - K*H*Pp;
				 _tracking_points_l[1] = xll;
				 _nDetect[0]++;

			 }else{
				  _nDetect[0]--;
			 }
		 }
	}

	//right lane points
	if(lanevalid1 != -1){
		 x1=  lanecoeff1[2]*_z_min*_z_min+lanecoeff1[1]*_z_min + lanecoeff1[0];
		 x2 = lanecoeff1[2]*_z_max*_z_max+lanecoeff1[1]*_z_max + lanecoeff1[0];

		 if(_nDetect[1] ==0){
			 _tracking_points_r[0]=x1;
			 _tracking_points_r[1]=x2;
			 _nDetect[1]++;
			 _nDetect[1] = min(_nDetect[1], 15);
		 }else{
			 error0 = abs(_tracking_points_r[0]-x1);
			 error1=  abs(_tracking_points_r[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				// _tracking_points_r[0] = alpha*_tracking_points_r[0] +(1-alpha)* x1;
				// _tracking_points_r[1] = alpha*_tracking_points_r[1] +(1-alpha)* x2;
				// right first point
				 xp = A*xrf;
				 Pp = A*Prf*A + Q;
				 K = (Pp*H)/(H*Pp*H + R);
				 xrf = xp + K*(x1-H*xp);
				 Prf = Pp - K*H*Pp;
				 _tracking_points_r[0] = xrf;
				 //right last point 
 				 xp = A*xrl;
				 Pp = A*Prl*A + Q;
				 K = (Pp*H)/(H*Pp*H + R);
				 xrl = xp + K*(x2-H*xp);
				 Prl = Pp - K*H*Pp;
				 _tracking_points_r[1] = xrl;
				 _nDetect[0]++;				
				 _nDetect[1]++;

			 }else{
				  _nDetect[1]--;
				  _nDetect[1] = max(_nDetect[1],0);
			 }
		 }
	}
	
	updateConfidence();

	if(lanevalid0 != -1 && lanevalid1 != -1){
		GPP[1]=_z_max; //12m

		// x1=  lanecoeff0[2]*GPP[1]*GPP[1]+lanecoeff0[1]*GPP[1] + lanecoeff0[0];
		// x2 = lanecoeff1[2]*GPP[1]*GPP[1]+lanecoeff1[1]*GPP[1] + lanecoeff1[0];
        
		//printf("%.2f, %.2f\n", lanecoeff0[0],lanecoeff1[0] );
		
		//GPP[0]= (x1+x2)/2;
		GPP[0]= (xll+xrl)/2;
	}

}

void TrackingLane::coeffupdate1st(int lanevalid0, int lanevalid1, float *lanecoeff0, float *lanecoeff1)
{ 
	//2차식일 경우의 계수 기반 kalman tracking
	double dt = 1.0/30.0;
	//재귀함수여서 static 변수가 많다.
	static double A[4][4]={1.0, dt, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, dt, 0, 0, 0, 1.0};
	double AT[4][4]={1.0, 0, 0, 0, dt, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, dt, 1.0};
	static double H[2][4]={1, 0, 0, 0, 0, 0, 1, 0}; 
	double HT[4][2]={1, 0, 0, 0, 0, 1, 0, 0}; 
	static double Q[4][4]={1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0};
	static double R[2][2]={50, 0, 0, 50};
	double xp[4][1] = {0,0,0,0};
	double Pp[4][4] = {0,};
	double K[4][2] = {0,};
	static double xl[4][1] = {0.0006, 0, 1.4, 0}; // x left point의 초기값 -1.4
	static double xr[4][1]={0.0006, 0, 1.4, 0}; // x right point의 초기값
	static double Pl[4][4] = {100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0};
	static double Pr[4][4] = {100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0};
	//초기 값에 대한 정보가 전혀 없을 경우, 오차 공분산을 크게 잡는 것이 좋다.
	float x1, x2;
	float error0, error1;
	int i, j, k;
	double sum;
	double temp[4][4] = {0,};
	double temp2[4][4] = {0,};
	double tmp1[4][2] = {0,};
	double tmp2[2][4] = {0,};
	double tmp3[2][2] = {0,};
	double tmp4[2][2] = {0,};
	double tmp5[2][1] = {0,};
    double determinant =0;
	double zl[2][1] = { (double)lanecoeff0[1],  (double)lanecoeff0[0]};
	double zr[2][1] = { (double)lanecoeff1[1],  (double)lanecoeff1[0]};
	//double z[2][1] = { (double)0.002,  (double)1.4};

	//left lane points
	if(lanevalid0 != -1){
		 x1=  lanecoeff0[1]*_z_min + lanecoeff0[0];
		 x2 = lanecoeff0[1]*_z_max + lanecoeff0[0];

		 if(_nDetect[0] ==0){
			 _tracking_points_l[0]=x1;
			 _tracking_points_l[1]=x2;
			 _nDetect[0]++;
		 }else{
			 error0 = abs(_tracking_points_l[0]-x1);
			 error1=  abs(_tracking_points_l[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				 //_tracking_points_l[0] = alpha*_tracking_points_l[0] +(1-alpha) *x1;
				 //_tracking_points_l[1] = alpha*_tracking_points_l[1] +(1-alpha) *x2;
				 //left first point
				 //xp = A*xl;
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*xl[k][j];	 	
						 }
						 xp[i][j] = sum;
					 }
				 }
				 //Pp = A*Pl*AT+ Q;
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*Pl[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*AT[k][j];	 	
						 }
						 Pp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pp[i][j] = Pp[i][j] + Q[i][j];
					 }
				 }
				 //K = (Pp*HT)*inv(H*Pp*HT + R);
				  for(i=0; i<4;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=Pp[i][k]*HT[k][j];	 	
						 }
						 tmp1[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*Pp[k][j];	 	
						 }
						 tmp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=tmp2[i][k]*HT[k][j];	 	
						 }
						 tmp3[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 tmp3[i][j] = tmp3[i][j] + R[i][j];
					 }
				 }		 

	//Inverse matrix of tmp3
		determinant=*(*(tmp3 + 0) + 0)* *(*(tmp3 + 1) + 1) - *(*(tmp3 + 0) + 1)* *(*(tmp3 + 1) + 0);
        tmp4[0][0] = (1 / determinant)* *(*(tmp3 + 1) + 1);
        tmp4[0][1] = -(1 / determinant)* *(*(tmp3 + 0) + 1);
        tmp4[1][0] = -(1 / determinant)* *(*(tmp3 + 1) + 0);
        tmp4[1][1] = (1 / determinant)* *(*(tmp3 + 0) + 0);
		
		 for(i=0; i<4;i++){
			 for(j=0; j<2; j++){
				 sum = 0.0;
					 for(k=0; k<2; k++){
						sum +=tmp1[i][k]*tmp4[k][j];	 	
					 }
				 K[i][j] = sum;
			 }
		 }

				 //xl = xp + K*(zl-H*xp);
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*xp[k][j];	 	
						 }
						 tmp5[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 tmp5[i][j] = zl[i][j] - tmp5[i][j];
					 }
				 }	
				  for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*tmp5[k][j];	 	
						 }
						 xl[i][j] = sum;
					 }
				 }		
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 xl[i][j] = xp[i][j] + xl[i][j];
					 }
				 }	
				 //Pl = Pp - K*H*Pp;
				for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*H[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*Pp[k][j];	 	
						 }
						 temp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pl[i][j] = Pp[i][j] - temp2[i][j];
					 }
				 }	
//계산 후, tracking points를 넘겨 줘야 한다.
			// if(lanecoeff0[0]<-2.5)
			// {
			// 	lanevalid0 = -1;	
			// 	 _nDetect[0]--;
			// 	 _nDetect[0] = max(_nDetect[0],0);
			// }
			// else{
				lanecoeff0[1] = xl[0][0];
				lanecoeff0[0] = xl[2][0];
				 _tracking_points_l[0] = lanecoeff0[1]*_z_min+ lanecoeff0[0];
				 _tracking_points_l[1] = lanecoeff0[1]*_z_max+ lanecoeff0[0];
				 _nDetect[0]++;
			// }
			 }else{
				  _nDetect[0]--;
				  _nDetect[0] = max(_nDetect[0],0);
			 }
		 }
	}

	//right lane points
	if(lanevalid1 != -1){
		 x1=  lanecoeff1[1]*_z_min + lanecoeff1[0];
		 x2 = lanecoeff1[1]*_z_max + lanecoeff1[0];

		 if(_nDetect[1] ==0){
			 _tracking_points_r[0]=x1;
			 _tracking_points_r[1]=x2;
			 _nDetect[1]++;
			 _nDetect[1] = min(_nDetect[1], 15);
		 }else{
			 error0 = abs(_tracking_points_r[0]-x1);
			 error1=  abs(_tracking_points_r[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				// _tracking_points_r[0] = alpha*_tracking_points_r[0] +(1-alpha)* x1;
				// _tracking_points_r[1] = alpha*_tracking_points_r[1] +(1-alpha)* x2;
				// right first point
				 //xp = A*xr;
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*xr[k][j];	 	
						 }
						 xp[i][j] = sum;
					 }
				 }
				 //	Pp = A*Pr*AT + Q;
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*Pr[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*AT[k][j];	 	
						 }
						 Pp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pp[i][j] = Pp[i][j] + Q[i][j];
					 }
				 }
				 //K = (Pp*HT)*inv(H*Pp*HT + R);
				  for(i=0; i<4;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=Pp[i][k]*HT[k][j];	 	
						 }
						 tmp1[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*Pp[k][j];	 	
						 }
						 tmp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=tmp2[i][k]*HT[k][j];	 	
						 }
						 tmp3[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 tmp3[i][j] = tmp3[i][j] + R[i][j];
					 }
				 }		 

	//Inverse matrix of tmp3
		determinant=*(*(tmp3 + 0) + 0)* *(*(tmp3 + 1) + 1) - *(*(tmp3 + 0) + 1)* *(*(tmp3 + 1) + 0);
        tmp4[0][0] = (1 / determinant)* *(*(tmp3 + 1) + 1);
        tmp4[0][1] = -(1 / determinant)* *(*(tmp3 + 0) + 1);
        tmp4[1][0] = -(1 / determinant)* *(*(tmp3 + 1) + 0);
        tmp4[1][1] = (1 / determinant)* *(*(tmp3 + 0) + 0);
		
		 for(i=0; i<4;i++){
			 for(j=0; j<2; j++){
				 sum = 0.0;
					 for(k=0; k<2; k++){
						sum +=tmp1[i][k]*tmp4[k][j];	 	
					 }
				 K[i][j] = sum;
			 }
		 }

				 //xr = xp + K*(zr-H*xp);
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*xp[k][j];	 	
						 }
						 tmp5[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 tmp5[i][j] = zr[i][j] - tmp5[i][j];
					 }
				 }	
				  for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*tmp5[k][j];	 	
						 }
						 xr[i][j] = sum;
					 }
				 }		
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 xr[i][j] = xp[i][j] + xr[i][j];
					 }
				 }	
				 //Pr = Pp - K*H*Pp;
				for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*H[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*Pp[k][j];	 	
						 }
						 temp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pr[i][j] = Pp[i][j] - temp2[i][j];
					 }
				 }					
				//계산 후, tracking points를 넘겨 줘야 한다.
				// if(lanecoeff1[0]>2.5)
				// {
				// 	lanevalid1 = -1;
				// 	 _nDetect[1]--;
				// 	 _nDetect[1] = max(_nDetect[1],0);
				// }
				// else{
				lanecoeff1[1] = xr[0][0];
				lanecoeff1[0] = xr[2][0];
				 _tracking_points_r[0] = lanecoeff1[1]*_z_min+ lanecoeff1[0];
				 _tracking_points_r[1] = lanecoeff1[1]*_z_max+ lanecoeff1[0];
				 _nDetect[1]++;
				// }

			 }else{
				  _nDetect[1]--;
				  _nDetect[1] = max(_nDetect[1],0);
			 }
		 }
	}
	
	updateConfidence();

	if(lanevalid0 != -1 && lanevalid1 != -1){
		GPP[1]=_z_max; //12m

       //GPP[0]= (x1+x2)/2;
		GPP[0]= (_tracking_points_l[1] +_tracking_points_r[1] )/2;
	}

}


void TrackingLane::coeffupdate(int lanevalid0, int lanevalid1, float *lanecoeff0, float *lanecoeff1)
{ 
	//2차식일 경우의 계수 기반 kalman tracking
	double dt = 1.0/30.0;
	//재귀함수여서 static 변수가 많다.
	static double A[4][4]={1.0, dt, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, dt, 0, 0, 0, 1.0};
	double AT[4][4]={1.0, 0, 0, 0, dt, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, dt, 1.0};
	static double H[2][4]={1, 0, 0, 0, 0, 0, 1, 0}; 
	double HT[4][2]={1, 0, 0, 0, 0, 1, 0, 0}; 
	static double Q[4][4]={1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0};
	static double R[2][2]={50, 0, 0, 50};
	double xp[4][1] = {0,0,0,0};
	double Pp[4][4] = {0,};
	double K[4][2] = {0,};
	static double xl[4][1] = {0.0006, 0, 1.4, 0}; // x left point의 초기값 -1.4
	static double xr[4][1]={0.0006, 0, 1.4, 0}; // x right point의 초기값
	static double Pl[4][4] = {100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0};
	static double Pr[4][4] = {100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0};
	//초기 값에 대한 정보가 전혀 없을 경우, 오차 공분산을 크게 잡는 것이 좋다.
	float x1, x2;
	float error0, error1;
	int i, j, k;
	double sum;
	double temp[4][4] = {0,};
	double temp2[4][4] = {0,};
	double tmp1[4][2] = {0,};
	double tmp2[2][4] = {0,};
	double tmp3[2][2] = {0,};
	double tmp4[2][2] = {0,};
	double tmp5[2][1] = {0,};
    double determinant =0;
	double zl[2][1] = { (double)lanecoeff0[2],  (double)lanecoeff0[0]};
	double zr[2][1] = { (double)lanecoeff1[2],  (double)lanecoeff1[0]};
	//double z[2][1] = { (double)0.002,  (double)1.4};

	//left lane points
	if(lanevalid0 != -1){
		 x1=  lanecoeff0[2]*_z_min*_z_min+lanecoeff0[1]*_z_min + lanecoeff0[0];
		 x2 = lanecoeff0[2]*_z_max*_z_max+lanecoeff0[1]*_z_max + lanecoeff0[0];

		 if(_nDetect[0] ==0){
			 _tracking_points_l[0]=x1;
			 _tracking_points_l[1]=x2;
			 _nDetect[0]++;
		 }else{
			 error0 = abs(_tracking_points_l[0]-x1);
			 error1=  abs(_tracking_points_l[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				 //_tracking_points_l[0] = alpha*_tracking_points_l[0] +(1-alpha) *x1;
				 //_tracking_points_l[1] = alpha*_tracking_points_l[1] +(1-alpha) *x2;
				 //left first point
				 //xp = A*xl;
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*xl[k][j];	 	
						 }
						 xp[i][j] = sum;
					 }
				 }
				 //Pp = A*Pl*AT+ Q;
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*Pl[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*AT[k][j];	 	
						 }
						 Pp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pp[i][j] = Pp[i][j] + Q[i][j];
					 }
				 }
				 //K = (Pp*HT)*inv(H*Pp*HT + R);
				  for(i=0; i<4;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=Pp[i][k]*HT[k][j];	 	
						 }
						 tmp1[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*Pp[k][j];	 	
						 }
						 tmp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=tmp2[i][k]*HT[k][j];	 	
						 }
						 tmp3[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 tmp3[i][j] = tmp3[i][j] + R[i][j];
					 }
				 }		 

	//Inverse matrix of tmp3
		determinant=*(*(tmp3 + 0) + 0)* *(*(tmp3 + 1) + 1) - *(*(tmp3 + 0) + 1)* *(*(tmp3 + 1) + 0);
        tmp4[0][0] = (1 / determinant)* *(*(tmp3 + 1) + 1);
        tmp4[0][1] = -(1 / determinant)* *(*(tmp3 + 0) + 1);
        tmp4[1][0] = -(1 / determinant)* *(*(tmp3 + 1) + 0);
        tmp4[1][1] = (1 / determinant)* *(*(tmp3 + 0) + 0);
		
		 for(i=0; i<4;i++){
			 for(j=0; j<2; j++){
				 sum = 0.0;
					 for(k=0; k<2; k++){
						sum +=tmp1[i][k]*tmp4[k][j];	 	
					 }
				 K[i][j] = sum;
			 }
		 }

				 //xl = xp + K*(zl-H*xp);
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*xp[k][j];	 	
						 }
						 tmp5[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 tmp5[i][j] = zl[i][j] - tmp5[i][j];
					 }
				 }	
				  for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*tmp5[k][j];	 	
						 }
						 xl[i][j] = sum;
					 }
				 }		
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 xl[i][j] = xp[i][j] + xl[i][j];
					 }
				 }	
				 //Pl = Pp - K*H*Pp;
				for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*H[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*Pp[k][j];	 	
						 }
						 temp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pl[i][j] = Pp[i][j] - temp2[i][j];
					 }
				 }	
//계산 후, tracking points를 넘겨 줘야 한다.
				lanecoeff0[2] = xl[0][0];
				lanecoeff0[1] = 0;
				lanecoeff0[0] = xl[2][0];
				 _tracking_points_l[0] = lanecoeff0[2]*_z_min*_z_min+ lanecoeff0[0];
				 _tracking_points_l[1] = lanecoeff0[2]*_z_max*_z_max+ lanecoeff0[0];
				 _nDetect[0]++;

			 }else{
				  _nDetect[0]--;
			 }
		 }
	}

	//right lane points
	if(lanevalid1 != -1){
		 x1=  lanecoeff1[2]*_z_min*_z_min+lanecoeff1[1]*_z_min + lanecoeff1[0];
		 x2 = lanecoeff1[2]*_z_max*_z_max+lanecoeff1[1]*_z_max + lanecoeff1[0];

		 if(_nDetect[1] ==0){
			 _tracking_points_r[0]=x1;
			 _tracking_points_r[1]=x2;
			 _nDetect[1]++;
			 _nDetect[1] = min(_nDetect[1], 15);
		 }else{
			 error0 = abs(_tracking_points_r[0]-x1);
			 error1=  abs(_tracking_points_r[1]-x2);

			 if( (error0< _error_th_0) && (error1 < _error_th_1))
			 { 
				// _tracking_points_r[0] = alpha*_tracking_points_r[0] +(1-alpha)* x1;
				// _tracking_points_r[1] = alpha*_tracking_points_r[1] +(1-alpha)* x2;
				// right first point
				 //xp = A*xr;
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*xr[k][j];	 	
						 }
						 xp[i][j] = sum;
					 }
				 }
				 //	Pp = A*Pr*AT + Q;
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=A[i][k]*Pr[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*AT[k][j];	 	
						 }
						 Pp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pp[i][j] = Pp[i][j] + Q[i][j];
					 }
				 }
				 //K = (Pp*HT)*inv(H*Pp*HT + R);
				  for(i=0; i<4;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=Pp[i][k]*HT[k][j];	 	
						 }
						 tmp1[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*Pp[k][j];	 	
						 }
						 tmp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=tmp2[i][k]*HT[k][j];	 	
						 }
						 tmp3[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<2; j++){
						 tmp3[i][j] = tmp3[i][j] + R[i][j];
					 }
				 }		 

	//Inverse matrix of tmp3
		determinant=*(*(tmp3 + 0) + 0)* *(*(tmp3 + 1) + 1) - *(*(tmp3 + 0) + 1)* *(*(tmp3 + 1) + 0);
        tmp4[0][0] = (1 / determinant)* *(*(tmp3 + 1) + 1);
        tmp4[0][1] = -(1 / determinant)* *(*(tmp3 + 0) + 1);
        tmp4[1][0] = -(1 / determinant)* *(*(tmp3 + 1) + 0);
        tmp4[1][1] = (1 / determinant)* *(*(tmp3 + 0) + 0);
		
		 for(i=0; i<4;i++){
			 for(j=0; j<2; j++){
				 sum = 0.0;
					 for(k=0; k<2; k++){
						sum +=tmp1[i][k]*tmp4[k][j];	 	
					 }
				 K[i][j] = sum;
			 }
		 }

				 //xr = xp + K*(zr-H*xp);
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=H[i][k]*xp[k][j];	 	
						 }
						 tmp5[i][j] = sum;
					 }
				 }		
				 for(i=0; i<2;i++){
					 for(j=0; j<1; j++){
						 tmp5[i][j] = zr[i][j] - tmp5[i][j];
					 }
				 }	
				  for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*tmp5[k][j];	 	
						 }
						 xr[i][j] = sum;
					 }
				 }		
				 for(i=0; i<4;i++){
					 for(j=0; j<1; j++){
						 xr[i][j] = xp[i][j] + xr[i][j];
					 }
				 }	
				 //Pr = Pp - K*H*Pp;
				for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<2; k++){
							sum +=K[i][k]*H[k][j];	 	
						 }
						 temp[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 sum = 0.0;
						 for(k=0; k<4; k++){
							sum +=temp[i][k]*Pp[k][j];	 	
						 }
						 temp2[i][j] = sum;
					 }
				 }
				 for(i=0; i<4;i++){
					 for(j=0; j<4; j++){
						 Pr[i][j] = Pp[i][j] - temp2[i][j];
					 }
				 }					
				//계산 후, tracking points를 넘겨 줘야 한다.
				lanecoeff1[2] = xr[0][0];
				lanecoeff1[1] = 0;
				lanecoeff1[0] = xr[2][0];
				 _tracking_points_r[0] = lanecoeff1[2]*_z_min*_z_min+ lanecoeff1[0];
				 _tracking_points_r[1] = lanecoeff1[2]*_z_max*_z_max+ lanecoeff1[0];
				 _nDetect[1]++;

			 }else{
				  _nDetect[1]--;
				  _nDetect[1] = max(_nDetect[1],0);
			 }
		 }
	}
	
	updateConfidence();

	if(lanevalid0 != -1 && lanevalid1 != -1){
		GPP[1]=_z_max; //12m

       //GPP[0]= (x1+x2)/2;
		GPP[0]= (_tracking_points_l[1] +_tracking_points_r[1] )/2;
	}

}

void TrackingLane::updateConfidence()
{
	for (int i=0; i<2; i++){
		if(_nDetect[i] > _confidence_th[ m_confidence[i] ]){
			m_confidence[i]++;
			m_confidence[i] = min(4, m_confidence[i]);
		}else{
			m_confidence[i]=m_confidence[i];
		}
		
	}
}
	 
	