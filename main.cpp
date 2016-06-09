				// -----------------SIDE CAM-------------------- //

#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include "blob_extrms.h"	// returns extrema(min. & max. x&y) points of each blob
//#include "framediff.h"
#include "tserial.cpp"
int* label1=new int[640*480];
int labelxy(IplImage* image)
{
int ht,w,c1=0,c2=0,c3=0,count=0;
ht=image->height;
w=image->width;
//int* label1=new int[ht*w];
for(int x=0;x<(ht*w);x++)
	label1[x]=0;
for(int y=1;y<ht;y++)
{
	uchar* ptr=(uchar*)(image->imageData+y*image->widthStep);
	for(int x=1;x<w;x++)
	{
		c1=label1[ht*(x-1)+y];
		c2=label1[ht*x+y-1];
		c3=label1[ht*x+y];
		//printf("%d",c3);
		if(*ptr==255)
		{
			if(c1!=0)
			{
				if(c2!=0)
				{
					label1[ht*x+y]=(c1<=c2)? c1:c2;
					if(c3==c2)
						label1[ht*(x-1)+y]=c2;
					else
						label1[ht*x+y-1]=c1;
				}
				else
					label1[ht*x+y]=c1;
			}
			else
			{
				if(c2!=0)
					label1[ht*x+y]=c2;
				else
				{
					count++;
					label1[ht*x+y]=count;
				}
			}
		}
		ptr++;
	}
}
for(int y=ht-2;y>=0;y--)
{
	for(int x=w-2;x>=0;x--)
	{
		c2=label1[ht*(x+1)+y];
		c3=label1[ht*x+y+1];
		c1=label1[ht*x+y];
		if(c1!=0)
		{
			if(c2!=0 && c1>c2)
			{
				if(c3!=0 && c1>c3)
				{
					label1[ht*x+y]=(c2<=c3)?c2:c3;
					if(c1==c2)
						label1[ht*x+y+1]=c2;
					else
						label1[ht*(x+1)+y]=c3;
				}
				else
					label1[ht*x+y]=c2;
			}
			else
			{
				if(c3!=0 && c1>c3)
					label1[ht*x+y]=c3;
			}
		}
	}
}
return count;
}
int centroid(IplImage* image, int count,long* sx,long* sy,long* c)
{
int ht,w,c1=0,c2=0,c3=0,counsp=0;
ht=image->height;
w=image->width;
for(int x=0;x<count;x++)
	{sx[x]=0;
	sy[x]=0;
	c[x]=0;}
for(int y=0;y<ht;y++)
{
	for(int x=0;x<w;x++)
	{
		c1=label1[ht*x+y];
		if(c1!=0)
		{
			sy[c1]+=y;
			sx[c1]+=x;
			c[c1]++;
			//if(c[maxc]<c[c1]) maxc=c1;
		}
	}
}
CvPoint p;
for(int z=0;z<count;z++)
{
	if(c[z]>4 && c[z]<400)
	{
		sx[z]/=c[z];
		sy[z]/=c[z];
		//printf("X=%d  &&  Y=%d\n",sx[z],sy[z]);
		p=cvPoint(sx[z],sy[z]);
		cvCircle(image,p,2,cvScalar(122),2);
		counsp++;
	}
}
return counsp;
}
void rectfill(IplImage* image,CvPoint p,CvPoint q,CvScalar s)
{
	for(int i=(p.y);i<(q.y);i++)
	{
		uchar* ptr= (uchar*)(image->imageData+i*image->widthStep);
		for(int j=(p.x);j<(q.x);j++)
		{
			ptr[3*j]=(s.val[0]);
			ptr[3*j+1]=(s.val[1]);
			ptr[3*j+2]=(s.val[2]);
		}
	}
}
#define HORIZON 300		//check for front camera ---> set this val. acc. to the our bot when it is closest
	// ***the below two values must also reflect in "bloborig.h" 
#define CRK_AMIN 4
#define CRK_AMAX 450	//check for front camera
#define FDIFF_THRESH 20
#define FRAME 4
#define VAL_A_THRESH 0.0001		//(threshold value for meana) check for front camera
#define PEAK_THRESH 400			//(threshhold value of the parabolic peak) check for front camera
#define thresH 50
#define threshL 1
#define THRESH_M 1000			//(thresh for the slope of st. line trajectory)
#define THRESH_D 1000			//(thresh for the intercept of st. line trajectory)
int MAX_BLOB=50;  // lower this value reasonably for optimizing framediff  (achieved faster framediff running time)

IplImage* framediff(CvCapture* g_capture,CvPoint* pixel,char* win)
{
//cvNamedWindow( win, CV_WINDOW_AUTOSIZE );
/*CvCapture* g_capture = cvCreateCameraCapture(1);
cvSetCaptureProperty(g_capture,CV_CAP_PROP_FPS,120);
cvSetCaptureProperty(g_capture,CV_CAP_PROP_FRAME_WIDTH,640);
cvSetCaptureProperty(g_capture,CV_CAP_PROP_FRAME_HEIGHT,480);*/

//CvCapture* g_capture = cvCreateFileCapture("f:/$robominton/Badminton2.avi");
//CvCapture* g_capture = cvCreateFileCapture("f:/$robominton/bad.mp4");
IplImage* frame1;
IplImage* frame,*imgA;
//cvWaitKey(1);
do{
 frame1=cvQueryFrame(g_capture);
}while(frame1==NULL);
do {
frame=cvQueryFrame(g_capture);
}while(frame==NULL);
CvSize s=cvGetSize(frame);
int tlX=s.height;
int tlY=s.width;
IplImage* frame3=cvCreateImage(s,frame->depth,1);
cvCvtColor(frame,frame3,CV_BGR2GRAY);
	IplImage* diff=cvCreateImage(s,frame->depth,1);
	double t=0,k=0;
	//int* label1;
long count=0;
long *sx,*sy,*cxy,couns=0,*sx1,*sy1,*c1;
//float thresh_m=1000,thresh_d=1000;
//CvPoint* pixel=new CvPoint[4];
float meanm,meand;
/*for(int ln=0;ln<9;ln++)
	pixel[ln]=cvPoint(0,0);*/
CvPoint2D32f* P=new CvPoint2D32f[MAX_BLOB*FRAME];
CvPoint2D32f* S=new CvPoint2D32f[FRAME];
for(int i=0;i<FRAME;i++)
{
	for(int j=0;j<MAX_BLOB;j++)
		P[FRAME*j+i]=cvPoint2D32f(1000,1000);
}
for(int i=0;i<FRAME;i++)
{
		S[i]=cvPoint2D32f(0,0);
}
//int thresH=100,threshL=1;
float a[FRAME-1];
float b[FRAME-1];
float c[FRAME-1];
float meanc=0,meana=0,meanb=0,peak=0;
int fr=0;
//IplImage* frame1;
IplImage* frame4=cvCreateImage(s,frame->depth,1);
while(1)
{
	//t=(double)cvGetTickCount();
	fr++;
	//printf("%lf\n",k);
	frame1=cvQueryFrame(g_capture);
	if(frame1==NULL)break;
	//IplImage* frame4=cvCreateImage(s,frame->depth,1);
	cvCvtColor(frame1,frame4,CV_BGR2GRAY);
	imgA=cvCloneImage(frame4);
	cvSetImageROI(frame3,cvRect(0,0,639,HORIZON));
	cvSetImageROI(frame4,cvRect(0,0,639,HORIZON));
	cvSetImageROI(diff,cvRect(0,0,639,HORIZON));
	cvAbsDiff(frame3, frame4, diff);
	cvResetImageROI(frame3);
	cvResetImageROI(frame4);
	cvResetImageROI(diff);
	cvThreshold(diff,diff,45,255,CV_THRESH_BINARY);
	//printf("label1\n");
//label1=(int*)malloc(tlX*tlY*sizeof(int));
//printf("label12\n");
count=labelxy(diff);  // labelling the image
sx=new long[count];
sy=new long[count];
//printf("\n%d\n",count);
cxy=(long*)malloc(count*sizeof(long));
couns=centroid(diff,count,sx,sy,cxy);  // centroid of each blob
sx1=new long[couns]; //COUNS
sy1=new long[couns];
c1=new long[couns];
if(couns>MAX_BLOB)    // condition added to avoid/filter noisy images with too much blobs & also avoids access violation error
		{
			//printf("3\n");
			//cvReleaseImage(&frame1);cvReleaseImage(&frame4);
			//cvReleaseImage(&frame);cvReleaseImage(&frame3);cvReleaseImage(&diff);
			//free( cxy);free( sy);free( sx);
			//free( sy1);free( sx1);free( c1);
			//free( label1);
			frame3=cvCloneImage(frame4);
			continue;
		}
//free(label1);
//printf("label1 freed\n");
int f1=0;
for(int i=0;i<count;i++)
{
	if(cxy[i]>CRK_AMIN && cxy[i]<CRK_AMAX)
	{
		sx1[f1]=sx[i];
		sy1[f1]=sy[i];
		c1[f1]=cxy[i];
		f1++;
	}
}

// storing of centroids
for(int i=FRAME-1;i>0;i--)
{
	for(int j=0;j<MAX_BLOB;j++)
		P[FRAME*j+i]=P[FRAME*j+i-1];
}
for(int i=0;i<MAX_BLOB;i++)
{
	if(i<couns)
		P[FRAME*i]=cvPoint2D32f(sx1[i],sy1[i]);
	else
		P[FRAME*i]=cvPoint2D32f(1000,1000);
}
float mind=0,dist=0;
int aj,m=0,J=1;
for(int i=0;i<FRAME;i++)
{
		S[i]=cvPoint2D32f(0,0);
}
//printf("\n%d",fr);
for(int i=0;i<couns;i++)
{
	m=i;
	S[0]=P[FRAME*i];
	J=1;
	for(int k=1;k<FRAME;k++)
	{
	aj=0;
	mind=sqrt(pow((P[FRAME*m+k-1].x-P[k].x),2)+pow((P[FRAME*m+k-1].y-P[k].y),2));
	for(int j=1;j<MAX_BLOB;j++)  // find adj nearest point
	{
		dist=sqrt(pow((P[FRAME*m+k-1].x-P[FRAME*j+k].x),2)+pow((P[FRAME*m+k-1].y-P[FRAME*j+k].y),2));
		if(dist<mind)
		{
			mind=dist;
			aj=j;
		}
	}
	if(mind<=thresH && mind>threshL)// && abs(P[6*aj+k-1].y-P[6*aj+k].y)>0)  // connect the dots...
	{
		m=aj;
		//count++;
		S[J]=P[FRAME*m+k];
		J++;
	}
	else
	{
		S[J]=cvPoint2D32f(0,0);
		break;
	}
	} // for 6 tyms
	if(J==FRAME)       // if found 6 connected dots..
	{
		for(int l=0;l<(FRAME-2);l++)
		{
		a[l]= (S[l].x*(S[l+2].y-S[l+1].y)+S[l+2].x*(S[l+1].y-S[l].y)+S[l+1].x*(S[l].y-S[l+2].y))/((S[l].x-S[l+1].x)*(S[l+1].x-S[l+2].x)*(S[l].x-S[l+2].x));
		b[l]= ((S[l].y-S[l+1].y)/(S[l].x-S[l+1].x)-a[l]*(S[l].x+S[l+1].x));
		c[l]= S[l].y-a[l]*S[l].x*S[l].x-b[l]*S[l].x;
		//printf("\n %f",a[l]);
		}
		meana=0;
		meanb=0;
		meanc=0;
		for(int l=0;l<(FRAME-2);l++)
		{
			meana+=a[l];
			meanb+=b[l];
			meanc+=c[l];
		}
		meana/=(FRAME-2);
		meanb/=(FRAME-2);
		meanc/=(FRAME-2);	
		
		//printf("\n (%f,%f,%f)",fr,a[1],b[1],meanc);
		peak=-1*(4*meana*meanc-meanb*meanb)/(2*meana);
		//printf("\n-------mean_a=%f: peak=%f ------\n",meana,peak);
		
		if(meana>VAL_A_THRESH && peak<PEAK_THRESH) // if 6 cnncted points are parabolic..  0.0001   400
		{
			printf("\n Meana = %f \n",meana);
			pixel[0]=cvPoint(S[0].x,S[0].y);
			pixel[1]=cvPoint(S[1].x,S[1].y);
			pixel[2]=cvPoint(S[2].x,S[2].y);
			pixel[3]=cvPoint(S[3].x,S[3].y);
			/*//cvReleaseImage(&frame);
            cvReleaseImage(&diff);
            cvReleaseImage(&frame3);
			cvReleaseImage(&frame4);
			//cvReleaseCapture(&g_capture);
			//cvReleaseImage(&frame);
			free(sx1);
			//free(sx);
			free(sy1);
			realloc(P,0);
			//realloc(S,0);
			//realloc(sy,0);
			//free(cxy);
			free(c1);
			//imgA=frame4;*/
			//cvWaitKey(1);
			//cvShowImage(win,imgA);
            return imgA;
            cvDestroyWindow( "Player3" );
			break;
		}
		meanm=0;
		meand=0;
		for(int l=0;l<FRAME-1;l++)  //start linear segmentation
		{
			a[l]=(S[l+1].y-S[l].y)/(S[l+1].x-S[l].x);
			b[l]=((S[l+1].x * S[l].y)-(S[l].x * S[l+1].y))/(S[l+1].x-S[l].x);
			meanm+=a[l];
			meand+=b[l];
		}
		meanm=fabs(meanm/(FRAME-1));
		meand=fabs(meand/(FRAME-1));
		if(meanm>THRESH_M && meand>THRESH_D)
		{   
			printf("\n Linear segmentation successful\n");
			pixel[0]=cvPoint(S[0].x,S[0].y);
			pixel[1]=cvPoint(S[1].x,S[1].y);
			pixel[2]=cvPoint(S[2].x,S[2].y);
			pixel[3]=cvPoint(S[3].x,S[3].y);
		  return imgA;
		}
		

	}

}
cvShowImage( win, diff );
	frame3=cvCloneImage(frame4);
    if(cvWaitKey(1)==' ')
		cvWaitKey(0);
	
	//delete label1;
	//k=(((double)cvGetTickCount()-t)/(cvGetTickFrequency()*1000))-13;
}
cvReleaseImage(&frame);
cvReleaseImage(&frame1);
cvReleaseImage(&frame3);
cvDestroyWindow( "Player3" );
return imgA;
}
int* label=new int[640*480];
int labelx(IplImage* image)
{
int ht,w,c1=0,c2=0,c3=0,count=0;
ht=image->height;
w=image->width;
//int* label=new int[ht*w];
for(int x=0;x<(ht*w);x++)
	label[x]=0;
for(int y=1;y<ht;y++)
{
	uchar* ptr=(uchar*)(image->imageData+y*image->widthStep);
	for(int x=1;x<w;x++)
	{
		c1=label[ht*(x-1)+y];
		c2=label[ht*x+y-1];
		c3=label[ht*x+y];
		//printf("%d",c3);
		if(*ptr==255)
		{
			if(c1!=0)
			{
				if(c2!=0)
				{
					label[ht*x+y]=(c1<=c2)? c1:c2;
					if(c3==c2)
						label[ht*(x-1)+y]=c2;
					else
						label[ht*x+y-1]=c1;
				}
				else
					label[ht*x+y]=c1;
			}
			else
			{
				if(c2!=0)
					label[ht*x+y]=c2;
				else
				{
					count++;
					label[ht*x+y]=count;
				}
			}
		}
		ptr++;
	}
}
for(int y=ht-2;y>=0;y--)
{
	for(int x=w-2;x>=0;x--)
	{
		c2=label[ht*(x+1)+y];
		c3=label[ht*x+y+1];
		c1=label[ht*x+y];
		if(c1!=0)
		{
			if(c2!=0 && c1>c2)
			{
				if(c3!=0 && c1>c3)
				{
					label[ht*x+y]=(c2<=c3)?c2:c3;
					if(c1==c2)
						label[ht*x+y+1]=c2;
					else
						label[ht*(x+1)+y]=c3;
				}
				else
					label[ht*x+y]=c2;
			}
			else
			{
				if(c3!=0 && c1>c3)
					label[ht*x+y]=c3;
			}
		}
	}
}
return count;
}
void centroid(IplImage* image, int count,CvPoint2D32f* cent,long* c,CvPoint2D32f* mins=NULL,CvPoint2D32f* maxs=NULL)
{
	float* sx=new float[count];
	float* sy=new float[count];
	//long* c=new long[count];
int ht,w,c1=0,c2=0,c3=0;
ht=image->height;
w=image->width;
for(int x=0;x<count;x++)  //initialise...
{	
	sx[x]=0;
	sy[x]=0;
	c[x]=0;
	mins[x].x=640;
	maxs[x].x=0;
	mins[x].y=480;
	maxs[x].y=0;
}
for(int y=0;y<ht;y++)
{
	for(int x=0;x<w;x++)
	{
		c1=label[ht*x+y];
		if(c1!=0)
		{
			sy[c1]+=x; //Summation x
			sx[c1]+=y; //Summation y
			c[c1]++; //increment area
			//if(c[maxc]<=c[c1]) {maxc=c1; max2=(int)c[maxc];}
			if(mins[c1].x>=x) {mins[c1].x=x;}
			if(mins[c1].y>=y) {mins[c1].y=y;}
			if(maxs[c1].x<=x) {maxs[c1].x=x;}
			if(maxs[c1].y<=y) {maxs[c1].y=y;}
		}
	}
}
/*
int k=0,k1,m1,m=0;
	char ch1[7]="000 000";
	CvFont f;
	cvInitFont(&f,1,1,1);*/
//CvPoint p;
//q=cvPoint(w/2,ht/2);
for(int z=0;z<count;z++)
{
	if(c[z]>10)
	{
		sx[z]/=c[z];
		sy[z]/=c[z];
		//max2=(int)c[z];
		cent[z]=cvPoint2D32f(sy[z],sx[z]);
		//printf("maxc=%d ; max2=%d ; c[maxc]=%ld \n",maxc,max2,c[maxc]);
			//cvWaitKey(0);
		//printf("X=%f  &&  Y=%f\n",sy[maxc],sx[maxc]);
		//p=cvPoint(sy[z],sx[z]);
		//cvCircle(image,p,2,cvScalar(122),2);
		//cvPutText(image,ch1,p,&f,cvScalar(122));
	}
}

}
#define peak_thresh 300		//for front camera---->set this value acc. to the opponent bot in view
#define MKR_HORIZON 300		//check for front camera (when our bot is closest)
#define DRAG_K 0.0109		//check for front camera
	// Adaptive ROI velocity increment factors
#define ROI_VEL_X 1.5		//check for front camera
#define ROI_VEL_YU 2.0		//check for front camera
#define ROI_VEL_YD 2.0		//check for front camera (this vel down is more than vel. up bcoz the cork will move faster when its close to cam)
#define KP 12
#define KD 0.2
#define MIN_ROI 25	// Min. ROI Size around cork
#define FUTURE_ITR_LMT 0	// Max. No. of Future Prediction(iterations) allowed 
#define CORK_INTNSTY_L 7	//(check for front camera) min. Intensity of Cork(in grayscale after frame difference) req. to avoid Future Prediction
#define MKR_AREA_THRESH 10	//check for front camera
#define MKR_AMIN_THRESH 150
#define MKR_AMAX_THRESH 7000
#define MKRw_AMIN_THRESH 80
//#define MKRw_DIST 40
int MKRwo_AMAX_THRESH=7000;
int MKRwb_AMAX_THRESH=7000;
int MKRwo_xROI=40;
int MKRwo_yROI=40;
int MKRwb_xROI=40;
int MKRwb_yROI=40;
#define fxROI 1
#define fyROI 1
#define BLUE_INTNSTY_THRESH 140
#define ORNGE_INTNSTY_THRESH 50
#define ANTI_ROI_Y 100	//Anti-ROI size around around BOT Marker.
#define ANTI_ROI_X 100
// Variables for "Marker Lost" condition.
#define MKR_X_VAR 40	// Marker's min. allowable x-position variance b/w two succesive frames (more for front camera if bot is close)
#define MKR_Y_VAR 35	// Marker's min. allowable y-position variance b/w two succesive frames
#define MKR_AREA_VAR 240	// Marker's min. allowable Area variance b/w two succesive frames
#define MKRLOST_LMT 10
#define GRDX_VAR 80		//check for front camera
int main()
{
	//CvCapture* cam=cvCaptureFromFile("f:/$robominton/kalman.avi");
	CvCapture* cam=cvCaptureFromCAM(2);
	cvSetCaptureProperty(cam,CV_CAP_PROP_FPS,30);
	cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_WIDTH,640);
	cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_HEIGHT,480);
	cvNamedWindow("Out",CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("Bin",CV_WINDOW_AUTOSIZE);
	Tserial com;
	//while(com.connect("COM2",9600,spNONE));
	//com.connect("COM15",9600,spNONE);
	int grdx=125,peak=0,fpd=0;
	/*******************PID Stuff********************************/
	float out,Kp1=KP,Ki1=0,Kd1=KD,Kp2=KP,Ki2=0,Kd2=KD,pre_error=0,error,esum=0;
	/*************Kalman Initialisation************************/
	CvKalman* kalman=cvCreateKalman(7,2,0);
	float k=DRAG_K;
	float F[49]={1,1,0.5,0,0,0,0,  0,1,1,0,0,0,0,   0,-k,-k,0,0,0,0,  0,0,0,1,1,0.5,0,  0,0,0,0,1,1,0,  0,0,0,0,-k,-k,1, 0,0,0,0,0,0,1};
	cvInitMatHeader(kalman->transition_matrix,7,7,CV_32FC1,F);
	float H[14]={1,0,0,0,0,0,0,  0,0,0,1,0,0,0};
	cvInitMatHeader(kalman->measurement_matrix,2,7,CV_32FC1,H);
	cvSetIdentity( kalman->process_noise_cov, cvRealScalar(1e-6) );
    cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-3) );
    cvSetIdentity( kalman->error_cov_post, cvRealScalar(1));
	const CvMat* y_k=cvCreateMat( 7, 1, CV_32FC1 );
	const CvMat* x_k = cvCreateMat(7, 1, CV_32FC1 );
	const CvMat z_k;// = cvCreateMat( 2, 1, CV_32FC1 );
	/******************Kalman Init End*********************/
	IplImage* image=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
	IplImage* diff=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* prev=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* gray=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* out1=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* final=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* out2=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* out3=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* red=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* blue=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* corner=cvCreateImage(cvSize(640,480),IPL_DEPTH_32F,1);
    bool first=true;
	float G=0.3,corr_x,corr_y,i,xroi,yroi,pvel=0;
	CvPoint* history=new CvPoint[4];             
	prev=framediff(cam,history,"Out"); // does frame diff w/ parabolic segmentation
	G=abs(history->y-(2*(history+1)->y)+(history+2)->y); // calculate approximate G (acceleration due to gravity)
	// Next we initialse Kalman "States" with x,x velocity,x acceleration etc. 
	//with some approximate(coz noise corrupted) values, which shall be corrected and filtered subsequently as the routine loops
	float X_k[7]={history->x,history->x-(history+1)->x,-k*(history->x-(history+1)->x),history->y,history->y-(history+1)->y,G-k*(history->y-(history+1)->y),G};
	
	cvInitMatHeader(kalman->state_post,7,1,CV_32FC1,X_k);
	cvCircle(prev,cvPoint(cvRound(history->x),cvRound(history->y)),3,CV_RGB(255,255,255),2);
	cvCvtColor(cvQueryFrame(cam),prev,CV_BGR2GRAY);

	int max=0,max_x,max_y,j,bsb=1;
	int rxl=MIN_ROI, rxr=MIN_ROI, ryu=MIN_ROI, ryd=MIN_ROI,xlroi,yuroi,xrroi,ydroi;
	int checko = 1,checkb = 1;
	float min;
	int cork_x;
	CvPoint2D32f  *cent,*mins,*maxs;
	int maxb=0,mb=-1,maxo=0,mo=-1,maxwo=0,maxwb=0,mwo=-1,mwb=-1,a=0,b=0,max2=0,bot=0;
	uchar* ptr1;  
	uchar* ptr2;
	int future=0;
	int hist_a = 0,hist_b = 0, hist_area = 0;			//Define History coordinates and area
	int flush=0;
	int hgrd1=125,hgrd0=125;
	while(1)
	{
		image=cvQueryFrame(cam); // say cheeese! :D`

		if(image!=NULL) 
		{
			cvCvtColor(image,gray,CV_BGR2GRAY);  // convert to grayscale...
			//1.Kalman Predict (state pre values updated)
			y_k=cvKalmanPredict(kalman,NULL); // Kalman Prediction Step
			//2.Adaptive current ROI finding to check for any cork for in the present frame
			if(future==0)
				i=0; // higher 'i' means more far into the future the updated ROI is..
			else if(future<=FUTURE_ITR_LMT)
				i=future;
			else
			{	//until the future iteration reaches 3 start framediff
				printf("Future Prediction failed\n");
				prev=framediff(cam,history,"Out");
				G=abs(history->y-(2*(history+1)->y)+(history+2)->y);
				float X_k[7]={history->x,history->x-(history+1)->x,-k*(history->x-(history+1)->x),history->y,history->y-(history+1)->y,G-k*(history->y-(history+1)->y),G};
				cvInitMatHeader(kalman->state_post,7,1,CV_32FC1,X_k);
				future=0;
				pvel=0;
				hist_a = 0,hist_b = 0, hist_area = 0;
				//cvCvtColor(cvQueryFrame(cam),prev,CV_BGR2GRAY);
				continue;
			}
			xroi=cvRound(kalman->state_post->data.fl[0]+ (i* kalman->state_post->data.fl[1])+ (0.5* i*i*kalman->state_post->data.fl[2]));// CALC ROI
			yroi=cvRound(kalman->state_post->data.fl[3]+ (i* kalman->state_post->data.fl[4])+ (0.5* i*i*kalman->state_post->data.fl[5]));
			if(kalman->state_post->data.fl[1]<0)
			{
				rxl=MIN_ROI-ROI_VEL_X*kalman->state_post->data.fl[1];
				rxr=MIN_ROI;
			}
			else
			{
				rxr=MIN_ROI+ROI_VEL_X*kalman->state_post->data.fl[1];
				rxl=MIN_ROI;
			}
			if(kalman->state_post->data.fl[4]<0)
			{
				ryu=MIN_ROI-ROI_VEL_YU*kalman->state_post->data.fl[4];
				ryd=MIN_ROI;
			}
			else
			{
				ryd=MIN_ROI+ROI_VEL_YD*kalman->state_post->data.fl[4];
				ryu=MIN_ROI;
			}
			xlroi=xroi-rxl;
			yuroi=yroi-ryu;
			xrroi=xroi+rxr;
			ydroi=yroi+ryd;
			if(yuroi<0 && yroi>0)
				yuroi=0;
			//cvRectangle(image,cvPoint(xlroi,yuroi),cvPoint(xrroi,ydroi),CV_RGB(255,0,0));  // draw ROI
			//3.ROI out of bounds condition 
			
			if((xlroi<0)||(yuroi<0)||(xrroi+1>639)||(ydroi+1>479))  // if the ROI box goes out of screen
		    {	
				//ydroi condition is not added below so as to prevent bot from stopping in case it falis to reach the expected position(anti-roi condition)
				if((xlroi<0)||(xrroi+1>639))
				{
					com.sendChar(253);
					/*com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);*/
					printf("grdx=253\n");
				}
				printf("\nROI Out of bounds\n");
			prev=framediff(cam,history,"Out");
			G=abs(history->y-(2*(history+1)->y)+(history+2)->y);
			float X_k[7]={history->x,history->x-(history+1)->x,-k*(history->x-(history+1)->x),history->y,history->y-(history+1)->y,G-k*(history->y-(history+1)->y),G};
			cvInitMatHeader(kalman->state_post,7,1,CV_32FC1,X_k);
			//cvWaitKey(0);
			bsb=1,checko=1,checkb=1,hist_a = 0,hist_b = 0, hist_area = 0,flush=0;
			pvel=0;// --------  search for bot again if out of bounds... prevents noisy blobs detection...
			hgrd1=125;
			hgrd0=125;
			//cvWaitKey(1);
			//cvCvtColor(cvQueryFrame(cam),prev,CV_BGR2GRAY);
			continue;
		    }
			//4.Finding the cork in the expected roi (max. intensity)
            cvAbsDiff(gray,prev,diff); // frame diff
			max=0;
			for(int y=yuroi;y<ydroi+1;y++) // find the brightest pixel within the ROI
			{
				uchar* ptr=(uchar*)(diff->imageData+ y*diff->widthStep);
				for(int x=xlroi;x<xrroi+1;x++)
				{
					if(ptr[x]>max)
					{
						max=ptr[x];
						max_x=x;
						max_y=y;
					}
				}
			}
			//printf("the Intensity value of cork = %d\n", max);
			//cvWaitKey(0);
			//5.Future ROI prediction if no cork is found in the presently expected roi just increment the  value of "i" until 2 iteration of future else code restart.
			//if(max<15  || (pvel*kalman->state_post->data.fl[1]<0 && kalman->state_post->data.fl[3]<peak_thresh))    // the execution of this if condition is independent of the existence of the bot marker
			if(max<CORK_INTNSTY_L)    // the execution of this if condition is independent of the existence of the bot marker
			{
						future++;  //..... Hence we try to find the shuttle somewhere up ahead; the higher is the value of i, the more 'futuristic' is the new ROI
						//pvel=kalman->state_post->data.fl[1];
						prev=cvCloneImage(gray);
						continue;
			}
			else
				future=0;
			//cvWaitKey(0);
			//6.Bot marker detection
			cvSetZero(out1);
			cvSetZero(out2);
			cvSetZero(out3);
			//cvSetZero(final);
			cvSplit(image,blue,NULL,NULL,NULL);
			cvSplit(image,NULL,NULL,red,NULL);
			//Check Orange
			if(checko == 1)
			{
				//printf("o");
				cvSetImageROI(blue,cvRect(0,MKR_HORIZON,639,460));
				cvSetImageROI(red,cvRect(0,MKR_HORIZON,639,460));
				cvSetImageROI(out1,cvRect(0,MKR_HORIZON,639,460));
				cvSub(red,blue,out1);	
				cvResetImageROI(blue);
				cvResetImageROI(red);
				cvResetImageROI(out1);
				cvThreshold(out1,out1,ORNGE_INTNSTY_THRESH,255,CV_THRESH_BINARY);
				//cvSmooth(out1,out1);
				//cvThreshold(out1,out1,250,255,CV_THRESH_BINARY);
			}

		//Check Blue
		   /* if(checkb == 1)
			{
				//printf("b");
				cvSetImageROI(blue,cvRect(0,MKR_HORIZON,639,460));
				cvSetImageROI(red,cvRect(0,MKR_HORIZON,639,460));
				cvSetImageROI(out2,cvRect(0,MKR_HORIZON,639,460));
					cvSub(blue,red,out2);
				cvResetImageROI(blue);
				cvResetImageROI(red);
				cvResetImageROI(out2);
					cvThreshold(out2,out2,BLUE_INTNSTY_THRESH,255,CV_THRESH_BINARY);
				cvSetImageROI(out2,cvRect(0,MKR_HORIZON,639,460));
					cvSmooth(out2,out2);
				cvResetImageROI(out2);
					cvThreshold(out2,out2,250,255,CV_THRESH_BINARY);	
				//cvShowImage("blue",out2);
			}*/
			//Check White
		    cvThreshold(gray,out3,253,255,CV_THRESH_BINARY);
			
			cvAdd(out1,out2,final);
			cvAdd(out3,final,final);
			//cvShowImage("Final",final);
			//cvShowImage("Bin",out2);
			//int* label=new int[image->width*image->height];
			int count=labelx(final);  // labelling the image
			cent=new CvPoint2D32f[count];
			mins=new CvPoint2D32f[count];
			maxs=new CvPoint2D32f[count];
			long* c=new long[count];		
			centroid(image,count,cent,c,mins,maxs);  // cent:centroid of each blob,c=area of each blob
            //delete[] label;
			// FINDING THE (max area)BLUE/ORNGE MKR (IN THE AREA THRESH && BELOW A SPECIFIC HORIZON level for ROI of bot)
			max2=0,maxb=0,mb=-1,maxo=0,mo=-1;
			for(int cnt=0;cnt<count;cnt++)
			{
				if(c[cnt]>MKR_AMIN_THRESH && c[cnt]<MKR_AMAX_THRESH && cent[cnt].y>MKR_HORIZON)
				{
					ptr1=(uchar*)(out1->imageData+(int)((cent[cnt].y+mins[cnt].y)/2)*out1->widthStep) + (int)cent[cnt].x;
					//orange mkr (max area blob) check under area thresh
					if(*ptr1>250 && maxo<=c[cnt])
					{
						maxo=c[cnt];
						mo=cnt;
					}
					ptr2=(uchar*)(out2->imageData+(int)cent[cnt].y*out2->widthStep) + (int)cent[cnt].x;
					//blue mkr (max area blob) check under area thresh
					if(*ptr2>250 && maxb<=c[cnt])
					{
						maxb=c[cnt];
						mb=cnt;
					}
				}
			}
			if(mo!=-1)
			{
			MKRwo_xROI= (int)(abs(maxs[mo].x-mins[mo].x)*fxROI)/2;
			MKRwo_yROI= (int)(abs(maxs[mo].y-mins[mo].y)*fyROI)/2;
			MKRwo_AMAX_THRESH=8*MKRwo_xROI*MKRwo_yROI;
			}
			if(mb!=-1)
			{
			MKRwb_xROI= (int)(abs(maxs[mb].x-mins[mb].x)*fxROI)/3;
			MKRwb_yROI= (int)(abs(maxs[mb].y-mins[mb].y)*fyROI)/2;
			MKRwb_AMAX_THRESH=8*MKRwb_xROI*MKRwb_yROI;
			}
			// FINDING THE (max area)WHITE MKR AROUND ANY OF THE BL/ORNGE MKRS IF FOUND (UNDER A SPECIFIED ROI AROUND THE COLOURED MKRS)
			maxwo=0,maxwb=0,mwo=-1,mwb=-1;
			if(mo!=-1 || mb!=-1)
			{
			for(int cnt=0;cnt<count;cnt++) 
			{
				if(c[cnt]>MKRw_AMIN_THRESH && c[cnt]<MKRwo_AMAX_THRESH)
				{
					ptr1=(uchar*)(out3->imageData+(int)cent[cnt].y*out3->widthStep) + (int)cent[cnt].x;
					//white(for orange) mkr (max area blob) check under area thresh & in that specific roi
					if(*ptr1>250 && maxwo<=c[cnt] && abs(cent[cnt].x-(maxs[mo].x-MKRwo_xROI))<MKRwo_xROI && abs(cent[cnt].y-(mins[mo].y-MKRwo_yROI))<MKRwo_yROI)
					{
						maxwo=c[cnt];
						mwo=cnt;
					}
				}
				if(c[cnt]>MKRw_AMIN_THRESH && c[cnt]<MKRwb_AMAX_THRESH)
				{
					ptr2=(uchar*)(out3->imageData+(int)cent[cnt].y*out3->widthStep) + (int)cent[cnt].x;
					//white(for blue) mkr (max area blob) check under area thresh & in that specific roi
					if(*ptr2>250 && maxwb<=c[cnt] && abs(cent[cnt].x-(maxs[mb].x-MKRwb_xROI))<MKRwb_xROI && abs(cent[cnt].y-(mins[mb].y-MKRwb_yROI))<MKRwb_yROI)
					{
						maxwb=c[cnt];
						mwb=cnt;
					}
				}
			}
			}
			if(bsb==1)
			{
				if(mwo!=-1 || mwb!=-1)  // IF WHITE MKR IS FOUND, THE GREATER OF THE 2 INSIDE THE ROI IS SELECTED AS THAT MKR IS "ON" !!! 
				{
					if(maxwo>maxwb)
					{
						bot=1;
						printf("orange bot found!!\n");
						com.sendChar(251);
						com.sendChar(125);
						//cvShowImage("Bin",image);
						cvCircle(image,cvPoint((int)cent[mwo].x,(int)cent[mwo].y),2,CV_RGB(100,255,100));
						checkb = 0;
						checko = 1;
						bsb++;
						flush=0;
					}
					else
					{
						bot=2;
						printf("blue bot found!!\n");
						com.sendChar(252);
						com.sendChar(125);
						//cvShowImage("Bin",image);
						cvCircle(image,cvPoint((int)cent[mwb].x,(int)cent[mwb].y),2,CV_RGB(100,255,100));
						checkb = 0;
						checko = 1;
						bsb++;
						flush=0;
					}
					hist_a = 0,hist_b = 0, hist_area = 0;
				}
				else
				{printf("Nothing Found - Max : %d\n",max2);
				flush++;
				if(flush==MKRLOST_LMT)	// will send the flush instruction only one time
					{com.sendChar(125);
				printf("Flush Data=125\n");}
				//cvShowImage("Out",image);
				//continue;
				}
			}
			if(bsb==2)		//STROING THE CENTROIDS OF THE FOUND MKRS IF FOUND
			{
			a=0,b=0,max2=0;
			if(maxwo>maxwb)
			{
				a=(int)cent[mwo].x;
				b=(int)cent[mwo].y;
				max2=maxwo;
			}
			else
			{
				a=(int)cent[mwb].x;
				b=(int)cent[mwb].y;
				max2=maxwb;
			}
			}
			//draw the roi
			//printf("\nMKRw_yROI = %d \n",MKRwb_yROI);
			cvRectangle(image,cvPoint(((int)maxs[mo].x-2*MKRwo_xROI),((int)mins[mo].y-2*MKRwo_yROI)),cvPoint((int)maxs[mo].x,(int)mins[mo].y),cvScalar(0,0,255),2);
			cvRectangle(image,cvPoint(((int)maxs[mb].x-2*MKRwb_xROI),((int)mins[mb].y-2*MKRwb_yROI)),cvPoint((int)maxs[mb].x,(int)mins[mb].y),cvScalar(255,0,0),2);
			//7.Check if the marker is switched off/not Also if the cork found above falls in the Anti-ROI of the bot marker else code restart
			if((checko==1 && mwo==-1) || (checkb==1 && mwb==-1) ||!(((bsb == 2) && ((abs(a-hist_a)<MKR_X_VAR)&&abs((b-hist_b)<MKR_Y_VAR)&&abs((max2-hist_area)<MKR_AREA_VAR))) || ((hist_area == 0)&&(hist_a==0)&&(hist_b==0))))   // [CHANGED] should only run if previous coordinates are near current coornites and area is nearly same or running first time.....
			{
				printf("Marker Lost\n");		// [CHANGED] if marker is lost history should be reset.
				printf("abs(a-hist_a)=%d ; abs((b-hist_b)=%d ; abs((max2-hist_area)=%d\n",abs(a-hist_a),abs(b-hist_b),abs(max2-hist_area));
				bsb=1,checko=1,checkb=1,hist_a = 0,hist_b = 0, hist_area = 0;flush=1;
				//com.sendChar(254);  // instruct transmitter arduino board to flush and clear all the previous data
			}
			if(bsb==2)    // the below part should only execute if the bot marker led is on, i.e. cent[j] exists
			{
				//printf("max=%d\n ---------------\n",max);
				// If max < threshold=10, means we have lost the shuttle in the current ROI....
				//cvRectangle(image,cvPoint((int)cent[j].x-50,(int)cent[j].y-50),cvPoint((int)cent[j].x+50,(int)cent[j].y+50),CV_RGB(0,0,255),2);
				if(abs(b-max_y)<ANTI_ROI_Y && abs(a-max_x)<ANTI_ROI_X) //the thresh value 45/50 is given on the basis of the ht. of the racket above the marker	
				{
						//printf("-------------------------------------------\n");
						printf("anti-roi or horizon\n");
						bsb=1,checko=1,checkb=1,hist_a = 0,hist_b = 0, hist_area = 0,flush=0;
						//printf("BULB OFFF\n");
						//grdx = 253;
						//com.sendChar(grdx);
						com.sendChar(253);
						/*com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);*/

						printf("grdx= 253\n");
						prev=framediff(cam,history,"Out");
						G=abs(history->y-(2*(history+1)->y)+(history+2)->y);
						float X_k[7]={history->x,history->x-(history+1)->x,-k*(history->x-(history+1)->x),history->y,history->y-(history+1)->y,G-k*(history->y-(history+1)->y),G};
						cvInitMatHeader(kalman->state_post,7,1,CV_32FC1,X_k);
						pvel=0;
						hgrd1=125;
						hgrd0=125;
						//cvCvtColor(cvQueryFrame(cam),prev,CV_BGR2GRAY);
						continue;
				}
					
			}
			//8.Otherwise store the x,y coord. of the cork found and reinitialize the MatHeader of the Kalman
			cvCircle(image,cvPoint(max_x,max_y),10,CV_RGB(0,255,0),2);  // Draw the measured point
			float Z_k[2]={max_x,max_y};
			cvInitMatHeader((CvMat*)&z_k,2,1,CV_32FC1,Z_k);
			//9.& correct & update the Kalman variables acc. to the new measured value of cork for better Kalman prediction in the next run. (state post values updated)
			pvel=kalman->state_post->data.fl[1];
			x_k=cvKalmanCorrect(kalman,&z_k);  // Incorporate measured point and do Kalman correction..
			//10.Plotting the trajectory acc. to newly updates kalman variables
			min=1000;
			peak=480;
			//peak=kalman->state_post->data.fl[3]+ (-100* kalman->state_post->data.fl[4])+ (0.5* -100*-100*kalman->state_post->data.fl[5]);
			for(float ip=-100;ip<=100;ip++) // Plot the future course
			{   // Use the clean and nice "corrected states" to plot the future course of flight
				if(kalman->state_post->data.fl[5]<0) kalman->state_post->data.fl[5]*=-1;	
				corr_x= kalman->state_post->data.fl[0]+ (ip* kalman->state_post->data.fl[1])+ (0.5* ip*ip*kalman->state_post->data.fl[2]);
				corr_y=kalman->state_post->data.fl[3]+ (ip* kalman->state_post->data.fl[4])+ (0.5* ip*ip*kalman->state_post->data.fl[5]);
				if(ip>0)
				{
				if((int)corr_y>480) break;
				if(bsb==2)  // the below part should only execute if the bot marker led is on, i.e. cent[j] exists
				{
		       if(abs(b-corr_y)<min)
				{
					cork_x=(int)corr_x;
					min=abs(b-corr_y);
				}
				}
				if((int)ip%2==0 && (corr_x>2 && corr_x<638 && corr_y>2 && corr_y<478))
				cvCircle(image,cvPoint(cvRound((double)(corr_x)),cvRound((double)(corr_y))),2,CV_RGB(255,255,0),2);
				}
			   if(corr_y<peak)
				   peak=corr_y;
			    //cvCircle(image,cvPoint(cvRound((double)(corr_x)),cvRound((double)(corr_y))),2,CV_RGB(255,255,0),2);
		    }
			if(cork_x>640) cork_x=640;
			if(cork_x<0) cork_x=0;
			//printf("the peak= %d; cork_x=%d\n",peak,cork_x);
			//cvLine(image,cvPoint(2,peak),cvPoint(630,peak),CV_RGB(122,0,122),2);
			//cvLine(image,cvPoint(20,peak_thresh),cvPoint(150,peak_thresh),CV_RGB(0,0,0),2);  //lessens execution time
			cvLine(image,cvPoint(80,peak_thresh),cvPoint(270,peak_thresh),CV_RGB(0,0,0),2);
			cvLine(image,cvPoint(450,peak_thresh),cvPoint(600,peak_thresh),CV_RGB(0,0,0),2);
			//11.Peak (below thresh) Value segmentation & Vel. dir seg. else Code restart 
			if(peak>peak_thresh || (pvel*kalman->state_post->data.fl[1]<0 && kalman->state_post->data.fl[3]>peak_thresh))
			{
				printf("Peak segmentation---- Code restart!\ngrdx=253\n");
				com.sendChar(253);
				/*com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);
						com.sendChar(253);*/
				prev=framediff(cam,history,"Out");
			G=abs(history->y-(2*(history+1)->y)+(history+2)->y);
			float X_k[7]={history->x,history->x-(history+1)->x,-k*(history->x-(history+1)->x),history->y,history->y-(history+1)->y,G-k*(history->y-(history+1)->y),G};
			cvInitMatHeader(kalman->state_post,7,1,CV_32FC1,X_k);
			//cvWaitKey(0);
			bsb=1,checko=1,checkb=1,hist_a = 0,hist_b = 0, hist_area = 0,flush=0;  // --------  search for bot again if out of bounds... prevents noisy blobs detection...
			pvel=0;
			hgrd1=125;
			hgrd0=125;
			//cvWaitKey(1);
			//cvCvtColor(cvQueryFrame(cam),prev,CV_BGR2GRAY);
			continue;
			}
			//12.PID of bot and sending data thro UART
			if(bsb==2)
				{
					hgrd1=hgrd0;
					hgrd0=grdx;
					/***PID**/
					cvLine(image,cvPoint(cvRound(a),cvRound(b)),cvPoint(cvRound(cork_x),cvRound(b)),CV_RGB(0,0,255),2);
					error=-1*(cork_x-a);

					if(bot == 1)
						out=Kp1*error+(Kd1*(error-pre_error))-(Ki1*esum);
					else if(bot == 2)
						out=Kp2*error+(Kd2*(error-pre_error))-(Ki2*esum);

					  grdx = (int)((128*(out)/(640))+127);
					 if(grdx>250)
						  grdx = 250;
					  if(grdx<1)
						  grdx = 1;
					 pre_error=error;
					 printf("10 times grdx=%d\t",grdx);
					// com.sendChar(125);
					 if(abs(hgrd1-hgrd0)<GRDX_VAR && abs(hgrd0-grdx)<GRDX_VAR)
						 {
							 com.sendChar(grdx);
							 /*com.sendChar(grdx);
							 com.sendChar(grdx);
							 com.sendChar(grdx);
							 com.sendChar(grdx);
							 com.sendChar(grdx);
							 com.sendChar(grdx);
							 com.sendChar(grdx);
							 com.sendChar(grdx);
							 com.sendChar(grdx);*/
					 }
					 else
						 printf("Invalid fluctuating value of grdx escape");
					 printf("\n");
					hist_area = max2;     // [CHANGED] Update history
					hist_a = a;
					hist_b = b;
			}
			else
			{
				//com.sendChar(stop);
				//com.sendChar(stop);
			}
			//printf("G=%f\n",kalman->state_post->data.fl[6]);
			/*printf("ax=%f\n",kalman->state_post->data.fl[2]);
			printf("ay=%f\n",kalman->state_post->data.fl[5]);
			printf("vx=%f\n",kalman->state_post->data.fl[1]);
			printf("vy=%f\n",kalman->state_post->data.fl[4]);*/
			cvShowImage("Out",image);
				prev=cvCloneImage(gray);  //. should/must be used just after cvAbsDiff
				//cvWaitKey(1);
			
		}// ........ See ya at the beginning of while(1) ! (:
		//cvShowImage("Bin",diff);
		
		if(cvWaitKey(1)==' ')
			cvWaitKey(0);
	}
	com.disconnect();
	return 0;
}