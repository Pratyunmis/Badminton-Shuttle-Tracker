				// -----------------SIDE CAM-------------------- //
#include <opencv2/highgui/highgui.hpp>
#include "iostream"
#include "opencv/cv.h"
//#include "bloborig.h"
#include <stdio.h>
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
