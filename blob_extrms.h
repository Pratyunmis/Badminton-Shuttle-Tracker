/*
   Credits To Gary Bradski & Adrian Kaehler (O'reilly) from Chirag :) 
   							                */

int labelx(IplImage* image,int* label)
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
void centroid(IplImage* image,int* label, int count,CvPoint2D32f* cent,long* c,CvPoint2D32f* mins=NULL,CvPoint2D32f* maxs=NULL)
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