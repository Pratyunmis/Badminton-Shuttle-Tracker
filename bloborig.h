/*
   Credits To Gary Bradski & Adrian Kaehler (O'reilly) from Chirag :) 
   							                */

int labelxy(IplImage* image,int* label)
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
int centroid(IplImage* image,int* label, int count,long* sx,long* sy,long* c)
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
		c1=label[ht*x+y];
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