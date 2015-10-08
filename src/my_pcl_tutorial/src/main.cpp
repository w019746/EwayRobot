#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "LmsData.h"
#include "DxData.h"
#include "PosMatrixData.h"
#include "YyfImage.h"


extern CameraParams cameraParams;
LMSDATA lmsData;
DXDATA dxData;
POSMATRIXDATA posData;
YYF_IMAGE yyfImage;

MATRIX3d dr_global;
point3d dt_global;



typedef struct	{
	double			x, y, z;
	unsigned char	r, g, b;
} rgbpoint3d;


#ifndef struct_point2d
#define struct_point2d
typedef struct {
	double x,y;
}point2d;
#endif

#ifndef struct_PointWeight
#define struct_PointWeight
typedef struct {
	double w[6];
}pointWeight;
#endif

#ifndef struct_PointReprojected
#define struct_PointReprojected
typedef struct {
	int Millisecond;
	point3d ptG;
	int GtFrame;
	vector <int> frames;
	vector <point2d> projected_result;
	vector <double> depth;
	vector <point2d> tracked_result;
	vector <pointWeight> weight;
}PointReprojected;
#endif


#define BOUND(x,min,max)	((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


double getPPDistance(const point2d& p1, const point2d& p2)
{
	return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

void UpdateRTNew(MATRIX3d& r, point3d& t, MATRIX3d& dr, point3d dt)
{
	rMatrixmultiLeft(dr,r);
	rotatePoint3d(t,dr);
	t.x = t.x + dt.x;
	t.y = t.y + dt.y;
	t.z = t.z + dt.z;
}

void UpdateRTNew(MATRIX3d& r, point3d t, MATRIX3d& dr, point3d dt, MATRIX3d& rnew, point3d& tnew)
{
	rMatrixCopy(rnew,r);
	rMatrixmultiLeft(dr,rnew);
	rotatePoint3d(t,dr);
	tnew.x = t.x + dt.x;
	tnew.y = t.y + dt.y;
	tnew.z = t.z + dt.z;
}

void UpdateDrDt(MATRIX3d& r, point3d t, MATRIX3d& rnew, point3d tnew, MATRIX3d& dr, point3d& dt)
{
	rMatrixCopy(dr,r);
	rMatrixInverse(dr);
	rMatrixmultiLeft(rnew,dr);
	
	rotatePoint3d(t,dr);
	dt.x = tnew.x - t.x;
	dt.y = tnew.y - t.y;
	dt.z = tnew.z - t.z;
}

void OutputRt(MATRIX3d& r, point3d t)
{
	for (int i=0; i<3; i++)
		cout << r[0][i] << "\t";
	cout << t.x << endl;
	for (int i=0; i<3; i++)
		cout << r[1][i] << "\t";
	cout << t.y << endl;
	for (int i=0; i<3; i++)
		cout << r[2][i] << "\t";
	cout << t.z << endl;
	cout << endl;
}


bool SavePts2File(point3d* pts, int ptNum, unsigned int* color, FILE*& fp, FILE*& fpc, double maxHeight=40, double minHeight=-60)
{
	int i;
	rgbpoint3d rgbpt;
	for (i=0; i<ptNum; i++)
	{
		rgbpt.x = pts[i].x;
		rgbpt.y = pts[i].y;
		rgbpt.z = pts[i].z;

		rgbpt.r = BOUND( (pts[i].z - minHeight) / (maxHeight - minHeight)*256, 0, 255);
		rgbpt.b = 255 - rgbpt.r;
		if (rgbpt.r<128)
			rgbpt.g = rgbpt.r*2;
		else
			rgbpt.g = rgbpt.b*2;

		fwrite(&rgbpt,sizeof(rgbpoint3d),1,fp);

		if (color[i]!=0)
		{
			rgbpt.r = (color[i]/256/256)%256;
			rgbpt.g = (color[i]/256)%256;
			rgbpt.b = color[i]%256;
			fwrite(&rgbpt,sizeof(rgbpoint3d),1,fpc);
		}
	}

	return true;
}

vector<point3d>			ptV;
vector<point3d>			ptVC;
int						ptVC_scan=2;
const int				ptVC_scan_max = 4;
vector<unsigned int>	colorV;

bool WriteRgbpoints2File(const int& Millisecond, const vector<point3d>& ptV, const vector<unsigned int>& colorV, FILE*& fp)
{
	unsigned int color;

	int i;
	fprintf(fp,"%d %d\n\t",Millisecond, ptV.size());
	for (i=0; i<ptV.size(); i++)
	{
		fprintf(fp,"%.4lf %.4lf %.4lf %d\t",ptV[i].x,ptV[i].y,ptV[i].z,colorV[i]);
	}
	fprintf(fp,"\n");
	return true;
}

bool SaveColorPoints(vector<point3d>& ptV, vector<unsigned int>& colorV, point3d* pts, int ptNum, unsigned int* color)
{
	int i;
	for (i=0; i<ptNum; i++)
	{
		ptV.push_back(pts[i]);
		colorV.push_back(color[i]);
	}
	return true;
}

bool DrawOneSweep2Image(vector<point3d> ptV, int Millisecond, MATRIX3d& r, point3d t, double maxHeight=2, double minHeight=-0.56)
{
	vector<unsigned int> colorGreyV;
	int i;
	point3d pt;
	rgbpoint3d rgbpt;


	rMatrixInverse(r);
	t.x = -t.x;
	t.y = -t.y;
	t.z = -t.z;

	for (i=0; i<ptV.size(); i++)
	{
		pt = ptV[i];
		shiftPoint3d(pt,t);
		rotatePoint3d(pt,r);
		ptV[i] = pt;
	}

	colorGreyV.clear();
	for (i=0; i<ptV.size(); i++)
	{

		rgbpt.r = BOUND( (ptV[i].z - minHeight) / (maxHeight - minHeight)*256, 0, 255);
//		rgbpt.r = BOUND( ( 1 - ((ptV[i].y - maxHeight) / (maxHeight - minHeight))*((ptV[i].y - maxHeight) / (maxHeight - minHeight)) ) * 256, 0, 255);

		rgbpt.b = 255 - rgbpt.r;
		if (rgbpt.r<128)
			rgbpt.g = rgbpt.r*2;
		else
			rgbpt.g = rgbpt.b*2;

		//rgbpt.g = (rgbpt.b*2)%256;

		unsigned int colorGrey = ( 256*rgbpt.r + rgbpt.g ) * 256 + rgbpt.b;
		//colorGrey = 256*256*rgbpt.r;
		colorGreyV.push_back(colorGrey);
	}

	yyfImage.DrawPointsOnImage(Millisecond,ptV,colorGreyV);
	return true;
}

void CalcPrsWeight1(vector <PointReprojected> &prs, const int& frameNow)
{
	// 遮挡点删除
	yyfImage.CalcOcclusionWeight(frameNow,prs);
}

void CalcPrsWeight4(vector <PointReprojected> &prs, const int& frameNow)
{
	// 投影点特征少的删除
	yyfImage.CalcImageFeatureWeight(frameNow,prs);
}

void CalcPrsWeight5(vector <PointReprojected> &prs, const int& frameNow)
{
	// 
	int i;
	for (i=0; i<prs.size(); i++)
	{
		if (*prs[i].frames.rbegin() != frameNow)
			continue;
		double x = *prs[i].depth.rbegin()/1000;
		prs[i].weight.rbegin()->w[5] = max(0.0, 1 - (x*x) / (40*40) );
	}
}

void CalcPrsWeight(vector <PointReprojected> &prs, const int& frameNow)
{
	int i;
	for (i=0; i<prs.size(); i++)
	{

		prs[i].weight.rbegin()->w[0] =  prs[i].weight.rbegin()->w[1] * prs[i].weight.rbegin()->w[2]
									  * prs[i].weight.rbegin()->w[3] * prs[i].weight.rbegin()->w[4]
									  * prs[i].weight.rbegin()->w[5];

/*
		prs[i].weight.rbegin()->w[0] =  prs[i].weight.rbegin()->w[1] * prs[i].weight.rbegin()->w[3]
		* prs[i].weight.rbegin()->w[5] * prs[i].weight.rbegin()->w[4];
*/


	}
	return;
}


void TrackNewFrame(vector <PointReprojected> &prs, const int& framePre, const int& frameNow)
{
	int i,j;
	vector <int> ptsIndex;
	vector <point2d> ptsTrack;

	MATRIX3d rC, irC;
	point3d tC, itC, pt;
	point2d ptI;
	
	pointWeight initWeight;
	initWeight.w[0] = initWeight.w[1] = initWeight.w[2] = initWeight.w[3] = initWeight.w[4] = initWeight.w[5] = 1.0;

	int imageTime;
	imageTime = yyfImage.GetTimeByFrame(frameNow);
	posData.GetPosData(imageTime,rC,tC);
	UpdateRTNew(rC,tC,dr_global,dt_global);
	rMatrixCopy(irC,rC);
	rMatrixInverse(irC);
	itC.x = -tC.x;
	itC.y = -tC.y;
	itC.z = -tC.z;


	ptsIndex.clear();
	

	for (i=0; i<prs.size(); i++)
	{
		if (*prs[i].frames.rbegin() != framePre)
			continue;
		prs[i].frames.push_back(frameNow);
		
		prs[i].weight.push_back(*prs[i].weight.rbegin());

		// 更新投影点
		double tmpZc;
		pt = prs[i].ptG;
		shiftPoint3d(pt,itC);
		rotatePoint3d(pt,irC);
		//WC2IC(pt.x*1000,pt.y*1000,pt.z*1000,&ptI.x,&ptI.y,&tmpZc);
		//ptI.y = 480-ptI.y;
		WC2IC_zhang(pt,ptI,tmpZc);


		// 判断是否保留该点
		double image_eps = 5;
		if ( !(	ptI.x<0+image_eps || ptI.x>=640-image_eps
			||	ptI.y<0+image_eps || ptI.y>=480-image_eps) )
		{
			prs[i].projected_result.push_back(ptI);
			prs[i].depth.push_back(tmpZc);
		}

		// 记录跟踪点
		ptsIndex.push_back(i);
	}

	// 复制跟踪点
	int cornerCount = ptsIndex.size();
	CvPoint2D32f* cornersA = new CvPoint2D32f [cornerCount];
	CvPoint2D32f* cornersB = new CvPoint2D32f [cornerCount];
	char* featureFound = new char [cornerCount];
	float* featureErrors = new float [cornerCount];
	for (i=0; i<cornerCount; i++)
	{
		cornersA[i].x = prs[ptsIndex[i]].tracked_result.rbegin()->x;
		cornersA[i].y = prs[ptsIndex[i]].tracked_result.rbegin()->y;
	}

	for (i=0; i<cornerCount; i++)
	{
		cornersB[i] = cornersA[i];
	}



	// 求解跟踪点
	const double errorTh = 0.1;
	if (cornerCount>=10)
		yyfImage.GetTrackedPoints(framePre,frameNow,cornersA,cornersB,cornerCount,featureFound,featureErrors);
	for (int i=0; i<cornerCount; i++)
	{
		if (featureFound[i]==0 || featureErrors[i]>100)
			continue;

		if (featureErrors[i]>100)
		{
			cout << cornersA[i].x << "\t" << cornersA[i].y << endl;
			cout << cornersB[i].x << "\t" << cornersB[i].y << endl;
			cout << featureErrors[i] << endl;
		
			char tmpch;
			cin >> tmpch;
		}

		ptI.x = cornersB[i].x;
		ptI.y = cornersB[i].y;
		prs[ptsIndex[i]].tracked_result.push_back(ptI);
		//prs[ptsIndex[i]].weight.rbegin()->w[2] = 0.99 * min(prs[ptsIndex[i]].weight.rbegin()->w[2],(double)(1-featureErrors[i]/20));
		prs[ptsIndex[i]].weight.rbegin()->w[2] = ( 0.01+0.99*prs[ptsIndex[i]].weight.rbegin()->w[2] )  * (1-featureErrors[i]/40);
		
		//prs[ptsIndex[i]].weight.rbegin()->w[2] = ( 0.01+0.99*prs[ptsIndex[i]].weight.rbegin()->w[2] ) * (1-featureErrors[i]/1000);
		//prs[ptsIndex[i]].weight.rbegin()->w[2] = min(prs[ptsIndex[i]].weight.rbegin()->w[2], (1-featureErrors[i]/600));
	}
	delete []cornersA;
	delete []cornersB;
	delete []featureFound;
	delete []featureErrors;

	// 清除无法跟踪或投影到的点
	for (i=0; i<prs.size(); i++)
	{
		int minSize;
		minSize = prs[i].frames.size();
		if (minSize > prs[i].projected_result.size())
			minSize = prs[i].projected_result.size();
		if (minSize >prs[i].tracked_result.size())
			minSize = prs[i].tracked_result.size();

		while (prs[i].frames.size()>minSize)
			prs[i].frames.pop_back();
		while (prs[i].projected_result.size()>minSize)
			prs[i].projected_result.pop_back();
		while (prs[i].tracked_result.size()>minSize)
			prs[i].tracked_result.pop_back();
		while (prs[i].depth.size()>minSize)
			prs[i].depth.pop_back();
		while (prs[i].weight.size()>minSize)
			prs[i].weight.pop_back();
	}

	CalcPrsWeight1(prs,frameNow);
	//CalcPrsWeight4(prs,frameNow);
	//CalcPrsWeight5(prs,frameNow);
	CalcPrsWeight(prs, frameNow);

	//cleaning prs;
	bool flag;
	int len = 0;
	for (int i=0; i<prs.size(); i++)
	{
		flag = true;
		if (flag && prs[i].weight.rbegin()->w[0] < 0.3)
		{
			flag = false;
		}
		if (flag && getPPDistance(*prs[i].tracked_result.rbegin(),*prs[i].projected_result.rbegin()) > 15)
		{
			flag = false;
		}

		if (flag && len<i)
			prs[len++] = prs[i];
	}
	prs.resize(len);


	yyfImage.ShowTrackedPoints(frameNow,prs);

	return;
}

double calcAngle(point3d& p1, point3d& p2, point3d& p3)
{
	point3d d1,d2;
	d1.x = p2.x-p1.x;
	d1.y = p2.y-p1.y;
	d1.z = p2.z-p1.z;
	d2.x = p3.x-p2.x;
	d2.y = p3.y-p2.y;
	d2.z = p3.z-p2.z;

	return acos( (d1.x*d2.x+d1.y*d2.y+d1.z*d2.z) / 
		(sqrt(d1.x*d1.x+d1.y*d1.y+d1.z*d1.z) * sqrt(d2.x*d2.x+d2.y*d2.y+d2.z*d2.z)) );
}


void outputMat(cv::Mat mat)
{
	for (int row=0; row<mat.rows; row++)
	{
		const double * mi = mat.ptr<double>(row); 
		for (int col=0; col<mat.cols; col++)
			cout << mi[col] << "\t";
		cout << endl;
	}
}

void CalcPose(vector <PointReprojected>& prs, int frameNow, MATRIX3d& r, point3d t, MATRIX3d& dr, point3d& dt)
{
	MATRIX3d rnew;
	point3d tnew;
	UpdateRTNew(r,t,dr,dt,rnew,tnew);
	//OutputRt(rnew,tnew);

	// 记录变换矩阵与内参数
	cv::Mat K, M_l2w, M_w2l, M_l2c, M_w2c;
	
	K = cv::Mat::eye(3,3,CV_64F);
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			K.at<double>(i,j) = cameraParams.K[i][j];
	M_l2c = cv::Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			M_l2c.at<double>(i,j) = cameraParams.R[i][j];
	M_l2c.at<double>(0,3) = cameraParams.T.x;
	M_l2c.at<double>(1,3) = cameraParams.T.y;
	M_l2c.at<double>(2,3) = cameraParams.T.z;
	

	M_l2w = cv::Mat::eye(4,4,CV_64F);
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			M_l2w.at<double>(i,j) = rnew[i][j];
	M_l2w.at<double>(0,3) = tnew.x;
	M_l2w.at<double>(1,3) = tnew.y;
	M_l2w.at<double>(2,3) = tnew.z;

	//outputMat(M_l2w);

	M_w2l = M_l2w.inv();
	//outputMat(M_w2l);
	M_w2c = M_l2c*M_w2l;
	//outputMat(M_w2c);

	// 记录对应点
	cv::Mat objectPoints, imagePoints, distCoeffs, rvec, tvec;
	vector<int> index;
	for (int i=0; i<prs.size(); i++)
	{
		if (*prs[i].frames.rbegin()==frameNow)
			index.push_back(i);
	}
	objectPoints.create(index.size(),3,CV_64F);
	imagePoints.create(index.size(),2,CV_64F);

	// 点数过少，直接退出
	if (index.size()<50)
		return;

	for (int i=0; i<index.size(); i++)
	{
		objectPoints.at<double>(i,0) = prs[index[i]].ptG.x;
		objectPoints.at<double>(i,1) = prs[index[i]].ptG.y;
		objectPoints.at<double>(i,2) = prs[index[i]].ptG.z;
		imagePoints.at<double>(i,0) = prs[index[i]].tracked_result.rbegin()->x;
		imagePoints.at<double>(i,1) = prs[index[i]].tracked_result.rbegin()->y;
	}

	distCoeffs = cv::Mat::zeros(1,4,CV_64F);

	cv::Mat R = M_w2c(cv::Rect(0,0,3,3));
	cv::Mat T = M_w2c(cv::Rect(3,0,1,3));

	//outputMat(K);
	//outputMat(R);
	//outputMat(T);


	cv::Rodrigues(R,rvec);
	tvec = T;

	//cv::Mat cameraPoints;
	//cameraPoints = R*objectPoints.t() + T;
	//outputMat(cameraPoints);

	CvMat cvObjectPoints = objectPoints;
	CvMat cvImagePoints = imagePoints;
	CvMat cvK = K;
	CvMat cvDistCoeffs = distCoeffs;
	CvMat cvRvec = rvec;
	CvMat cvTvec = tvec;

	//outputMat(rvec);
	//outputMat(tvec);

	//cv::solvePnP(objectPoints,imagePoints,K,distCoeffs,rvec,tvec,true);
	
	cvFindExtrinsicCameraParams2(&cvObjectPoints,&cvImagePoints,&cvK,
		&cvDistCoeffs,&cvRvec,&cvTvec,true);

	//rvec = cv::Mat(&cvRvec);
	//tvec = cv::Mat(&cvTvec);
	//outputMat(rvec);
	//outputMat(tvec);

	cv::Mat R_ref,T_ref, M_w2c_ref, M_c2w_ref, M_l2w_ref;
	cv::Rodrigues(rvec,R_ref);
	T_ref = tvec;

	//outputMat(R_ref);
	//outputMat(T_ref);

	M_w2c_ref = cv::Mat::eye(4,4,CV_64F);
	R_ref.copyTo(M_w2c_ref(cv::Rect(0,0,3,3)));
	T_ref.copyTo(M_w2c_ref(cv::Rect(3,0,1,3)));


	//outputMat(M_w2c_ref);

	M_c2w_ref = M_w2c_ref.inv();
	//outputMat(M_c2w_ref);
	M_l2w_ref = M_c2w_ref*M_l2c;
	//outputMat(M_l2w_ref);

	MATRIX3d rt;
	point3d pt;
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			rt[i][j] = M_l2w_ref.at<double>(i,j);
	pt.x = M_l2w_ref.at<double>(0,3);
	pt.y = M_l2w_ref.at<double>(1,3);
	pt.z = M_l2w_ref.at<double>(2,3);

	UpdateDrDt(r,t,rt,pt,dr,dt);
	OutputRt(dr,dt);

	
	// yyf for test 20150728
	MATRIX3d rt_new,drt;
	point3d pt_new,dpt;
	UpdateRTNew(r,t,dr,dt,rt_new,pt_new);

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			drt[i][j] = rt_new[i][j] - rt[i][j];
	dpt.x = pt_new.x - pt.x;
	dpt.y = pt_new.y - pt.y;
	dpt.z = pt_new.z - pt.z;
	//OutputRt(drt,dpt);
	

	return;
}

FILE* fileForPoseSave = NULL;
void savePose2File(int Millisecond, MATRIX3d r, point3d t)
{
	//if (fileForPoseSave==NULL)
	fileForPoseSave = fopen("posRefined.csv","a");

	fprintf(fileForPoseSave,"%d,",Millisecond);
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
			fprintf(fileForPoseSave,"%.5lf,",r[j][i]);
		fprintf(fileForPoseSave,"%.5lf,",0.0);
	}
	fprintf(fileForPoseSave,"%.5lf,%.5lf,%.5lf,%.5lf\n",t.x,t.y,t.z,0.0);
	fclose(fileForPoseSave);
}

int main(int argc, char** argv)
{


	double maxHeight = 2;
	double minHeight = -1;

	if (argc<7)
	{

/*
		lmsData.LoadLmsData("D:/workspace/data/20141106/lake_round_1/20141106_162604_685v.lms");
		dxData.LoadDxData("D:/workspace/data/20141106/lake_round_1/20141106_162605_169v.dx");
		posData.LoadPosData("D:/workspace/data/20141106/lake_round_1/Analysis/pos_laser.csv");
		yyfImage.LoadData(	"D:/workspace/data/20141106/lake_round_1/1234.camera",
							"D:/workspace/data/20141106/lake_round_1/FileListRight.txt",
							"D:/workspace/data/20141106/lake_round_1/20141106_162604_895l.avi_timestamp.log");
*/		

/*
#define date_20141212
		lmsData.LoadLmsData("d:/workspace/data/20141212/1414/20141212_141102_899v.lms");
		dxData.LoadDxData(	"d:/workspace/data/20141212/1414/20141212_141103_291v.dx");
		posData.LoadPosData("d:/workspace/data/20141212/1414/Analysis/pos_laser.csv");
		yyfImage.LoadData(	"d:/workspace/data/20141212/1414/1234.camera",
							"d:/workspace/data/20141212/1414/FileListRight.txt",
							"d:/workspace/data/20141212/1414/20141212_141103_077l.avi_timestamp.log");
*/

/*
		lmsData.LoadLmsData("J:/workspace/data/20141212/1518/20141212_151318_338v.lms");
		dxData.LoadDxData("J:/workspace/data/20141212/1518/20141212_151318_741v.dx");
		posData.LoadPosData("J:/workspace/data/20141212/1518/Analysis/pos_stereo.csv");
		yyfImage.LoadData(	"J:/workspace/data/20141212/1518/1234.camera",
							"J:/workspace/data/20141212/1518/FileListRight.txt",
							"J:/workspace/data/20141212/1518/20141212_151318_532l.avi_timestamp.log");
*/

/*
		lmsData.LoadLmsData("D:/workspace/data/20141212/1418/20141212_141830_028v.lms");
		dxData.LoadDxData("D:/workspace/data/20141212/1418/20141212_141830_450v.dx");
		posData.LoadPosData("D:/workspace/data/20141212/1418/Analysis/pos_laser.csv");
		yyfImage.LoadData(	"D:/workspace/data/20141212/1418/1234.camera",
							"D:/workspace/data/20141212/1418/FileListRight.txt",
							"D:/workspace/data/20141212/1418/20141212_141830_216l.avi_timestamp.log");
*/

/*
		lmsData.LoadLmsData("D:/workspace/data/20141226/1226_1/20141226_140900_034v.lms");
		dxData.LoadDxData(	"D:/workspace/data/20141226/1226_1/20141226_140900_435v.dx");
		posData.LoadPosData("D:/workspace/data/20141226/1226_1/Analysis/pos_stereo_inter.csv");
		yyfImage.LoadData(	"D:/workspace/data/20141226/1226_1/1234.camera",
							"D:/workspace/data/20141226/1226_1/FileListRight.txt",
							"D:/workspace/data/20141226/1226_1/20141226_140900_212l.avi_timestamp.log");
*/

/*
		lmsData.LoadLmsData("D:/workspace/data/20150730/ForCheck/20150730_171233_415v.lms");
		dxData.LoadDxData(	"D:/workspace/data/20150730/ForCheck/20150730_171233_836v.dx");
		posData.LoadPosData("D:/workspace/data/20150730/ForCheck/Analysis/pos_stereo_inter.csv");
		yyfImage.LoadData(	"D:/workspace/data/20150730/ForCheck/1234.camera",
							"D:/workspace/data/20150730/ForCheck/FileListRight.txt",
							"D:/workspace/data/20150730/ForCheck/20150730_171233_611l.avi_timestamp.log");
*/

		lmsData.LoadLmsData("D:/workspace/data/20150730/10/20150730_160902_257v.lms");
		dxData.LoadDxData(	"D:/workspace/data/20150730/10/20150730_160902_679v.dx");
		posData.LoadPosData("D:/workspace/data/20150730/10/Analysis/pos_stereo_inter.csv");
		yyfImage.LoadData(	"D:/workspace/data/20150730/10/1234.camera",
							"D:/workspace/data/20150730/10/FileList.txt",
							"D:/workspace/data/20150730/10/20150730_160902_463l.avi_timestamp.log");


	}else
	{
		lmsData.LoadLmsData(argv[1]);
		dxData.LoadDxData(argv[2]);
		posData.LoadPosData(argv[3]);
		yyfImage.LoadData(argv[4],argv[5],argv[6]);
	}
	if (argc>=8)
		maxHeight = atoi(argv[7]);
	if (argc>=9)
		minHeight = atoi(argv[8]);

	int i,j;
	int Millisecond;
	int isLUBound = 0;
	int preLUBound = 0;
	point3d pts[2000];
	point3d ptsC[2000];
	point2d ptsI[2000];
	bool valid[2000];
	unsigned int color[2000];
	int ptNum;
	MATRIX3d rL,rG, rC, irC;
	point3d tL,tG, tC, itC;
	rgbpoint3d rgbpt;

	// 保存一次摇摆周期的时间序列
	vector <point3d> ptV;
	vector <point3d> ptVPre;
	ptVPre.clear();
	int readyToDraw = -2;
	vector <int> timeVector;
	//bool circleInited = false;
	timeVector.clear();


	// 获取所有
	vector <PointReprojected> prs;
	PointReprojected pr;
	prs.clear();

	rMatrixInit(dr_global);
	dt_global.x = 0;
	dt_global.y = 0;
	dt_global.z = 0;

	// yyf for test
	//createRotMatrix_XYZ(dr_global,1.0,1.0,1.0);
	//dt_global.x = 10;
	//dt_global.y = 10;
	//dt_global.z = 10;

	//1106: from frame 320 3000
	//1212-1418: from frame 1000 15000

	int startFrame = 0; // 1106

	//int startFrame = 4800;

	int endFrame = startFrame+5000000;
	int imageFrame = -1;
	int imageFramePre = -1;

	for (i=startFrame; i<lmsData.data.size(); i++)
	{
		Millisecond = lmsData.data[i].Millisecond;
		printf("%d: %d\n", i, Millisecond);

		imageFrame = yyfImage.GetNearestFrame(Millisecond);
		int imageTime = yyfImage.GetTimeByFrame(imageFrame);
		
		if (imageFramePre>=0 && imageFrame>imageFramePre)
		{
			TrackNewFrame(prs,imageFramePre,imageFrame);
			if (readyToDraw>0)
			{
				posData.GetPosData(imageTime,rC,tC);
				//CalcPose(prs,imageFrame,rC,tC,dr_global,dt_global);
				//cout << Millisecond << ":" << endl;
				//OutputRt(dr_global,dt_global);
				//OutputRt(rC,tC);
			}
		}

		posData.GetPosData(imageTime,rC,tC);
		UpdateRTNew(rC,tC,dr_global,dt_global);
		//OutputRt(rC,tC);
		
		lmsData.GetLmsData(Millisecond,pts,ptNum);
		dxData.GetDxData(Millisecond,rL,tL,isLUBound);
		posData.GetPosData(Millisecond,rG,tG);
		UpdateRTNew(rG,tG,dr_global,dt_global);
		savePose2File(Millisecond,rG, tG);



		// 到局部坐标系
		for (j=0; j<ptNum; j++)
		{
			rotatePoint3d(pts[j],rL);
			shiftPoint3d(pts[j],tL);
		}

		// 到全局坐标系
  		for (j=0; j<ptNum; j++)
		{
			rotatePoint3d(pts[j],rG);
			shiftPoint3d(pts[j],tG);
		}

		// 到图像坐标系
		for (j=0; j<ptNum; j++)
			ptsC[j] = pts[j];
		rMatrixCopy(irC,rC);
		rMatrixInverse(irC);
		itC.x = -tC.x;
		itC.y = -tC.y;
		itC.z = -tC.z;

		double tmpZc[2000];
		for (j=0; j<ptNum; j++)
		{
			// 先转到局部坐标系
			shiftPoint3d(ptsC[j],itC);
			rotatePoint3d(ptsC[j],irC);

			//if (ptsC[j].y>=1 && ptsC[j].x>=-2 && ptsC[j].x<=2)
			//	j=j;

			//WC2IC(ptsC[j].x*1000,ptsC[j].y*1000,ptsC[j].z*1000,&ptsI[j].x,&ptsI[j].y);
			//ptsI[j].y = 480 - ptsI[j].y;

			//point3d tmpPW;
			//WC2IC(ptsC[j].x*1000, ptsC[j].y*1000, ptsC[j].z*1000, & ptsI[j].x, &ptsI[j].y, &tmpZc[j]);
			//ptsI[j].y = 480 - ptsI[j].y;
			//yyf_IC2WC(ptsI[j].x,ptsI[j].y,tmpZc, &tmpPW.x, &tmpPW.y, &tmpPW.z);
			WC2IC_zhang(ptsC[j],ptsI[j],tmpZc[j]);


			//判断是否保留该点
			double image_eps = 5;
			if (	ptsI[j].x<0+image_eps || ptsI[j].x>=640-image_eps
				||	ptsI[j].y<0+image_eps || ptsI[j].y>=480-image_eps)
			{
				valid[j] = false;
			}else
				valid[j] = true;
		}
		
		//vector<point3d> ptsForShow(ptNum);
		//for (int i=0; i<ptNum; i++)
		//	ptsForShow[i] = ptsC[i];

		//vector<unsigned int> color;
		//yyfImage.DrawPointsOnImage(Millisecond,ptsForShow,color);

		pointWeight initWeight;
		initWeight.w[0] = initWeight.w[1] = initWeight.w[2] = initWeight.w[3] = initWeight.w[4] = initWeight.w[5] = 1.0;

		int sp,ep;
		sp = prs.size();

		// 保存有效点
		if (i<endFrame)
		{
			for (j=0; j<ptNum; j++)
			{
				if (valid[j])
				{
					pr.frames.clear();
					pr.projected_result.clear();
					pr.depth.clear();
					pr.tracked_result.clear();
					pr.weight.clear();

					pr.Millisecond = Millisecond;
					pr.ptG = pts[j];
					pr.GtFrame = imageFrame;
					pr.frames.push_back(imageFrame);
					pr.projected_result.push_back(ptsI[j]);
					pr.depth.push_back(tmpZc[j]);
					pr.tracked_result.push_back(ptsI[j]);
					pr.weight.push_back(initWeight);
					prs.push_back(pr);
				}
			}
		}
		ep = prs.size();

	
		// 计算投影点特征
		yyfImage.CalcImageFeatureWeight(imageFrame,prs);


		// 计算w3，即同一条扫描线上的权值
		for (j=sp; j<ep; j++)
		{
			if (j==sp || j==ep-1)
				prs[j].weight[0].w[3] = 0.1;
			else
			{
				double tmp=calcAngle(prs[max(sp,j-2)].ptG,prs[j].ptG,prs[min(ep-1,j+2)].ptG);
				if (tmp>=1.2) //夹角大于45度
					prs[j].weight[0].w[3] = 0.1;
			}
		}


		// 求解深度图，计算遮挡的影响
		for (j=0; j<ptNum; j++)
		{
			if (valid[j])
			{
				ptV.push_back(pts[j]);
			}
		}
		if (isLUBound!=0)
		{
			readyToDraw++;
			if (isLUBound*preLUBound<0 && readyToDraw)
			{
				// 保存cirlce.txt
				ptVPre = ptV;
			}
			preLUBound = isLUBound;
			ptV.clear();
		}
		DrawOneSweep2Image(ptV,Millisecond,rC,tC);

		if (ptVPre.size()>0)
		{
			rMatrixCopy(irC,rC);
			rMatrixInverse(irC);
			itC.x = -tC.x;
			itC.y = -tC.y;
			itC.z = -tC.z;
			vector<point3d> ptVC(ptVPre);
			for (j=0; j<ptVC.size(); j++)
			{
				shiftPoint3d(ptVC[j],itC);
				rotatePoint3d(ptVC[j],irC);
			}
			yyfImage.CalcDepthImage(imageFrame,ptVC);
		}
		//yyfImage.CalcOcclusionWeight(imageFrame,prs);

		CalcPrsWeight(prs,imageFrame);

		/*
		//cleaning prs;
		bool flag;
		int len = 0;
		for (int i=0; i<prs.size(); i++)
		{
			flag = true;
			if (flag && prs[i].weight.rbegin()->w[0] < 0.3)
			{
				flag = false;
			}
			if (flag && getPPDistance(*prs[i].tracked_result.rbegin(),*prs[i].projected_result.rbegin()) > 15)
			{
				flag = false;
			}

			if (flag && len<i)
				prs[len++] = prs[i];
		}
		prs.resize(len);
		*/

		//yyfImage.ShowTrackedPoints(imageFrame,prs);

		imageFramePre = imageFrame;
	}
	return 0;
}