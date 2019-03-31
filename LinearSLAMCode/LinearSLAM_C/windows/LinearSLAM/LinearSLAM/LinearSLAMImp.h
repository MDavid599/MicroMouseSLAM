//==============================================================================================================
// University of Technology, Sydney, Australia
// 
// Authors:  Liang Zhao         -- liang.zhao@imperial.ac.uk 
// 		  Shoudong Huang     -- Shoudong.Huang@uts.edu.au
// 		  Gamini Dissanayake -- Gamini.Dissanayake@uts.edu.au
// 
// 		  Centre for Autonomous Systems
// 
// 		  Faculty of Engineering and Information Technology
// 
// 		  University of Technology, Sydney
// 
// 		  NSW 2007, Australia
// 
// 		  License
// 
// 		  Linear SLAM by Liang Zhao, Shoudong Huang, Gamini Dissanayake is licensed under a 
// 
// 		  Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
// 
// 		  Please contact Liang Zhao {liang.zhao@imperial.ac.uk} if you have any questions/comments about the code.
//==============================================================================================================

#pragma once
#define MAXSTRLEN  2048 /* 2K */
#define SKIP_LINE(f){                                                       \
	char buf[MAXSTRLEN];                                                        \
	while(!feof(f))                                                           \
	if(!fgets(buf, MAXSTRLEN-1, f) || buf[strlen(buf)-1]=='\n') break;      \
}

#include "cholmod.h"

#pragma  comment( lib, "../../bin/libcholmod.lib")
#pragma  comment( lib, "../../bin/libamd.lib")
#pragma  comment( lib, "../../bin/libcamd.lib")
#pragma  comment( lib, "../../bin/libccolamd.lib")
#pragma  comment( lib, "../../bin/libcolamd.lib")
#pragma  comment( lib, "../../bin/libmetis_CHOLMOD.lib")
#pragma  comment( lib, "../../bin/libgoto2.lib")
#include <vector>
#include <map>
#include <set>
#include <algorithm>  
#include <time.h>
using namespace std;

#define PI 3.1415926

typedef enum LinearSLAM_Method
{	
	LinearSLAM_Sequential = -1, //fix the longest axis  
	LinearSLAM_DivideConquer = 0,
} LinearSLAM_Method; 

typedef enum LinearSLAM_DataType
{
	LinearSLAM_2DPoseFeature = -1,
	LinearSLAM_3DPoseFeature = 0,
	LinearSLAM_2DPoseGraph = 1,
	LinearSLAM_3DPoseGraph = 2,
}LinearSLAM_DataType;

typedef struct LocalMapInfo
{
public:
	int r, Ref;
	int	*stno;
	int nzero;
	int sp;

	cholmod_sparse *  I;				//Information Matrix
	cholmod_dense*	 st;				//State Vector

	LocalMapInfo(){};
	LocalMapInfo( int row ){  r = row;	}

	void setDimension( int row ) { r = row; }

	LocalMapInfo& operator = (const LocalMapInfo& lm) 	
	{
		sp = lm.sp;
		nzero = lm.nzero;
		I = lm.I;
		st = lm.st;
		stno = lm.stno;
		r = lm.r;	Ref = lm.Ref;		
		return *this;
	}

} LocalMapInfo;


class CLinearSLAMImp
{
public:
	CLinearSLAMImp(void);
	~CLinearSLAMImp(void);

	//========================================================================================
	void	run( char* szState, char* szData, int nMapCount, LinearSLAM_DataType dataType, LinearSLAM_Method method = LinearSLAM_Sequential, char* szInfo = NULL, char* szPose = NULL, char* szFea = NULL );
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

private:


	//========================================================================================
	//load local maps
	void	lmj_loadLocalMaps( int nMapCount );
	int		lmj_readStateV( FILE* fpSt, LocalMapInfo &lm );	
	void	lmj_SaveStateVector( char* szSt, int* stno, double* st, int n );
	void	lmj_SaveInfomationMatrix( char* szInfo, double* I, int* Si, int *Sp, int n );

	void	lmj_SavePoses_2DPF( char* szPose, char* szFea, int* stno, double* st, int n );
	void	lmj_SavePoses_3DPF( char* szPose, char* szFea, int* stno, double* st, int n );
	void	lmj_SavePoses_2DPG( char* szPose, int* stno, double* st, int n );
	void	lmj_SavePoses_3DPG( char* szPose, int* stno, double* st, int n );
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//2D Pose Feature
	void	lmj_PF2D_Sequential( int nMapCount );
	void	lmj_PF2D_Divide_Conquer( int nLocalMapCount );
	void	lmj_Transform_PF2D( LocalMapInfo& GMap_End, int Ref );
	void	lmj_LinearLS_PF2D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur );
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//3D Pose Feature
	void	lmj_PF3D_Sequential( int nMapCount );
	void	lmj_PF3D_Divide_Conquer( int nLocalMapCount );
	void	lmj_Transform_PF3D( LocalMapInfo& GMap_End, int Ref );
	void	lmj_LinearLS_PF3D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur );
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//2D Pose Graph
	void	lmj_Wrapst( LocalMapInfo& GMap );
	void	lmj_PG2D_Sequential( int nMapCount );
	void	lmj_PG2D_Divide_Conquer( int nLocalMapCount );
	void	lmj_Transform_PG2D( LocalMapInfo& GMap_End, int Ref );
	void	lmj_LinearLS_PG2D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur );
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//3D Pose Graph
	void	lmj_Wrapst_3D( LocalMapInfo &GMap );
	void	lmj_PG3D_Sequential( int nMapCount );
	void	lmj_PG3D_Divide_Conquer( int nLocalMapCount );
	void	lmj_Transform_PG3D( LocalMapInfo& GMap_End, int Ref );
	void	lmj_LinearLS_PG3D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur );
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

private:
	char*			m_szPath;
	char*			m_szInfo;
	char*			m_szSt;
	char*			m_szPose;
	char*			m_szFea;
	LocalMapInfo*	m_LMset;
	LocalMapInfo	m_GMap;
	int				m_nMapCount;
	LinearSLAM_Method  m_method;
	LinearSLAM_DataType m_data;

private:
	//use cholmod mathmatic software package to solve linear equation
	cholmod_sparse *m_sparseS; 
	cholmod_factor *m_factorS; 
	cholmod_common m_cS; 
	cholmod_dense  *m_sparseR, *m_sparseE;
};

