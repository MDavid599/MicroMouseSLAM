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
#include "stdafx.h"
#include "LinearSLAMImp.h"


CLinearSLAMImp::CLinearSLAMImp(void)
{
	m_szSt   = NULL;
	m_szInfo = NULL;
	m_szPose = NULL;
	m_szFea  = NULL;
	cholmod_start (&m_cS);
}


CLinearSLAMImp::~CLinearSLAMImp(void)
{
	cholmod_finish (&m_cS);
}

bool IsOdd (int i) {  return (i<0);}

void CLinearSLAMImp::run( char* szState, char* szData, int nMapCount, LinearSLAM_DataType dataType, LinearSLAM_Method method, char* szInfo, char* szPose, char* szFea)
{
	m_szPath = szData;
	m_szSt   = szState; 
	m_szInfo = szInfo;
	m_szPose = szPose;
	m_szFea  = szFea;

	m_LMset = new LocalMapInfo[nMapCount];
	lmj_loadLocalMaps( nMapCount );


	if ( method == LinearSLAM_Sequential )
	{
		if ( dataType == LinearSLAM_2DPoseFeature )
			lmj_PF2D_Sequential( nMapCount );

		if ( dataType == LinearSLAM_3DPoseFeature )
			lmj_PF3D_Sequential( nMapCount );

		if ( dataType == LinearSLAM_2DPoseGraph )
			lmj_PG2D_Sequential( nMapCount );

		if ( dataType == LinearSLAM_3DPoseGraph )
			lmj_PG3D_Sequential( nMapCount );
	}

	if ( method == LinearSLAM_DivideConquer )
	{
		if ( dataType == LinearSLAM_2DPoseFeature )
			lmj_PF2D_Divide_Conquer( nMapCount );

		if ( dataType == LinearSLAM_3DPoseFeature )
			lmj_PF3D_Divide_Conquer( nMapCount );

		if ( dataType == LinearSLAM_2DPoseGraph )
			lmj_PG2D_Divide_Conquer( nMapCount );

		if ( dataType == LinearSLAM_3DPoseGraph )
			lmj_PG3D_Divide_Conquer( nMapCount );
	}

	free( m_LMset );
}


void CLinearSLAMImp::lmj_loadLocalMaps( int nMapCount )
{
	FILE* fpSt;
	int i, rows, tmp, n;

	//LocalMapInfo  *info = new LocalMapInfo[nMapCount];

	for ( i = 0; i < nMapCount; i++)
	{
		char szSt[200];
		strcpy( szSt, m_szPath );	
		char szPathExt[20];
		sprintf( szPathExt, "/localmap_%d.txt", i+1 );
		strcat( szSt, szPathExt );

		fpSt = fopen( szSt, "r" );

		if( fpSt == NULL )
		{ printf("There are no data in this path\n");	return ;}

		n = fscanf( fpSt, "%d", &tmp );
		if( n==-1)
			break; 
		n = fscanf( fpSt, "%d", &rows );
		if( n==-1)
			break;
		LocalMapInfo lm( rows );
		lm.Ref = tmp;

		int nMax = (rows*rows-rows)/2 + rows;
		lm.I      = cholmod_allocate_sparse( rows, rows, nMax,true,true,-1,CHOLMOD_REAL,&m_cS);
		lm.st     = cholmod_zeros( rows, 1, CHOLMOD_REAL, &m_cS);
		lm.stno   = (int*)malloc( rows*sizeof(int));

		lmj_readStateV( fpSt, lm );

		lm.Ref = i;
		m_LMset[i] = lm;		
		fclose(fpSt);
	}
}

int CLinearSLAMImp::lmj_readStateV( FILE* fpSt, LocalMapInfo &lm  )
{
	int i, j;
	i = 0;

	double* ptr1 = (double*)lm.st->x;
	int   * ptr2 = lm.stno;

	//read state vector
	int row = lm.r;
	for ( i = 0; i < row; i++ )
		j = fscanf( fpSt, "%d %lf", ptr2+i, ptr1+i );

	double val = 0;
	//read Information vector

	int* Si = (int*)lm.I->i;
	int* Sp = (int*)lm.I->p;
	double* Sx = (double*)lm.I->x;
	int nZ = 0;
	;
	for ( i = 0; i < row; i++ )
	{
		*Sp = nZ;
		for ( j = 0; j < row; j++ )
		{
			int n = fscanf( fpSt, "%lf", &val );
			if( n== -1 )
				break;

			if( j >= i )
			{
				if ( val != 0 )
				{
					*Si++ = j;
					*Sx++ = val;
					nZ++;
				}
			}
		}
		Sp++;
	}	
	*Sp = nZ;

	return i;
}



void CLinearSLAMImp::lmj_PF2D_Sequential( int nMapCount )
{
	int i;
	double t0, t1, t2;
	t0 = clock();
	for ( i = 0; i < nMapCount; i++ )
	{
		printf(  "Join Local Map %d\n", i+1 );

		if ( i == 0 )
		{
			m_GMap = m_LMset[0];			
		}
		else
		{
			LocalMapInfo GMap_End;
			lmj_Transform_PF2D( GMap_End, m_LMset[i].Ref );

			lmj_LinearLS_PF2D(GMap_End,m_LMset[i]);

			free(m_LMset[i].stno);
			cholmod_free_sparse( &m_LMset[i].I, &m_cS );
			cholmod_free_dense(&m_LMset[i].st, &m_cS );

			free(GMap_End.stno);
			cholmod_free_sparse( &GMap_End.I, &m_cS );
			cholmod_free_dense(&GMap_End.st, &m_cS );
		}
	}

	set<int> setLM;
	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = 0;
	for ( set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
	{
		if (*it<=0)
			m = -(*it);
		else
			break;
	}

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PF2D( GMapTmp, m );

		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;		
	}
	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_2DPF( m_szPose, m_szFea, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}

void CLinearSLAMImp::lmj_Transform_PF2D( LocalMapInfo& GMap_End, int Ref )
{

	int pos, i, n; 
	double t[2], phi, R[4], dR[4], dRt[2];
	double* ptr1, *ptr2;
	int* iter;
	int   * stno;
	n = m_GMap.r;

	if (m_GMap.Ref == Ref )
	{
		GMap_End = m_GMap;
	}
	else
	{
		//========================================================================================
		ptr1 = (double*)m_GMap.st->x;
		stno = m_GMap.stno;

		iter = find( stno, stno+n, -Ref );
		pos = iter - stno;

		t[0] = ptr1[pos];
		t[1] = ptr1[pos+1];
		phi  = ptr1[pos+2];

		R[0] = cos(phi);
		R[1] = sin(phi);
		R[2] = -sin(phi);
		R[3] = cos(phi);

		GMap_End.setDimension( n );
		GMap_End.st     = cholmod_zeros( n, 1, CHOLMOD_REAL, &m_cS);
		GMap_End.stno   = (int*)malloc( n*sizeof(int));

		ptr2 = (double*)GMap_End.st->x;
		memcpy( GMap_End.stno, stno, n*sizeof(int) );
		GMap_End.Ref = Ref;
		GMap_End.stno[pos] = GMap_End.stno[pos+1] = GMap_End.stno[pos+2] = -m_GMap.Ref;
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		for ( i=0; i<n; i++ )
		{
			if ( stno[i] <=0 )
			{
				if ( i == pos )
				{
					ptr2[i] = -(R[0]*t[0]+R[1]*t[1]);
					ptr2[i+1] = -(R[2]*t[0]+R[3]*t[1]);
					ptr2[i+2] = -phi;
				}
				else
				{
					ptr2[i]   =  R[0]*(ptr1[i]-t[0])+R[1]*(ptr1[i+1]-t[1]);
					ptr2[i+1] =  R[2]*(ptr1[i]-t[0])+R[3]*(ptr1[i+1]-t[1]);
					ptr2[i+2] = ptr1[i+2] - phi;
				}

				i+=2;
			}
			else
			{
				ptr2[i] =  R[0]*(ptr1[i]-t[0]) + R[1]*(ptr1[i+1]-t[1]);
				ptr2[i+1] =  R[2]*(ptr1[i]-t[0]) + R[3]*(ptr1[i+1]-t[1]);

				i = i+1;
			}
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		t[0] =	ptr2[pos];
		t[1] =	ptr2[pos+1];
		phi  =	ptr2[pos+2];

		R[0] = cos(phi);
		R[1] = sin(phi);
		R[2] = -sin(phi);
		R[3] = cos(phi);

		dR[0] = -sin(phi);
		dR[1] = cos(phi);
		dR[2] = -cos(phi);
		dR[3] = -sin(phi);

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		int nMax = (n-3)/2*10 + 12;
		cholmod_triplet* J = cholmod_allocate_triplet(n,n,nMax,0,CHOLMOD_REAL,&m_cS); 	
		int * triI = (int*)(J->i); 
		int * triJ =(int*)(J->j); 
		double* triVal=(double*)(J->x); 
		J->nnz =0; 		
		int sum = 0;
		for ( i = 0; i < n; i++ )
		{
			if (GMap_End.stno[i]<=0)
			{
				if ( i == pos )
				{
					dRt[0] = dR[0]*t[0] + dR[1]*t[1];
					dRt[1] = dR[2]*t[0] + dR[3]*t[1];

					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = pos;	triVal[sum] = -R[0];	
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = pos;	triVal[sum] = -R[2];	
					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = pos+1;	triVal[sum] = -R[1];	
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = pos+1;	triVal[sum] = -R[3];	
					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = pos+2;	triVal[sum] = -dRt[0];	
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = pos+2;	triVal[sum] = -dRt[1];
					sum = J->nnz++;
					triI[sum] = i+2;	triJ[sum] = pos+2;	triVal[sum] = -1;		

				}
				else
				{
					dRt[0] = dR[0]*(ptr2[i]-t[0]) + dR[1]*(ptr2[i+1]-t[1]);
					dRt[1] = dR[2]*(ptr2[i]-t[0]) + dR[3]*(ptr2[i+1]-t[1]);

					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = pos;		triVal[sum] = -R[0];	
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = pos;		triVal[sum] = -R[2];	
					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = pos+1;		triVal[sum] = -R[1];	
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = pos+1;		triVal[sum] = -R[3];	
					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = pos+2;		triVal[sum] = dRt[0];	
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = pos+2;		triVal[sum] = dRt[1];	
					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = i;			triVal[sum] = R[0];		
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = i;			triVal[sum] = R[2];		
					sum = J->nnz++;
					triI[sum] = i;		triJ[sum] = i+1;		triVal[sum] = R[1];		
					sum = J->nnz++;
					triI[sum] = i+1;	triJ[sum] = i+1;		triVal[sum] = R[3];		
					sum = J->nnz++;
					triI[sum] = i+2;	triJ[sum] = pos+2;		triVal[sum] = -1;		
					sum = J->nnz++;
					triI[sum] = i+2;	triJ[sum] = i+2;		triVal[sum] = 1;		

				}
				i += 2;
			}
			else
			{
				dRt[0] = dR[0]*(ptr2[i]-t[0]) + dR[1]*(ptr2[i+1]-t[1]);
				dRt[1] = dR[2]*(ptr2[i]-t[0]) + dR[3]*(ptr2[i+1]-t[1]);

				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos;		triVal[sum] = -R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos;		triVal[sum] = -R[2];
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos+1;		triVal[sum] = -R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos+1;		triVal[sum] = -R[3];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos+2;		triVal[sum] = dRt[0];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos+2;		triVal[sum] = dRt[1];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = i;			triVal[sum] = R[0];		
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = i;			triVal[sum] = R[2];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = i+1;		triVal[sum] = R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = i+1;		triVal[sum] = R[3];	

				i += 1;
			}
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		//J^T*I*J
		cholmod_sparse* Jsparse = cholmod_triplet_to_sparse( J, nMax, &m_cS);
		cholmod_free_triplet(&J, &m_cS) ;

		cholmod_sparse* JT = cholmod_transpose( Jsparse, Jsparse->xtype, &m_cS );
		cholmod_sparse* JTI = cholmod_ssmult( JT, m_GMap.I, 0, 1, 1, &m_cS);
		cholmod_sparse* JTIJ = cholmod_ssmult( JTI, Jsparse,-1, 1, 1, &m_cS);

		GMap_End.I = JTIJ;		

		cholmod_free_sparse(&Jsparse, &m_cS) ; 
		cholmod_free_sparse(&JT, &m_cS) ; 
		cholmod_free_sparse(&JTI, &m_cS) ; 
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}

}


void CLinearSLAMImp::lmj_LinearLS_PF2D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur )
{
	int n1, n2;
	int pos;
	int *ptr1, *ptr2;
	double *ptr3;
	int *ptr4, *ptr5;
	n1 = GMap_End.r;
	n2 = GMap_Cur.r;

	//st = [LM1_st, LM2_st]
	cholmod_dense* b = cholmod_zeros( n1+n2, 1, CHOLMOD_REAL, &m_cS);
	double* bx = (double*)b->x;
	memcpy( bx, (double*)GMap_End.st->x, sizeof(double)*n1 );
	memcpy( bx+n1, (double*)GMap_Cur.st->x, sizeof(double)*n2 );

	int nMax = GMap_End.I->nzmax + GMap_Cur.I->nzmax;

	//I = [LM1_I, LM2_I]
	cholmod_sparse* Inew  = cholmod_allocate_sparse( n1+n2, n1+n2, nMax,true,true,-1,CHOLMOD_REAL,&m_cS);

	ptr1 = (int*)Inew->i;
	ptr2 = (int*)Inew->p;
	ptr3 = (double *)Inew->x;

	ptr4 = (int*)GMap_Cur.I->i;
	ptr5 = (int*)GMap_Cur.I->p;

	memcpy( ptr1, (int*)GMap_End.I->i, sizeof(int)*GMap_End.I->nzmax );
	memcpy( ptr2, (int*)GMap_End.I->p, sizeof(int)*GMap_End.r );
	memcpy( ptr3, (double*)GMap_End.I->x, sizeof(double)*GMap_End.I->nzmax );

	int sum = GMap_End.I->nzmax;
	memcpy( ptr3+sum, (double*)GMap_Cur.I->x, sizeof(double)*GMap_Cur.I->nzmax );
	for ( int i = 0; i < (int)GMap_Cur.I->nzmax; i++ )
		ptr1[sum+i] = ptr4[i] + n1;

	for ( int i = 0; i <= n2; i++ )
		ptr2[n1+i] = ptr5[i] + sum;


	//count column of A. #feature*2 + pose*3
	nMax = n1 + (n2-3) + 3;
	set<int> setLM;
	setLM.insert( GMap_End.stno, GMap_End.stno+n1 );
	setLM.insert( GMap_Cur.stno, GMap_Cur.stno+n2 );
	int nLess0 = 0;

	for (set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
	{
		if ( *it > 0 )
			break;
		else
			nLess0++;
	}

	int nAcol = (setLM.size() - nLess0)*2 + nLess0*3;	
	cholmod_triplet* triA = cholmod_allocate_triplet( n1+n2, nAcol, nMax, 0, CHOLMOD_REAL, &m_cS ); 	

	int*  GMap_End_stno = GMap_End.stno;
	int*  GMap_Cur_stno = GMap_Cur.stno;

	int* XGID = (int*)malloc((nAcol)*sizeof(int));
	memcpy( XGID, GMap_End_stno, n1*sizeof(int) );

	int* triAI = (int*)triA->i;
	int* triAJ = (int*)triA->j;
	double* triAVal = (double*)triA->x;
	triA->nnz=0;

	int Pn, Fn;
	int* iter;
	sum = n1;
	int Xn = n1;

	for (int i = 0; i < n1; i++ ) 
	{
		pos = triA->nnz++;
		triAI[pos] = i;		triAJ[pos] = i;		triAVal[pos] = 1;
	}

	for ( int i = 0; i < n2; i++ )
	{
		if (GMap_Cur_stno[i]<=0)
		{
			Pn = GMap_Cur_stno[i];

			XGID[sum] = XGID[sum+1] = XGID[sum+2] = Pn;

			pos = triA->nnz++;
			triAI[pos] = n1+i;		triAJ[pos] = Xn;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+1;	triAJ[pos] = Xn+1;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+2;	triAJ[pos] = Xn+2;	triAVal[pos] = 1;

			i = i+2;
			sum += 3;
			Xn +=3;
		}
		else
		{
			Fn = GMap_Cur_stno[i];
			iter = find( XGID, XGID+sum, Fn );

			if (iter!=XGID+sum)
			{
				pos = triA->nnz++;
				triAI[pos] = n1+i;		triAJ[pos] = iter-XGID;		triAVal[pos] = 1;
				pos = triA->nnz++;
				triAI[pos] = n1+i+1;	triAJ[pos] = iter-XGID+1;	triAVal[pos] = 1;
			}
			else
			{
				XGID[sum] = XGID[sum+1] = Fn;

				pos = triA->nnz++;
				triAI[pos] = n1+i;		triAJ[pos] = Xn;		triAVal[pos] = 1;
				pos = triA->nnz++;
				triAI[pos] = n1+i+1;	triAJ[pos] = Xn+1;		triAVal[pos] = 1;

				Xn+=2;
				sum += 2;
			}

			i+=1;
		}
	}

	//========================================================================================
	cholmod_sparse* A = cholmod_triplet_to_sparse( triA, triA->nnz, &m_cS );
	cholmod_free_triplet(&triA, &m_cS);

	//A^T*I*A
	cholmod_sparse* AT = cholmod_transpose( A, A->xtype, &m_cS );
	cholmod_sparse* ATI = cholmod_ssmult( AT, Inew, 0, 1, 1, &m_cS);

	cholmod_sparse* ATIA = cholmod_ssmult( ATI, A, -1, 1, 1, &m_cS);

	//A^T*I*b
	double alpha[2] = {1, 1}, belta[2] = {0,0};
	cholmod_dense* ATIb = cholmod_zeros( nAcol, 1, CHOLMOD_REAL, &m_cS);
	cholmod_sdmult(ATI, 0, alpha, belta, b, ATIb, &m_cS );
	cholmod_free_dense( &b, &m_cS );
	cholmod_free_sparse(&A, &m_cS);
	cholmod_free_sparse(&AT, &m_cS);
	cholmod_free_sparse(&ATI, &m_cS );
	cholmod_free_sparse( &Inew, &m_cS );

	//solve (A^T*I*A)*X = A^T*I*b
	cholmod_factor* factor = cholmod_analyze( ATIA, &m_cS);
	cholmod_factorize( ATIA, factor, &m_cS);
	cholmod_dense* denseG = cholmod_solve ( CHOLMOD_A, factor, ATIb, &m_cS ) ;	

	cholmod_free_sparse( &m_GMap.I, &m_cS );
	m_GMap.I = ATIA;
	cholmod_free_factor( &factor, &m_cS );

	m_GMap.r = nAcol;
	cholmod_free_dense( &m_GMap.st, &m_cS );
	m_GMap.st = denseG;
	free( m_GMap.stno );
	m_GMap.stno = (int*)malloc(nAcol*sizeof(double));
	memcpy( m_GMap.stno, XGID, sizeof(int)*nAcol );
	m_GMap.Ref = GMap_Cur.Ref;
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	free(XGID);
}

void CLinearSLAMImp::lmj_PF2D_Divide_Conquer( int nLocalMapCount )
{
	double t0, t1, t2;
	t0 = clock();
	int L = 1;
	set<int> setLM;
	while( nLocalMapCount>1 )
	{
		int N2 = nLocalMapCount%2;
		nLocalMapCount = int(nLocalMapCount/2.0 + 0.5);

		int NumLM;
		for ( int i = 0; i < nLocalMapCount; i++ )
		{
			if ( i < nLocalMapCount-1 )
				NumLM = 2;
			else
			{
				if ( N2 == 0 )
					NumLM = 2;
				else
					NumLM = 1;
			}

			for ( int j = 0; j < NumLM; j++ )
			{
				printf( "Join Level %d Local Map %d\n", L, 2*i+j+1 );

				if ( j == 0 )
				{
					m_GMap = m_LMset[2*i+j];
				}
				else
				{
					LocalMapInfo GMap_End;
					lmj_Transform_PF2D( GMap_End, m_LMset[2*i+j].Ref );

					lmj_LinearLS_PF2D(GMap_End,m_LMset[2*i+j]);

					free(m_LMset[2*i+j].stno);
					cholmod_free_sparse( &m_LMset[2*i+j].I, &m_cS);
					cholmod_l_free_dense(&m_LMset[2*i+j].st, &m_cS );

					free(GMap_End.stno);
					cholmod_free_sparse(&GMap_End.I, &m_cS) ; 
					cholmod_l_free_dense(&GMap_End.st, &m_cS );
				}


			}

printf( "Generate Level %d Local Map %d\n\n", L+1, i+1 );


			setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
			int tmp = 0;
			for ( set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
			{
				if ( *it<=0)
					tmp = -(*it);
				else
					break;
			}
			setLM.clear();

			if ( (i+1)%2 == 0 )
			{
				int m = tmp;

				if ( m_GMap.Ref > m )
				{
					LocalMapInfo GMapTmp;
					lmj_Transform_PF2D( GMapTmp, m );

					free(m_GMap.stno);
					cholmod_free_sparse(&m_GMap.I, &m_cS) ;
					cholmod_l_free_dense(&m_GMap.st, &m_cS );
					m_GMap = GMapTmp;
				}
			}

			m_LMset[i] = m_GMap;
		}

		L++;
	}

	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = 0;
	for ( set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
	{
		if ( *it<=0)
			m = -(*it);
		else
			break;
	}

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PF2D( GMapTmp, m );

		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;
	}

	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_2DPF( m_szPose, m_szFea, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}

void CLinearSLAMImp::lmj_PF3D_Sequential( int nMapCount )
{
	int i;
	double t0, t1, t2;
	t0 = clock();
	for ( i = 0; i < nMapCount; i++ )
	{
		printf(  "Join Local Map %d\n", i+1 );

		if ( i == 0 )
		{
			m_GMap = m_LMset[0];			
		}
		else
		{
			LocalMapInfo GMap_End;
			lmj_Transform_PF3D( GMap_End, m_LMset[i].Ref );

			lmj_LinearLS_PF3D(GMap_End,m_LMset[i]);

			free(m_LMset[i].stno);
			cholmod_free_sparse( &m_LMset[i].I, &m_cS );
			cholmod_free_dense(&m_LMset[i].st, &m_cS );

			free(GMap_End.stno);
			cholmod_free_sparse( &GMap_End.I, &m_cS );
			cholmod_free_dense(&GMap_End.st, &m_cS );
		}
	}

	set<int> setLM;
	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = 0;
	for ( set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
	{
		if (*it<=0)
			m = -(*it);
		else
			break;
	}

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PF3D( GMapTmp, m );

		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;		
	}
	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_3DPF( m_szPose, m_szFea, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}

static void lmj_RMatrixYPR22( double* R, double Alpha, double Beta, double Gamma )
{
	R[0] = cos(Beta)*cos(Alpha);
	R[1] = cos(Beta)*sin(Alpha);
	R[2] = -sin(Beta);
	R[3] = sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha);
	R[4] = sin(Gamma)*sin(Beta)*sin(Alpha)+cos(Gamma)*cos(Alpha);
	R[5] = sin(Gamma)*cos(Beta);
	R[6] = cos(Gamma)*sin(Beta)*cos(Alpha)+sin(Gamma)*sin(Alpha);
	R[7] = cos(Gamma)*sin(Beta)*sin(Alpha)-sin(Gamma)*cos(Alpha);
	R[8] = cos(Gamma)*cos(Beta);
}

static void lmj_InvRotMatrixYPR22T(double*R, double &alpha, double &beta, double &gamma )
{
	beta = atan2( -R[6], sqrt(R[0]*R[0]+R[3]*R[3]) );

	if ( cos(beta)==0 )
	{
		alpha = 0; 
		beta = PI/2;
		gamma = atan2( R[3], R[4]);
	}
	else
	{
		alpha = atan2( R[3]/cos(beta), R[0]/cos(beta) );
		gamma = atan2( R[7]/cos(beta), R[8]/cos(beta) );
	}
}

static void lmj_InvRotMatrixYPR22(double*R, double &alpha, double &beta, double &gamma )
{
	beta = atan2( -R[2], sqrt(R[0]*R[0]+R[1]*R[1]) );

	if ( cos(beta)==0 )
	{
		alpha = 0; 
		beta = PI/2;
		gamma = atan2( R[1], R[4]);
	}
	else
	{
		alpha = atan2( R[1]/cos(beta), R[0]/cos(beta) );
		gamma = atan2( R[5]/cos(beta), R[8]/cos(beta) );
	}
}

static void lmj_Rderivation( double Alpha, double Beta, double Gamma, double* matR, double* dRA, double* dRB, double*dRG )
{
	matR[0] = cos(Beta) * cos(Alpha);
	matR[1] = cos(Beta) * sin(Alpha);
	matR[2] = -sin(Beta);
	matR[3] = sin(Gamma)*sin(Beta)*cos(Alpha)-cos(Gamma)*sin(Alpha);
	matR[4] = sin(Gamma)*sin(Beta)*sin(Alpha)+cos(Gamma)*cos(Alpha);
	matR[5] = sin(Gamma)*cos(Beta);
	matR[6] = cos(Gamma)*sin(Beta)*cos(Alpha) + sin(Gamma)*sin(Alpha);
	matR[7] = cos(Gamma)*sin(Beta)*sin(Alpha) - sin(Gamma)*cos(Alpha);
	matR[8] = cos(Gamma)*cos(Beta);

	double matRG[9], matRB[9], matRA[9];
	matRG[0] = 1;		matRG[1] = 0;				matRG[2] = 0;
	matRG[3] = 0;		matRG[4] = cos(Gamma);		matRG[5] = sin(Gamma);
	matRG[6] = 0;		matRG[7] = -sin(Gamma);		matRG[8] = cos(Gamma);

	matRB[0] = cos(Beta);		matRB[1] = 0;		matRB[2] = -sin(Beta);
	matRB[3] = 0;				matRB[4] = 1;		matRB[5] = 0;
	matRB[6] = sin(Beta);		matRB[7] = 0;		matRB[8] = cos(Beta);

	matRA[0] = cos(Alpha);		matRA[1] = sin(Alpha);			matRA[2] = 0;
	matRA[3] = -sin(Alpha);		matRA[4] = cos(Alpha);			matRA[5] = 0;
	matRA[6] = 0;				matRA[7] = 0;					matRA[8] = 1;

	double matDRG[9], matDRB[9], matDRA[9];
	matDRG[0] = 0;		matDRG[1] = 0;				matDRG[2] = 0;
	matDRG[3] = 0;		matDRG[4] = -sin(Gamma);	matDRG[5] = cos(Gamma);
	matDRG[6] = 0;		matDRG[7] = -cos(Gamma);	matDRG[8] = -sin(Gamma);

	matDRB[0] = -sin(Beta);		matDRB[1] = 0;		matDRB[2] = -cos(Beta);
	matDRB[3] = 0;				matDRB[4] = 0;		matDRB[5] = 0;
	matDRB[6] = cos(Beta);		matDRB[7] = 0;		matDRB[8] = -sin(Beta);

	matDRA[0] = -sin(Alpha);		matDRA[1] = cos(Alpha);			matDRA[2] = 0;
	matDRA[3] = -cos(Alpha);		matDRA[4] = -sin(Alpha);		matDRA[5] = 0;
	matDRA[6] = 0;					matDRA[7] = 0;					matDRA[8] = 0;

	//dRG
	double tmp1[9];
	tmp1[0] = matDRG[0]*matRB[0]+matDRG[1]*matRB[3]+matDRG[2]*matRB[6];
	tmp1[1] = matDRG[0]*matRB[1]+matDRG[1]*matRB[4]+matDRG[2]*matRB[7];
	tmp1[2] = matDRG[0]*matRB[2]+matDRG[1]*matRB[5]+matDRG[2]*matRB[8];
	tmp1[3] = matDRG[3]*matRB[0]+matDRG[4]*matRB[3]+matDRG[5]*matRB[6];
	tmp1[4] = matDRG[3]*matRB[1]+matDRG[4]*matRB[4]+matDRG[5]*matRB[7];
	tmp1[5] = matDRG[3]*matRB[2]+matDRG[4]*matRB[5]+matDRG[5]*matRB[8];
	tmp1[6] = matDRG[6]*matRB[0]+matDRG[7]*matRB[3]+matDRG[8]*matRB[6];
	tmp1[7] = matDRG[6]*matRB[1]+matDRG[7]*matRB[4]+matDRG[8]*matRB[7];
	tmp1[8] = matDRG[6]*matRB[2]+matDRG[7]*matRB[5]+matDRG[8]*matRB[8];

	dRG[0] = tmp1[0]*matRA[0]+tmp1[1]*matRA[3]+tmp1[2]*matRA[6];
	dRG[1] = tmp1[0]*matRA[1]+tmp1[1]*matRA[4]+tmp1[2]*matRA[7];
	dRG[2] = tmp1[0]*matRA[2]+tmp1[1]*matRA[5]+tmp1[2]*matRA[8];
	dRG[3] = tmp1[3]*matRA[0]+tmp1[4]*matRA[3]+tmp1[5]*matRA[6];
	dRG[4] = tmp1[3]*matRA[1]+tmp1[4]*matRA[4]+tmp1[5]*matRA[7];
	dRG[5] = tmp1[3]*matRA[2]+tmp1[4]*matRA[5]+tmp1[5]*matRA[8];
	dRG[6] = tmp1[6]*matRA[0]+tmp1[7]*matRA[3]+tmp1[8]*matRA[6];
	dRG[7] = tmp1[6]*matRA[1]+tmp1[7]*matRA[4]+tmp1[8]*matRA[7];
	dRG[8] = tmp1[6]*matRA[2]+tmp1[7]*matRA[5]+tmp1[8]*matRA[8];

	//dRB
	tmp1[0] = matRG[0]*matDRB[0]+matRG[1]*matDRB[3]+matRG[2]*matDRB[6];
	tmp1[1] = matRG[0]*matDRB[1]+matRG[1]*matDRB[4]+matRG[2]*matDRB[7];
	tmp1[2] = matRG[0]*matDRB[2]+matRG[1]*matDRB[5]+matRG[2]*matDRB[8];
	tmp1[3] = matRG[3]*matDRB[0]+matRG[4]*matDRB[3]+matRG[5]*matDRB[6];
	tmp1[4] = matRG[3]*matDRB[1]+matRG[4]*matDRB[4]+matRG[5]*matDRB[7];
	tmp1[5] = matRG[3]*matDRB[2]+matRG[4]*matDRB[5]+matRG[5]*matDRB[8];
	tmp1[6] = matRG[6]*matDRB[0]+matRG[7]*matDRB[3]+matRG[8]*matDRB[6];
	tmp1[7] = matRG[6]*matDRB[1]+matRG[7]*matDRB[4]+matRG[8]*matDRB[7];
	tmp1[8] = matRG[6]*matDRB[2]+matRG[7]*matDRB[5]+matRG[8]*matDRB[8];

	dRB[0] = tmp1[0]*matRA[0]+tmp1[1]*matRA[3]+tmp1[2]*matRA[6];
	dRB[1] = tmp1[0]*matRA[1]+tmp1[1]*matRA[4]+tmp1[2]*matRA[7];
	dRB[2] = tmp1[0]*matRA[2]+tmp1[1]*matRA[5]+tmp1[2]*matRA[8];
	dRB[3] = tmp1[3]*matRA[0]+tmp1[4]*matRA[3]+tmp1[5]*matRA[6];
	dRB[4] = tmp1[3]*matRA[1]+tmp1[4]*matRA[4]+tmp1[5]*matRA[7];
	dRB[5] = tmp1[3]*matRA[2]+tmp1[4]*matRA[5]+tmp1[5]*matRA[8];
	dRB[6] = tmp1[6]*matRA[0]+tmp1[7]*matRA[3]+tmp1[8]*matRA[6];
	dRB[7] = tmp1[6]*matRA[1]+tmp1[7]*matRA[4]+tmp1[8]*matRA[7];
	dRB[8] = tmp1[6]*matRA[2]+tmp1[7]*matRA[5]+tmp1[8]*matRA[8];

	//dRA
	tmp1[0] = matRG[0]*matRB[0]+matRG[1]*matRB[3]+matRG[2]*matRB[6];
	tmp1[1] = matRG[0]*matRB[1]+matRG[1]*matRB[4]+matRG[2]*matRB[7];
	tmp1[2] = matRG[0]*matRB[2]+matRG[1]*matRB[5]+matRG[2]*matRB[8];
	tmp1[3] = matRG[3]*matRB[0]+matRG[4]*matRB[3]+matRG[5]*matRB[6];
	tmp1[4] = matRG[3]*matRB[1]+matRG[4]*matRB[4]+matRG[5]*matRB[7];
	tmp1[5] = matRG[3]*matRB[2]+matRG[4]*matRB[5]+matRG[5]*matRB[8];
	tmp1[6] = matRG[6]*matRB[0]+matRG[7]*matRB[3]+matRG[8]*matRB[6];
	tmp1[7] = matRG[6]*matRB[1]+matRG[7]*matRB[4]+matRG[8]*matRB[7];
	tmp1[8] = matRG[6]*matRB[2]+matRG[7]*matRB[5]+matRG[8]*matRB[8];

	dRA[0] = tmp1[0]*matDRA[0]+tmp1[1]*matDRA[3]+tmp1[2]*matDRA[6];
	dRA[1] = tmp1[0]*matDRA[1]+tmp1[1]*matDRA[4]+tmp1[2]*matDRA[7];
	dRA[2] = tmp1[0]*matDRA[2]+tmp1[1]*matDRA[5]+tmp1[2]*matDRA[8];
	dRA[3] = tmp1[3]*matDRA[0]+tmp1[4]*matDRA[3]+tmp1[5]*matDRA[6];
	dRA[4] = tmp1[3]*matDRA[1]+tmp1[4]*matDRA[4]+tmp1[5]*matDRA[7];
	dRA[5] = tmp1[3]*matDRA[2]+tmp1[4]*matDRA[5]+tmp1[5]*matDRA[8];
	dRA[6] = tmp1[6]*matDRA[0]+tmp1[7]*matDRA[3]+tmp1[8]*matDRA[6];
	dRA[7] = tmp1[6]*matDRA[1]+tmp1[7]*matDRA[4]+tmp1[8]*matDRA[7];
	dRA[8] = tmp1[6]*matDRA[2]+tmp1[7]*matDRA[5]+tmp1[8]*matDRA[8];
}

static void lmj_dRi( double* dRid, double* dRi, double* Ri )
{
	double F1, F2, F3, F4, F5, dAdF1, dBdF2, dGdF3, dF1d, dF2d, dF3d, dF4d, dF5d, dF4dF5;

	F1 = Ri[1]/Ri[0];
	F3 = Ri[5]/Ri[8];
	F5 = Ri[0]*Ri[0] + Ri[1]*Ri[1];
	F4 = sqrt(F5);
	F2 = -Ri[2]/F4;

	dAdF1 = 1.0/(1+F1*F1);
	dBdF2 = 1.0/(1+F2*F2);
	dGdF3 = 1.0/(1+F3*F3);

	dF1d = (dRi[1]*Ri[0]-Ri[1]*dRi[0])/(Ri[0]*Ri[0]);
	dF3d = (dRi[5]*Ri[8]-Ri[5]*dRi[8])/(Ri[8]*Ri[8]);

	dF4dF5 = 1.0/(2*sqrt(F5));
	dF5d = 2*Ri[0]*dRi[0] + 2*Ri[1]*dRi[1];
	dF4d = dF4dF5 *dF5d;

	dF2d = (-dRi[2]*F4+Ri[2]*dF4d)/F5;
	dRid[0] = dAdF1*dF1d;
	dRid[1] = dBdF2*dF2d;
	dRid[2] = dGdF3*dF3d;
}

static void lmj_dRiTT( double* dRid, double* dRi, double* Ri )
{
	double F1, F2, F3, F4, F5, dAdF1, dBdF2, dGdF3, dF1d, dF2d, dF3d, dF4d, dF5d, dF4dF5;

	F1 = Ri[3]/Ri[0];
	F3 = Ri[7]/Ri[8];
	F5 = Ri[0]*Ri[0] + Ri[3]*Ri[3];
	F4 = sqrt(F5);
	F2 = -Ri[6]/F4;

	dAdF1 = 1.0/(1+F1*F1);
	dBdF2 = 1.0/(1+F2*F2);
	dGdF3 = 1.0/(1+F3*F3);

	dF1d = (dRi[3]*Ri[0]-Ri[3]*dRi[0])/(Ri[0]*Ri[0]);
	dF3d = (dRi[7]*Ri[8]-Ri[7]*dRi[8])/(Ri[8]*Ri[8]);

	dF4dF5 = 1.0/(2*sqrt(F5));
	dF5d = 2*Ri[0]*dRi[0] + 2*Ri[3]*dRi[3];
	dF4d = dF4dF5 *dF5d;

	dF2d = (-dRi[6]*F4+Ri[6]*dF4d)/F5;
	dRid[0] = dAdF1*dF1d;
	dRid[1] = dBdF2*dF2d;
	dRid[2] = dGdF3*dF3d;
}

static void lmj_TimesRRT( double*R3, double* R1, double* R2)
{
	R3[0] = R1[0]*R2[0] + R1[1]*R2[1] + R1[2]*R2[2];
	R3[1] = R1[0]*R2[3] + R1[1]*R2[4] + R1[2]*R2[5];
	R3[2] = R1[0]*R2[6] + R1[1]*R2[7] + R1[2]*R2[8];
	R3[3] = R1[3]*R2[0] + R1[4]*R2[1] + R1[5]*R2[2];
	R3[4] = R1[3]*R2[3] + R1[4]*R2[4] + R1[5]*R2[5];
	R3[5] = R1[3]*R2[6] + R1[4]*R2[7] + R1[5]*R2[8];
	R3[6] = R1[6]*R2[0] + R1[7]*R2[1] + R1[8]*R2[2];
	R3[7] = R1[6]*R2[3] + R1[7]*R2[4] + R1[8]*R2[5];
	R3[8] = R1[6]*R2[6] + R1[7]*R2[7] + R1[8]*R2[8];
}

void CLinearSLAMImp::lmj_Transform_PF3D( LocalMapInfo& GMap_End, int Ref )
{

	int pos, i, n; 
	double t[3], Alpha, Beta, Gamma, R[9], R2[9], R3[9], dRA[9], dRB[9], dRG[9];
	double dA[3], dB[3], dG[3], dRA2[9], dRB2[9], dRG2[9];
	double t2[3],  Alpha2, Beta2, Gamma2, Ri[9], tmp1[3], tmp2[3], tmp3[3];
	double ddA2[3], ddB2[3], ddG2[3], ddA[3], ddB[3], ddG[3];
	double* ptr1, *ptr2;
	int* iter;
	int   * stno;
	n = m_GMap.r;

	if (m_GMap.Ref == Ref )
	{
		GMap_End = m_GMap;
	}
	else
	{
		//========================================================================================
		ptr1 = (double*)m_GMap.st->x;
		stno = m_GMap.stno;

		iter = find( stno, stno+n, -Ref );
		pos = iter - stno;

		t[0] = ptr1[pos];
		t[1] = ptr1[pos+1];
		t[2] = ptr1[pos+2];

		Alpha  = ptr1[pos+3];
		Beta   = ptr1[pos+4];
		Gamma  = ptr1[pos+5];

		lmj_RMatrixYPR22( R, Alpha, Beta, Gamma );

		GMap_End.setDimension( n );
		GMap_End.st     = cholmod_zeros( n, 1, CHOLMOD_REAL, &m_cS);
		GMap_End.stno   = (int*)malloc( n*sizeof(int));

		ptr2 = (double*)GMap_End.st->x;
		memcpy( GMap_End.stno, stno, n*sizeof(int) );
		GMap_End.Ref = Ref;
		GMap_End.stno[pos] = GMap_End.stno[pos+1] = GMap_End.stno[pos+2] =
			GMap_End.stno[pos+3] = GMap_End.stno[pos+4] = GMap_End.stno[pos+5] = -m_GMap.Ref;
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		for ( i=0; i<n; i++ )
		{
			if ( stno[i] <=0 )
			{
				if ( i == pos )
				{
					ptr2[i]	  = -( R[0]*t[0]+R[1]*t[1]+R[2]*t[2] );
					ptr2[i+1] = -( R[3]*t[0]+R[4]*t[1]+R[5]*t[2] );
					ptr2[i+2] = -( R[6]*t[0]+R[7]*t[1]+R[8]*t[2] );

					lmj_InvRotMatrixYPR22T( R, ptr2[i+3], ptr2[i+4], ptr2[i+5] );					
				}
				else
				{
					ptr2[i]	  = ( R[0]*(ptr1[i]-t[0])+R[1]*(ptr1[i+1]-t[1])+R[2]*(ptr1[i+2]-t[2]) );
					ptr2[i+1] = ( R[3]*(ptr1[i]-t[0])+R[4]*(ptr1[i+1]-t[1])+R[5]*(ptr1[i+2]-t[2]) );
					ptr2[i+2] = ( R[6]*(ptr1[i]-t[0])+R[7]*(ptr1[i+1]-t[1])+R[8]*(ptr1[i+2]-t[2]) );

					lmj_RMatrixYPR22( R2, ptr1[i+3], ptr1[i+4], ptr1[i+5] );

					lmj_TimesRRT( R3, R2, R );
					lmj_InvRotMatrixYPR22( R3, ptr2[i+3], ptr2[i+4], ptr2[i+5] );
				}

				i+=5;
			}
			else
			{
				ptr2[i]	  = ( R[0]*(ptr1[i]-t[0])+R[1]*(ptr1[i+1]-t[1])+R[2]*(ptr1[i+2]-t[2]) );
				ptr2[i+1] = ( R[3]*(ptr1[i]-t[0])+R[4]*(ptr1[i+1]-t[1])+R[5]*(ptr1[i+2]-t[2]) );
				ptr2[i+2] = ( R[6]*(ptr1[i]-t[0])+R[7]*(ptr1[i+1]-t[1])+R[8]*(ptr1[i+2]-t[2]) );

				i = i+2;
			}
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		t[0] =	ptr2[pos];
		t[1] =	ptr2[pos+1];
		t[2]  =	ptr2[pos+2];

		Alpha = ptr2[pos+3];
		Beta  = ptr2[pos+4];
		Gamma = ptr2[pos+5];

		lmj_Rderivation( Alpha, Beta, Gamma, R, dRA, dRB, dRG );

		lmj_dRiTT( dA, dRA, R );
		lmj_dRiTT( dB, dRB, R );
		lmj_dRiTT( dG, dRG, R );
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		int nMax = (n-3)/2*27 + 16*3;
		cholmod_triplet* J = cholmod_allocate_triplet(n,n,nMax,0,CHOLMOD_REAL,&m_cS); 	
		int * triI = (int*)(J->i); 
		int * triJ =(int*)(J->j); 
		double* triVal=(double*)(J->x); 
		J->nnz =0; 		
		int sum = 0;
		for ( i = 0; i < n; i++ )
		{

			if (GMap_End.stno[i]<=0)
			{
				if ( i == pos )
				{
					tmp1[0] = (dRA[0]*t[0]+dRA[1]*t[1]+dRA[2]*t[2]);
					tmp1[1] = (dRA[3]*t[0]+dRA[4]*t[1]+dRA[5]*t[2]);
					tmp1[2] = (dRA[6]*t[0]+dRA[7]*t[1]+dRA[8]*t[2]);

					tmp2[0] = (dRB[0]*t[0]+dRB[1]*t[1]+dRB[2]*t[2]);
					tmp2[1] = (dRB[3]*t[0]+dRB[4]*t[1]+dRB[5]*t[2]);
					tmp2[2] = (dRB[6]*t[0]+dRB[7]*t[1]+dRB[8]*t[2]);

					tmp3[0] = (dRG[0]*t[0]+dRG[1]*t[1]+dRG[2]*t[2]);
					tmp3[1] = (dRG[3]*t[0]+dRG[4]*t[1]+dRG[5]*t[2]);
					tmp3[2] = (dRG[6]*t[0]+dRG[7]*t[1]+dRG[8]*t[2]);

					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos;	triVal[sum] = -R[0];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos;	triVal[sum] = -R[3];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos;	triVal[sum] = -R[6];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+1;	triVal[sum] = -R[1];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+1;	triVal[sum] = -R[4];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+1;	triVal[sum] = -R[7];
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+2;	triVal[sum] = -R[2];		
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+2;	triVal[sum] = -R[5];
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+2;	triVal[sum] = -R[8];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+3;	triVal[sum] = -tmp1[0];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+3;	triVal[sum] = -tmp1[1];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+3;	triVal[sum] = -tmp1[2];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+4;	triVal[sum] = -tmp2[0];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+4;	triVal[sum] = -tmp2[1];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+4;	triVal[sum] = -tmp2[2];
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+5;	triVal[sum] = -tmp3[0];		
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+5;	triVal[sum] = -tmp3[1];
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+5;	triVal[sum] = -tmp3[2];	
					sum = J->nnz++;
					triI[sum] = i+3;		triJ[sum] = pos+3;	triVal[sum] = dA[0];	
					sum = J->nnz++;
					triI[sum] = i+4;		triJ[sum] = pos+3;	triVal[sum] = dA[1];	
					sum = J->nnz++;
					triI[sum] = i+5;		triJ[sum] = pos+3;	triVal[sum] = dA[2];	
					sum = J->nnz++;
					triI[sum] = i+3;		triJ[sum] = pos+4;	triVal[sum] = dB[0];	
					sum = J->nnz++;
					triI[sum] = i+4;		triJ[sum] = pos+4;	triVal[sum] = dB[1];	
					sum = J->nnz++;
					triI[sum] = i+5;		triJ[sum] = pos+4;	triVal[sum] = dB[2];
					sum = J->nnz++;
					triI[sum] = i+3;		triJ[sum] = pos+5;	triVal[sum] = dG[0];		
					sum = J->nnz++;
					triI[sum] = i+4;		triJ[sum] = pos+5;	triVal[sum] = dG[1];
					sum = J->nnz++;
					triI[sum] = i+5;		triJ[sum] = pos+5;	triVal[sum] = dG[2];	

				}
				else
				{
					t2[0] =	ptr2[i];
					t2[1] =	ptr2[i+1];
					t2[2] = ptr2[i+2];

					Alpha2 = ptr2[i+3];
					Beta2  = ptr2[i+4];
					Gamma2 = ptr2[i+5];

					lmj_Rderivation( Alpha2, Beta2, Gamma2, R2, dRA2, dRB2, dRG2 );

					lmj_TimesRRT( Ri, R2, R );

					double dRidA2[9], dRidB2[9], dRidG2[9];

					lmj_TimesRRT( dRidA2, dRA2, R );
					lmj_TimesRRT( dRidB2, dRB2, R );
					lmj_TimesRRT( dRidG2, dRG2, R );


					double dRidA[9], dRidB[9], dRidG[9];
					lmj_TimesRRT( dRidA, R2, dRA );
					lmj_TimesRRT( dRidB, R2, dRB );
					lmj_TimesRRT( dRidG, R2, dRG );


					lmj_dRi( ddA2, dRidA2, Ri );
					lmj_dRi( ddB2, dRidB2, Ri );
					lmj_dRi( ddG2, dRidG2, Ri );

					lmj_dRi( ddA, dRidA, Ri );
					lmj_dRi( ddB, dRidB, Ri );
					lmj_dRi( ddG, dRidG, Ri );

					tmp1[0] = dRA[0]*(t2[0]-t[0]) + dRA[1]*(t2[1]-t[1]) + dRA[2]*(t2[2]-t[2]);
					tmp1[1] = dRA[3]*(t2[0]-t[0]) + dRA[4]*(t2[1]-t[1]) + dRA[5]*(t2[2]-t[2]);
					tmp1[2] = dRA[6]*(t2[0]-t[0]) + dRA[7]*(t2[1]-t[1]) + dRA[8]*(t2[2]-t[2]);

					tmp2[0] = dRB[0]*(t2[0]-t[0]) + dRB[1]*(t2[1]-t[1]) + dRB[2]*(t2[2]-t[2]);
					tmp2[1] = dRB[3]*(t2[0]-t[0]) + dRB[4]*(t2[1]-t[1]) + dRB[5]*(t2[2]-t[2]);
					tmp2[2] = dRB[6]*(t2[0]-t[0]) + dRB[7]*(t2[1]-t[1]) + dRB[8]*(t2[2]-t[2]);

					tmp3[0] = dRG[0]*(t2[0]-t[0]) + dRG[1]*(t2[1]-t[1]) + dRG[2]*(t2[2]-t[2]);
					tmp3[1] = dRG[3]*(t2[0]-t[0]) + dRG[4]*(t2[1]-t[1]) + dRG[5]*(t2[2]-t[2]);
					tmp3[2] = dRG[6]*(t2[0]-t[0]) + dRG[7]*(t2[1]-t[1]) + dRG[8]*(t2[2]-t[2]);

					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos;	triVal[sum] = -R[0];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos;	triVal[sum] = -R[3];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos;	triVal[sum] = -R[6];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+1;	triVal[sum] = -R[1];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+1;	triVal[sum] = -R[4];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+1;	triVal[sum] = -R[7];
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+2;	triVal[sum] = -R[2];		
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+2;	triVal[sum] = -R[5];
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+2;	triVal[sum] = -R[8];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+3;	triVal[sum] = tmp1[0];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+3;	triVal[sum] = tmp1[1];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+3;	triVal[sum] = tmp1[2];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+4;	triVal[sum] = tmp2[0];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+4;	triVal[sum] = tmp2[1];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+4;	triVal[sum] = tmp2[2];
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = pos+5;	triVal[sum] = tmp3[0];		
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = pos+5;	triVal[sum] = tmp3[1];
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = pos+5;	triVal[sum] = tmp3[2];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = i;		triVal[sum] = R[0];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = i;		triVal[sum] = R[3];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = i;		triVal[sum] = R[6];	
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = i+1;	triVal[sum] = R[1];	
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = i+1;	triVal[sum] = R[4];	
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = i+1;	triVal[sum] = R[7];
					sum = J->nnz++;
					triI[sum] = i;			triJ[sum] = i+2;	triVal[sum] = R[2];		
					sum = J->nnz++;
					triI[sum] = i+1;		triJ[sum] = i+2;	triVal[sum] = R[5];
					sum = J->nnz++;
					triI[sum] = i+2;		triJ[sum] = i+2;	triVal[sum] = R[8];	


					sum = J->nnz++;
					triI[sum] = i+3;			triJ[sum] = pos+3;	triVal[sum] = ddA[0];	
					sum = J->nnz++;
					triI[sum] = i+4;			triJ[sum] = pos+3;	triVal[sum] = ddA[1];	
					sum = J->nnz++;
					triI[sum] = i+5;			triJ[sum] = pos+3;	triVal[sum] = ddA[2];	
					sum = J->nnz++;
					triI[sum] = i+3;			triJ[sum] = pos+4;	triVal[sum] = ddB[0];	
					sum = J->nnz++;
					triI[sum] = i+4;			triJ[sum] = pos+4;	triVal[sum] = ddB[1];	
					sum = J->nnz++;
					triI[sum] = i+5;			triJ[sum] = pos+4;	triVal[sum] = ddB[2];
					sum = J->nnz++;
					triI[sum] = i+3;			triJ[sum] = pos+5;	triVal[sum] = ddG[0];		
					sum = J->nnz++;
					triI[sum] = i+4;			triJ[sum] = pos+5;	triVal[sum] = ddG[1];
					sum = J->nnz++;
					triI[sum] = i+5;			triJ[sum] = pos+5;	triVal[sum] = ddG[2];	
					sum = J->nnz++;
					triI[sum] = i+3;			triJ[sum] = i+3;	triVal[sum] = ddA2[0];	
					sum = J->nnz++;
					triI[sum] = i+4;			triJ[sum] = i+3;	triVal[sum] = ddA2[1];	
					sum = J->nnz++;
					triI[sum] = i+5;			triJ[sum] = i+3;	triVal[sum] = ddA2[2];	
					sum = J->nnz++;
					triI[sum] = i+3;			triJ[sum] = i+4;	triVal[sum] = ddB2[0];	
					sum = J->nnz++;
					triI[sum] = i+4;			triJ[sum] = i+4;	triVal[sum] = ddB2[1];	
					sum = J->nnz++;
					triI[sum] = i+5;			triJ[sum] = i+4;	triVal[sum] = ddB2[2];
					sum = J->nnz++;
					triI[sum] = i+3;			triJ[sum] = i+5;	triVal[sum] = ddG2[0];		
					sum = J->nnz++;
					triI[sum] = i+4;			triJ[sum] = i+5;	triVal[sum] = ddG2[1];
					sum = J->nnz++;
					triI[sum] = i+5;			triJ[sum] = i+5;	triVal[sum] = ddG2[2];	
				}
				i += 5;
			}
			else
			{
				tmp1[0] = dRA[0]*(ptr2[i]-t[0])+dRA[1]*(ptr2[i+1]-t[1])+dRA[2]*(ptr2[i+2]-t[2]);
				tmp1[1] = dRA[3]*(ptr2[i]-t[0])+dRA[4]*(ptr2[i+1]-t[1])+dRA[5]*(ptr2[i+2]-t[2]);
				tmp1[2] = dRA[6]*(ptr2[i]-t[0])+dRA[7]*(ptr2[i+1]-t[1])+dRA[8]*(ptr2[i+2]-t[2]);

				tmp2[0] = dRB[0]*(ptr2[i]-t[0])+dRB[1]*(ptr2[i+1]-t[1])+dRB[2]*(ptr2[i+2]-t[2]);
				tmp2[1] = dRB[3]*(ptr2[i]-t[0])+dRB[4]*(ptr2[i+1]-t[1])+dRB[5]*(ptr2[i+2]-t[2]);
				tmp2[2] = dRB[6]*(ptr2[i]-t[0])+dRB[7]*(ptr2[i+1]-t[1])+dRB[8]*(ptr2[i+2]-t[2]);

				tmp3[0] = dRG[0]*(ptr2[i]-t[0])+dRG[1]*(ptr2[i+1]-t[1])+dRG[2]*(ptr2[i+2]-t[2]);
				tmp3[1] = dRG[3]*(ptr2[i]-t[0])+dRG[4]*(ptr2[i+1]-t[1])+dRG[5]*(ptr2[i+2]-t[2]);
				tmp3[2] = dRG[6]*(ptr2[i]-t[0])+dRG[7]*(ptr2[i+1]-t[1])+dRG[8]*(ptr2[i+2]-t[2]);

				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos;	triVal[sum] = -R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos;	triVal[sum] = -R[3];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos;	triVal[sum] = -R[6];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+1;	triVal[sum] = -R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+1;	triVal[sum] = -R[4];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+1;	triVal[sum] = -R[7];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+2;	triVal[sum] = -R[2];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+2;	triVal[sum] = -R[5];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+2;	triVal[sum] = -R[8];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+3;	triVal[sum] = tmp1[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+3;	triVal[sum] = tmp1[1];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+3;	triVal[sum] = tmp1[2];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+4;	triVal[sum] = tmp2[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+4;	triVal[sum] = tmp2[1];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+4;	triVal[sum] = tmp2[2];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+5;	triVal[sum] = tmp3[0];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+5;	triVal[sum] = tmp3[1];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+5;	triVal[sum] = tmp3[2];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = i;		triVal[sum] = R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = i;		triVal[sum] = R[3];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = i;		triVal[sum] = R[6];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = i+1;	triVal[sum] = R[1];	
				sum = J->nnz++;	
				triI[sum] = i+1;		triJ[sum] = i+1;	triVal[sum] = R[4];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = i+1;	triVal[sum] = R[7];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = i+2;	triVal[sum] = R[2];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = i+2;	triVal[sum] = R[5];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = i+2;	triVal[sum] = R[8];	
				i += 2;
			}
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		//J^T*I*J
		cholmod_sparse* Jsparse = cholmod_triplet_to_sparse( J, nMax, &m_cS);
		cholmod_free_triplet(&J, &m_cS) ;

		cholmod_sparse* JT = cholmod_transpose( Jsparse, Jsparse->xtype, &m_cS );
		cholmod_sparse* JTI = cholmod_ssmult( JT, m_GMap.I, 0, 1, 1, &m_cS);
		cholmod_sparse* JTIJ = cholmod_ssmult( JTI, Jsparse,-1, 1, 1, &m_cS);

		GMap_End.I = JTIJ;		

		cholmod_free_sparse(&Jsparse, &m_cS) ; 
		cholmod_free_sparse(&JT, &m_cS) ; 
		cholmod_free_sparse(&JTI, &m_cS) ; 
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}

}


void CLinearSLAMImp::lmj_LinearLS_PF3D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur )
{
	int n1, n2;
	int pos;
	int *ptr1, *ptr2;
	double *ptr3;
	int *ptr4, *ptr5;
	n1 = GMap_End.r;
	n2 = GMap_Cur.r;

	//st = [LM1_st, LM2_st]
	cholmod_dense* b = cholmod_zeros( n1+n2, 1, CHOLMOD_REAL, &m_cS);
	double* bx = (double*)b->x;
	memcpy( bx, (double*)GMap_End.st->x, sizeof(double)*n1 );
	memcpy( bx+n1, (double*)GMap_Cur.st->x, sizeof(double)*n2 );

	int nMax = GMap_End.I->nzmax + GMap_Cur.I->nzmax;

	//I = [LM1_I, LM2_I]
	cholmod_sparse* Inew  = cholmod_allocate_sparse( n1+n2, n1+n2, nMax,true,true,-1,CHOLMOD_REAL,&m_cS);

	ptr1 = (int*)Inew->i;
	ptr2 = (int*)Inew->p;
	ptr3 = (double *)Inew->x;

	ptr4 = (int*)GMap_Cur.I->i;
	ptr5 = (int*)GMap_Cur.I->p;

	memcpy( ptr1, (int*)GMap_End.I->i, sizeof(int)*GMap_End.I->nzmax );
	memcpy( ptr2, (int*)GMap_End.I->p, sizeof(int)*GMap_End.r );
	memcpy( ptr3, (double*)GMap_End.I->x, sizeof(double)*GMap_End.I->nzmax );

	int sum = GMap_End.I->nzmax;
	memcpy( ptr3+sum, (double*)GMap_Cur.I->x, sizeof(double)*GMap_Cur.I->nzmax );
	for ( int i = 0; i < (int)GMap_Cur.I->nzmax; i++ )
		ptr1[sum+i] = ptr4[i] + n1;

	for ( int i = 0; i <= n2; i++ )
		ptr2[n1+i] = ptr5[i] + sum;


	//count column of A. #feature*2 + pose*3
	nMax = n1 + (n2-3) + 3;
	set<int> setLM;
	setLM.insert( GMap_End.stno, GMap_End.stno+n1 );
	setLM.insert( GMap_Cur.stno, GMap_Cur.stno+n2 );
	int nLess0 = 0;

	for (set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
	{
		if ( *it > 0 )
			break;
		else
			nLess0++;
	}

	int nAcol = (setLM.size() - nLess0)*3 + nLess0*6;	
	cholmod_triplet* triA = cholmod_allocate_triplet( n1+n2, nAcol, nMax, 0, CHOLMOD_REAL, &m_cS ); 	

	int*  GMap_End_stno = GMap_End.stno;
	int*  GMap_Cur_stno = GMap_Cur.stno;

	int* XGID = (int*)malloc((nAcol)*sizeof(int));
	memcpy( XGID, GMap_End_stno, n1*sizeof(int) );

	int* triAI = (int*)triA->i;
	int* triAJ = (int*)triA->j;
	double* triAVal = (double*)triA->x;
	triA->nnz=0;

	int Pn, Fn;
	int* iter;
	sum = n1;
	int Xn = n1;

	for (int i = 0; i < n1; i++ ) 
	{
		pos = triA->nnz++;
		triAI[pos] = i;		triAJ[pos] = i;		triAVal[pos] = 1;
	}

	for ( int i = 0; i < n2; i++ )
	{
		if (GMap_Cur_stno[i]<=0)
		{
			Pn = GMap_Cur_stno[i];

			XGID[sum] = XGID[sum+1] = XGID[sum+2]= XGID[sum+3]= XGID[sum+4]= XGID[sum+5] = Pn;

			pos = triA->nnz++;
			triAI[pos] = n1+i;			triAJ[pos] = Xn;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+1;		triAJ[pos] = Xn+1;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+2;		triAJ[pos] = Xn+2;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+3;		triAJ[pos] = Xn+3;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+4;		triAJ[pos] = Xn+4;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+5;		triAJ[pos] = Xn+5;	triAVal[pos] = 1;

			i += 5;
			sum += 6;
			Xn +=6;
		}
		else
		{
			Fn = GMap_Cur_stno[i];
			iter = find( XGID, XGID+sum, Fn );

			if (iter!=XGID+sum)
			{
				pos = triA->nnz++;
				triAI[pos] = n1+i;		triAJ[pos] = iter-XGID;		triAVal[pos] = 1;
				pos = triA->nnz++;
				triAI[pos] = n1+i+1;	triAJ[pos] = iter-XGID+1;	triAVal[pos] = 1;
				pos = triA->nnz++;
				triAI[pos] = n1+i+2;	triAJ[pos] = iter-XGID+2;	triAVal[pos] = 1;
			}
			else
			{
				XGID[sum] = XGID[sum+1] = XGID[sum+2] = Fn;

				pos = triA->nnz++;
				triAI[pos] = n1+i;		triAJ[pos] = Xn;		triAVal[pos] = 1;
				pos = triA->nnz++;
				triAI[pos] = n1+i+1;	triAJ[pos] = Xn+1;		triAVal[pos] = 1;
				pos = triA->nnz++;
				triAI[pos] = n1+i+2;	triAJ[pos] = Xn+2;		triAVal[pos] = 1;

				Xn+=3;
				sum += 3;
			}

			i+=2;
		}
	}

	//========================================================================================
	cholmod_sparse* A = cholmod_triplet_to_sparse( triA, triA->nnz, &m_cS );
	cholmod_free_triplet(&triA, &m_cS);

	//A^T*I*A
	cholmod_sparse* AT = cholmod_transpose( A, A->xtype, &m_cS );
	cholmod_sparse* ATI = cholmod_ssmult( AT, Inew, 0, 1, 1, &m_cS);

	cholmod_sparse* ATIA = cholmod_ssmult( ATI, A, -1, 1, 1, &m_cS);

	//A^T*I*b
	double alpha[2] = {1, 1}, belta[2] = {0,0};
	cholmod_dense* ATIb = cholmod_zeros( nAcol, 1, CHOLMOD_REAL, &m_cS);
	cholmod_sdmult(ATI, 0, alpha, belta, b, ATIb, &m_cS );

	cholmod_free_dense( &b, &m_cS );
	cholmod_free_sparse(&A, &m_cS);
	cholmod_free_sparse(&AT, &m_cS);
	cholmod_free_sparse(&ATI, &m_cS );
	cholmod_free_sparse( &Inew, &m_cS );

	//solve (A^T*I*A)*X = A^T*I*b
	cholmod_factor* factor = cholmod_analyze( ATIA, &m_cS);
	cholmod_factorize( ATIA, factor, &m_cS);
	cholmod_dense* denseG = cholmod_solve ( CHOLMOD_A, factor, ATIb, &m_cS ) ;	

	cholmod_free_sparse( &m_GMap.I, &m_cS );
	m_GMap.I = ATIA;
	cholmod_free_factor( &factor, &m_cS );

	m_GMap.r = nAcol;
	cholmod_free_dense( &m_GMap.st, &m_cS );
	m_GMap.st = denseG;
	free( m_GMap.stno );
	m_GMap.stno = (int*)malloc(nAcol*sizeof(double));
	memcpy( m_GMap.stno, XGID, sizeof(int)*nAcol );
	m_GMap.Ref = GMap_Cur.Ref;
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	free(XGID);
}

void CLinearSLAMImp::lmj_PF3D_Divide_Conquer( int nLocalMapCount )
{
	double t0, t1, t2;
	t0 = clock();
	int L = 1;
	set<int> setLM;
	while( nLocalMapCount>1 )
	{
		int N2 = nLocalMapCount%2;
		nLocalMapCount = int(nLocalMapCount/2.0 + 0.5);

		int NumLM;
		for ( int i = 0; i < nLocalMapCount; i++ )
		{
			if ( i < nLocalMapCount-1 )
				NumLM = 2;
			else
			{
				if ( N2 == 0 )
					NumLM = 2;
				else
					NumLM = 1;
			}

			for ( int j = 0; j < NumLM; j++ )
			{
				printf( "Join Level %d Local Map %d\n", L, 2*i+j+1 );

				if ( j == 0 )
				{
					m_GMap = m_LMset[2*i+j];
				}
				else
				{
					LocalMapInfo GMap_End;
					lmj_Transform_PF3D( GMap_End, m_LMset[2*i+j].Ref );

					lmj_LinearLS_PF3D(GMap_End,m_LMset[2*i+j]);

					free(m_LMset[2*i+j].stno);
					cholmod_free_sparse( &m_LMset[2*i+j].I, &m_cS);
					cholmod_l_free_dense(&m_LMset[2*i+j].st, &m_cS );

					free(GMap_End.stno);
					cholmod_free_sparse(&GMap_End.I, &m_cS) ; 
					cholmod_l_free_dense(&GMap_End.st, &m_cS );
				}


			}

printf( "Generate Level %d Local Map %d\n\n", L+1, i+1 );


			setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
			int tmp = 0;
			for ( set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
			{
				if ( *it<=0)
					tmp = -(*it);
				else
					break;
			}
			setLM.clear();

			if ( (i+1)%2 == 0 )
			{
				int m = tmp;

				if ( m_GMap.Ref > m )
				{
					LocalMapInfo GMapTmp;
					lmj_Transform_PF3D( GMapTmp, m );

					free(m_GMap.stno);
					cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
					cholmod_l_free_dense(&m_GMap.st, &m_cS );
					m_GMap = GMapTmp;
				}
			}

			m_LMset[i] = m_GMap;
		}

		L++;
	}

	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = 0;
	for ( set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
	{
		if ( *it<=0)
			m = -(*it);
		else
			break;
	}

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PF3D( GMapTmp, m );

		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;
	}

	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_3DPF( m_szPose, m_szFea, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}

static void lmj_WrapstEach(double angle, double &angleWrapst)
{
	double tmp;

	if ( angle > PI )
	{
		tmp = (int)(angle/(2*PI));
		angleWrapst = angle - (tmp+1)*(2*PI);
	}

	if ( angle < -PI )
	{
		tmp = (int)(angle/(2*PI));
		angleWrapst = angle - (tmp-1)*(2*PI);
	}
}

void CLinearSLAMImp::lmj_PG2D_Sequential( int nMapCount )
{
	double t0, t1, t2;
	t0 = clock();
	int i;

	for ( i = 0; i < nMapCount; i++ )
	{
		printf(  "Join Local Map %d\n", i+1 );

		//========================================================================================
		m_LMset[i].sp = m_LMset[i].Ref;
		lmj_Wrapst( m_LMset[i] );
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		if ( i == 0 )
		{
			m_GMap = m_LMset[0];			
		}
		else
		{
			LocalMapInfo GMap_End;
			lmj_Transform_PG2D( GMap_End, m_LMset[i].Ref );
			lmj_Wrapst( GMap_End );

			lmj_LinearLS_PG2D(GMap_End,m_LMset[i]);
			lmj_Wrapst( m_GMap );

			free(m_LMset[i].stno);
			cholmod_free_sparse( &m_LMset[i].I, &m_cS );
			cholmod_free_dense(&m_LMset[i].st, &m_cS );

			free(GMap_End.stno);
			cholmod_free_sparse( &GMap_End.I, &m_cS );
			cholmod_free_dense(&GMap_End.st, &m_cS );
		}
	}

	set<int> setLM;
	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = *(setLM.begin());

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PG2D( GMapTmp, m );

		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;		
	}
	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_2DPG( m_szPose, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}

void CLinearSLAMImp::lmj_Transform_PG2D( LocalMapInfo& GMap_End, int Ref )
{

	int pos, i, n; 
	double t[2], phi, R[4], dR[4], dRt[2];
	double* ptr1, *ptr2, tmp;
	int* iter;
	int   * stno;
	n = m_GMap.r;

	if (m_GMap.Ref == Ref )
	{
		GMap_End = m_GMap;
	}
	else
	{
		//========================================================================================
		ptr1 = (double*)m_GMap.st->x;
		stno = m_GMap.stno;

		iter = find( stno, stno+n, Ref );
		pos = iter - stno;

		t[0] = ptr1[pos];
		t[1] = ptr1[pos+1];
		phi  = ptr1[pos+2];

		R[0] = cos(phi);
		R[1] = sin(phi);
		R[2] = -sin(phi);
		R[3] = cos(phi);

		GMap_End.setDimension( n );
		GMap_End.st     = cholmod_zeros( n, 1, CHOLMOD_REAL, &m_cS);
		GMap_End.stno   = (int*)malloc( n*sizeof(int));

		ptr2 = (double*)GMap_End.st->x;
		memcpy( GMap_End.stno, stno, n*sizeof(int) );
		GMap_End.Ref = Ref;
		GMap_End.sp  = m_GMap.sp;
		GMap_End.stno[pos] = GMap_End.stno[pos+1] = GMap_End.stno[pos+2] = m_GMap.Ref;
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		for ( i=0; i<n; i++ )
		{
			if ( i == pos )
			{
				ptr2[i] = -(R[0]*t[0]+R[1]*t[1]);
				ptr2[i+1] = -(R[2]*t[0]+R[3]*t[1]);
				ptr2[i+2] = -phi;
			}
			else
			{
				ptr2[i]   =  R[0]*(ptr1[i]-t[0])+R[1]*(ptr1[i+1]-t[1]);
				ptr2[i+1] =  R[2]*(ptr1[i]-t[0])+R[3]*(ptr1[i+1]-t[1]);
				ptr2[i+2] = ptr1[i+2] - phi;

				if ( ptr2[i+2]>PI || ptr2[i+2]<-PI )
				{
					lmj_WrapstEach(ptr2[i+2], tmp );
					ptr2[i+2] = tmp;
				}								
			}

			i+=2;

		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		t[0] =	ptr2[pos];
		t[1] =	ptr2[pos+1];
		phi  =	ptr2[pos+2];

		R[0] = cos(phi);
		R[1] = sin(phi);
		R[2] = -sin(phi);
		R[3] = cos(phi);

		dR[0] = -sin(phi);
		dR[1] = cos(phi);
		dR[2] = -cos(phi);
		dR[3] = -sin(phi);

		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		int nMax = (n-3)/2*12 + 7;
		cholmod_triplet* J = cholmod_allocate_triplet(n,n,nMax,0,CHOLMOD_REAL,&m_cS); 	
		int * triI = (int*)(J->i); 
		int * triJ =(int*)(J->j); 
		double* triVal=(double*)(J->x); 
		J->nnz =0; 		
		int sum = 0;
		for ( i = 0; i < n; i++ )
		{
			if ( i == pos )
			{
				dRt[0] = dR[0]*t[0] + dR[1]*t[1];
				dRt[1] = dR[2]*t[0] + dR[3]*t[1];

				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos;	triVal[sum] = -R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos;	triVal[sum] = -R[2];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos+1;	triVal[sum] = -R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos+1;	triVal[sum] = -R[3];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos+2;	triVal[sum] = -dRt[0];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos+2;	triVal[sum] = -dRt[1];
				sum = J->nnz++;
				triI[sum] = i+2;	triJ[sum] = pos+2;	triVal[sum] = -1;		

			}
			else
			{
				dRt[0] = dR[0]*(ptr2[i]-t[0]) + dR[1]*(ptr2[i+1]-t[1]);
				dRt[1] = dR[2]*(ptr2[i]-t[0]) + dR[3]*(ptr2[i+1]-t[1]);

				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos;		triVal[sum] = -R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos;		triVal[sum] = -R[2];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos+1;		triVal[sum] = -R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos+1;		triVal[sum] = -R[3];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = pos+2;		triVal[sum] = dRt[0];	
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = pos+2;		triVal[sum] = dRt[1];	
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = i;			triVal[sum] = R[0];		
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = i;			triVal[sum] = R[2];		
				sum = J->nnz++;
				triI[sum] = i;		triJ[sum] = i+1;		triVal[sum] = R[1];		
				sum = J->nnz++;
				triI[sum] = i+1;	triJ[sum] = i+1;		triVal[sum] = R[3];		
				sum = J->nnz++;
				triI[sum] = i+2;	triJ[sum] = pos+2;		triVal[sum] = -1;		
				sum = J->nnz++;
				triI[sum] = i+2;	triJ[sum] = i+2;		triVal[sum] = 1;		

			}

			i += 2;			
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		//J^T*I*J
		cholmod_sparse* Jsparse = cholmod_triplet_to_sparse( J, nMax, &m_cS);
		cholmod_free_triplet(&J, &m_cS) ;

		cholmod_sparse* JT = cholmod_transpose( Jsparse, Jsparse->xtype, &m_cS );
		cholmod_sparse* JTI = cholmod_ssmult( JT, m_GMap.I, 0, 1, 1, &m_cS);
		cholmod_sparse* JTIJ = cholmod_ssmult( JTI, Jsparse,-1, 1, 1, &m_cS);

		GMap_End.I = JTIJ;		

		cholmod_free_sparse(&Jsparse, &m_cS) ; 
		cholmod_free_sparse(&JT, &m_cS) ; 
		cholmod_free_sparse(&JTI, &m_cS) ; 
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}

}

void CLinearSLAMImp::lmj_LinearLS_PG2D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur )
{
	int n1, n2;
	int pos;
	int *ptr1, *ptr2;
	double *ptr3;
	int *ptr4, *ptr5;
	n1 = GMap_End.r;
	n2 = GMap_Cur.r;

	//st = [LM1_st, LM2_st]
	cholmod_dense* b = cholmod_zeros( n1+n2, 1, CHOLMOD_REAL, &m_cS);
	double* bx = (double*)b->x;
	memcpy( bx, (double*)GMap_End.st->x, sizeof(double)*n1 );
	memcpy( bx+n1, (double*)GMap_Cur.st->x, sizeof(double)*n2 );

	int nMax = GMap_End.I->nzmax + GMap_Cur.I->nzmax;

	//I = [LM1_I, LM2_I]
	cholmod_sparse* Inew  = cholmod_allocate_sparse( n1+n2, n1+n2, nMax,true,true,-1,CHOLMOD_REAL,&m_cS);

	ptr1 = (int*)Inew->i;
	ptr2 = (int*)Inew->p;
	ptr3 = (double *)Inew->x;

	ptr4 = (int*)GMap_Cur.I->i;
	ptr5 = (int*)GMap_Cur.I->p;

	memcpy( ptr1, (int*)GMap_End.I->i, sizeof(int)*GMap_End.I->nzmax );
	memcpy( ptr2, (int*)GMap_End.I->p, sizeof(int)*GMap_End.r );
	memcpy( ptr3, (double*)GMap_End.I->x, sizeof(double)*GMap_End.I->nzmax );

	int sum = GMap_End.I->nzmax;
	memcpy( ptr3+sum, (double*)GMap_Cur.I->x, sizeof(double)*GMap_Cur.I->nzmax );
	for ( int i = 0; i < (int)GMap_Cur.I->nzmax; i++ )
		ptr1[sum+i] = ptr4[i] + n1;

	for ( int i = 0; i <= n2; i++ )
		ptr2[n1+i] = ptr5[i] + sum;


	//count column of A. #feature*2 + pose*3
	set<int> setLM, setLM1;
	setLM1.insert( GMap_End.stno, GMap_End.stno+n1 );
	setLM.insert( GMap_End.stno, GMap_End.stno+n1 );
	setLM.insert( GMap_Cur.stno, GMap_Cur.stno+n2 );


	int nAcol = n1 + (setLM.size()-setLM1.size())*3;	

	nMax = n1+n2*3;

	//int nAcol = setLM.size() * 3;	
	cholmod_triplet* triA = cholmod_allocate_triplet( n1+n2, nAcol, nMax, 0, CHOLMOD_REAL, &m_cS ); 	

	int*  GMap_End_stno = GMap_End.stno;
	int*  GMap_Cur_stno = GMap_Cur.stno;

	int* XGID = (int*)malloc((nAcol)*sizeof(int));
	memcpy( XGID, GMap_End_stno, n1*sizeof(int) );

	//double* tt2 = (double*)b->x;

	int* triAI = (int*)triA->i;
	int* triAJ = (int*)triA->j;
	double* triAVal = (double*)triA->x;
	triA->nnz=0;

	int Pn;
	int* iter;
	sum = n1;
	int Xn = n1;

	for (int i = 0; i < n1; i++ ) 
	{
		pos = triA->nnz++;
		triAI[pos] = i;		triAJ[pos] = i;		triAVal[pos] = 1;
	}

	double err;
	for ( int i = 0; i < n2; i++ )
	{
		Pn = GMap_Cur_stno[i];
		iter = find( XGID, XGID+sum, Pn );

		if (iter!=XGID+sum)
		{
			pos = triA->nnz++;
			triAI[pos] = n1+i;		triAJ[pos] = iter-XGID;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+1;	triAJ[pos] = iter-XGID+1;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+2;	triAJ[pos] = iter-XGID+2;	triAVal[pos] = 1;

			err = bx[n1+i+2] - bx[(iter-XGID+2)];
			if ( err > PI)
				bx[n1+i+2] = bx[n1+i+2] - 2*PI;
			else if ( err < -PI )
				bx[n1+i+2] = bx[n1+i+2] + 2*PI;
		}
		else
		{
			XGID[sum] = XGID[sum+1] = XGID[sum+2] = Pn;

			pos = triA->nnz++;
			triAI[pos] = n1+i;		triAJ[pos] = Xn;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+1;	triAJ[pos] = Xn+1;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+2;	triAJ[pos] = Xn+2;		triAVal[pos] = 1;

			Xn+=3;
			sum += 3;
		}		

		i+=2;

	}

	//========================================================================================
	cholmod_sparse* A = cholmod_triplet_to_sparse( triA, triA->nnz, &m_cS );
	cholmod_free_triplet(&triA, &m_cS);

	//A^T*I*A
	cholmod_sparse* AT = cholmod_transpose( A, A->xtype, &m_cS );
	cholmod_sparse* ATI = cholmod_ssmult( AT, Inew, 0, 1, 1, &m_cS);

	cholmod_sparse* ATIA = cholmod_ssmult( ATI, A, -1, 1, 1, &m_cS);

	//A^T*I*b
	double alpha[2] = {1, 1}, belta[2] = {0,0};
	cholmod_dense* ATIb = cholmod_zeros( nAcol, 1, CHOLMOD_REAL, &m_cS);
	cholmod_sdmult(ATI, 0, alpha, belta, b, ATIb, &m_cS );

	cholmod_free_dense( &b, &m_cS );
	cholmod_free_sparse(&A, &m_cS);
	cholmod_free_sparse(&AT, &m_cS);
	cholmod_free_sparse(&ATI, &m_cS );
	cholmod_free_sparse( &Inew, &m_cS );

	//solve (A^T*I*A)*X = A^T*I*b
	cholmod_factor* factor = cholmod_analyze( ATIA, &m_cS);
	cholmod_factorize( ATIA, factor, &m_cS);
	cholmod_dense* denseG = cholmod_solve ( CHOLMOD_A, factor, ATIb, &m_cS ) ;	

	cholmod_free_sparse( &m_GMap.I, &m_cS );
	m_GMap.I = ATIA;
	cholmod_free_factor( &factor, &m_cS );

	m_GMap.r = nAcol;
	cholmod_free_dense( &m_GMap.st, &m_cS );
	m_GMap.st = denseG;
	free( m_GMap.stno );
	m_GMap.stno = (int*)malloc(nAcol*sizeof(double));
	memcpy( m_GMap.stno, XGID, sizeof(int)*nAcol );
	m_GMap.Ref = GMap_Cur.Ref;
	m_GMap.sp  = GMap_End.sp;
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	free(XGID);
}


void CLinearSLAMImp::lmj_Wrapst( LocalMapInfo& GMap )
{
	int i;
	int size = GMap.r;
	double* ptr = (double*)GMap.st->x;
	double tmp;

	for ( i = 2; i < size; i++ )
	{
		if ( ptr[i] > PI )
		{
			tmp = (int)(ptr[i]/(2*PI));
			ptr[i] -= (tmp+1)*(2*PI);
		}

		if ( ptr[i] < -PI )
		{
			tmp = (int)(ptr[i]/(2*PI));
			ptr[i] -= (tmp-1)*(2*PI);
		}

		i += 2;
	}
}

void CLinearSLAMImp::lmj_PG2D_Divide_Conquer( int nLocalMapCount )
{
	double t0, t1, t2;
	t0 = clock();
	for( int i = 0; i < nLocalMapCount; i++ )
	{
		m_LMset[i].sp = m_LMset[i].Ref;
		lmj_Wrapst( m_LMset[i] );
	}

	int L = 1;
	set<int> setLM;
	while( nLocalMapCount>1 )
	{
		int N2 = nLocalMapCount%2;
		nLocalMapCount = int(nLocalMapCount/2.0 + 0.5);

		int NumLM;
		for ( int i = 0; i < nLocalMapCount; i++ )
		{
			if ( i < nLocalMapCount-1 )
				NumLM = 2;
			else
			{
				if ( N2 == 0 )
					NumLM = 2;
				else
					NumLM = 1;
			}

			for ( int j = 0; j < NumLM; j++ )
			{
				printf( "Join Level %d Local Map %d\n", L, 2*i+j+1 );

				if ( j == 0 )
				{
					m_GMap = m_LMset[2*i+j];
				}
				else
				{
					LocalMapInfo GMap_End;
					lmj_Transform_PG2D( GMap_End, m_LMset[2*i+j].Ref );
					lmj_Wrapst( GMap_End );

					lmj_LinearLS_PG2D(GMap_End,m_LMset[2*i+j]);

					free(m_LMset[2*i+j].stno);
					cholmod_free_sparse( &m_LMset[2*i+j].I, &m_cS);
					cholmod_l_free_dense(&m_LMset[2*i+j].st, &m_cS );

					free(GMap_End.stno);
					cholmod_free_sparse(&GMap_End.I, &m_cS) ; 
					cholmod_l_free_dense(&GMap_End.st, &m_cS );
				}
			}

printf( "Generate Level %d Local Map %d\n\n", L+1, i+1 );


			lmj_Wrapst( m_GMap );

			if ( (i+1)%2 == 0 )
			{
				if ( m_GMap.Ref > m_GMap.sp )
				{
					LocalMapInfo GMapTmp;
					lmj_Transform_PG2D( GMapTmp, m_GMap.sp );

					free(m_GMap.stno);
					cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
					cholmod_l_free_dense(&m_GMap.st, &m_cS );
					m_GMap = GMapTmp;
				}
			}

			lmj_Wrapst( m_GMap );

			m_LMset[i] = m_GMap;
		}

		L++;
		printf("\n");
	}

	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = *(setLM.begin());

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PG2D( GMapTmp, m );

		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;
	}

	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_2DPG( m_szPose, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}

void CLinearSLAMImp::lmj_Wrapst_3D( LocalMapInfo &GMap )
{
	int i, j;
	int size = GMap.r;
	double* ptr = (double*)GMap.st->x;
	double tmp;

	for ( i = 2; i < size; i++ )
	{
		for ( j = 1; j <= 3; j++ )
		{
			if ( ptr[i+j] > PI )
			{
				tmp = (int)(ptr[i+1]/(2*PI));
				ptr[i+j] -= (tmp+1)*(2*PI);
			}

			if ( ptr[i+j] < -PI )
			{
				tmp = (int)(ptr[i+1]/(2*PI));
				ptr[i+j] -= (tmp-1)*(2*PI);
			}
		}		

		i += 5;
	}
}


void CLinearSLAMImp::lmj_PG3D_Sequential( int nMapCount )
{
	int i;
	double t0, t1, t2;
	t0 = clock();

	for ( i = 0; i < nMapCount; i++ )
	{
		printf(  "Join Local Map %d\n", i+1 );

		//========================================================================================
		m_LMset[i].sp = m_LMset[i].Ref;
		lmj_Wrapst_3D( m_LMset[i] );
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		if ( i == 0 )
		{
			m_GMap = m_LMset[0];			
		}
		else
		{
			LocalMapInfo GMap_End;

			lmj_Transform_PG3D( GMap_End, m_LMset[i].Ref );
			lmj_LinearLS_PG3D( GMap_End,m_LMset[i]);
			lmj_Wrapst_3D( m_GMap );

			free(m_LMset[i].stno);
			cholmod_free_sparse( &m_LMset[i].I, &m_cS );
			cholmod_free_dense(&m_LMset[i].st, &m_cS );

			free(GMap_End.stno);
			cholmod_free_sparse( &GMap_End.I, &m_cS );
			cholmod_free_dense(&GMap_End.st, &m_cS );
		}
	}

	set<int> setLM;
	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = *(setLM.begin());

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PG3D( GMapTmp, m );
		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;		
	}
	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_3DPG( m_szPose, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}


void CLinearSLAMImp::lmj_Transform_PG3D( LocalMapInfo& GMap_End, int Ref )
{

	int pos, i, n; 
	double t[3], Alpha, Beta, Gamma, R[9], R2[9], R3[9], dRA[9], dRB[9], dRG[9];
	double dA[3], dB[3], dG[3],dRA2[9], dRB2[9], dRG2[9];
	double t2[3], Alpha2, Beta2, Gamma2, Ri[9], tmp1[3], tmp2[3], tmp3[3];
	double ddA2[3], ddB2[3], ddG2[3], ddA[3], ddB[3], ddG[3];
	double* ptr1, *ptr2;
	int* iter;
	int   * stno;
	n = m_GMap.r;

	if (m_GMap.Ref == Ref )
	{
		GMap_End = m_GMap;
	}
	else
	{
		//========================================================================================
		ptr1 = (double*)m_GMap.st->x;
		stno = m_GMap.stno;

		iter = find( stno, stno+n, Ref );
		pos = iter - stno;

		t[0] = ptr1[pos];
		t[1] = ptr1[pos+1];
		t[2] = ptr1[pos+2];

		Alpha  = ptr1[pos+3];
		Beta   = ptr1[pos+4];
		Gamma  = ptr1[pos+5];

		lmj_RMatrixYPR22( R, Alpha, Beta, Gamma );

		GMap_End.setDimension( n );
		GMap_End.st     = cholmod_zeros( n, 1, CHOLMOD_REAL, &m_cS);
		GMap_End.stno   = (int*)malloc( n*sizeof(int));

		ptr2 = (double*)GMap_End.st->x;
		memcpy( GMap_End.stno, stno, n*sizeof(int) );
		GMap_End.Ref = Ref;
		GMap_End.stno[pos] = GMap_End.stno[pos+1] = GMap_End.stno[pos+2] =
			GMap_End.stno[pos+3] = GMap_End.stno[pos+4] = GMap_End.stno[pos+5] = m_GMap.Ref;
		GMap_End.sp = m_GMap.sp;
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		for ( i=0; i<n; i++ )
		{
			if ( i == pos )
			{
				ptr2[i]	  = -( R[0]*t[0]+R[1]*t[1]+R[2]*t[2] );
				ptr2[i+1] = -( R[3]*t[0]+R[4]*t[1]+R[5]*t[2] );
				ptr2[i+2] = -( R[6]*t[0]+R[7]*t[1]+R[8]*t[2] );

				lmj_InvRotMatrixYPR22T( R, ptr2[i+3], ptr2[i+4], ptr2[i+5] );					
			}
			else
			{
				ptr2[i]	  = ( R[0]*(ptr1[i]-t[0])+R[1]*(ptr1[i+1]-t[1])+R[2]*(ptr1[i+2]-t[2]) );
				ptr2[i+1] = ( R[3]*(ptr1[i]-t[0])+R[4]*(ptr1[i+1]-t[1])+R[5]*(ptr1[i+2]-t[2]) );
				ptr2[i+2] = ( R[6]*(ptr1[i]-t[0])+R[7]*(ptr1[i+1]-t[1])+R[8]*(ptr1[i+2]-t[2]) );

				lmj_RMatrixYPR22( R2, ptr1[i+3], ptr1[i+4], ptr1[i+5] );

				lmj_TimesRRT( R3, R2, R );
				lmj_InvRotMatrixYPR22( R3, ptr2[i+3], ptr2[i+4], ptr2[i+5] );
			}

			i+=5;
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		t[0] =	ptr2[pos];
		t[1] =	ptr2[pos+1];
		t[2]  =	ptr2[pos+2];

		Alpha = ptr2[pos+3];
		Beta  = ptr2[pos+4];
		Gamma = ptr2[pos+5];

		lmj_Rderivation( Alpha, Beta, Gamma, R, dRA, dRB, dRG );

		lmj_dRiTT( dA, dRA, R );
		lmj_dRiTT( dB, dRB, R );
		lmj_dRiTT( dG, dRG, R );
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		int nMax = n/2*45 + 27;
		cholmod_triplet* J = cholmod_allocate_triplet(n,n,nMax,0,CHOLMOD_REAL,&m_cS); 	
		int * triI = (int*)(J->i); 
		int * triJ =(int*)(J->j); 
		double* triVal=(double*)(J->x); 
		J->nnz =0; 		
		int sum = 0;
		for ( i = 0; i < n; i++ )
		{
			if ( i == pos )
			{
				tmp1[0] = (dRA[0]*t[0]+dRA[1]*t[1]+dRA[2]*t[2]);
				tmp1[1] = (dRA[3]*t[0]+dRA[4]*t[1]+dRA[5]*t[2]);
				tmp1[2] = (dRA[6]*t[0]+dRA[7]*t[1]+dRA[8]*t[2]);

				tmp2[0] = (dRB[0]*t[0]+dRB[1]*t[1]+dRB[2]*t[2]);
				tmp2[1] = (dRB[3]*t[0]+dRB[4]*t[1]+dRB[5]*t[2]);
				tmp2[2] = (dRB[6]*t[0]+dRB[7]*t[1]+dRB[8]*t[2]);

				tmp3[0] = (dRG[0]*t[0]+dRG[1]*t[1]+dRG[2]*t[2]);
				tmp3[1] = (dRG[3]*t[0]+dRG[4]*t[1]+dRG[5]*t[2]);
				tmp3[2] = (dRG[6]*t[0]+dRG[7]*t[1]+dRG[8]*t[2]);

				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos;	triVal[sum] = -R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos;	triVal[sum] = -R[3];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos;	triVal[sum] = -R[6];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+1;	triVal[sum] = -R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+1;	triVal[sum] = -R[4];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+1;	triVal[sum] = -R[7];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+2;	triVal[sum] = -R[2];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+2;	triVal[sum] = -R[5];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+2;	triVal[sum] = -R[8];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+3;	triVal[sum] = -tmp1[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+3;	triVal[sum] = -tmp1[1];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+3;	triVal[sum] = -tmp1[2];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+4;	triVal[sum] = -tmp2[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+4;	triVal[sum] = -tmp2[1];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+4;	triVal[sum] = -tmp2[2];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+5;	triVal[sum] = -tmp3[0];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+5;	triVal[sum] = -tmp3[1];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+5;	triVal[sum] = -tmp3[2];	
				sum = J->nnz++;
				triI[sum] = i+3;		triJ[sum] = pos+3;	triVal[sum] = dA[0];	
				sum = J->nnz++;
				triI[sum] = i+4;		triJ[sum] = pos+3;	triVal[sum] = dA[1];	
				sum = J->nnz++;
				triI[sum] = i+5;		triJ[sum] = pos+3;	triVal[sum] = dA[2];	
				sum = J->nnz++;
				triI[sum] = i+3;		triJ[sum] = pos+4;	triVal[sum] = dB[0];	
				sum = J->nnz++;
				triI[sum] = i+4;		triJ[sum] = pos+4;	triVal[sum] = dB[1];	
				sum = J->nnz++;
				triI[sum] = i+5;		triJ[sum] = pos+4;	triVal[sum] = dB[2];
				sum = J->nnz++;
				triI[sum] = i+3;		triJ[sum] = pos+5;	triVal[sum] = dG[0];		
				sum = J->nnz++;
				triI[sum] = i+4;		triJ[sum] = pos+5;	triVal[sum] = dG[1];
				sum = J->nnz++;
				triI[sum] = i+5;		triJ[sum] = pos+5;	triVal[sum] = dG[2];	

			}
			else
			{
				t2[0] =	ptr2[i];
				t2[1] =	ptr2[i+1];
				t2[2] = ptr2[i+2];

				Alpha2 = ptr2[i+3];
				Beta2  = ptr2[i+4];
				Gamma2 = ptr2[i+5];

				lmj_Rderivation( Alpha2, Beta2, Gamma2, R2, dRA2, dRB2, dRG2 );

				lmj_TimesRRT( Ri, R2, R );

				double dRidA2[9], dRidB2[9], dRidG2[9];

				lmj_TimesRRT( dRidA2, dRA2, R );
				lmj_TimesRRT( dRidB2, dRB2, R );
				lmj_TimesRRT( dRidG2, dRG2, R );

				double dRidA[9], dRidB[9], dRidG[9];
				lmj_TimesRRT( dRidA, R2, dRA );
				lmj_TimesRRT( dRidB, R2, dRB );
				lmj_TimesRRT( dRidG, R2, dRG );


				lmj_dRi( ddA2, dRidA2, Ri );
				lmj_dRi( ddB2, dRidB2, Ri );
				lmj_dRi( ddG2, dRidG2, Ri );

				lmj_dRi( ddA, dRidA, Ri );
				lmj_dRi( ddB, dRidB, Ri );
				lmj_dRi( ddG, dRidG, Ri );

				tmp1[0] = dRA[0]*(t2[0]-t[0]) + dRA[1]*(t2[1]-t[1]) + dRA[2]*(t2[2]-t[2]);
				tmp1[1] = dRA[3]*(t2[0]-t[0]) + dRA[4]*(t2[1]-t[1]) + dRA[5]*(t2[2]-t[2]);
				tmp1[2] = dRA[6]*(t2[0]-t[0]) + dRA[7]*(t2[1]-t[1]) + dRA[8]*(t2[2]-t[2]);

				tmp2[0] = dRB[0]*(t2[0]-t[0]) + dRB[1]*(t2[1]-t[1]) + dRB[2]*(t2[2]-t[2]);
				tmp2[1] = dRB[3]*(t2[0]-t[0]) + dRB[4]*(t2[1]-t[1]) + dRB[5]*(t2[2]-t[2]);
				tmp2[2] = dRB[6]*(t2[0]-t[0]) + dRB[7]*(t2[1]-t[1]) + dRB[8]*(t2[2]-t[2]);

				tmp3[0] = dRG[0]*(t2[0]-t[0]) + dRG[1]*(t2[1]-t[1]) + dRG[2]*(t2[2]-t[2]);
				tmp3[1] = dRG[3]*(t2[0]-t[0]) + dRG[4]*(t2[1]-t[1]) + dRG[5]*(t2[2]-t[2]);
				tmp3[2] = dRG[6]*(t2[0]-t[0]) + dRG[7]*(t2[1]-t[1]) + dRG[8]*(t2[2]-t[2]);

				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos;	triVal[sum] = -R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos;	triVal[sum] = -R[3];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos;	triVal[sum] = -R[6];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+1;	triVal[sum] = -R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+1;	triVal[sum] = -R[4];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+1;	triVal[sum] = -R[7];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+2;	triVal[sum] = -R[2];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+2;	triVal[sum] = -R[5];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+2;	triVal[sum] = -R[8];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+3;	triVal[sum] = tmp1[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+3;	triVal[sum] = tmp1[1];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+3;	triVal[sum] = tmp1[2];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+4;	triVal[sum] = tmp2[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+4;	triVal[sum] = tmp2[1];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+4;	triVal[sum] = tmp2[2];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = pos+5;	triVal[sum] = tmp3[0];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = pos+5;	triVal[sum] = tmp3[1];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = pos+5;	triVal[sum] = tmp3[2];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = i;		triVal[sum] = R[0];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = i;		triVal[sum] = R[3];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = i;		triVal[sum] = R[6];	
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = i+1;	triVal[sum] = R[1];	
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = i+1;	triVal[sum] = R[4];	
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = i+1;	triVal[sum] = R[7];
				sum = J->nnz++;
				triI[sum] = i;			triJ[sum] = i+2;	triVal[sum] = R[2];		
				sum = J->nnz++;
				triI[sum] = i+1;		triJ[sum] = i+2;	triVal[sum] = R[5];
				sum = J->nnz++;
				triI[sum] = i+2;		triJ[sum] = i+2;	triVal[sum] = R[8];	

				sum = J->nnz++;
				triI[sum] = i+3;			triJ[sum] = pos+3;	triVal[sum] = ddA[0];	
				sum = J->nnz++;
				triI[sum] = i+4;			triJ[sum] = pos+3;	triVal[sum] = ddA[1];	
				sum = J->nnz++;
				triI[sum] = i+5;			triJ[sum] = pos+3;	triVal[sum] = ddA[2];	
				sum = J->nnz++;
				triI[sum] = i+3;			triJ[sum] = pos+4;	triVal[sum] = ddB[0];	
				sum = J->nnz++;
				triI[sum] = i+4;			triJ[sum] = pos+4;	triVal[sum] = ddB[1];	
				sum = J->nnz++;
				triI[sum] = i+5;			triJ[sum] = pos+4;	triVal[sum] = ddB[2];
				sum = J->nnz++;
				triI[sum] = i+3;			triJ[sum] = pos+5;	triVal[sum] = ddG[0];		
				sum = J->nnz++;
				triI[sum] = i+4;			triJ[sum] = pos+5;	triVal[sum] = ddG[1];
				sum = J->nnz++;
				triI[sum] = i+5;			triJ[sum] = pos+5;	triVal[sum] = ddG[2];	
				sum = J->nnz++;
				triI[sum] = i+3;			triJ[sum] = i+3;	triVal[sum] = ddA2[0];	
				sum = J->nnz++;
				triI[sum] = i+4;			triJ[sum] = i+3;	triVal[sum] = ddA2[1];	
				sum = J->nnz++;
				triI[sum] = i+5;			triJ[sum] = i+3;	triVal[sum] = ddA2[2];	
				sum = J->nnz++;
				triI[sum] = i+3;			triJ[sum] = i+4;	triVal[sum] = ddB2[0];	
				sum = J->nnz++;
				triI[sum] = i+4;			triJ[sum] = i+4;	triVal[sum] = ddB2[1];	
				sum = J->nnz++;
				triI[sum] = i+5;			triJ[sum] = i+4;	triVal[sum] = ddB2[2];
				sum = J->nnz++;
				triI[sum] = i+3;			triJ[sum] = i+5;	triVal[sum] = ddG2[0];		
				sum = J->nnz++;
				triI[sum] = i+4;			triJ[sum] = i+5;	triVal[sum] = ddG2[1];
				sum = J->nnz++;
				triI[sum] = i+5;			triJ[sum] = i+5;	triVal[sum] = ddG2[2];	

			}
			i += 5;
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//========================================================================================
		//J^T*I*J
		cholmod_sparse* Jsparse = cholmod_triplet_to_sparse( J, nMax, &m_cS);
		cholmod_free_triplet(&J, &m_cS) ;

		cholmod_sparse* JT = cholmod_transpose( Jsparse, Jsparse->xtype, &m_cS );
		cholmod_sparse* JTI = cholmod_ssmult( JT, m_GMap.I, 0, 1, 1, &m_cS);
		cholmod_sparse* JTIJ = cholmod_ssmult( JTI, Jsparse,-1, 1, 1, &m_cS);

		GMap_End.I = JTIJ;		

		cholmod_free_sparse(&Jsparse, &m_cS) ; 
		cholmod_free_sparse(&JT, &m_cS) ; 
		cholmod_free_sparse(&JTI, &m_cS) ; 
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}

}


void CLinearSLAMImp::lmj_LinearLS_PG3D( LocalMapInfo& GMap_End, LocalMapInfo& GMap_Cur )
{
	int n1, n2;
	int pos;
	int *ptr1, *ptr2;
	double *ptr3;
	int *ptr4, *ptr5;
	n1 = GMap_End.r;
	n2 = GMap_Cur.r;

	//st = [LM1_st, LM2_st]
	cholmod_dense* b = cholmod_zeros( n1+n2, 1, CHOLMOD_REAL, &m_cS);
	double* bx = (double*)b->x;
	memcpy( bx, (double*)GMap_End.st->x, sizeof(double)*n1 );
	memcpy( bx+n1, (double*)GMap_Cur.st->x, sizeof(double)*n2 );

	int nMax = GMap_End.I->nzmax + GMap_Cur.I->nzmax;

	//I = [LM1_I, LM2_I]
	cholmod_sparse* Inew  = cholmod_allocate_sparse( n1+n2, n1+n2, nMax,true,true,-1,CHOLMOD_REAL,&m_cS);

	ptr1 = (int*)Inew->i;
	ptr2 = (int*)Inew->p;
	ptr3 = (double *)Inew->x;

	ptr4 = (int*)GMap_Cur.I->i;
	ptr5 = (int*)GMap_Cur.I->p;

	memcpy( ptr1, (int*)GMap_End.I->i, sizeof(int)*GMap_End.I->nzmax );
	memcpy( ptr2, (int*)GMap_End.I->p, sizeof(int)*GMap_End.r );
	memcpy( ptr3, (double*)GMap_End.I->x, sizeof(double)*GMap_End.I->nzmax );

	int sum = GMap_End.I->nzmax;
	memcpy( ptr3+sum, (double*)GMap_Cur.I->x, sizeof(double)*GMap_Cur.I->nzmax );
	for ( int i = 0; i < (int)GMap_Cur.I->nzmax; i++ )
		ptr1[sum+i] = ptr4[i] + n1;

	for ( int i = 0; i <= n2; i++ )
		ptr2[n1+i] = ptr5[i] + sum;


	//count column of A. #feature*2 + pose*3
	nMax = n1 + (n2-3) + 3;
	set<int> setLM;
	setLM.insert( GMap_End.stno, GMap_End.stno+n1 );
	setLM.insert( GMap_Cur.stno, GMap_Cur.stno+n2 );
	int nLess0 = 0;

	for (set<int>::iterator it = setLM.begin(); it != setLM.end(); it++ )
	{
		if ( *it > 0 )
			break;
		else
			nLess0++;
	}

	int nAcol = (setLM.size() - nLess0)*6 + nLess0*6;	
	cholmod_triplet* triA = cholmod_allocate_triplet( n1+n2, nAcol, nMax, 0, CHOLMOD_REAL, &m_cS ); 	

	int*  GMap_End_stno = GMap_End.stno;
	int*  GMap_Cur_stno = GMap_Cur.stno;

	int* XGID = (int*)malloc((nAcol)*sizeof(int));
	memcpy( XGID, GMap_End_stno, n1*sizeof(int) );

	int* triAI = (int*)triA->i;
	int* triAJ = (int*)triA->j;
	double* triAVal = (double*)triA->x;
	triA->nnz=0;

	int Pn;
	int* iter;
	sum = n1;
	int Xn = n1;

	for (int i = 0; i < n1; i++ ) 
	{
		pos = triA->nnz++;
		triAI[pos] = i;		triAJ[pos] = i;		triAVal[pos] = 1;
	}

	double err;
	for ( int i = 0; i < n2; i++ )
	{
		Pn = GMap_Cur_stno[i];

		iter = find( XGID, XGID+sum, Pn );

		if (iter!=XGID+sum)
		{
			pos = triA->nnz++;
			triAI[pos] = n1+i;			triAJ[pos] = iter-XGID;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+1;		triAJ[pos] = iter-XGID+1;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+2;		triAJ[pos] = iter-XGID+2;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+3;		triAJ[pos] = iter-XGID+3;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+4;		triAJ[pos] = iter-XGID+4;	triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+5;		triAJ[pos] = iter-XGID+5;	triAVal[pos] = 1;

			for ( int j = 3; j < 6; j++ )
			{
				err = bx[n1+i+j] - bx[iter-XGID+j];
				if ( err > PI )
					bx[n1+i+j] = bx[n1+i+j] - 2*PI;
				else if ( err < -PI )
					bx[n1+i+j] = bx[n1+i+j] + 2*PI;
			}	
		}
		else
		{
			XGID[sum] = XGID[sum+1] = XGID[sum+2] = XGID[sum+3] = XGID[sum+4] = XGID[sum+5] = Pn;

			pos = triA->nnz++;
			triAI[pos] = n1+i;		triAJ[pos] = Xn;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+1;	triAJ[pos] = Xn+1;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+2;	triAJ[pos] = Xn+2;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+3;	triAJ[pos] = Xn+3;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+4;	triAJ[pos] = Xn+4;		triAVal[pos] = 1;
			pos = triA->nnz++;
			triAI[pos] = n1+i+5;	triAJ[pos] = Xn+5;		triAVal[pos] = 1;

			Xn+=6;
			sum += 6;
		}

		i+=5;
	}

	//========================================================================================
	cholmod_sparse* A = cholmod_triplet_to_sparse( triA, triA->nnz, &m_cS );
	cholmod_free_triplet(&triA, &m_cS);

	//A^T*I*A
	cholmod_sparse* AT = cholmod_transpose( A, A->xtype, &m_cS );
	cholmod_sparse* ATI = cholmod_ssmult( AT, Inew, 0, 1, 1, &m_cS);

	cholmod_sparse* ATIA = cholmod_ssmult( ATI, A, -1, 1, 1, &m_cS);

	//A^T*I*b
	double alpha[2] = {1, 1}, belta[2] = {0,0};
	cholmod_dense* ATIb = cholmod_zeros( nAcol, 1, CHOLMOD_REAL, &m_cS);

	cholmod_sdmult(ATI, 0, alpha, belta, b, ATIb, &m_cS );

	cholmod_free_dense( &b, &m_cS );
	cholmod_free_sparse(&A, &m_cS);
	cholmod_free_sparse(&AT, &m_cS);
	cholmod_free_sparse(&ATI, &m_cS );
	cholmod_free_sparse( &Inew, &m_cS );

	//solve (A^T*I*A)*X = A^T*I*b
	cholmod_factor* factor = cholmod_analyze( ATIA, &m_cS);
	cholmod_factorize( ATIA, factor, &m_cS);
	cholmod_dense* denseG = cholmod_solve ( CHOLMOD_A, factor, ATIb, &m_cS ) ;	

	cholmod_free_sparse( &m_GMap.I, &m_cS );
	m_GMap.I = ATIA;
	cholmod_free_factor( &factor, &m_cS );

	m_GMap.r = nAcol;
	cholmod_free_dense( &m_GMap.st, &m_cS );
	m_GMap.st = denseG;
	free( m_GMap.stno );
	m_GMap.stno = (int*)malloc(nAcol*sizeof(double));
	memcpy( m_GMap.stno, XGID, sizeof(int)*nAcol );
	m_GMap.Ref = GMap_Cur.Ref;
	m_GMap.sp = GMap_End.sp;
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	free(XGID);
}

void CLinearSLAMImp::lmj_PG3D_Divide_Conquer( int nLocalMapCount )
{
	double t0, t1, t2;
	t0 = clock();
	int L = 1;
	set<int> setLM;

	for ( int i = 0; i < nLocalMapCount; i++ )
	{
		m_LMset[i].sp = m_LMset[i].Ref;
		lmj_Wrapst_3D( m_LMset[i] );
	}

	while( nLocalMapCount>1 )
	{
		int N2 = nLocalMapCount%2;
		nLocalMapCount = int(nLocalMapCount/2.0 + 0.5);

		int NumLM;
		for ( int i = 0; i < nLocalMapCount; i++ )
		{
			if ( i < nLocalMapCount-1 )
				NumLM = 2;
			else
			{
				if ( N2 == 0 )
					NumLM = 2;
				else
					NumLM = 1;
			}

			for ( int j = 0; j < NumLM; j++ )
			{
				printf( "Join Level %d Local Map %d\n", L, 2*i+j+1 );

				if ( j == 0 )
				{
					m_GMap = m_LMset[2*i+j];
				}
				else
				{
					LocalMapInfo GMap_End;
					lmj_Transform_PG3D( GMap_End, m_LMset[2*i+j].Ref );

					lmj_LinearLS_PG3D(GMap_End,m_LMset[2*i+j]);

					free(m_LMset[2*i+j].stno);
					cholmod_free_sparse( &m_LMset[2*i+j].I, &m_cS);
					cholmod_l_free_dense(&m_LMset[2*i+j].st, &m_cS );

					free(GMap_End.stno);
					cholmod_free_sparse(&GMap_End.I, &m_cS) ; 
					cholmod_l_free_dense(&GMap_End.st, &m_cS );
				}


			}

printf( "Generate Level %d Local Map %d\n\n", L+1, i+1 );


			lmj_Wrapst_3D( m_GMap );

			if ( (i+1)%2 == 0 )
			{
				if ( m_GMap.Ref > m_GMap.sp )
				{
					LocalMapInfo GMapTmp;
					lmj_Transform_PG3D( GMapTmp, m_GMap.sp );

					free(m_GMap.stno);
					cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
					cholmod_l_free_dense(&m_GMap.st, &m_cS );
					m_GMap = GMapTmp;
				}
			}

			m_LMset[i] = m_GMap;
		}

		L++;
	}

	setLM.clear();
	setLM.insert( m_GMap.stno, m_GMap.stno+m_GMap.r );
	int m = *(setLM.begin());

	if ( m_GMap.Ref > m )
	{
		LocalMapInfo GMapTmp;
		lmj_Transform_PG3D( GMapTmp, m );

		free(m_GMap.stno);
		cholmod_free_sparse(&m_GMap.I, &m_cS) ; 
		cholmod_l_free_dense(&m_GMap.st, &m_cS );
		m_GMap = GMapTmp;
	}
	t1 = clock();

	t2 = (t1-t0)*0.001;

	printf( "Used Time:  %lf  sec", t2 );

	//Save State Vector
	if( m_szSt != NULL)
		lmj_SaveStateVector( m_szSt, m_GMap.stno, (double*)m_GMap.st->x, m_GMap.st->nrow );

	if ( m_szInfo != NULL )
		lmj_SaveInfomationMatrix( m_szInfo, (double*)m_GMap.I->x, (int*)m_GMap.I->i, (int*)m_GMap.I->p, (int)m_GMap.I->ncol );

	lmj_SavePoses_3DPG( m_szPose, m_GMap.stno, (double*)m_GMap.st->x, (int)m_GMap.st->nrow );
}

void CLinearSLAMImp::lmj_SaveStateVector( char* szSt, int* stno, double* st, int n )
{
	FILE *fp = fopen( szSt, "w" );
	if ( fp == NULL )
	{
		printf( "Please Input Path to Save Final State Vector!" );
		return;
	}

	for ( int i = 0; i < n; i++ )
	{
		fprintf( fp, "%d  %lf\n", stno[i], st[i] );
	}

	fclose( fp );
}

void CLinearSLAMImp::lmj_SaveInfomationMatrix( char* szInfo, double* I, int* Si, int *Sp, int n )
{
	FILE *fp = fopen( szInfo, "w" );
	if ( fp == NULL )
	{
		return;
	}

	int cols = 0, sum = 0;
	int lastNum = 0;
	for ( int i = 0; i < n; i++ )
	{
		cols = Sp[i+1] - lastNum;
		for ( int j = 0; j < cols; j++ )
		{
			if ( I[sum] != 0 )
				fprintf( fp, "%d  %d  %lf\n", Si[sum], i, I[sum]);

			sum++;
		}
		lastNum = Sp[i+1];
	}

	fclose( fp );
}

void CLinearSLAMImp::lmj_SavePoses_2DPF( char* szPose, char* szFea, int* stno, double* st, int n )
{
	bool bSavePose = false, bSaveFea = false;
	FILE *fpPose = NULL, *fpFea = NULL;

	set<int> poseID, featureID;
	vector<double> pose, feature;
	map<int, int>  mapIndexPose, mapIndexFea;

	if (szPose)
	{
		bSavePose = true;
		fpPose    = fopen( szPose, "w" );
	}

	if (szFea)
	{
		bSaveFea  = true;
		fpFea     = fopen( szFea, "w" );
	}

	if (!szPose && !szFea)
		return;

	int nIndexPose = 0, nIndexFea = 0;
	for ( int i = 0; i < n; i++ )
	{
		if (stno[i] <= 0 )
		{
			if ( bSavePose )
			{
				mapIndexPose[-stno[i]] = nIndexPose;
				poseID.insert( -stno[i]);
				pose.push_back( st[i] );
				pose.push_back( st[i+1] );
				pose.push_back( st[i+2] );

				nIndexPose++;
			}
				
			i += 2;
		}
		else
		{
			if ( bSaveFea )
			{
				mapIndexFea[stno[i]] = nIndexFea;	
				featureID.insert(stno[i]);
				feature.push_back( st[i] );
				feature.push_back( st[i+1] );
				nIndexFea++;
			}	
			i += 1;
		}
	}

	int m1 = 0, n1 = 0;
	map<int, int>::iterator find1, find2;
	if (bSavePose)
	{
		for (set<int>::iterator it1 = poseID.begin(); it1 != poseID.end(); it1++ )
		{
			m1 = *it1;
			find1 = mapIndexPose.find(m1);
			n1 = find1->second;

			fprintf( fpPose, "%d  %lf  %lf  %lf\n", m1, pose[n1*3], pose[n1*3+1], pose[n1*3+2] );
		}
	}

	if (bSaveFea)
	{
		for (set<int>::iterator it2 = featureID.begin(); it2 != featureID.end(); it2++ )
		{
			m1 = *it2;
			find1 = mapIndexFea.find(m1);
			n1 = find1->second;

			fprintf( fpFea, "%d  %lf  %lf\n", m1, feature[n1*2], feature[n1*2+1] );
		}
	}

	if( fpPose )
		fclose( fpPose );
	if( fpFea)
		fclose( fpFea );
}

void CLinearSLAMImp::lmj_SavePoses_2DPG( char* szPose, int* stno, double* st, int n )
{
	bool bSavePose = false;
	FILE *fpPose = NULL;

	set<int> poseID;
	vector<double> pose;
	map<int, int>  mapIndexPose;

	if (szPose)
	{
		bSavePose = true;
		fpPose    = fopen( szPose, "w" );
	}

	if (!szPose )
		return;

	int nIndexPose = 0;
	for ( int i = 0; i < n; i++ )
	{
		mapIndexPose[stno[i]] = nIndexPose;
		poseID.insert( stno[i]);
		pose.push_back( st[i] );
		pose.push_back( st[i+1] );
		pose.push_back( st[i+2] );

		nIndexPose++;

		i += 2;
	}


	int m1 = 0, n1 = 0;
	map<int, int>::iterator find1, find2;
	if (bSavePose)
	{
		for (set<int>::iterator it1 = poseID.begin(); it1 != poseID.end(); it1++ )
		{
			m1 = *it1;
			find1 = mapIndexPose.find(m1);
			n1 = find1->second;

			fprintf( fpPose, "%d  %lf  %lf  %lf\n", m1, pose[n1*3], pose[n1*3+1], pose[n1*3+2] );
		}
	}

	fclose( fpPose );
}

void CLinearSLAMImp::lmj_SavePoses_3DPF( char* szPose, char* szFea, int* stno, double* st, int n )
{
	bool bSavePose = false, bSaveFea = false;
	FILE *fpPose = NULL, *fpFea = NULL;

	set<int> poseID, featureID;
	vector<double> pose, feature;
	map<int, int>  mapIndexPose, mapIndexFea;

	if (szPose)
	{
		bSavePose = true;
		fpPose    = fopen( szPose, "w" );
	}

	if (szFea)
	{
		bSaveFea  = true;
		fpFea     = fopen( szFea, "w" );
	}

	if (!szPose && !szFea)
		return;

	int nIndexPose = 0, nIndexFea = 0;
	for ( int i = 0; i < n; i++ )
	{
		if (stno[i] <= 0 )
		{
			if ( bSavePose )
			{
				mapIndexPose[-stno[i]] = nIndexPose;
				poseID.insert( -stno[i]);
				pose.push_back( st[i] );
				pose.push_back( st[i+1] );
				pose.push_back( st[i+2] );
				pose.push_back( st[i+3] );
				pose.push_back( st[i+4] );
				pose.push_back( st[i+5] );

				nIndexPose++;
			}

			i += 5;
		}
		else
		{
			if ( bSaveFea )
			{
				mapIndexFea[stno[i]] = nIndexFea;	
				featureID.insert(stno[i]);
				feature.push_back( st[i] );
				feature.push_back( st[i+1] );
				feature.push_back( st[i+2] );
				nIndexFea++;
			}	
			i += 2;
		}
	}

	int m1 = 0, n1 = 0;
	map<int, int>::iterator find1, find2;
	if (bSavePose)
	{
		for (set<int>::iterator it1 = poseID.begin(); it1 != poseID.end(); it1++ )
		{
			m1 = *it1;
			find1 = mapIndexPose.find(m1);
			n1 = find1->second;

			fprintf( fpPose, "%d  %lf  %lf  %lf %lf  %lf  %lf\n", m1, pose[n1*6], pose[n1*6+1], pose[n1*6+2],
								pose[n1*6+3], pose[n1*6+4], pose[n1*6+5]);
		}
	}

	if (bSaveFea)
	{
		for (set<int>::iterator it2 = featureID.begin(); it2 != featureID.end(); it2++ )
		{
			m1 = *it2;
			find1 = mapIndexFea.find(m1);
			n1 = find1->second;

			fprintf( fpFea, "%d  %lf  %lf %lf\n", m1, feature[n1*3], feature[n1*3+1], feature[n1*3+2] );
		}
	}

	if( fpPose )
		fclose( fpPose );
	if( fpFea)
		fclose( fpFea );
}

void CLinearSLAMImp::lmj_SavePoses_3DPG( char* szPose, int* stno, double* st, int n )
{
	bool bSavePose = false;
	FILE *fpPose = NULL;

	set<int> poseID;
	vector<double> pose;
	map<int, int>  mapIndexPose;

	if (szPose)
	{
		bSavePose = true;
		fpPose    = fopen( szPose, "w" );
	}

	if (!szPose)
		return;

	int nIndexPose = 0;
	for ( int i = 0; i < n; i++ )
	{
		mapIndexPose[stno[i]] = nIndexPose;
		poseID.insert( stno[i]);
		pose.push_back( st[i] );
		pose.push_back( st[i+1] );
		pose.push_back( st[i+2] );
		pose.push_back( st[i+3] );
		pose.push_back( st[i+4] );
		pose.push_back( st[i+5] );

		nIndexPose++;


		i += 5;
	}

	int m1 = 0, n1 = 0;
	map<int, int>::iterator find1, find2;
	if (bSavePose)
	{
		for (set<int>::iterator it1 = poseID.begin(); it1 != poseID.end(); it1++ )
		{
			m1 = *it1;
			find1 = mapIndexPose.find(m1);
			n1 = find1->second;

			fprintf( fpPose, "%d  %lf  %lf  %lf %lf  %lf  %lf\n", m1, pose[n1*6], pose[n1*6+1], pose[n1*6+2],
				pose[n1*6+3], pose[n1*6+4], pose[n1*6+5]);
		}
	}
	
	fclose( fpPose );
}
