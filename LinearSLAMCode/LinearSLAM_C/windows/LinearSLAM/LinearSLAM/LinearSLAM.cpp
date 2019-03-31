// LinearSLAM.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "LinearSLAMImp.h"


int _tmain(int argc, _TCHAR* argv[])
{
	 CLinearSLAMImp ptr;
	 int nMapCount;

    //========================================================================================
	//Select Dataset
	//2D Pose Feature Dataset
	char* szData = "../../../DatasetsForC/VicPark_200_local_maps";	nMapCount = 200;
	//char* szData = "../../../DatasetsForC/VicPark_6898_local_maps";	nMapCount = 6898;
	//char* szData = "../../../DatasetsForC/DLR_200_local_maps";		nMapCount = 200;
	//char* szData = "../../../DatasetsForC/DLR_3298_local_maps";		nMapCount = 3298;
	//char* szData = "../../../DatasetsForC/8240_data_50_local_maps";	nMapCount = 50;
	//char* szData = "../../../DatasetsForC/35188_data_700_local_maps";	nMapCount = 700;


	//3D Pose Feature Dataset
	//char* szData = "../../../DatasetsForC/Simu_3D_870_Loop";			nMapCount = 870;


	//2D Pose Graph Dataset
	//char* szData = "../../../DatasetsForC/Intel";					nMapCount = 942;
	//char* szData = "../../../DatasetsForC/manhattanOlson3500";		nMapCount = 3499;
	//char* szData = "../../../DatasetsForC/city10000";				nMapCount = 9999;

	//3D Pose Graph Dataset
	//char* szData = "../../../DatasetsForC/parking-garage";			nMapCount = 1660;
	//char* szData = "../../../DatasetsForC/sphere2500";				nMapCount = 2499;

//Test Your Own Dataset
	//char* szData = "../../../DatasetsForC/test_your_own_dataset";			nMapCount = 22;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//Select Process Method( Sequential & DivideConquer )
	//LinearSLAM_Method method = LinearSLAM_Sequential;					
	LinearSLAM_Method method = LinearSLAM_DivideConquer;		
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	 //========================================================================================
	 //Select Date Type
	LinearSLAM_DataType dataType = LinearSLAM_2DPoseFeature;
	//LinearSLAM_DataType dataType = LinearSLAM_3DPoseFeature;
	//LinearSLAM_DataType dataType = LinearSLAM_2DPoseGraph;
	//LinearSLAM_DataType dataType = LinearSLAM_3DPoseGraph;
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//Set Path to Save Final State Vector
	char* szSt = NULL, *szPose = NULL, *szFeature = NULL;
	//szSt = "../../../State.txt";
	szPose = "../../../pose.txt";
	szFeature = "../../../feature.txt";
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//Set Path to Save Information Matrix
	char* szInfo = NULL;
	//szInfo = "../../../Info.txt";
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//========================================================================================
	//First parameter:  file path to save final state vector
    //Second parameter: Dataset location
    //Third  parameter; Number of local maps
	//Fourth parameter: Dataset Type
	//Fifth  parameter: Process Method
	ptr.run( szSt, szData, nMapCount, dataType, method, szInfo, szPose, szFeature );
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	system("pause");
	return 0;
}

