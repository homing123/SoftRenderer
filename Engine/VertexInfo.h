#pragma once
#include "EngineUtil.h"

namespace HM
{
	class VertexInfo
	{
	public:
		VertexInfo() = default;

		void Setting(const int objCount, Camera& mainCam)
		{
			GameObjectCount = objCount;
			WorldToViewMat = mainCam.GetWorldToViewMat();
			WorldToNDCMat = mainCam.GetWorldToNDCMat();
			mainCam.GetNearFar(CamNear, CamFar);
			Inv_CamDis = 1 / (CamFar - CamNear);

			if (isInit == false) 
			{
				isInit = true;
				arr_v_Vertex = new vector<Vertex>[ThreadCountWithMainThread];
			}

		}
		void Release() 
		{
			for (int i = 0; i < ThreadCountWithMainThread; i++)
			{
				arr_v_Vertex[i].clear();
			}
		}
		bool isInit = false;
		int GameObjectCount;
		Matrix4x4 WorldToViewMat;
		Matrix4x4 WorldToNDCMat;
		float CamNear, CamFar, Inv_CamDis;
		vector<Vertex>* arr_v_Vertex;

	};
}