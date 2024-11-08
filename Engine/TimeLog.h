#pragma once
#include "EngineUtil.h"
namespace HM
{
	class TimeLog
	{
	public:
		TimeLog() = default;

		void Reset() {
			_UpdateTime = 0;
			_RenderTime = 0;

			CU_BG_Depth_CamInfoSetting = 0;
			CU_DrawCallSetting = 0;
			CU_BoundCheckBox = 0;
			CU_VertexArraySetting = 0;
			CU_Culling_ScanLine_SetBuffer = 0;
			CU_SetBuffer = 0;
			CU_BufferCopy = 0;
			CU_Test = 0;
			CU_Test1 = 0;
		}

		float _UpdateTime;
		float _RenderTime;

		float CU_BG_Depth_CamInfoSetting;
		float CU_DrawCallSetting;
		float CU_BoundCheckBox;
		float CU_VertexArraySetting;
		float CU_Culling_ScanLine_SetBuffer;
		float CU_SetBuffer;
		float CU_BufferCopy;
		float CU_Test;
		float CU_Test1;

		
	};
}