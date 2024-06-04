#include "PID.h"

void Pos_PID_Init(Pos_PID_t* plant,float Kp,float Ki,float Kd)
{
	plant->Error = 0;
	plant->Value_I = 0;
	plant->Value_D = 0;
	plant->Past_Error = 0;
	plant->Kp = Kp;
	plant->Ki = Ki;
	plant->Kd = Kd;
}

float Pos_PID_Controller(Pos_PID_t* plant ,float ValueCurrent)
{
	plant->Error = plant->Ref - ValueCurrent;
	plant->Value_I += plant->Error;
	plant->Value_D = fabs(plant->Error-plant->Past_Error);
	plant->Output = plant->Kp * plant->Error + plant->Ki * plant->Value_I - plant->Value_D*plant->Kd;
	plant->Past_Error = plant->Error;
	
	if(plant->Value_I > plant->Value_I_Max)
	{
		plant->Value_I = plant->Value_I_Max;
	}
	if(plant->Value_I < -plant->Value_I_Max)
	{
		plant->Value_I = -plant->Value_I_Max;
	}

	if(plant->Output > plant->Output_Max)
		 return plant->Output_Max;
	if(plant->Output < plant->Output_Min)
		return plant->Output_Min;
	
	return plant->Output;
}