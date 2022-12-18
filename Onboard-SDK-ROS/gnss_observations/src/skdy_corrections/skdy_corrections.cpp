#include <skdy_corrections.h>

static      pppcorr_msg_t           skdy_Corr_Data_Info_t;
static      skdy_init_t             skdy_Init_Info_t;
static      skdy_uint8_t            skdy_CurrArea_u8;

static      pppcorr_msg_t           skdy_CorrData_st;                                     // Database to store the correction data
static      skdy_uint8_t            skdy_MqttMissCount_u8;
static      skdy_uint32_t           skdy_MqttArriveData_u32;

// data callback function
static void skdy_Sdk_Data_Callback(pppcorr_msg_t* data);

// check if mqtt need to be re-initialized
static void skdy_CheckMqtt_Validation(void);

// update the current mqtt connection according to the position
static int skdy_UpdateMqttConnection(skdy_point_position_t* pos);

/*
* get the correction data which decoded by the client library
* output:
*      data       : pointer of the structure pppcorr_msg_t
* return          : void
*/
void skdy_CorrDataHndr_GetCorrData(pppcorr_msg_t* data)
{
	memcpy(data, &skdy_Corr_Data_Info_t, sizeof(pppcorr_msg_t));
}

/*
* initialization function to inialize the client library and datbase etc.
* input           : none
* output          : none
* return          :
*      1          : success
*      other value: fail
*/
int skdy_CorrDataHndr_Initialization(char* key, char* secret)
{
	//skdy_sdk_need_int = 0;
	skdy_point_position_t initPos_st;
	skdy_int8_t  retVal_s8 = -1;
	skdy_int32_t  area_u32 = 0;

	memset(&skdy_Corr_Data_Info_t, 0, sizeof(pppcorr_msg_t));
	memset(&initPos_st, 0, sizeof(skdy_point_position_t));

	memset(&skdy_CorrData_st, 0, sizeof(pppcorr_msg_t));

	initPos_st.latitude = 31.5;
	initPos_st.longitude = 121.75;

	skdy_MqttMissCount_u8 = 0;
	skdy_MqttArriveData_u32 = 0;

	memcpy(skdy_Init_Info_t.key, key,sizeof(skdy_Init_Info_t.key));
	memcpy(skdy_Init_Info_t.secret, secret, sizeof(skdy_Init_Info_t.secret));

	skdy_Init_Info_t.data_callback = skdy_Sdk_Data_Callback;

	SKDY_Ser_SDK_Init(&skdy_Init_Info_t);
	printf(">>>>>>>>Begin to SKDY_Ser_CertAuth()!\n");
	//check the authorization
	retVal_s8 = SKDY_Ser_CertAuth();
	
	if (retVal_s8 == 1)
	{
		printf(">>>>>>>>>>SKDY_Ser_CertAuth() successful!\n");
		SKDY_Point_Position_Update(&initPos_st);
		skdy_CurrArea_u8 = skdy_nowArea;

		//start the mqtt service and receive correction data
		SKDY_Rec_Start();
	}

	return retVal_s8;
}

void skdy_CorrDataHndr_ProcessPppRtk(double *pos, pppcorr_msg_t* skdy_CorrData)
{
	skdy_CheckMqtt_Validation();

	skdy_point_position_t CurrPos_st;

	skdy_CorrDataHndr_GetCorrData(&skdy_CorrData_st);
	*skdy_CorrData = skdy_CorrData_st;

	CurrPos_st.latitude = pos[0];
	CurrPos_st.longitude = pos[1];
	skdy_UpdateMqttConnection(&CurrPos_st);
}

/*
* data callback function which will be used to initialize the client library,
* and also, the decoded SSR data will be stored here
* input           :
*      data       : pointer to the structure pppcorr_msg_t which means the decoded SSR data 
* output          : none
* return          : none
*/
static void skdy_Sdk_Data_Callback(pppcorr_msg_t* data)
{
	memcpy(&skdy_Corr_Data_Info_t, data, sizeof(pppcorr_msg_t));
}

/* ---------------------------------------------------------------------------
* re-connect mqtt function
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
static void skdy_CheckMqtt_Validation(void)
{
	skdy_point_position_t initPos_st;
	skdy_int8_t retVal_s8 = 1;

	if (skdy_sdk_data_arrive == skdy_MqttArriveData_u32)
	{
		skdy_MqttMissCount_u8++;

		if (skdy_MqttMissCount_u8 >= 30)
		{
			skdy_MqttMissCount_u8 = 0;
			skdy_sdk_need_int = 1;
		}
	}
	else
	{	
		skdy_MqttArriveData_u32 = skdy_sdk_data_arrive;
		skdy_MqttMissCount_u8 = 0;
	}

	if (skdy_sdk_need_int)
	{
		skdy_Init_Info_t.data_callback = skdy_Sdk_Data_Callback;

		SKDY_Ser_SDK_Init(&skdy_Init_Info_t);

		retVal_s8 = SKDY_Ser_CertAuth();

		if (retVal_s8 == 1)
		{
			initPos_st.latitude = 31.5;
			initPos_st.longitude = 121.75;

			SKDY_Point_Position_Update(&initPos_st);
			skdy_CurrArea_u8 = skdy_nowArea;

			SKDY_Rec_Start();

			skdy_sdk_need_int = 0;
		}
	}
}

/*
* update the mqtt connection according to the position
* input           :
*      pos        : pointer to the structure skdy_point_position_t, which means the current
*                   receiver's position
* output          : none
* return          :
*      1          : success
*      other value: fail
*/
static int skdy_UpdateMqttConnection(skdy_point_position_t* pos)
{
	skdy_int8_t retVal_s8 = 1;

	if (pos)
	{
		SKDY_Point_Position_Update(pos);

		if (skdy_nowArea != skdy_CurrArea_u8)
		{
			//strcpy(skdy_Init_Info_t.key, SKDY_USER_KEY);
			//strcpy(skdy_Init_Info_t.secret, SKDY_USER_SECRET);

			skdy_Init_Info_t.data_callback = skdy_Sdk_Data_Callback;

			SKDY_Ser_SDK_Init(&skdy_Init_Info_t);

			retVal_s8 = SKDY_Ser_CertAuth();

			if (retVal_s8 == 1)
			{
				SKDY_Point_Position_Update(pos);
				skdy_CurrArea_u8 = skdy_nowArea;

				SKDY_Rec_Start();
			}
		}
	}

	return retVal_s8;
}