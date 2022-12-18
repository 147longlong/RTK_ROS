/*------------------------------------------------------------------------------
* usericd.h : for user ICD data types struct
*          Copyright (C) 2020 by SPARK.GENG, All rights reserved.
* version : $Revision: 1.0  $Date: 2020/08/18 15:41:00 $
* history : 2020/08/18 1.0  usericd ver.1.0.0
*-----------------------------------------------------------------------------*/
#ifndef ICDSTRUCT_H
#define ICDSTRUCT_H
#include "skdy_types.h"
#include "time.h"
#ifdef __cplusplus
extern "C" {
#endif

#define NSYS 4											/* max number of sat system */
#define MAXSAT 180										/* max number of sattlites */
#define NSATMASK 66										/* max number of sat mask */
#define MAXCODE     48                 					/* max number of obs code */
#define MAXAREA     256                 				/* max number of area count */
	//changed by liupeng
#define MAXGRID     1024                 					/* max number of grid point count */

#define GPS_OCB 1
#define GLONASS_OCB 4
#define BDS_OCB 2
#define GALILEO_OCB 3

#define GPS_OCB_SWITCH 1
#define GLONASS_OCB_SWITCH 2
#define BDS_OCB_SWITCH 3
#define GALILEO_OCB_SWITCH 4

#define GAD 10
#define STEC 11
#define TROP 12
#define VTEC 13

#define PREAMBLE_NUM 21323
#define MSGTYPE 4061

#define MSGHEADLENGTH 96



#define MIN_GPS_PRN   1
#define MAX_GPS_PRN  32
/*----------satellite id list of GLO----------*/
#define MIN_GLO_PRN  33
#define MAX_GLO_PRN  59
/*----------satellite id list of GAL----------*/
#define MIN_GAL_PRN  60
#define MAX_GAL_PRN  95
/*----------satellite id list of BDS----------*/
#define MIN_BDS_PRN 96
#define MAX_BDS_PRN 113

/*----------satellite id list of BDS3----------*/
#define MIN_BDS3_PRN 114
#define MAX_BDS3_PRN 155


	typedef struct {        								/* time struct */
		time_t 				time;        					/* time (s) expressed by standard time_t */
		double 				sec;         					/* fraction of second under 1 s */
	} gs_gtime_t;

//==================================================================
//结构体名：wstime_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    时间信息（周/秒）
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* time struct */
		long 				week;        					/* time (s) expressed by standard time_t */
		double 				sec;         					/* fraction of second under 1 s */
	} gs_wstime_t;

//==================================================================
//结构体名：ssr_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    状态空间数据
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* SSR correction type */
		gs_wstime_t 			t0[6];     						/* epoch time (GPST) {eph,clk,cbias,pbias,hrclk,ura} */
		unsigned char 		sys;							/* sattlite system */
		unsigned char 		refdatum; 						/* Satellite reference datum */
		unsigned char 		ephtype; 						/* Ephemeris type */
		unsigned char 		pres_flag;						/* OCB present flags */
		unsigned char 		IODE[2];							/* IODE */
		double 				deph[3];    					/* delta orbit {radial,along,cross} (m) */
		double 				ddeph[3];    					/* dot delta orbit {radial,along,cross} (m/s) */
		double 				dclk[3];    					/* delta clock {c0,c1,c2} (m,m/s,m/s^2) */
		unsigned char 		URE;							/* User range error */
		unsigned char 		cbias_source;					/* Phase bias indicator */
		unsigned short 		cbias_mask[MAXCODE];			/* Phase bias indicator */
		unsigned char 		pbias_source;					/* Phase bias indicator */
		unsigned short 		pbias_mask[MAXCODE];			/* Phase bias indicator */
		unsigned char 		pbias_fix_flg[MAXCODE];					/* Phase bias Fix flag */
		float 				cbias[MAXCODE]; 				/* code biases (m) */
		float 				pbias[MAXCODE]; 				/* phase biases (m) */
		double 				udi[5];      					/* SSR update interval (s) */
		int 				iod[5];        					/* iod ssr {eph,clk,hrclk,ura,bias} */
		unsigned char 		update; 						/* update flag (0:no update,1:update) */
	} gs_ssr_t;

//==================================================================
//结构体名：iono_grid_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    电离层格网块
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* Ionosphere grid block msg */
		unsigned short 		grid_id;						/* Ionosphere Grid ID msg */
		double 				iono_residuals;					/* Ionosphere grid residuals */
	} gs_iono_grid_t;
//==================================================================
//结构体名：iono_sat_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    电离层卫星块
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* Ionosphere satellite block msg */


		unsigned char 		iono_quality;					/* Ionosphere quality */
		unsigned char		fixFlag;
		unsigned char 		iono_fixflg;					/* Ionosphere fix flag */
		double 				iono_c00;						/* Ionosphere coefficient C00  */
		double 				iono_c01;						/* Ionosphere coefficient C01  */
		double 				iono_c10;						/* Ionosphere coefficient C10  */
		double 				iono_c11;						/* Ionosphere coefficient C11  */
		gs_iono_grid_t			iono_grid_block[MAXGRID];
	} gs_iono_sat_t;
//==================================================================
//结构体名：stec_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    斜路径电离层改正
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* Slant Ionosphere Correction msg */
		gs_wstime_t			t0;								/* referance time */
		unsigned char 		area_id;  						/* Area ID  */
		unsigned char 		iono_ind;  						/* Area Ionosphere available indicator  */
		unsigned char 		iono_sat_mask[NSYS][NSATMASK];  /* Area Ionosphere available indicator  */
		unsigned char		grid_count;
		unsigned char		iono_equa_type;
		gs_iono_sat_t 		iono_sat_block[MAXSAT];				/* Ionosphere grid block  */
	} gs_stec_t;




	typedef struct {
		unsigned short 		grid_id;  						/* grid ID  */
		double				trop_residuals;
	}gs_trop_grid_t;
//==================================================================
//结构体名：trop_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    对流层改正
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* Troposphere Correction msg */

		unsigned char 		trop_equatype;					/* Troposphere equation type  */
		unsigned char 		trop_quality;					/* Troposphere quality */
		double 				trop_vhydro;  					/* Area average vertical hydrostatic delay  */
		double 				trop_t00;						/* Troposphere coefficient T00  */
		double 				trop_t01;						/* Troposphere coefficient T01  */
		double 				trop_t10;						/* Troposphere coefficient T10  */
		double 				trop_t11;						/* Troposphere coefficient T11  */

		gs_trop_grid_t			trop_grid_block[MAXGRID];


	} gs_trop_t;

//==================================================================
//结构体名：gtrop_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    格网对流层改正
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* Grid Troposphere Correction msg */
		gs_wstime_t			t0;								/* referance time */
		unsigned char 		area_id;  						/* Area ID  */
		unsigned char 		iono_ind;  						/* Area trop available indicator  */
		unsigned char		grid_count;
		gs_trop_t 				trop_grid;				/* Troposphere grid correction  */
	} gs_gtrop_t;

//==================================================================
//结构体名：iono_vtec_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    垂直电离层改正数据
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* Vertical Ionosphere Correction msg */
		unsigned char 		grid_id;  						/* grid ID  */
		unsigned char 		vtec_quality;  					/* VTEC Ionosphere quality  */
		double 				vtec_res;  						/* VTEC residual  */
	} gs_iono_vtec_t;


//==================================================================
//结构体名：  vtec_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    垂直电离层改正数据
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* Vertical Ionosphere Correction msg */
		gs_wstime_t			t0;								/* referance time */
		unsigned char 		area_id;  						/* Area ID  */
		unsigned char 		iono_ind;  						/* Area Ionosphere available indicator  */
		double 				aver_area_vtec;  				/* Average area VTEC  */
		unsigned char		grid_count;
		gs_iono_vtec_t 		iono_vtec[MAXGRID];				/* Ionosphere grid block  */
	} gs_vtec_t;

//==================================================================
//结构体名：  ocb_msg_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    轨道、钟差、码偏差及相位偏差改正
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* OCB correction msg */
		unsigned char 		n, ns[NSYS];					/* number of sattlites */
		gs_ssr_t 				ssr[MAXSAT];  					/* SSR corrections */
		unsigned char 		refdatum[NSYS]; 				/* Satellite reference datum */
		unsigned char 		ephtype[NSYS];
		unsigned char		sat_mask[NSYS][NSATMASK];		/* Satellite mask */
		unsigned char		ssr_mask[MAXSAT][6];
		unsigned char 		IODE[NSYS]; 					/* IODE */
	} gs_ocb_msg_t;

//==================================================================
//结构体名：  atmos_msg_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    对流层/电离层改正数据结构体
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {       									/* Iono/Trop correction msg */
		gs_stec_t 				slant_iono;						/* Slant Ionosphere correction msg */
		gs_gtrop_t 			grid_trop;						/* Grid Troposphere correction msg */
		gs_vtec_t  			vtec;							/* Vertical Ionosphere Correction(VTEC) */
	} gs_atmos_msg_t;

//==================================================================
//结构体名：  gad_msg_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    区域数据结构体，保存每个区域的格网信息
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* GAD correction msg */
		unsigned char 		area_id;  						/* Area ID  */
		double 				ref_pos[2];						/* Area reference position : {latitude , longitude} (degrees) */
		unsigned char 		node_cnt[2]; 					/* Area grid node count : {latitude , longitude} */
		float 				grid_span[2];					/* Area grid node span : {latitude , longitude} (degrees)*/
	} gs_gad_msg_t;


//==================================================================
//结构体名：  pppcorr_msg_t
//作者：    Gengsq
//日期：    2020-08-27
//功能：    网络播发数据接收结构体，存放解析后网络播发数据
//输入参数：无
//返回值：  无
//修改记录：
//==================================================================
	typedef struct {        								/* PPP correction msg */
		gs_wstime_t 			wst;
		gs_gtime_t 			t;
		gs_ocb_msg_t 			ocb;							/* OCB correction */
		gs_atmos_msg_t 		atmos; 							/* Iono/Trop correction */
		unsigned char 		area_cnt;						/* Area Count */
		unsigned char 		area_mask[MAXAREA];				/* Area Available indicator */
		gs_gad_msg_t 			gad[MAXAREA];      				/* geographic area definition msg */

		unsigned char msgSubType;
		unsigned char solusionProviderID;
		unsigned char GAD_Version[12];
	} pppcorr_msg_t;

	int ICDParser(skdy_int16_t* msgCount, unsigned char* data, pppcorr_msg_t* m_pppcorr_msg_t);

#ifdef __cplusplus
}
#endif

#endif

