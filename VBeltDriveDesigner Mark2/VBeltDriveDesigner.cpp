#include "Universal.h"

#define S 1
#define W 2
#define O 3
#define P 4
#define HT150 7860	//(KG/M^3)

typedef struct DoublePOINT { double x = NULL; double y = NULL; }DoublePOINT;
typedef struct StringPOINT { string x = NULL; string y = NULL; }StringPOINT;

typedef struct Prerequisites {
	float KA = 1.0;								//指定工作情况系数(默认为1);
	float NominalPower = 1;						//指定电动机名义功率(kW)
	string Type = "";							//指定带型;
	int MinWrapAngle = 120;						//指定指定允许的最小包角(默认120°);
	double Epsilon = 0.015;						//指定指定弹性滑动率,一般取[0.01,0.02](默认0.015);

	double ImagTransmissionRatio = 1;			//指定理论传动比;
	double TransmissionRatioTolerance = 0.05;	//指定理论与实际传动比的允许容差率(默认5%);

	double SmallWheelRotatingSpeed;				//指定小带轮转速(r/min);
	double BigWheelRotatingSpeed;				//指定大带轮转速(r/min);

	float SmallWheelAxleDameter = NULL;				//指定小轮传动轴直径(mm);
	float BigWheelAxleDameter = NULL;					//指定大轮传动轴直径(mm);
}Prerequisites;

typedef struct VBelt {
	double RealTransmissionRatio = 1;			//实际传动比;

	vector<int> WheelBenchmarkDiameter1;		//选中的小带轮基准直径;
	vector<int> WheelBenchmarkDiameter2;		//选中的大带轮基准直径;

	struct Result {
		vector<int> SmallWheelDiameter;				//储存小带轮直径;
		vector<int> BigWheelDiameter;				//储存大带轮直径;
		vector<double> TransmissionRatio;			//储存传动比;
		vector<int> RealBeltLength;					//带的基准长度;
		vector<double> RealWheelbase;				//实际中心矩(mm);
		vector<double> SmallWheelWrapAngle;			//小带轮包角(°);
		vector<double> RealBeltSpeed;				//实际带速(m/s);
		vector<int> BeltNumber_Z;					//确定带的根数z;
	}Result;
}VBelt;

typedef struct VWheel {
	//储存轮槽几何参数(x=小轮,y=大轮);
	DoublePOINT Bd;
	DoublePOINT Hamin;
	DoublePOINT Hfmin;
	DoublePOINT e;
	DoublePOINT Fmin;
	DoublePOINT deltaMin;
	vector<DoublePOINT> phi;

	vector<DoublePOINT> MinWheelBroad;				//储存最小带轮宽B(mm);
	vector<DoublePOINT> MinWheelOutsideDameter;		//储存最小带轮外径da(mm);
	DoublePOINT WheelHubDameter;					//储存轮毂直径d1(mm);

	vector<double> SmallWheelHubBroad;				//储存小轮轮毂宽度L(mm);
	vector<double> BigWheelHubBroad;				//储存大轮轮毂宽度L(mm);

	vector<DoublePOINT> WheelRimINDameter;			//储存轮缘内径D1(mm);
	DoublePOINT WheelRimThick;						//储存最小轮缘厚度(mm)

	vector<string> SmallWheelStruct;				//推荐的小带轮结构形式;
	vector<string> BigWheelStruct;					//推荐的大带轮结构形式;

	vector<DoublePOINT> SmallWebThickness;			//小轮腹板厚度C';
	vector<DoublePOINT> BigWebThickness;			//大轮腹板厚度C';

	vector<DoublePOINT> EstimateMass;						//预估质量EM;
}VWheel;

class VBeltDriveDesigner {
private:
	//Ordinary_V_Pulley_WheelBenchmarkDiameter_Diameter_Series(普通V带轮基准直径系列);
	int WheelBenchmarkDiameter_Z[24] = { 50,56,63,71,75,80,90,100,112,125,132,140,150,160,180,200,224,250,280,315,355,400,500,630 };
	int WheelBenchmarkDiameter_A[28] = { 75,80,85,90,95,100,106,112,118,125,132,140,150,160,180,200,224,250,280,315,355,400,450,500,560,630,710,800 };
	int WheelBenchmarkDiameter_B[25] = { 125,132,140,150,160,170,180,200,224,250,280,315,355,400,450,500,560,600,630,710,750,800,900,1000,1120 };
	int WheelBenchmarkDiameter_C[27] = { 200,212,224,236,250,265,280,300,315,335,355,400,450,500,560,600,630,710,750,800,900,1000,1120,1250,1400,1600,2000 };
	int WheelBenchmarkDiameter_D[23] = { 355,375,400,425,450,475,500,560,600,630,710,750,800,900,1000,1060,1120,1250,1400,1500,1600,1800,2000 };
	int WheelBenchmarkDiameter_E[19] = { 500,530,560,600,630,670,710,800,900,1000,1120,1250,1400,1500,1600,1800,2000,2240,2500 };

	//BeltStandardLength(普通V带基准长度: Ld)；
	int Ld_Z[11] = { 405,475,530,625,700,780,920,1080,1330,1420,1540 };
	float KL_Z[11] = { 0.87,0.90,0.93,0.96,0.99,1.00,1.04,1.07,1.13,1.14,1.54 };
	int Ld_A[17] = { 630,700,790,890,990,1100,1250,1430,1550,1640,1750,1940,2050,2200,2300,2480,2700 };
	float KL_A[17] = { 0.81,0.83,0.85,0.87,0.89,0.91,0.93,0.96,0.98,0.99,1.00,1.02,1.04,1.06,1.07,1.09,1.10 };
	int Ld_B[20] = { 930,1000,1100,1210,1370,1560,1760,1950,2180,2300,2500,2700,2870,3200,3600,4060,4430,4820,5370,6070 };
	float KL_B[20] = { 0.83,0.84,0.86,0.87,0.90,0.92,0.94,0.97,0.99,1.01,1.03,1.04,1.05,1.07,1.09,1.13,1.15,1.17,1.20,1.24 };
	int Ld_C[17] = { 1565,1760,1950,2195,2420,2715,2880,3080,3520,4060,4600,5380,6100,6815,7600,9100,10700 };
	float KL_C[17] = { 0.82,0.85,0.87,0.90,0.92,0.94,0.95,0.97,0.99,1.02,1.05,1.08,1.11,1.14,1.17,1.21,1.24 };
	int Ld_D[15] = { 2740,3100,3330,3730,4080,4620,5400,6100,6840,7620,9140,10700,12200,13700,15200 };
	float KL_D[15] = { 0.82,0.86,0.87,0.90,0.91,0.94,0.97,0.99,1.02,1.05,1.08,1.13,1.16,1.19,1.21 };
	int Ld_E[11] = { 4660,5040,5420,6100,6850,7650,9150,12230,13750,15280,16800 };
	float KL_E[11] = { 0.91,0.92,0.94,0.96,0.99,1.01,1.05,1.11,1.15,1.17,1.19 };
	//带长修正系数索引程序KL;
	float KL_Indexer(int Ld, string Type) {
		if (PSET.Type == "Z") {
			for (int i = 0; i < 11; ++i)if (Ld_Z[i] == Ld)return KL_Z[i];
		} else if (PSET.Type == "A") {
			for (int i = 0; i < 17; ++i)if (Ld_A[i] == Ld)return KL_A[i];
		} else if (PSET.Type == "B") {
			for (int i = 0; i < 20; ++i)if (Ld_B[i] == Ld)return KL_B[i];
		} else if (PSET.Type == "C") {
			for (int i = 0; i < 17; ++i)if (Ld_C[i] == Ld)return KL_C[i];
		} else if (PSET.Type == "D") {
			for (int i = 0; i < 15; ++i)if (Ld_D[i] == Ld)return KL_D[i];
		} else if (PSET.Type == "E") {
			for (int i = 0; i < 11; ++i)if (Ld_E[i] == Ld)return KL_E[i];
		}
	}

	//单根普通V带的基本额定功率P0;
	//索引头块;
	int Index_SmallWheelRotatingSpeed[10] = { 400,700,800,950,1200,1450,1600,2000,2400,2800 };
	int Index_WheelBenchmarkDiameter_Z[6] = { 50,56,63,71,80,90 };
	int Index_WheelBenchmarkDiameter_A[8] = { 75,90,100,112,125,140,160,180 };
	int Index_WheelBenchmarkDiameter_B[8] = { 125,140,160,180,200,224,250,280 };
	int Index_WheelBenchmarkDiameter_C[8] = { 200,224,250,280,315,355,400,450 };
	int Index_WheelBenchmarkDiameter_D[8] = { 355,400,450,500,560,630,710,800 };
	//Z块;
	float Z50[10] = { 0.06,0.09,0.10,0.12,0.14,0.16,0.17,0.20,0.22,0.26 };
	float Z56[10] = { 0.06,0.11,0.12,0.14,0.17,0.19,0.20,0.25,0.30,0.33 };
	float Z63[10] = { 0.08,0.13,0.15,0.18,0.22,0.25,0.27,0.32,0.37,0.41 };
	float Z71[10] = { 0.09,0.17,0.20,0.23,0.27,0.30,0.33,0.39,0.46,0.50 };
	float Z80[10] = { 0.14,0.20,0.22,0.26,0.30,0.35,0.39,0.44,0.50,0.56 };
	float Z90[10] = { 0.14,0.22,0.24,0.28,0.33,0.36,0.40,0.48,0.54,0.60 };
	//A块;
	float A75[10] = { 0.26,0.40,0.45,0.51,0.60,0.68,0.73,0.84,0.92,1.00 };
	float A90[10] = { 0.39,0.61,0.68,0.77,0.93,1.07,1.15,1.34,1.50,1.64 };
	float A100[10] = { 0.47,0.74,0.83,0.95,1.14,1.32,1.42,1.66,1.87,2.05 };
	float A112[10] = { 0.56,0.90,1.00,1.15,1.39,1.61,1.74,2.04,2.30,2.51 };
	float A125[10] = { 0.67,1.07,1.19,1.37,1.66,1.92,2.07,2.44,2.74,2.98 };
	float A140[10] = { 0.78,1.26,1.41,1.62,1.96,2.28,2.45,2.87,3.22,3.48 };
	float A160[10] = { 0.94,1.51,1.69,1.95,2.36,2.73,2.94,3.42,3.80,4.06 };
	float A180[10] = { 1.09,1.76,1.97,2.27,2.74,3.16,3.40,3.93,4.32,4.54 };
	//B块;
	float B125[10] = { 0.84,1.30,1.44,1.64,1.93,2.19,2.33,2.64,2.85,2.96 };
	float B140[10] = { 1.05,1.64,1.82,2.08,2.47,2.82,3.00,3.42,3.70,3.85 };
	float B160[10] = { 1.32,2.09,2.32,2.66,3.17,3.62,3.86,4.40,4.75,4.89 };
	float B180[10] = { 1.59,2.53,2.81,3.22,3.85,4.39,4.68,5.30,5.67,5.76 };
	float B200[10] = { 1.85,2.96,3.30,3.77,4.50,5.13,5.46,6.13,6.47,6.43 };
	float B224[10] = { 2.17,3.47,3.86,4.42,5.26,5.97,6.33,7.02,7.25,6.95 };
	float B250[10] = { 2.50,4.00,4.46,5.10,6.04,6.82,7.20,7.87,7.89,7.14 };
	float B280[10] = { 2.89,4.61,5.13,5.85,6.90,7.76,8.13,8.60,8.22,6.80 };
	//C块;
	float C200[10] = { 2.41,3.69,4.07,4.58,5.29,5.84,6.07,6.34,6.02,5.01 };
	float C224[10] = { 2.99,4.64,5.12,5.78,6.71,7.45,7.75,8.06,7.57,6.08 };
	float C250[10] = { 3.62,5.64,6.23,7.04,8.21,9.04,9.38,9.62,8.75,6.56 };
	float C280[10] = { 4.32,6.76,7.52,8.49,9.81,10.72,11.06,11.04,9.50,6.13 };
	float C315[10] = { 5.14,8.09,8.92,10.05,11.53,12.46,12.72,12.14,9.43,4.16 };
	float C355[9] = { 6.05,9.50,10.46,11.73,13.31,14.12,14.19,12.59,7.98 };
	float C400[9] = { 7.06,11.02,12.10,13.48,15.04,15.53,15.24,11.95,4.34 };
	float C450[8] = { 8.20,12.63,13.80,15.23,16.59,16.47,15.57,9.64 };
	//D块;
	float D355[7] = { 9.24,13.70,14.83,16.15,17.25,16.77,15.63 };
	float D400[7] = { 11.45,17.07,18.46,20.06,21.20,20.15,18.31 };
	float D450[7] = { 13.85,20.63,22.25,24.01,24.84,22.02,19.59 };
	float D500[7] = { 16.20,23.99,25.76,27.50,26.71,23.59,18.88 };
	float D560[7] = { 18.95,27.73,29.55,31.04,29.67,22.58,15.13 };
	float D630[7] = { 22.05,31.68,33.38,34.19,30.15,18.06,6.25 };
	float D710[6] = { 25.45,35.59,36.87,36.35,27.88,7.99 };
	float D800[5] = { 29.08,39.14,39.55,36.6,21.32 };
	//P0表格索引程序;
	float P0_Indexer(int WheelBenchmarkDiameter1, double SmallWheelRotatingSpeed, string Type) {
		int Y = 0;//表格列指针位置;

		//设计转速匹配最近的基准转速;
		int Index_RotatingSpeed = StaticGetNearestElement(Index_SmallWheelRotatingSpeed, SmallWheelRotatingSpeed, 10);
		//获取列位置;
		if (Index_RotatingSpeed == 400) {
			Y = 0;
		} else if (Index_RotatingSpeed == 700) {
			Y = 1;
		} else if (Index_RotatingSpeed == 800) {
			Y = 2;
		} else if (Index_RotatingSpeed == 950) {
			Y = 3;
		} else if (Index_RotatingSpeed == 1200) {
			Y = 4;
		} else if (Index_RotatingSpeed == 1450) {
			Y = 5;
		} else if (Index_RotatingSpeed == 1600) {
			Y = 6;
		} else if (Index_RotatingSpeed == 2000) {
			Y = 7;
		} else if (Index_RotatingSpeed == 2400) {
			Y = 8;
		} else if (Index_RotatingSpeed == 2800) {
			Y = 9;
		}

		//小带轮实际直径匹配最近的基准直径;
		int Index_BenchmarkDiameter = NULL;
		//匹配块位置;
		if (Type == "Z") {
			Index_BenchmarkDiameter = StaticGetNearestElement(Index_WheelBenchmarkDiameter_Z, WheelBenchmarkDiameter1, 6);

			//匹配行位置;
			if (Index_BenchmarkDiameter == 50) {
				return Z50[Y];
			} else if (Index_BenchmarkDiameter == 56) {
				return Z56[Y];
			} else if (Index_BenchmarkDiameter == 63) {
				return Z63[Y];
			} else if (Index_BenchmarkDiameter == 71) {
				return Z71[Y];
			} else if (Index_BenchmarkDiameter == 80) {
				return Z80[Y];
			} else if (Index_BenchmarkDiameter == 90) {
				return Z90[Y];
			}
		} else if (Type == "A") {
			Index_BenchmarkDiameter = StaticGetNearestElement(Index_WheelBenchmarkDiameter_A, WheelBenchmarkDiameter1, 8);

			//匹配行位置;
			if (Index_BenchmarkDiameter == 75) {
				return A75[Y];
			} else if (Index_BenchmarkDiameter == 90) {
				return A90[Y];
			} else if (Index_BenchmarkDiameter == 100) {
				return A100[Y];
			} else if (Index_BenchmarkDiameter == 112) {
				return A112[Y];
			} else if (Index_BenchmarkDiameter == 125) {
				return A125[Y];
			} else if (Index_BenchmarkDiameter == 140) {
				return A140[Y];
			} else if (Index_BenchmarkDiameter == 160) {
				return A160[Y];
			} else if (Index_BenchmarkDiameter == 180) {
				return A180[Y];
			}
		} else if (Type == "B") {
			Index_BenchmarkDiameter = StaticGetNearestElement(Index_WheelBenchmarkDiameter_B, WheelBenchmarkDiameter1, 8);

			//匹配行位置;
			if (Index_BenchmarkDiameter == 125) {
				return B125[Y];
			} else if (Index_BenchmarkDiameter == 140) {
				return B140[Y];
			} else if (Index_BenchmarkDiameter == 160) {
				return B160[Y];
			} else if (Index_BenchmarkDiameter == 180) {
				return B180[Y];
			} else if (Index_BenchmarkDiameter == 200) {
				return B200[Y];
			} else if (Index_BenchmarkDiameter == 224) {
				return B224[Y];
			} else if (Index_BenchmarkDiameter == 250) {
				return B250[Y];
			} else if (Index_BenchmarkDiameter == 280) {
				return B280[Y];
			}
		} else if (Type == "C") {
			Index_BenchmarkDiameter = StaticGetNearestElement(Index_WheelBenchmarkDiameter_C, WheelBenchmarkDiameter1, 8);

			//匹配行位置;
			if (Index_BenchmarkDiameter == 200) {
				return C200[Y];
			} else if (Index_BenchmarkDiameter == 224) {
				return C224[Y];
			} else if (Index_BenchmarkDiameter == 250) {
				return C250[Y];
			} else if (Index_BenchmarkDiameter == 280) {
				return C280[Y];
			} else if (Index_BenchmarkDiameter == 315) {
				return C315[Y];
			} else if (Index_BenchmarkDiameter == 355) {
				return C355[Y];
			} else if (Index_BenchmarkDiameter == 400) {
				return C400[Y];
			} else if (Index_BenchmarkDiameter == 450) {
				return C450[Y];
			}
		} else if (Type == "D") {
			Index_BenchmarkDiameter = StaticGetNearestElement(Index_WheelBenchmarkDiameter_D, WheelBenchmarkDiameter1, 8);

			//匹配行位置;
			if (Index_BenchmarkDiameter == 355) {
				return D355[Y];
			} else if (Index_BenchmarkDiameter == 400) {
				return D400[Y];
			} else if (Index_BenchmarkDiameter == 450) {
				return D450[Y];
			} else if (Index_BenchmarkDiameter == 500) {
				return D500[Y];
			} else if (Index_BenchmarkDiameter == 560) {
				return D560[Y];
			} else if (Index_BenchmarkDiameter == 630) {
				return D630[Y];
			} else if (Index_BenchmarkDiameter == 710) {
				return D710[Y];
			} else if (Index_BenchmarkDiameter == 800) {
				return D800[Y];
			}
		} else if (Type == "E") {
			Index_BenchmarkDiameter = NULL; cout << "TypeE: P0 does not exist." << endl;
		}
		return NULL;
	}

	//单根普通V带额定功率的增量DeltaP0;
	DoublePOINT Index_TransmissionRatio[10] = { {1.00,1.01},{1.02,1.04},{1.05,1.08},{1.09,1.12},{1.13,1.18},{1.19,1.24},{1.25,1.34},{1.35,1.50},{1.51,1.99},{2.00,INT_MAX} };
	//Z块;
	float Z100D[10] = { 0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00 };
	float Z102D[10] = { 0.00,0.00,0.00,0.00,0.00,0.00,0.01,0.01,0.01,0.01 };
	float Z105D[10] = { 0.00,0.00,0.00,0.00,0.01,0.01,0.01,0.01,0.02,0.02 };
	float Z109D[10] = { 0.00,0.00,0.00,0.01,0.01,0.01,0.01,0.02,0.02,0.02 };
	float Z113D[10] = { 0.00,0.00,0.01,0.01,0.01,0.01,0.01,0.02,0.02,0.03 };
	float Z119D[10] = { 0.00,0.00,0.01,0.01,0.01,0.02,0.02,0.02,0.03,0.03 };
	float Z125D[10] = { 0.00,0.01,0.01,0.01,0.02,0.02,0.02,0.02,0.03,0.03 };
	float Z135D[10] = { 0.00,0.01,0.01,0.02,0.02,0.02,0.02,0.03,0.03,0.04 };
	float Z151D[10] = { 0.01,0.01,0.02,0.02,0.02,0.02,0.03,0.03,0.04,0.04 };
	float Z200D[10] = { 0.01,0.02,0.02,0.02,0.03,0.03,0.03,0.04,0.04,0.04 };
	//A块;
	float A100D[10] = { 0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00 };
	float A102D[10] = { 0.01,0.01,0.01,0.01,0.02,0.02,0.02,0.03,0.03,0.04 };
	float A105D[10] = { 0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.06,0.07,0.08 };
	float A109D[10] = { 0.02,0.03,0.03,0.04,0.05,0.06,0.06,0.08,0.10,0.11 };
	float A113D[10] = { 0.02,0.04,0.04,0.05,0.07,0.08,0.09,0.11,0.13,0.15 };
	float A119D[10] = { 0.03,0.05,0.05,0.06,0.08,0.09,0.11,0.13,0.16,0.19 };
	float A125D[10] = { 0.03,0.06,0.06,0.07,0.10,0.11,0.13,0.16,0.19,0.23 };
	float A135D[10] = { 0.04,0.07,0.08,0.08,0.11,0.13,0.15,0.19,0.23,0.26 };
	float A151D[10] = { 0.04,0.08,0.09,0.10,0.13,0.15,0.17,0.22,0.26,0.30 };
	float A200D[10] = { 0.05,0.09,0.10,0.11,0.15,0.17,0.19,0.24,0.29,0.34 };
	//B块;
	float B100D[10] = { 0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00 };
	float B102D[10] = { 0.01,0.02,0.03,0.03,0.04,0.05,0.06,0.07,0.08,0.10 };
	float B105D[10] = { 0.03,0.05,0.06,0.07,0.08,0.10,0.11,0.14,0.17,0.20 };
	float B109D[10] = { 0.04,0.07,0.08,0.10,0.13,0.15,0.17,0.21,0.25,0.29 };
	float B113D[10] = { 0.06,0.10,0.11,0.13,0.17,0.20,0.23,0.28,0.34,0.39 };
	float B119D[10] = { 0.07,0.12,0.14,0.17,0.21,0.25,0.28,0.35,0.42,0.49 };
	float B125D[10] = { 0.08,0.15,0.17,0.20,0.25,0.31,0.34,0.42,0.51,0.59 };
	float B135D[10] = { 0.10,0.17,0.20,0.23,0.30,0.36,0.39,0.49,0.59,0.69 };
	float B151D[10] = { 0.11,0.20,0.23,0.26,0.34,0.40,0.45,0.56,0.68,0.79 };
	float B200D[10] = { 0.13,0.22,0.25,0.30,0.38,0.46,0.51,0.63,0.76,0.89 };
	//C块;
	float C100D[10] = { 0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00 };
	float C102D[10] = { 0.04,0.07,0.08,0.09,0.12,0.14,0.16,0.20,0.23,0.27 };
	float C105D[10] = { 0.08,0.14,0.16,0.19,0.24,0.28,0.31,0.39,0.47,0.55 };
	float C109D[10] = { 0.12,0.21,0.23,0.27,0.35,0.42,0.47,0.59,0.70,0.82 };
	float C113D[10] = { 0.16,0.27,0.31,0.37,0.47,0.58,0.63,0.78,0.94,1.10 };
	float C119D[10] = { 0.20,0.34,0.39,0.47,0.59,0.71,0.78,0.98,1.18,1.37 };
	float C125D[10] = { 0.23,0.41,0.47,0.56,0.70,0.85,0.94,1.17,1.41,1.64 };
	float C135D[10] = { 0.27,0.48,0.55,0.65,0.82,0.99,1.10,1.37,1.65,1.92 };
	float C151D[10] = { 0.31,0.55,0.63,0.74,0.94,1.14,1.25,1.57,1.88,2.19 };
	float C200D[10] = { 0.35,0.62,0.71,0.83,1.06,1.27,1.41,1.76,2.12,2.47 };
	//D块;
	float D100D[7] = { 0.00,0.00,0.00,0.00,0.00,0.00,0.00 };
	float D102D[7] = { 0.14,0.24,0.28,0.33,0.42,0.51,0.56 };
	float D105D[7] = { 0.28,0.49,0.56,0.66,0.84,1.01,1.11 };
	float D109D[7] = { 0.42,0.73,0.83,0.99,1.25,1.51,1.67 };
	float D113D[7] = { 0.56,0.97,1.11,1.32,1.67,2.02,2.23 };
	float D119D[7] = { 0.70,1.22,1.39,1.60,2.09,2.52,2.78 };
	float D125D[7] = { 0.83,1.46,1.67,1.92,2.50,3.02,3.33 };
	float D135D[7] = { 0.97,1.70,1.95,2.31,2.92,3.52,3.89 };
	float D151D[7] = { 1.11,1.95,2.22,2.64,3.34,4.03,4.45 };
	float D200D[7] = { 1.25,2.19,2.50,2.97,3.75,4.53,5.00 };
	//DeltaP0表格索引程序;
	float DeltaP0_Indexer(double RealTransmissionRatio, double SmallWheelRotatingSpeed, string Type) {
		int X = 0, Y = 0;//表格指针位置;

		//设计转速匹配最近的基准转速;
		int Index_RotatingSpeed = StaticGetNearestElement(Index_SmallWheelRotatingSpeed, SmallWheelRotatingSpeed, 10);
		//获取列位置;
		if (Index_RotatingSpeed == 400) {
			Y = 0;
		} else if (Index_RotatingSpeed == 700) {
			Y = 1;
		} else if (Index_RotatingSpeed == 800) {
			Y = 2;
		} else if (Index_RotatingSpeed == 950) {
			Y = 3;
		} else if (Index_RotatingSpeed == 1200) {
			Y = 4;
		} else if (Index_RotatingSpeed == 1450) {
			Y = 5;
		} else if (Index_RotatingSpeed == 1600) {
			Y = 6;
		} else if (Index_RotatingSpeed == 2000) {
			Y = 7;
		} else if (Index_RotatingSpeed == 2400) {
			Y = 8;
		} else if (Index_RotatingSpeed == 2800) {
			Y = 9;
		}

		//匹配传动比(获取块中的行指针位置);
		for (int i = 0; i < 10; ++i) if (RealTransmissionRatio >= Index_TransmissionRatio[i].x && RealTransmissionRatio <= Index_TransmissionRatio[i].y)X = i;

		//匹配块位置;
		if (Type == "Z") {
			if (X == 0)return Z100D[Y];
			if (X == 1)return Z102D[Y];
			if (X == 3)return Z105D[Y];
			if (X == 4)return Z109D[Y];
			if (X == 5)return Z113D[Y];
			if (X == 6)return Z119D[Y];
			if (X == 7)return Z125D[Y];
			if (X == 8)return Z135D[Y];
			if (X == 9)return Z151D[Y];
			if (X == 10)return Z200D[Y];
		} else if (Type == "A") {
			if (X == 0)return A100D[Y];
			if (X == 1)return A102D[Y];
			if (X == 3)return A105D[Y];
			if (X == 4)return A109D[Y];
			if (X == 5)return A113D[Y];
			if (X == 6)return A119D[Y];
			if (X == 7)return A125D[Y];
			if (X == 8)return A135D[Y];
			if (X == 9)return A151D[Y];
			if (X == 10)return A200D[Y];
		} else if (Type == "B") {
			if (X == 0)return B100D[Y];
			if (X == 1)return B102D[Y];
			if (X == 3)return B105D[Y];
			if (X == 4)return B109D[Y];
			if (X == 5)return B113D[Y];
			if (X == 6)return B119D[Y];
			if (X == 7)return B125D[Y];
			if (X == 8)return B135D[Y];
			if (X == 9)return B151D[Y];
			if (X == 10)return B200D[Y];
		} else if (Type == "C") {
			if (X == 0)return C100D[Y];
			if (X == 1)return C102D[Y];
			if (X == 3)return C105D[Y];
			if (X == 4)return C109D[Y];
			if (X == 5)return C113D[Y];
			if (X == 6)return C119D[Y];
			if (X == 7)return C125D[Y];
			if (X == 8)return C135D[Y];
			if (X == 9)return C151D[Y];
			if (X == 10)return C200D[Y];
		} else if (Type == "D") {
			if (X == 0)return D100D[Y];
			if (X == 1)return D102D[Y];
			if (X == 3)return D105D[Y];
			if (X == 4)return D109D[Y];
			if (X == 5)return D113D[Y];
			if (X == 6)return D119D[Y];
			if (X == 7)return D125D[Y];
			if (X == 8)return D135D[Y];
			if (X == 9)return D151D[Y];
			if (X == 10)return D200D[Y];
		} else if (Type == "E") {
			cout << "TypeE: DeltaP0 does not exist." << endl;
		}
		return NULL;
	}

	//包角修正系数Ka;
	int BaseWrapAngle[13] = { 120,125,130,135,140,145,150,155,160,165,170,175,180 };
	float WrapAngleFix[13] = { 0.82,0.84,0.86,0.88,0.89,0.91,0.92,0.93,0.95,0.96,0.98,0.99,1.00 };
	//Ka索引程序;
	float WrapAngleFix_Indexer(double SmallWheelWrapAngle) {
		for (int i = 0; i < 13; ++i) if (StaticGetNearestElement(BaseWrapAngle, SmallWheelWrapAngle, 13) == BaseWrapAngle[i])return WrapAngleFix[i];
		return NULL;
	}

	//V带轮槽选型表(Bd, Hamin, Hfmin, e, Fmin, Delta_min);
	float Z_Notch[6] = { 8.5,2.0,7.0,12,7,5.5 };
	float A_Notch[6] = { 11.0,2.75,8.7,15,9,6 };
	float B_Notch[6] = { 14.0,3.50,10.8,19,11.5,7.5 };
	float C_Notch[6] = { 19.0,4.80,14.3,25.5,16,10 };
	float D_Notch[6] = { 27.0,8.10,19.9,37,23,12 };
	float E_Notch[6] = { 32.0,9.60,23.4,44.5,28,15 };
	//V带轮槽选型表索引程序;
	vector<float> WheelNotch_Indexer(int WheelDiameter, string Type) {
		vector<float> WheelNotch;
		if (Type == "Z") {
			for (int i = 0; i < 6; ++i)WheelNotch.push_back(Z_Notch[i]);
			if (WheelDiameter <= 80) WheelNotch.push_back(34); else WheelNotch.push_back(38);
		} else if (Type == "A") {
			for (int i = 0; i < 6; ++i)WheelNotch.push_back(A_Notch[i]);
			if (WheelDiameter <= 118) WheelNotch.push_back(34); else WheelNotch.push_back(38);
		} else if (Type == "B") {
			for (int i = 0; i < 6; ++i)WheelNotch.push_back(B_Notch[i]);
			if (WheelDiameter <= 190) WheelNotch.push_back(34); else WheelNotch.push_back(38);
		} else if (Type == "C") {
			for (int i = 0; i < 6; ++i)WheelNotch.push_back(C_Notch[i]);
			if (WheelDiameter <= 315) WheelNotch.push_back(34); else WheelNotch.push_back(38);
		} else if (Type == "D") {
			for (int i = 0; i < 6; ++i)WheelNotch.push_back(D_Notch[i]);
			if (WheelDiameter <= 475) WheelNotch.push_back(36); else WheelNotch.push_back(38);
		} else if (Type == "E") {
			for (int i = 0; i < 6; ++i)WheelNotch.push_back(E_Notch[i]);
			if (WheelDiameter <= 600) WheelNotch.push_back(36); else WheelNotch.push_back(38);
		}
		return WheelNotch;
	}

	//V带轮的结构形式和辐板厚度选型表;
	//轴孔径系列索引块;
	POINT Index_Zd[6] = { {12,14},{16,18},{20,22},{24,25},{28,30},{32,35} };
	POINT Index_Ad[7] = { {10,18},{20,22},{24,25},{28,30},{32,35},{38,40},{42,45} };
	POINT Index_Bd[5] = { {32,35},{38,40},{42,45},{50,55},{60,65} };
	POINT Index_Cd[5] = { {42,45},{50,55},{60,65},{70,75},{80,85} };
	POINT Index_Dd[5] = { {60,65},{70,75},{80,85},{90,95},{100,110} };
	POINT Index_Ed[5] = { {80,85},{90,95},{100,110},{120,130},{140,150} };
	//轮结构索引块;
	//实心:S; 腹板:W; 孔板:O; 轮辐:P;
	/*int D63[33] = {S,S};
	int D71[33] = { S,S,S };
	int D75[33] = { S,S,S,S };
	int D80[33] = { W6,S,S,S };
	int D90[33] = { W6,W7,S,S,S,0,S,S,S,S };
	int D95[33] = { W6,W7,S,S,S,0,S,S,S,S };
	int D100[33] = { W6,W7,W7,S,S,S,S,S,S,S,S };
	int D106[33] = { W6,W7,W7,S,S,S,W,S,S,S,S,S };
	int D112[33] = { W7,W8,W8,W8,W8,S,W10,S,S,S,S,S };
	int D118[33] = { W7,W8,W8,W8,W8,S,W10,W10,S,S,S,S,S };
	int D125[33] = {W7,W8,W8,W8,W8,S,W10,W10,S,S,S,S,S,S,S,S};
	int D132[33] = {W7,W8,W8,W8,W8,W8,W8,W11,W11,W11};
	int D140[33] = {W7,W9,W9,W9,W9,W9,W11,W11,W12,W12,S,S,S,S,S,S,S};
	int D150[33] = {W7,W9,W9,W9,W9,W9,W11,W11,W12,W12,W12,S,S,S,S,S,S,S};
	int D160[33] = {0,W9,W9,W9,W9,W9,W11,W11,W12,W12,W12,W12,S,W14,S,S,S,S};
	int D170[33] = {0,O4_9,W9,W9,W9,W9,W12,W12,W13,W13,W13,W13,S,W14,W14,S,S,S};
	int D180[33] = {0,O4_10,O4_10,O4_10 };
	int D200[33] = {};
	int D212[33] = {};
	int D224[33] = {};
	int D236[33] = {};
	int D250[33] = {};
	int D265[33] = {};
	int D280[33] = {};
	int D300[33] = {};
	int D315[33] = {};
	int D355[33] = {};
	int D375[33] = {};
	int D400[33] = {};
	int D425[33] = {};
	int D450[33] = {};
	int D475[33] = {};
	int D500[33] = {};
	int D530[33] = {};
	int D560[33] = {};
	int D600[33] = {};
	int D630[33] = {};
	int D710[33] = {};

	int D750[33] = {};
	int D2500[33] = {};*/

private:
	//计算空心圆柱体体积(mm^3);
	double HollowCylinder(double D, double d, double h) { return PI * h / 4 * (pow(D, 2) - pow(d, 2)); }

	int getNearest(int x, int y, double target) { if (target - x >= y - target)return y; else return x; }
	int getNearestElement(vector<int> arr, double target) {
		int n = arr.size();
		if (target <= arr[0])return arr[0];
		if (target >= arr[n - 1])return arr[n - 1];
		int left = 0, right = n, mid = 0;
		while (left < right) {
			mid = (left + right) / 2;
			if (arr[mid] == target)return arr[mid];
			if (target < arr[mid]) {
				if (mid > 0 && target > arr[mid - 1])return getNearest(arr[mid - 1], arr[mid], target);
				right = mid;
			} else {
				if (mid < n - 1 && target < arr[mid + 1])return getNearest(arr[mid], arr[mid + 1], target);
				left = mid + 1;
			}
		}
		return arr[mid];
	}
	int StaticGetNearestElement(int arr[], double target, int Num) {
		if (target <= arr[0])return arr[0];
		if (target >= arr[Num - 1])return arr[Num - 1];
		int left = 0, right = Num, mid = 0;
		while (left < right) {
			mid = (left + right) / 2;
			if (arr[mid] == target)return arr[mid];
			if (target < arr[mid]) {
				if (mid > 0 && target > arr[mid - 1])return getNearest(arr[mid - 1], arr[mid], target);
				right = mid;
			} else {
				if (mid < Num - 1 && target < arr[mid + 1])return getNearest(arr[mid], arr[mid + 1], target);
				left = mid + 1;
			}
		}
		return arr[mid];
	}

	void AutoWheelBenchmarkDiameter() {
		//根据带型选择小带轮基准直径;
		//验算小带轮带速度,排除不符合带速条件的小带轮基准直径;
		if (PSET.Type == "Z") {
			for (int i = 0; i < 24; ++i) {
				double BeltVelocity = (PI * WheelBenchmarkDiameter_Z[i] * PSET.SmallWheelRotatingSpeed) / 60000;
				if (BeltVelocity >= 5 && BeltVelocity <= 30) Belt.WheelBenchmarkDiameter1.push_back(WheelBenchmarkDiameter_Z[i]);
			}
		} else if (PSET.Type == "A") {
			for (int i = 0; i < 28; ++i) {
				double BeltVelocity = (PI * WheelBenchmarkDiameter_A[i] * PSET.SmallWheelRotatingSpeed) / 60000;
				if (BeltVelocity >= 5 && BeltVelocity <= 30) Belt.WheelBenchmarkDiameter1.push_back(WheelBenchmarkDiameter_A[i]);
			}
		} else if (PSET.Type == "B") {
			for (int i = 0; i < 25; ++i) {
				double BeltVelocity = (PI * WheelBenchmarkDiameter_B[i] * PSET.SmallWheelRotatingSpeed) / 60000;
				if (BeltVelocity >= 5 && BeltVelocity <= 30) Belt.WheelBenchmarkDiameter1.push_back(WheelBenchmarkDiameter_B[i]);
			}
		} else if (PSET.Type == "C") {
			for (int i = 0; i < 27; ++i) {
				double BeltVelocity = (PI * WheelBenchmarkDiameter_C[i] * PSET.SmallWheelRotatingSpeed) / 60000;
				if (BeltVelocity >= 5 && BeltVelocity <= 30) Belt.WheelBenchmarkDiameter1.push_back(WheelBenchmarkDiameter_C[i]);
			}
		} else if (PSET.Type == "D") {
			for (int i = 0; i < 23; ++i) {
				double BeltVelocity = (PI * WheelBenchmarkDiameter_D[i] * PSET.SmallWheelRotatingSpeed) / 60000;
				if (BeltVelocity >= 5 && BeltVelocity <= 30) Belt.WheelBenchmarkDiameter1.push_back(WheelBenchmarkDiameter_D[i]);
			}
		} else if (PSET.Type == "E") {
			for (int i = 0; i < 19; ++i) {
				double BeltVelocity = (PI * WheelBenchmarkDiameter_E[i] * PSET.SmallWheelRotatingSpeed) / 60000;
				if (BeltVelocity >= 5 && BeltVelocity <= 30) Belt.WheelBenchmarkDiameter1.push_back(WheelBenchmarkDiameter_E[i]);
			}
		}

		//计算并圆整大带轮基准直径;
		vector<int> WheelBenchmarkDiameter1_TEMP;
		for (int i = 0; i < Belt.WheelBenchmarkDiameter1.size(); ++i) {
			double Dd2 = getNearestElement(Belt.WheelBenchmarkDiameter1, PSET.ImagTransmissionRatio * Belt.WheelBenchmarkDiameter1[i] * (1 - PSET.Epsilon));

			//保证实际传动比在可接受的范围内;
			double RealTransmissionRatio = Dd2 / Belt.WheelBenchmarkDiameter1[i];					//计算实际传动比;
			double ToleranceValue = PSET.ImagTransmissionRatio * PSET.TransmissionRatioTolerance;	//计算传动比容许的误差值;

			if (RealTransmissionRatio >= PSET.ImagTransmissionRatio - ToleranceValue && RealTransmissionRatio <= PSET.ImagTransmissionRatio + ToleranceValue) {
				Belt.WheelBenchmarkDiameter2.push_back(Dd2);							//实际传动比可行的大带轮基准直径;
				WheelBenchmarkDiameter1_TEMP.push_back(Belt.WheelBenchmarkDiameter1[i]);//实际传动比可行的小带轮基准直径;
			}
		}

		//用实际传动比可行的小带轮基准直径替换掉原本的小带轮基准直径;
		Belt.WheelBenchmarkDiameter1.clear();
		for (int i = 0; i < WheelBenchmarkDiameter1_TEMP.size(); ++i) Belt.WheelBenchmarkDiameter1.push_back(WheelBenchmarkDiameter1_TEMP[i]);
	}

	void AutoBeltGeometricParameters(double Accuracy = 20) {
		for (int i = 0; i < Belt.WheelBenchmarkDiameter1.size(); ++i) {
			//初定两轮中心距;
			double WheelDiameterSum = Belt.WheelBenchmarkDiameter1[i] + Belt.WheelBenchmarkDiameter2[i];		//计算两轮直径和;
			double WheelDiameterDiff = Belt.WheelBenchmarkDiameter2[i] - Belt.WheelBenchmarkDiameter1[i];		//计算两轮直径差;

			//计算实际传动比;
			double RealTransmissionRatio = double(Belt.WheelBenchmarkDiameter2[i]) / double(Belt.WheelBenchmarkDiameter1[i]);
			//查表->单根普通V带额定功率的增量DeltaP0;
			float DeltaP0 = DeltaP0_Indexer(RealTransmissionRatio, PSET.SmallWheelRotatingSpeed, PSET.Type);

			for (double RoughWheelbase = 0.7 * WheelDiameterSum; RoughWheelbase < 2 * WheelDiameterSum; RoughWheelbase += Accuracy) {
				double ImagBeltLength = 2 * RoughWheelbase + (PI * 0.5) * WheelDiameterSum + pow(WheelDiameterDiff, 2) / (4 * RoughWheelbase);	//计算理论带长;

				//根据理论带长查找基准带长;
				int RealBeltLength = NULL;	//储存选出的带基准长度(也是实际带长);
				if (PSET.Type == "Z") {
					RealBeltLength = StaticGetNearestElement(Ld_Z, ImagBeltLength, 11);
				} else if (PSET.Type == "A") {
					RealBeltLength = StaticGetNearestElement(Ld_A, ImagBeltLength, 17);
				} else if (PSET.Type == "B") {
					RealBeltLength = StaticGetNearestElement(Ld_B, ImagBeltLength, 20);
				} else if (PSET.Type == "C") {
					RealBeltLength = StaticGetNearestElement(Ld_C, ImagBeltLength, 17);
				} else if (PSET.Type == "D") {
					RealBeltLength = StaticGetNearestElement(Ld_D, ImagBeltLength, 15);
				} else if (PSET.Type == "E") {
					RealBeltLength = StaticGetNearestElement(Ld_E, ImagBeltLength, 11);
				}

				//计算实际中心距;
				double _RealWheelbase = RoughWheelbase + (RealBeltLength - ImagBeltLength) * 0.5;

				//验算小带轮包角;
				double WrapAngle = 180 - WheelDiameterDiff * (57.29578 / _RealWheelbase);

				//计算实际带速;
				double _RealBeltSpeed = (PI * Belt.WheelBenchmarkDiameter1[i] * PSET.SmallWheelRotatingSpeed) / 60000;

				//确定带的条数z;
				float P0 = P0_Indexer(Belt.WheelBenchmarkDiameter1[i], PSET.SmallWheelRotatingSpeed, PSET.Type);	//查表->单根普通V带的基本额定功率P0;
				float Ka = WrapAngleFix_Indexer(WrapAngle);			//查表->包角修正系数Ka;
				float KL = KL_Indexer(RealBeltLength, PSET.Type);	//查表->带长修正系数KL
				double Z = (PSET.KA * PSET.NominalPower) / ((P0 + DeltaP0) * Ka * KL);

				//保存最终结果;
				if (WrapAngle >= PSET.MinWrapAngle && Z < 10) {
					Belt.Result.SmallWheelDiameter.push_back(Belt.WheelBenchmarkDiameter1[i]);	//储存小带轮直径;
					Belt.Result.BigWheelDiameter.push_back(Belt.WheelBenchmarkDiameter2[i]);	//储存大带轮直径;
					Belt.Result.TransmissionRatio.push_back(RealTransmissionRatio);				//储存传动比;
					Belt.Result.RealBeltLength.push_back(RealBeltLength);						//储存带的基准长度;
					Belt.Result.RealWheelbase.push_back(_RealWheelbase);						//储存实际中心矩;
					Belt.Result.SmallWheelWrapAngle.push_back(WrapAngle);						//储存小带轮包角;
					Belt.Result.RealBeltSpeed.push_back(_RealBeltSpeed);						//储存实际带速;
					Belt.Result.BeltNumber_Z.push_back(round(Z));								//储存带的根数;
				}
			}
		}
	}

	void AutoWheelStruct() {
		for (int i = 0; i < Belt.Result.SmallWheelDiameter.size(); ++i) {
			//计算轮缘内径D1(小轮,大轮)
			Wheel.WheelRimINDameter.push_back({ Wheel.MinWheelOutsideDameter[i].x - 2 * Wheel.WheelRimThick.x, Wheel.MinWheelOutsideDameter[i].y - 2 * Wheel.WheelRimThick.y });

			//小带轮结构形式选型;
			if (Belt.Result.SmallWheelDiameter[i] <= 2.5 * PSET.SmallWheelAxleDameter) {
				Wheel.SmallWheelStruct.push_back("实心");
			} else if (Belt.Result.SmallWheelDiameter[i] <= 300) {
				if (Wheel.WheelRimINDameter[i].x - Wheel.WheelHubDameter.x >= 100)Wheel.SmallWheelStruct.push_back("孔板"); else Wheel.SmallWheelStruct.push_back("腹板");
			} else if (Belt.Result.SmallWheelDiameter[i] > 300) {
				Wheel.SmallWheelStruct.push_back("轮辐");
			}

			//大带轮结构形式选型;
			if (Belt.Result.BigWheelDiameter[i] <= 2.5 * PSET.BigWheelAxleDameter) {
				Wheel.BigWheelStruct.push_back("实心");
			} else if (Belt.Result.BigWheelDiameter[i] <= 300) {
				if (Wheel.WheelRimINDameter[i].y - Wheel.WheelHubDameter.y >= 100)Wheel.BigWheelStruct.push_back("孔板"); else Wheel.BigWheelStruct.push_back("腹板");
			} else if (Belt.Result.BigWheelDiameter[i] > 300) {
				Wheel.BigWheelStruct.push_back("轮辐");
			}

			//Wheel.SmallWheelStruct.push_back("实心");
			//Wheel.BigWheelStruct.push_back("孔板");

			if (Wheel.SmallWheelStruct[i] == "实心")Wheel.SmallWebThickness.clear();
			if (Wheel.BigWheelStruct[i] == "实心")Wheel.BigWebThickness.clear();
		}
	}

	void AutoWheelDesign() {
		vector<float> SmallWheelNotch, BigWheelNotch;
		for (int i = 0; i < Belt.Result.SmallWheelDiameter.size(); ++i) {
			//小带轮槽型选型;
			SmallWheelNotch = WheelNotch_Indexer(Belt.Result.SmallWheelDiameter[i], PSET.Type);
			BigWheelNotch = WheelNotch_Indexer(Belt.Result.BigWheelDiameter[i], PSET.Type);
			Wheel.phi.push_back({ SmallWheelNotch[6], BigWheelNotch[6] });

			//计算最小带轮宽: B＝(z-1)e+2fmin;
			double MinSmallWheelBroad = (Belt.Result.BeltNumber_Z[i] - 1) * SmallWheelNotch[3] + 2 * SmallWheelNotch[4];
			double MinBigWheelBroad = (Belt.Result.BeltNumber_Z[i] - 1) * BigWheelNotch[3] + 2 * BigWheelNotch[4];
			Wheel.MinWheelBroad.push_back({ MinSmallWheelBroad, MinBigWheelBroad });

			//计算腹板厚度;
			Wheel.SmallWebThickness.push_back({ round(static_cast<double>(1) / 7 * MinSmallWheelBroad) , round(static_cast<double>(1) / 4 * MinSmallWheelBroad) });	//计算小轮腹板厚度;
			Wheel.BigWebThickness.push_back({ round(static_cast<double>(1) / 7 * MinBigWheelBroad) , round(static_cast<double>(1) / 4 * MinBigWheelBroad) });		//计算大轮腹板厚度;

			//计算带轮最小外径: da＝dd+2hamin;
			double MinSmallWheelDa = Belt.Result.SmallWheelDiameter[i] + 2 * SmallWheelNotch[1];
			double MinBigWheelDa = Belt.Result.BigWheelDiameter[i] + 2 * BigWheelNotch[1];
			Wheel.MinWheelOutsideDameter.push_back({ MinSmallWheelDa, MinBigWheelDa });

			//计算轮毂宽度L;
			if (PSET.BigWheelAxleDameter == NULL)PSET.BigWheelAxleDameter = PSET.SmallWheelAxleDameter;
			if (PSET.SmallWheelAxleDameter == NULL)PSET.SmallWheelAxleDameter = PSET.BigWheelAxleDameter;
			if (PSET.SmallWheelAxleDameter == PSET.BigWheelAxleDameter == NULL)cout << "Error: Please Set WheelAxleDameter" << endl;

			if (MinSmallWheelBroad < 1.5 * PSET.SmallWheelAxleDameter) {
				Wheel.SmallWheelHubBroad.push_back(MinSmallWheelBroad);
			} else {
				Wheel.SmallWheelHubBroad.push_back(2 * PSET.SmallWheelAxleDameter);
			}
			if (MinBigWheelBroad < 1.5 * PSET.BigWheelAxleDameter) {
				Wheel.BigWheelHubBroad.push_back(MinBigWheelBroad);
			} else {
				Wheel.BigWheelHubBroad.push_back(2 * PSET.BigWheelAxleDameter);
			}
		}

		//储存轮槽截面尺寸数据(小轮,大轮);
		Wheel.Bd = { SmallWheelNotch[0], BigWheelNotch[0] };
		Wheel.Hamin = { SmallWheelNotch[1], BigWheelNotch[1] };
		Wheel.Hfmin = { SmallWheelNotch[2], BigWheelNotch[2] };
		Wheel.e = { SmallWheelNotch[3], BigWheelNotch[3] };
		Wheel.Fmin = { SmallWheelNotch[4], BigWheelNotch[4] };
		Wheel.deltaMin = { SmallWheelNotch[5], BigWheelNotch[5] };
		//计算最小轮缘厚度(小轮,大轮);
		Wheel.WheelRimThick = { Wheel.Hamin.x + Wheel.Hfmin.x + Wheel.deltaMin.x, Wheel.Hamin.y + Wheel.Hfmin.y + Wheel.deltaMin.y };
		//计算轮毂直径d1(小轮,大轮);
		if (PSET.BigWheelAxleDameter == NULL)PSET.BigWheelAxleDameter = PSET.SmallWheelAxleDameter;
		if (PSET.SmallWheelAxleDameter == NULL)PSET.SmallWheelAxleDameter = PSET.BigWheelAxleDameter;
		if (PSET.SmallWheelAxleDameter == PSET.BigWheelAxleDameter == NULL)cout << "Error: Please Set WheelAxleDameter" << endl;
		Wheel.WheelHubDameter = { 2 * PSET.SmallWheelAxleDameter , 2 * PSET.BigWheelAxleDameter };

		AutoWheelStruct();//带轮结构形式选型;

		//估计带轮毛坯质量;
		for (int i = 0; i < Belt.Result.SmallWheelDiameter.size(); ++i) {
			//计算小带轮轮缘体积;
			double SmallWheelRimVolume = HollowCylinder(Wheel.MinWheelOutsideDameter[i].x, Wheel.WheelRimINDameter[i].x, Wheel.MinWheelBroad[i].x);
			//计算小带轮腹板体积;
			double SmallWheelWebVolume = 0; if (Wheel.SmallWheelStruct[i] == "腹板" || Wheel.SmallWheelStruct[i] == "孔板") SmallWheelWebVolume = HollowCylinder(Wheel.WheelRimINDameter[i].x, Wheel.WheelHubDameter.x, 0.5 * (Wheel.SmallWebThickness[i].y - Wheel.SmallWebThickness[i].x));
			//计算小带轮轮毂体积;
			double SmallWheelHubVolume = HollowCylinder(Wheel.WheelHubDameter.x, PSET.SmallWheelAxleDameter, Wheel.SmallWheelHubBroad[i]);
			//小带轮预估毛坯总体积;
			double SmallWheelVolume = (SmallWheelRimVolume + SmallWheelWebVolume + SmallWheelHubVolume) * pow(10, -9);

			//计算大带轮轮缘体积;
			double BigWheelRimVolume = HollowCylinder(Wheel.MinWheelOutsideDameter[i].y, Wheel.WheelRimINDameter[i].y, Wheel.MinWheelBroad[i].y);
			//计算大带轮腹板体积;
			double BigWheelWebVolume = 0; if (Wheel.BigWheelStruct[i] == "腹板" || Wheel.BigWheelStruct[i] == "孔板") BigWheelWebVolume = HollowCylinder(Wheel.WheelRimINDameter[i].y, Wheel.WheelHubDameter.y, 0.5 * (Wheel.BigWebThickness[i].y - Wheel.BigWebThickness[i].y));
			//计算大带轮轮毂体积;
			double BigWheelHubVolume = HollowCylinder(Wheel.WheelHubDameter.y, PSET.BigWheelAxleDameter, Wheel.BigWheelHubBroad[i]);
			//大带轮预估毛坯总体积;
			double BigWheelVolume = (BigWheelRimVolume + BigWheelWebVolume + BigWheelHubVolume) * pow(10, -9);

			//分别计算两轮预估毛坯质量(小轮,大轮);
			Wheel.EstimateMass.push_back({ SmallWheelVolume * HT150 , BigWheelVolume * HT150 });
		}
	}

public:
	VBelt Belt;
	VWheel Wheel;
	Prerequisites PSET;
	void AutoVBelt() {
		AutoWheelBenchmarkDiameter();
		AutoBeltGeometricParameters(40);
		AutoWheelDesign();
	}

	void OutputData() {
		ofstream outFile;
		outFile.open("VBeltDesignerData.csv", ios::out);

		//setiosflags(ios::fixed);

		outFile << "KA" << "," << "名义功率(kW)" << "," << "小轮转速(r/min)" << "," << "带型" << "," << "基准宽度Bd" << "," << "Hamin" << "," << "Hfmin"
			<< "," << "槽间距e" << "," << "Fmin" << "," << "δmin" << "," << "轴孔径d" << endl;

		outFile << PSET.KA << "," << PSET.NominalPower << "," << PSET.SmallWheelRotatingSpeed << "," << PSET.Type << "," << Wheel.Bd.x << " / " << Wheel.Bd.y << ","
			<< Wheel.Hamin.x << " / " << Wheel.Hamin.y << "," << Wheel.Hfmin.x << " / " << Wheel.Hfmin.y << "," << Wheel.e.x << " / " << Wheel.e.y << "," << Wheel.Fmin.x << " / " << Wheel.Fmin.y
			<< "," << Wheel.deltaMin.x << " / " << Wheel.deltaMin.y << "," << PSET.SmallWheelAxleDameter << " / " << PSET.BigWheelAxleDameter << "," << "\n" << endl;

		outFile << "小轮" << "/" << "大轮直径" << ", " << "传动比" << ", " << "带基准长度" << ", " << "中心矩" << ", " << "小轮包角(°)" << ", "
			<< "带速(m/s)" << ", " << "带的根数" << "," << "建议带轮结构" << "," << "槽轮角φ(°)" << "," << "最小带轮宽B" << "," << "轮毂宽度L" << "," << "最小外径da" << ","
			<< "轮毂直径d1" << "," << "轮缘内径D1" << "," << "腹板厚度C'" << "," << "预估毛坯质量(Kg)" << endl;

		for (int i = 0; i < Belt.Result.SmallWheelDiameter.size(); ++i) {
			outFile << Belt.Result.SmallWheelDiameter[i] << " / " << Belt.Result.BigWheelDiameter[i] << "," << Belt.Result.TransmissionRatio[i]
				<< "," << Belt.Result.RealBeltLength[i] << "," << Belt.Result.RealWheelbase[i] << "," << Belt.Result.SmallWheelWrapAngle[i]
				<< "," << Belt.Result.RealBeltSpeed[i] << "," << Belt.Result.BeltNumber_Z[i] << "," << Wheel.SmallWheelStruct[i] << " / " << Wheel.BigWheelStruct[i]
				<< "," << (int)Wheel.phi[i].x << " / " << (int)Wheel.phi[i].y << "," << Wheel.MinWheelBroad[i].x << " / " << Wheel.MinWheelBroad[i].y << "," << Wheel.SmallWheelHubBroad[i] << " / " << Wheel.BigWheelHubBroad[i] << ","
				<< Wheel.MinWheelOutsideDameter[i].x << " / " << Wheel.MinWheelOutsideDameter[i].y << "," << Wheel.WheelHubDameter.x << " / "
				<< Wheel.WheelHubDameter.y << "," << Wheel.WheelRimINDameter[i].x << " / " << Wheel.WheelRimINDameter[i].y << "," << Wheel.SmallWebThickness[i].x << "~"
				<< Wheel.SmallWebThickness[i].y << " / " << Wheel.BigWebThickness[i].x << "~" << Wheel.BigWebThickness[i].y << "," << to_string(Wheel.EstimateMass[i].x).substr(0, 4) << " / "
				<< to_string(Wheel.EstimateMass[i].y).substr(0, 4) << endl;
		}
	}
};

int main() {
	VBeltDriveDesigner VBDD;

	VBDD.PSET.KA = 1.2;
	VBDD.PSET.NominalPower = 7;
	VBDD.PSET.Type = "B";
	VBDD.PSET.ImagTransmissionRatio = 2.909091;
	VBDD.PSET.SmallWheelRotatingSpeed = 960;
	VBDD.PSET.SmallWheelAxleDameter = 32;

	VBDD.AutoVBelt();

	VBDD.OutputData();

	system("pause");
}