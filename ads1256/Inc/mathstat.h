#include "inttypes.h"

typedef struct{
int N;
float k[5];
} StudentsKoeff;

typedef enum {
	P0_80  = 0,
	P0_90  = 1,
	P0_95  = 2,
	P0_99  = 3,
	P0_999 = 4
} PStud;

typedef enum {
	STABLE,
	UNSTABLE,
} PStability;

typedef enum {
	LOW,
	MED,
	HIGH
} PQuality;

typedef struct{
float value;
float delta;
PStud p;
PStability stabil;
PQuality qual;
} StatResult;

uint16_t MaskCount(int ch);
float Mean(int ch);
float MeanStandardDeviation(int ch,float mean);
float GetStudentsKoefficient(uint16_t N, PStud P);
float StudentDelta(int ch, float Sr, uint8_t P);
uint16_t StudentFilter(int ch, float mean, float delta);
uint8_t FillData(int ch, float val_n);
StatResult Calculate(int ch, float high_Q, float low_Q);
void SetLength(uint16_t Len);
uint16_t GetLength();
