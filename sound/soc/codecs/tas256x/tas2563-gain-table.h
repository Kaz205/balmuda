/*
 This is the program used to generate below table.

#include <stdio.h>
#include <math.h>
int main()
{
	double gain;
	double dec_ratio;
	int setval;
	printf("/" "* This file is only included exactly once!\n");
	printf("*" "/\n");
	printf("static int tas_gaintable[] = {\n");
	for (gain=-60,setval=0;gain<=0;gain++,setval++) {
		dec_ratio = round(pow(10,(gain/20)) * pow(2,30));
		printf("	0x%.8x, /" "* %02d : %-2.1f dB *" "/\n", (unsigned int)dec_ratio, setval, gain);
	}
	printf("};\n\n");
}
*/
/* This file is only included exactly once!
 */
static int tas_gaintable[] = {
	0x0010624e, /* 00 : -60.0 dB */
	0x00126216, /* 01 : -59.0 dB */
	0x0014a051, /* 02 : -58.0 dB */
	0x0017249d, /* 03 : -57.0 dB */
	0x0019f786, /* 04 : -56.0 dB */
	0x001d22a5, /* 05 : -55.0 dB */
	0x0020b0bd, /* 06 : -54.0 dB */
	0x0024ade1, /* 07 : -53.0 dB */
	0x0029279e, /* 08 : -52.0 dB */
	0x002e2d28, /* 09 : -51.0 dB */
	0x0033cf8e, /* 10 : -50.0 dB */
	0x003a21f4, /* 11 : -49.0 dB */
	0x004139d3, /* 12 : -48.0 dB */
	0x00492f45, /* 13 : -47.0 dB */
	0x00521d51, /* 14 : -46.0 dB */
	0x005c224e, /* 15 : -45.0 dB */
	0x00676045, /* 16 : -44.0 dB */
	0x0073fd66, /* 17 : -43.0 dB */
	0x0082248a, /* 18 : -42.0 dB */
	0x009205c6, /* 19 : -41.0 dB */
	0x00a3d70a, /* 20 : -40.0 dB */
	0x00b7d4dd, /* 21 : -39.0 dB */
	0x00ce4329, /* 22 : -38.0 dB */
	0x00e76e1e, /* 23 : -37.0 dB */
	0x0103ab3d, /* 24 : -36.0 dB */
	0x01235a72, /* 25 : -35.0 dB */
	0x0146e75e, /* 26 : -34.0 dB */
	0x016ecac5, /* 27 : -33.0 dB */
	0x019b8c27, /* 28 : -32.0 dB */
	0x01cdc38c, /* 29 : -31.0 dB */
	0x02061b8a, /* 30 : -30.0 dB */
	0x02455386, /* 31 : -29.0 dB */
	0x028c4240, /* 32 : -28.0 dB */
	0x02dbd8ad, /* 33 : -27.0 dB */
	0x03352529, /* 34 : -26.0 dB */
	0x0399570c, /* 35 : -25.0 dB */
	0x0409c2b1, /* 36 : -24.0 dB */
	0x0487e5fc, /* 37 : -23.0 dB */
	0x05156d69, /* 38 : -22.0 dB */
	0x05b439bd, /* 39 : -21.0 dB */
	0x06666666, /* 40 : -20.0 dB */
	0x072e50a6, /* 41 : -19.0 dB */
	0x080e9f97, /* 42 : -18.0 dB */
	0x090a4d30, /* 43 : -17.0 dB */
	0x0a24b063, /* 44 : -16.0 dB */
	0x0b618872, /* 45 : -15.0 dB */
	0x0cc509ac, /* 46 : -14.0 dB */
	0x0e53ebb4, /* 47 : -13.0 dB */
	0x10137988, /* 48 : -12.0 dB */
	0x1209a37b, /* 49 : -11.0 dB */
	0x143d1362, /* 50 : -10.0 dB */
	0x16b54338, /* 51 : -9.0 dB */
	0x197a967f, /* 52 : -8.0 dB */
	0x1c9676c7, /* 53 : -7.0 dB */
	0x2013739e, /* 54 : -6.0 dB */
	0x23fd6678, /* 55 : -5.0 dB */
	0x28619aea, /* 56 : -4.0 dB */
	0x2d4efbd6, /* 57 : -3.0 dB */
	0x32d64618, /* 58 : -2.0 dB */
	0x390a4160, /* 59 : -1.0 dB */
	0x40000000, /* 60 : 0.0 dB */
};
