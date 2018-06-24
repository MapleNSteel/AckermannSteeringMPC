/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 5;
/* Vector of auxiliary variables; number of elements: 27. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[1]));
a[1] = (atan(((od[2]*a[0])/(od[1]+od[2]))));
a[2] = (sin(a[1]));
a[3] = (cos(xd[1]));
a[4] = (tan(u[1]));
a[5] = (atan(((od[2]*a[4])/(od[1]+od[2]))));
a[6] = (cos(a[5]));
a[7] = (sin(xd[1]));
a[8] = (tan(u[1]));
a[9] = (atan(((od[2]*a[8])/(od[1]+od[2]))));
a[10] = (sin(a[9]));
a[11] = (tan(u[1]));
a[12] = (atan(((od[2]*a[11])/(od[1]+od[2]))));
a[13] = (cos(a[12]));
a[14] = (cos(xd[1]));
a[15] = (tan(u[1]));
a[16] = (atan(((od[2]*a[15])/(od[1]+od[2]))));
a[17] = (sin(a[16]));
a[18] = (sin(xd[1]));
a[19] = (tan(u[1]));
a[20] = (atan(((od[2]*a[19])/(od[1]+od[2]))));
a[21] = (cos(a[20]));
a[22] = (cos(xd[1]));
a[23] = (tan(u[1]));
a[24] = (atan(((od[2]*a[23])/(od[1]+od[2]))));
a[25] = (sin(a[24]));
a[26] = (sin(xd[1]));

/* Compute outputs: */
out[0] = (((u[0]*a[2])*a[3])+((u[0]*a[6])*a[7]));
out[1] = (((u[0]*a[10])/od[2])-(((od[0]*(((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18])))/(od[0]-xd[0]))/od[0]));
out[2] = ((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))/(od[0]-xd[0]));
}

void acado_rhs_ext(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 18;
const real_t* od = in + 20;
/* Vector of auxiliary variables; number of elements: 111. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(u[1]));
a[1] = (atan(((od[2]*a[0])/(od[1]+od[2]))));
a[2] = (sin(a[1]));
a[3] = (cos(xd[1]));
a[4] = (tan(u[1]));
a[5] = (atan(((od[2]*a[4])/(od[1]+od[2]))));
a[6] = (cos(a[5]));
a[7] = (sin(xd[1]));
a[8] = (tan(u[1]));
a[9] = (atan(((od[2]*a[8])/(od[1]+od[2]))));
a[10] = (sin(a[9]));
a[11] = (tan(u[1]));
a[12] = (atan(((od[2]*a[11])/(od[1]+od[2]))));
a[13] = (cos(a[12]));
a[14] = (cos(xd[1]));
a[15] = (tan(u[1]));
a[16] = (atan(((od[2]*a[15])/(od[1]+od[2]))));
a[17] = (sin(a[16]));
a[18] = (sin(xd[1]));
a[19] = (tan(u[1]));
a[20] = (atan(((od[2]*a[19])/(od[1]+od[2]))));
a[21] = (cos(a[20]));
a[22] = (cos(xd[1]));
a[23] = (tan(u[1]));
a[24] = (atan(((od[2]*a[23])/(od[1]+od[2]))));
a[25] = (sin(a[24]));
a[26] = (sin(xd[1]));
a[27] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[28] = (xd[6]*a[27]);
a[29] = (cos(xd[1]));
a[30] = (xd[6]*a[29]);
a[31] = (xd[7]*a[27]);
a[32] = (xd[7]*a[29]);
a[33] = (xd[8]*a[27]);
a[34] = (xd[8]*a[29]);
a[35] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[36] = (xd[6]*a[35]);
a[37] = (cos(xd[1]));
a[38] = (xd[6]*a[37]);
a[39] = ((real_t)(1.0000000000000000e+00)/(od[0]-xd[0]));
a[40] = (a[39]*a[39]);
a[41] = ((real_t)(1.0000000000000000e+00)/od[0]);
a[42] = (xd[7]*a[35]);
a[43] = (xd[7]*a[37]);
a[44] = (xd[8]*a[35]);
a[45] = (xd[8]*a[37]);
a[46] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[47] = (xd[6]*a[46]);
a[48] = (cos(xd[1]));
a[49] = (xd[6]*a[48]);
a[50] = ((real_t)(1.0000000000000000e+00)/(od[0]-xd[0]));
a[51] = (a[50]*a[50]);
a[52] = (xd[7]*a[46]);
a[53] = (xd[7]*a[48]);
a[54] = (xd[8]*a[46]);
a[55] = (xd[8]*a[48]);
a[56] = (xd[14]*a[27]);
a[57] = (xd[14]*a[29]);
a[58] = (xd[15]*a[27]);
a[59] = (xd[15]*a[29]);
a[60] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[61] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[2]));
a[62] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[2]*a[0])/(od[1]+od[2])),2))));
a[63] = (((od[2]*a[60])*a[61])*a[62]);
a[64] = (cos(a[1]));
a[65] = (a[63]*a[64]);
a[66] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[67] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[2]));
a[68] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[2]*a[4])/(od[1]+od[2])),2))));
a[69] = (((od[2]*a[66])*a[67])*a[68]);
a[70] = ((real_t)(-1.0000000000000000e+00)*(sin(a[5])));
a[71] = (a[69]*a[70]);
a[72] = (xd[14]*a[35]);
a[73] = (xd[14]*a[37]);
a[74] = ((real_t)(1.0000000000000000e+00)/od[2]);
a[75] = (xd[15]*a[35]);
a[76] = (xd[15]*a[37]);
a[77] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[78] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[2]));
a[79] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[2]*a[8])/(od[1]+od[2])),2))));
a[80] = (((od[2]*a[77])*a[78])*a[79]);
a[81] = (cos(a[9]));
a[82] = (a[80]*a[81]);
a[83] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[84] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[2]));
a[85] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[2]*a[11])/(od[1]+od[2])),2))));
a[86] = (((od[2]*a[83])*a[84])*a[85]);
a[87] = ((real_t)(-1.0000000000000000e+00)*(sin(a[12])));
a[88] = (a[86]*a[87]);
a[89] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[90] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[2]));
a[91] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[2]*a[15])/(od[1]+od[2])),2))));
a[92] = (((od[2]*a[89])*a[90])*a[91]);
a[93] = (cos(a[16]));
a[94] = (a[92]*a[93]);
a[95] = (xd[14]*a[46]);
a[96] = (xd[14]*a[48]);
a[97] = (xd[15]*a[46]);
a[98] = (xd[15]*a[48]);
a[99] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[100] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[2]));
a[101] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[2]*a[19])/(od[1]+od[2])),2))));
a[102] = (((od[2]*a[99])*a[100])*a[101]);
a[103] = ((real_t)(-1.0000000000000000e+00)*(sin(a[20])));
a[104] = (a[102]*a[103]);
a[105] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[1])),2)));
a[106] = ((real_t)(1.0000000000000000e+00)/(od[1]+od[2]));
a[107] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((od[2]*a[23])/(od[1]+od[2])),2))));
a[108] = (((od[2]*a[105])*a[106])*a[107]);
a[109] = (cos(a[24]));
a[110] = (a[108]*a[109]);

/* Compute outputs: */
out[0] = (((u[0]*a[2])*a[3])+((u[0]*a[6])*a[7]));
out[1] = (((u[0]*a[10])/od[2])-(((od[0]*(((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18])))/(od[0]-xd[0]))/od[0]));
out[2] = ((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))/(od[0]-xd[0]));
out[3] = (((u[0]*a[2])*a[28])+((u[0]*a[6])*a[30]));
out[4] = (((u[0]*a[2])*a[31])+((u[0]*a[6])*a[32]));
out[5] = (((u[0]*a[2])*a[33])+((u[0]*a[6])*a[34]));
out[6] = ((real_t)(0.0000000000000000e+00)-((((od[0]*(((u[0]*a[13])*a[36])-((u[0]*a[17])*a[38])))*a[39])-(((od[0]*(((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18])))*((real_t)(0.0000000000000000e+00)-xd[3]))*a[40]))*a[41]));
out[7] = ((real_t)(0.0000000000000000e+00)-((((od[0]*(((u[0]*a[13])*a[42])-((u[0]*a[17])*a[43])))*a[39])-(((od[0]*(((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18])))*((real_t)(0.0000000000000000e+00)-xd[4]))*a[40]))*a[41]));
out[8] = ((real_t)(0.0000000000000000e+00)-((((od[0]*(((u[0]*a[13])*a[44])-((u[0]*a[17])*a[45])))*a[39])-(((od[0]*(((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18])))*((real_t)(0.0000000000000000e+00)-xd[5]))*a[40]))*a[41]));
out[9] = (((od[0]*(((u[0]*a[21])*a[47])-((u[0]*a[25])*a[49])))*a[50])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[3]))*a[51]));
out[10] = (((od[0]*(((u[0]*a[21])*a[52])-((u[0]*a[25])*a[53])))*a[50])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[4]))*a[51]));
out[11] = (((od[0]*(((u[0]*a[21])*a[54])-((u[0]*a[25])*a[55])))*a[50])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[5]))*a[51]));
out[12] = ((((u[0]*a[2])*a[56])+((u[0]*a[6])*a[57]))+((a[2]*a[3])+(a[6]*a[7])));
out[13] = ((((u[0]*a[2])*a[58])+((u[0]*a[6])*a[59]))+(((u[0]*a[65])*a[3])+((u[0]*a[71])*a[7])));
out[14] = (((real_t)(0.0000000000000000e+00)-((((od[0]*(((u[0]*a[13])*a[72])-((u[0]*a[17])*a[73])))*a[39])-(((od[0]*(((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18])))*((real_t)(0.0000000000000000e+00)-xd[12]))*a[40]))*a[41]))+((a[10]*a[74])-(((od[0]*((a[13]*a[14])-(a[17]*a[18])))*a[39])*a[41])));
out[15] = (((real_t)(0.0000000000000000e+00)-((((od[0]*(((u[0]*a[13])*a[75])-((u[0]*a[17])*a[76])))*a[39])-(((od[0]*(((u[0]*a[13])*a[14])-((u[0]*a[17])*a[18])))*((real_t)(0.0000000000000000e+00)-xd[13]))*a[40]))*a[41]))+(((u[0]*a[82])*a[74])-(((od[0]*(((u[0]*a[88])*a[14])-((u[0]*a[94])*a[18])))*a[39])*a[41])));
out[16] = ((((od[0]*(((u[0]*a[21])*a[95])-((u[0]*a[25])*a[96])))*a[50])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[12]))*a[51]))+((od[0]*((a[21]*a[22])-(a[25]*a[26])))*a[50]));
out[17] = ((((od[0]*(((u[0]*a[21])*a[97])-((u[0]*a[25])*a[98])))*a[50])-(((od[0]*(((u[0]*a[21])*a[22])-((u[0]*a[25])*a[26])))*((real_t)(0.0000000000000000e+00)-xd[13]))*a[51]))+((od[0]*(((u[0]*a[104])*a[22])-((u[0]*a[110])*a[26])))*a[50]));
}

/* Fixed step size:0.0333333 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
rk_eta[3] = 1.0000000000000000e+00;
rk_eta[4] = 0.0000000000000000e+00;
rk_eta[5] = 0.0000000000000000e+00;
rk_eta[6] = 0.0000000000000000e+00;
rk_eta[7] = 1.0000000000000000e+00;
rk_eta[8] = 0.0000000000000000e+00;
rk_eta[9] = 0.0000000000000000e+00;
rk_eta[10] = 0.0000000000000000e+00;
rk_eta[11] = 1.0000000000000000e+00;
rk_eta[12] = 0.0000000000000000e+00;
rk_eta[13] = 0.0000000000000000e+00;
rk_eta[14] = 0.0000000000000000e+00;
rk_eta[15] = 0.0000000000000000e+00;
rk_eta[16] = 0.0000000000000000e+00;
rk_eta[17] = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[18] = rk_eta[18];
acadoWorkspace.rk_xxx[19] = rk_eta[19];
acadoWorkspace.rk_xxx[20] = rk_eta[20];
acadoWorkspace.rk_xxx[21] = rk_eta[21];
acadoWorkspace.rk_xxx[22] = rk_eta[22];

for (run1 = 0; run1 < 1; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + rk_eta[17];
acado_rhs_ext( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[6] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[7] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[8] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[9] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[10] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[11] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[12] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[13] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[14] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[15] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[16] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[17] + rk_eta[17];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 18 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[18] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[19] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[20] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[21] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[22] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[23] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[24] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[25] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[26] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[27] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[28] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[29] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[30] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[31] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[32] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[33] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[34] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)1.6666666666666666e-02*acadoWorkspace.rk_kkk[35] + rk_eta[17];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 36 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[36] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[37] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[38] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[39] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[40] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[41] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[42] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[43] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[44] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[45] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[46] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[47] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[48] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[49] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[50] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[51] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[52] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)3.3333333333333333e-02*acadoWorkspace.rk_kkk[53] + rk_eta[17];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 54 ]) );
rk_eta[0] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[0] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[18] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[36] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[54];
rk_eta[1] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[1] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[19] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[37] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[55];
rk_eta[2] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[2] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[20] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[38] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[56];
rk_eta[3] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[3] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[21] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[39] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[57];
rk_eta[4] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[4] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[22] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[40] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[58];
rk_eta[5] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[5] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[23] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[41] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[59];
rk_eta[6] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[6] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[24] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[42] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[60];
rk_eta[7] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[7] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[25] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[43] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[61];
rk_eta[8] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[8] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[26] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[44] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[62];
rk_eta[9] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[9] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[27] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[45] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[63];
rk_eta[10] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[10] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[28] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[46] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[64];
rk_eta[11] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[11] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[29] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[47] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[65];
rk_eta[12] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[12] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[30] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[48] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[66];
rk_eta[13] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[13] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[31] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[49] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[67];
rk_eta[14] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[14] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[32] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[50] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[68];
rk_eta[15] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[15] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[33] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[51] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[69];
rk_eta[16] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[16] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[34] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[52] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[70];
rk_eta[17] += + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[17] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[35] + (real_t)1.1111111111111110e-02*acadoWorkspace.rk_kkk[53] + (real_t)5.5555555555555549e-03*acadoWorkspace.rk_kkk[71];
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
error = 0;
return error;
}

