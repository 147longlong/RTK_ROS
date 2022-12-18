#ifndef SKDY_CORRECTIONS_H
#define SKDY_CORRECTIONS_H

#include <iostream>
#include <string.h>
#include <skdy_types.h>
#include <skdy_sdk.h>
#include <icdParser.h>

// initialization function to inialize the client library and datbase etc.
int skdy_CorrDataHndr_Initialization(char* key, char* secret);

// get correction data
void skdy_CorrDataHndr_ProcessPppRtk(double *pos, pppcorr_msg_t* skdy_CorrData);

void skdy_CorrDataHndr_ProcessPppRtk(bool status);

#endif