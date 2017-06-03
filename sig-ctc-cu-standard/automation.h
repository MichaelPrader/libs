#ifndef AUTOMATION_H
#define AUTOMATION_H

#include "ctc-SafetyProcesses.h"
#include "ctc-CTCCmdProcesses.h"
#include "../../libs/block/block-com.h"
#include "../../libs/typedefs/typedef-go.h"
#include "../../libs/typedefs/typedef-gi.h"
#include "../../libs/typedefs/typedef-ir.h"
#include "../../libs/typedefs/typedef-rt.h"
#include "../../libs/typedefs/typedef-sig.h"
#include "../../libs/typedefs/typedef-tc.h"
#include "../../libs/typedefs/typedef-to.h"

extern TSignal ObjSignal[N_SIGNALS];
extern TCircuit ObjCircuit[N_TRACK_CIRCUITS];
extern TTurnout ObjTurnout[N_TURNOUTS];
extern TIR_Sensor ObjIR_Sensor[N_IR_SENSORS];
extern TGenericOutput ObjGenericOutput[N_GENERIC_OUTPUTS];
extern TGenericInput ObjGenericInput[N_GENERIC_INPUTS];
extern uint8_t AutomationLockout[N_SIGNALS];



void Automation(unsigned char init_def);



#endif
