

#ifndef CTC_CMD_PROCESSES_H
#define CTC_CMD_PROCESSES_H


void SetTurnout(uint8_t turnout, uint8_t command);
void DropFleet(uint8_t signal);
void SetFleet(uint8_t signal);
void DropSignal(uint8_t signal);
void ClearSignal(uint8_t signal);

#endif