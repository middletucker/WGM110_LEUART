/***************************************************************************//**
 * @file
 * @brief uartdrv examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
// Define module name for Power Manager debuging feature.
#define CURRENT_MODULE_NAME    "APP_COMMON_EXAMPLE_UARTDRV"

#include <stdio.h>
#include <string.h>
#include "uartdrv_app.h"
#include "sl_uartdrv_instances.h"
#include "sl_power_manager.h"
#include "em_core.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
#define getTicks sl_sleeptimer_get_tick_count64
#define vcom sl_uartdrv_leuart_vcom_handle
typedef enum {SEND,RECV,HEADER,REMAINDER,COMPLETE} MessageState;



/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
uint32_t timeOut;
uint32_t evtWait;

// Hook for power manager. The application will not prevent the
// power manager from entering sleep.
bool app_is_ok_to_sleep(void)
{
  return true;
}

// Hook for power manager. The application will not prevent the
// power manager from re-entering sleep after an interrupt is serviced.
sl_power_manager_on_isr_exit_t app_sleep_on_isr_exit(void)
{
  return SL_POWER_MANAGER_SLEEP;
}

void initGpio(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortB, 6, gpioModePushPull, 0); // Reset Not

  GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);
  GPIO_DriveStrengthSet(gpioPortF,gpioDriveStrengthStrongAlternateStrong);
}

void bounceWGM110(){
  GPIO_PinOutClear(gpioPortB,6);
  sl_sleeptimer_delay_millisecond(10);
  GPIO_PinOutSet(gpioPortB,6);
}

/***************************************************************************//**
 * Initialize example.
 ******************************************************************************/
void uartdrv_app_init(void)
{
  // Require at least EM2 from Power Manager
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);

  initGpio();
  bounceWGM110();
  timeOut=sl_sleeptimer_ms_to_tick(250);
  evtWait=10; // ms
}

uint8_t incoming[64];
uint8_t outgoing[64];
uint32_t inN,outN;

int getN(unsigned char octet0,unsigned char octet1) {
  int n=0;
  if (octet0==0x08 || octet0==0x88) {  //reasonably valid first byte
    n=octet1;
    if (n>60) n=60;
  } else n=0;
  return n;
}


void setMessage4(uint32_t m){
    outgoing[0]=(m&0xff000000)>>24;
    outgoing[1]=(m&0xff0000)>>16;
    outgoing[2]=(m&0xff00)>>8;
    outgoing[3]=(m&0xff);
    outN=getN(outgoing[0],outgoing[1]);
}

MessageState txState;
void afterTransmit(struct UARTDRV_HandleData *handle,
                           Ecode_t transferStatus,
                           uint8_t *data,
                           UARTDRV_Count_t transferCount){
  if (txState==SEND && transferStatus==ECODE_OK && outN+4==transferCount)
      txState=COMPLETE;  // goto next stage looking for response
}

bool send() {
  txState=SEND;
  uint64_t start=getTicks();
  UARTDRV_Transmit(sl_uartdrv_leuart_vcom_handle,outgoing,outN+4,afterTransmit);
  while (txState!=COMPLETE && (getTicks())-start<timeOut);
  return (txState!=COMPLETE);
}

MessageState rxState; // The state of a message to the WGM110
bool compare(uint32_t m){
 //   if (rxState!=COMPLETE) return false;
    return
      incoming[0]==((m&0xff000000)>>24) &&
      incoming[1]==((m&0xff0000)>>16) &&
      incoming[2]==((m&0xff00)>>8) &&
      incoming[3]==(m&0xff);
}

uint8_t big[64];
uint8_t pos=0;

uint8_t next;
uint8_t errors=0;
void getOneByte(struct UARTDRV_HandleData *handle,
                           Ecode_t transferStatus,
                           uint8_t *data,
                           UARTDRV_Count_t transferCount){
    big[pos]=data[0];
    pos++;
    if (pos==64) pos=0;

    incoming[next]=data[0];
    if (next==0 && data[0]==0x07) { // Patch missing 0x88 for MAC response
        incoming[0]=0x88;
        incoming[1]=0x07;
        next=1;
    }
    if (next==1)    // second byte  get length of packet
        inN=getN(incoming[0],incoming[1]);
    if (next==0) {  // first byte
      if (data[0]==0x088 || data[0]==0x08) // is 0x08 or 0x88
        next++;
    } else
      next++;
    if (next>0 && next<=4) rxState=HEADER;
    else if (next<(inN+4)) rxState=REMAINDER;
    else rxState=COMPLETE;
    if (next==64) next=0;
    if (rxState!=COMPLETE) UARTDRV_Receive(handle,data,1,getOneByte);
}

bool receive() {
  uint8_t nextC;
  for (int i=0;i<64;i++)
    incoming[i]=0;
  next=0;
  inN=0;
  rxState=RECV;
  UARTDRV_Receive(vcom,&nextC,1,getOneByte);
  uint64_t start=getTicks();
  while ((getTicks()-start)<timeOut && rxState!=COMPLETE)
    sl_sleeptimer_delay_millisecond(10);
  return (rxState==COMPLETE);
}

uint32_t helloMsg=0x08000102;
uint32_t helloMsgResp=0x08000102;
void sayHello() {
  setMessage4(helloMsg);  // cmd_system_hello
  send();
}

uint32_t getMacMsg=0x08010200;
uint32_t getMacResp=0x08030200;
uint32_t getMacEvt=0x88070200;
uint8_t validMAC=0;
uint8_t MAC[6]={0,0,0,0,0,0};
void requestMAC() {
  validMAC=0;
  setMessage4(getMacMsg);
  outgoing[4]=0; // Wi-Fi
  send();
}
bool checkMACEvt() {
  if (compare(getMacEvt)){ // Check for actual MAC message
    validMAC=1;
    for (int i=0;i<6;i++)
      MAC[i]=incoming[5+i];
    return true;
  }
  return false;
}

uint32_t wifiOnMsg=0x08000300;
uint32_t wifiOnResp=0x08020300;
void wifiOn() {
  setMessage4(wifiOnMsg);
  send();
}
bool checkWifiOnAck() {
  if(compare(wifiOnResp))
    return ((incoming[4]<<8)|incoming[5])==0;
  return false;
}

uint32_t wifiOffMsg=0x08000301;
uint32_t wifiOffResp=0x08020301;
uint32_t wifiOffEvt=0x88020301;
void wifiOff() {
  setMessage4(wifiOffMsg);
  send();
}

typedef enum {START,
  HELLO,HELLOACK,
  REQMAC,REQMACACK,REQMACEVT,
  WIFION,WIFIONACK,
  WIFIOFF,WIFIOFFACK,WIFIOFFEVT,
  SSID,PWD,FINISH} ConState;

#define MAXRETRY 4
int retry=MAXRETRY;
ConState conState=START;

void resetRetry() {
  retry=MAXRETRY;
}
void testRetry(ConState cs) {
  retry--;
  if (retry==0) conState=cs;
}

/***************************************************************************//**
 * Ticking function.
 ******************************************************************************/
void uartdrv_app_process_action(void) {
  receive();
  switch(conState) {
      case START:
        GPIO_PinOutClear(gpioPortF,5);
        GPIO_PinOutClear(gpioPortF,4);
        bounceWGM110();
        sl_sleeptimer_delay_millisecond(500);
        GPIO_PinOutClear(gpioPortF,5);
        GPIO_PinOutSet(gpioPortF,4);
        sl_sleeptimer_delay_millisecond(500);
        conState=HELLO;
        break;
      case HELLO:
        sayHello();
        conState=HELLOACK;
        resetRetry();
        break;
      case HELLOACK:
        if (compare(helloMsgResp)) {
          conState=REQMAC;
          rxState=RECV; // Message matched and consumed
        } else testRetry(START);
        break;
      case REQMAC:
        requestMAC();
        conState=REQMACACK;
        resetRetry();
        break;
      case REQMACACK:
        if (compare(getMacResp)) conState=REQMACEVT;
        else testRetry(START);
        break;
      case REQMACEVT:
        if (checkMACEvt()) conState=WIFION;
        else testRetry(REQMAC);
        break;
      case WIFION:
        wifiOn();
        conState=WIFIONACK;
        resetRetry();
        break;
      case WIFIONACK:
        if (checkWifiOnAck()) conState=WIFIOFF;
        else testRetry(START);
        break;
      case WIFIOFF:
        wifiOff();
        conState=WIFIOFFACK;
        resetRetry();
        break;
      case WIFIOFFACK:
        if (compare(wifiOffResp)) conState=WIFIOFFEVT;
        else testRetry(START);
        break;
      case WIFIOFFEVT:
        if (compare(wifiOffEvt)) conState=FINISH;
        else testRetry(WIFIOFF);
        break;
      case FINISH:
        GPIO_PinOutSet(gpioPortF,5);
        GPIO_PinOutSet(gpioPortF,4);
        sl_sleeptimer_delay_millisecond(1000);
        conState=START;
        break;
      default:
        break;
    }
  }
