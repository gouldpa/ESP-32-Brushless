//#define MINI 1

#include <SPI.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/spi_master.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_reg.h"
#include "soc/dport_reg.h"
#include "soc/spi_struct.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "esp_adc_cal.h"
#include <CAN.h>

#include <stdint.h>
#include <string.h>

#define LED_PIN                             (2)

int32_t other_motor_pos;

//#include "wiring_private.h" // pinPeripheral() function

int current_config = 0;
#include "EEPROM.h"

int addr = 0;
#define EEPROM_SIZE 16
int count;
byte eeprom_data[EEPROM_SIZE];

#define EEPROM_SIZE 16
#define CONFIG_ARRAY_SIZE 6
int16_t config_array[CONFIG_ARRAY_SIZE];

#define ID_OS           0
#define MAG_OFFSET_OS   1
#define PHASE_OFFSET_OS 2
#define TEMP_MAX_OS     3
#define KP_OS           4
#define REV_OS           5
//GPIO.out_w1tc and GPIO.out_w1ts

static const int spiClk = 2000000; // 1 MHz
SPIClass * hspi = NULL;

 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
     int hi_byte=0;
    int lo_byte=0;
    int encoder_int =0;
    int encoder_ext = 0;
    //int drv_spi_value = 0;

///////////////////////////////////////////////////////////////////////////////////////
int man_offset=142;
#define SIN_ARRAY_SIZE_BITS 12
#define SIN_ARRAY_SIZE (1<<SIN_ARRAY_SIZE_BITS)
#define MOTOR_POLE_PAIRS    7
#define FULL_ANGLE 360.0
#define DEG_RAD (3.1415/180.0)
#define FULL_PWM_BITS 10
#define FULL_PWM (1<<FULL_PWM_BITS)

//#define MAGNET_OFFSET -140 //0
int PHASE_OFFSET = -156;
#define MAX_PWM 400
int step_print=0;
    int drive;    

    
float sin_scale;
int phase_120;
int16_t SinArray[SIN_ARRAY_SIZE];
int Temp_Max = 90;
int32_t PWMUU;
int32_t PWMVV;
int32_t PWMWW;

int Maximum_PWM=0;
int32_t Motor_Position=0;
int32_t Motor_Position_Old=0;
int32_t Motor_Abs=0; 
int Motor_Count_Start=0;
int32_t Joint_Position;
int32_t Tar_Position = 0;
int32_t Tar_Position_Speed = 0;
int32_t Tar_Speed = 0;
int32_t Tar_Move =0;
int32_t Motor_Pos_Change=0;
int32_t off_set=0;
int Kp = 1;
int Target_Phase;
int8_t Servo_ID=1;
unsigned int magnetoffset = 0;
unsigned int phase_direction = 0;
int32_t Motor_Joint;
unsigned int Mag_Sensor_Data;
int Motor_on = 0;
int incomplete = 1;
//int time = 0;
int forward=0;
int Motor_Error;
// common
uint32_t counter=0;
int start=0;
int tpwm=0;
char cat;
int fault=0;
int temp = 22;
int d_counter=0;
int s1, s2, s3, f_temp, v_mon;
float current;
  int motor_output;

unsigned int drv_spi_value[5];

int s1_nom;
int s2_nom;
int s3_nom;

int tt;
int tm;
int cur_cur;

#if MINI
////////////////////////////////////////
// MCPWM Pins
#define DRV_HA 5   //Set GPIO 15 as PWM0A
#define DRV_HB 18   //Set GPIO 00 as PWM1A
#define DRV_HC 19   //Set GPIO 16 as PWM2A

// DRV Pins
#define DRV_nFAULT 25 // Input Fault low
#define DRV_nSCS   14
#define DRV_EN_GATE 12 // Output Enable high

// SPI Pins Default SPI2 HSPI
#define SPI_SCLK 15
#define SPI_MISO 4
#define SPI_MOSI 2

// INT ENCODER Pin
#define INT_ENC_nCS 13

// EXT ENCODER Pin
//#define EXT_ENC_nCS 26

// ADC
#define DRV_SO3 35
#define DRV_SO1 34
#define DRV_SO2 32
#define MOTOR_TEMP 37
#define FET_TEMP 36
#define VOLT_MON 34

// CAN Pins (Default ???) can also be spare UART
#define CAN_RX 16
#define CAN_TX 17

// I2C Pins (Default)
#define I2C_DATA 21
#define I2C_CLK 22

// UART Pins Default
#define TXD0 1
#define RXD0 3

// BOOT / AUX Button
#define BOOT_BUTTON 0

// Test Pins
#define TEST_IO1 23
#define TEST_IO2 26
#define TEST_IO4 27

#else
////////////////////////////////////////
// MCPWM Pins
#define DRV_HA 5   //Set GPIO 15 as PWM0A
#define DRV_HB 18   //Set GPIO 00 as PWM1A
#define DRV_HC 19   //Set GPIO 16 as PWM2A

// DRV Pins
#define DRV_nFAULT 27 // Input Fault low
#define DRV_nSCS   14
#define DRV_EN_GATE 12 // Output Enable high

// SPI Pins Default SPI2 HSPI
#define SPI_SCLK 13
#define SPI_MISO 4
#define SPI_MOSI 2

// INT ENCODER Pin
#define INT_ENC_nCS 15

// EXT ENCODER Pin
#define EXT_ENC_nCS 26

// ADC
#define DRV_SO3 35
#define DRV_SO1 34
#define DRV_SO2 32
#define MOTOR_TEMP 37
#define FET_TEMP 33
#define VOLT_MON 36

// CAN Pins (Default ???) can also be spare UART
#define CAN_RX 16
#define CAN_TX 17

// I2C Pins (Default)
#define I2C_DATA 21
#define I2C_CLK 22

// UART Pins Default
#define TXD0 1
#define RXD0 3

// BOOT / AUX Button
#define BOOT_BUTTON 0

// Test Pins
#define TEST_IO2 23
#define TEST_IO4 25


#endif


// SPI ////////////////////////////////////////////////////////////////////
//https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-spi.c
void IRAM_ATTR SPIInit(void)
{
  hspi = new SPIClass(HSPI);
  hspi->begin(SPI_SCLK,SPI_MISO, SPI_MOSI,0); 
  pinMode(INT_ENC_nCS, OUTPUT);
  #if MINI
  #else
  pinMode(EXT_ENC_nCS, OUTPUT); 
  GPIO.out_w1ts = ((uint32_t)1 << EXT_ENC_nCS);
  #endif
  pinMode(DRV_nSCS, OUTPUT);
  GPIO.out_w1ts = ((uint32_t)1 << INT_ENC_nCS);

  GPIO.out_w1ts = ((uint32_t)1 << DRV_nSCS);
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); //MA702

  SPI2.mosi_dlen.usr_mosi_dbitlen = (1 * 8) - 1;
  SPI2.miso_dlen.usr_miso_dbitlen = (1 * 8) - 1;
}

int  even=0;
void IRAM_ATTR SPI_Read_Encoder(){

  GPIO.out_w1tc = ((uint32_t)1 << INT_ENC_nCS);
  encoder_int = SPI2.data_buf[0];
  SPI2.data_buf[0] = 0;
  SPI2.cmd.usr = 1;
  while(SPI2.cmd.usr);
  encoder_int = (SPI2.data_buf[0] & 0x00FF)*256 ;
  
  SPI2.data_buf[0] = 0;
  SPI2.cmd.usr = 1;
  while(SPI2.cmd.usr);
  
  GPIO.out_w1ts = ((uint32_t)1 << INT_ENC_nCS);
  encoder_int = (SPI2.data_buf[0]& 0x00FF) + encoder_int;  
  encoder_int = encoder_int>>4;
}

int insta_speed;
int phase_forward;

void IRAM_ATTR Motor_CTRL(void)
{
    Motor_Position_Old = Motor_Position;
    Mag_Sensor_Data  = encoder_int;
    Motor_Position = (Mag_Sensor_Data); //& 0x2FFF;
    
   if (start<2)
   {  
    start++; 
    Motor_Abs = 0;
   }
    Motor_Pos_Change = -(Motor_Position-Motor_Position_Old);
    insta_speed = abs(Motor_Pos_Change);
    
    if (Motor_Pos_Change>2048)
        off_set =  Motor_Pos_Change-4096;
    else if (Motor_Pos_Change<-2048)
        off_set = Motor_Pos_Change + 4096;
    else 
        off_set = Motor_Pos_Change; 
    
    Motor_Abs += off_set;
    
    //Joint_Position = encoder_int;

   // PID Control - Just P for now
   Motor_Error = (Motor_Abs - Tar_Position);
    Motor_Joint = config_array[KP_OS] * Motor_Error;
    motor_output = Motor_Joint;

  if (abs(motor_output)<2)
    motor_output=0;
    
    if (insta_speed>30)
      insta_speed=30;
    if (config_array[PHASE_OFFSET_OS]>0)
    {
      phase_forward = config_array[PHASE_OFFSET_OS]-insta_speed;
    }
    else
    {
      phase_forward = config_array[PHASE_OFFSET_OS]+insta_speed;
    }
    
    if (motor_output > 0) // Forward or Reverse
    {
        drive = phase_forward; //phase_set;
    }
    else
    {
        drive = -phase_forward; //phase_set;
        motor_output = -motor_output;
    }

    if (motor_output > Maximum_PWM)
        motor_output = Maximum_PWM;
/*
    if (cur_cur>2000)
      motor_output = motor_output/2;
    else if (cur_cur>3000)
      motor_output = 0;
    */  
    // Drive Motor at ~90deg electrically ahead of magnets 
    Target_Phase = Motor_Position + drive + config_array[MAG_OFFSET_OS];//MAGNET_OFFSET; // MAGNET_OFFSET;

    //Handle wrap-around
    if (Target_Phase >= SIN_ARRAY_SIZE)
        Target_Phase = Target_Phase - SIN_ARRAY_SIZE;
    else if (Target_Phase < 0)
        Target_Phase = Target_Phase + SIN_ARRAY_SIZE;
    else
    {} 
}

/*
 * 
 */
void IRAM_ATTR Motor_Vector_Phases(int Mot_Phase, int Motor_PWM)
{
    int U_Ang = Mot_Phase;
    int V_Ang = Mot_Phase + phase_120;
    int W_Ang = Mot_Phase - phase_120;

    if (V_Ang >= SIN_ARRAY_SIZE)
        V_Ang -=SIN_ARRAY_SIZE;
        
    if (W_Ang < 0)
        W_Ang +=SIN_ARRAY_SIZE;
    
    PWMUU = (SinArray[U_Ang] * Motor_PWM) >>FULL_PWM_BITS;
    PWMVV = (SinArray[V_Ang] * Motor_PWM) >>FULL_PWM_BITS;
    PWMWW = (SinArray[W_Ang] * Motor_PWM) >>FULL_PWM_BITS;
}

void IRAM_ATTR Three_Phases(void)
{    
  if (Motor_on==0)
  {
      PWMUU = 0;
      PWMVV = 0;
      PWMWW = 0;
  }
  if (config_array[REV_OS]==1)
  {
  MCPWM0.channel[0].cmpr_value[MCPWM_OPR_A].cmpr_val = PWMUU;
  MCPWM0.channel[1].cmpr_value[MCPWM_OPR_A].cmpr_val = PWMVV;
  MCPWM0.channel[2].cmpr_value[MCPWM_OPR_A].cmpr_val = PWMWW;
  }
  else
  {
  MCPWM0.channel[0].cmpr_value[MCPWM_OPR_A].cmpr_val = PWMVV;
  MCPWM0.channel[1].cmpr_value[MCPWM_OPR_A].cmpr_val = PWMUU;
  MCPWM0.channel[2].cmpr_value[MCPWM_OPR_A].cmpr_val = PWMWW;
  }
}
/*
 * 
 */
void IRAM_ATTR Setup_Sin_array(void)
{
    int step1;
    float tempa;
    float tempb;
    int phase;
    sin_scale = 1.0/((sin(60*DEG_RAD))+(sin(60*DEG_RAD)));
    phase_120 = SIN_ARRAY_SIZE/(MOTOR_POLE_PAIRS*3);
    Serial.print("phase_120 ");
    Serial.println(phase_120);
    for (step1=0; step1<SIN_ARRAY_SIZE; step1++)
    {
        tempa = (float)step1 / ((float)SIN_ARRAY_SIZE/(float)MOTOR_POLE_PAIRS);
        phase = floor(tempa);
        tempb = (tempa-(float)phase) * FULL_ANGLE;
        
        if (tempb<(FULL_ANGLE/3.0))
        {
            SinArray[step1] = (int)(( sin((tempb-30.0)* DEG_RAD) - sin((tempb -( 30.0 + 120.0 ))* DEG_RAD) ) * sin_scale * (float)FULL_PWM);
        }
        else if (tempb<((FULL_ANGLE*2.0)/3.0))
        {    
            SinArray[step1] = (int)(( sin(((240.0-tempb)-30.0)* DEG_RAD) - sin(((240.0-tempb) -( 30.0 + 120.0 ))* DEG_RAD) ) * sin_scale * (float)FULL_PWM);
        }
        else
        {
            SinArray[step1] = 0;
        }      
    }
}



static void IRAM_ATTR setup_mcpwm_pins()
{
    Serial.println("initializing mcpwm control gpio...n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, DRV_HA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, DRV_HB);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, DRV_HC);
} // setup_pins()

#define TIMER_CLK_PRESCALE 1
#define MCPWM_CLK_PRESCALE 0
#define MCPWM_PERIOD_PRESCALE 2
#define MCPWM_PERIOD_PERIOD 1024
static void IRAM_ATTR setup_mcpwm()
{
     setup_mcpwm_pins();

     mcpwm_config_t pwm_config;
     pwm_config.frequency = 4000000;  //frequency = 20000Hz
     pwm_config.cmpr_a = 0;      //duty cycle of PWMxA = 50.0%
     pwm_config.cmpr_b = 0;      //duty cycle of PWMxB = 50.0%
     pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; // Up-down counter (triangle wave)
     pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active HIGH
     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);    //Configure PWM0A & PWM0B with above settings
     Serial.print("clk_cfg.prescale = ");
     Serial.println( MCPWM0.clk_cfg.prescale);
     Serial.print("timer[0].period.prescale = ");
     Serial.println( MCPWM0.timer[0].period.prescale);
     Serial.print("timer[0].period.period = ");
     Serial.println( MCPWM0.timer[0].period.period);
     Serial.print("timer[0].period.upmethod = ");
     Serial.println( MCPWM0.timer[0].period.upmethod);

     mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
     mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
     mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_2);
     MCPWM0.clk_cfg.prescale = MCPWM_CLK_PRESCALE;
  
     MCPWM0.timer[0].period.prescale = MCPWM_PERIOD_PRESCALE;
     MCPWM0.timer[1].period.prescale = MCPWM_PERIOD_PRESCALE;
     MCPWM0.timer[2].period.prescale = MCPWM_PERIOD_PRESCALE;    
     delay(1);
     MCPWM0.timer[0].period.period = MCPWM_PERIOD_PERIOD;
     MCPWM0.timer[1].period.period = MCPWM_PERIOD_PERIOD;
     MCPWM0.timer[2].period.period = MCPWM_PERIOD_PERIOD;
     delay(1);
     MCPWM0.timer[0].period.upmethod =0;
     MCPWM0.timer[1].period.upmethod =0;
     MCPWM0.timer[2].period.upmethod =0;
     delay(1); 
     Serial.print("clk_cfg.prescale = ");
     Serial.println( MCPWM0.clk_cfg.prescale);
     Serial.print("timer[0].period.prescale = ");
     Serial.println( MCPWM0.timer[0].period.prescale);
     Serial.print("timer[0].period.period = ");
     Serial.println( MCPWM0.timer[0].period.period);
     mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
     mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
     mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);
   
     mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_SYNC_INT0, 0); //Have to modify the mcpwm.h file
     mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC_INT0, 0);
     mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_SYNC_INT0, 0);
     delayMicroseconds(1000);
     MCPWM0.timer[0].sync.out_sel = 1;
     delayMicroseconds(1000);
     MCPWM0.timer[0].sync.out_sel = 0;



} // setup_mcpwm

// 5KHz Timer /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR onTimer() {
  //static int lc_counter =0;
  GPIO.out_w1ts = ((uint32_t)1 << TEST_IO2); //TESTPINB
  if (config_array[ID_OS] !=100)
  {
    if (fault)
    {
      SPI_Read_Encoder();
    }
    Motor_CTRL();
    Motor_Vector_Phases(Target_Phase, motor_output);
    Three_Phases();
  }
  GPIO.out_w1tc = ((uint32_t)1 << TEST_IO2); //TESTPINB
}

void IRAM_ATTR TimerInit(void)
{
  timer = timerBegin(0, 8, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2000, true);
  timerAlarmEnable(timer);
}


/**
 * @brief  Initialization for the SSD1306 OLED display via I2C
 * @retval void
 */
 /////////////////////////////////////////
// EEPROM
void IRAM_ATTR EepromInit()
{
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM"); delay(1000);
  }
}

void print_config(void)
{
    Serial.println("e = return & save ");
    Serial.print ("i = id");
    Serial.println(config_array[ID_OS]);
    Serial.print("m = magnet offset ");
    Serial.println(config_array[MAG_OFFSET_OS]);
    Serial.print("p = phase offset ");
    Serial.println(config_array[PHASE_OFFSET_OS]);
    Serial.print("t = pwm_max ");
    Serial.println(config_array[TEMP_MAX_OS]);   
    Serial.print("s = kp ");
    Serial.println(config_array[KP_OS]); 
    Serial.print("d = REV ");
    Serial.println(config_array[REV_OS]);
     
    if (current_config==0)
        Serial.println("ID\n"); 
    else if (current_config==1)
        Serial.println("MAG_OFFSET\n"); 
    else if (current_config==2)
        Serial.println("PHASE_OFFSET\n");
    else if (current_config==3)
        Serial.println("TEMP_MAX\n");
    else if (current_config==4)
        Serial.println("KP\n");
    else if (current_config==5)
        Serial.println("REV\n");
    else
        Serial.println("??\n");
}
void edit_config(void)
{
    incomplete = 1;
    print_config();

    while(incomplete)
    {
        char c;
        c = Serial.read();
        if      (c=='i')
        {
            current_config = 0;
            print_config();
        }
        else if (c=='m')
        {
            current_config = 1;
            print_config();
        }
        else if (c=='p')
        {
            current_config = 2;
            print_config();
        }
        else if (c=='t')
        {
            current_config = 3;
            print_config();
        }
        else if (c=='s')
        {
            current_config = 4;
            print_config();
        }
        else if (c=='d')
        {
            current_config = 5;
            print_config();
        }
        else if (c==',')
        {
            config_array[current_config]--;
            print_config();
        }
        else if (c=='.')
        {
            config_array[current_config]++;
            print_config();
        }        
        else if (c=='k')
        {
            config_to_eeprom();
            incomplete=0;
            //return and save
        }
        else if (c=='r')
        {
            eeprom_to_config();
            //incomplete=0;
            //return and save
        }
        else if (c=='z')
        {
          zero_eeprom();
        }
        else if (c=='e')
        {
          incomplete=0;
        }
    }
}

void config_to_eeprom(void)
{   
    int i;
    int16_t config_temp; 
    unsigned long address = 0;
    for (i=0; i< (CONFIG_ARRAY_SIZE); i++)
    {
        config_temp = (int16_t)config_array[i];
        eeprom_data[i*2] =  (config_temp) & 0xFF;
        eeprom_data[i*2 + 1]=  (config_temp >>8) & 0xFF;
    }

    EEPROM.put(address, eeprom_data);
    if (EEPROM.commit())
      Serial.println("save config to eeprom");
    else
      Serial.println("failsave config to eeprom");
}

void zero_eeprom(void)
{   
    int i;
    int16_t config_temp;
    unsigned long address = 0;
    for (i=0; i< (CONFIG_ARRAY_SIZE); i++)
    {
        config_temp = 0;
        config_array[i] = config_temp;
        eeprom_data[i*2] =  (config_temp) & 0xFF;
        eeprom_data[i*2 + 1]=  (config_temp >>8) & 0xFF;
    }

    EEPROM.put(address, eeprom_data);
    if (EEPROM.commit())
      Serial.println("save zero eeprom");
    else
      Serial.println("failsave save zero eeprom");
    
}

void eeprom_to_config(void)
{
    unsigned long address = 0;
    int i;    
    int32_t config_temp=0;
    Serial.println("read eeprom to config");
    EEPROM.get(address, eeprom_data);
    for (i=0; i< (CONFIG_ARRAY_SIZE); i++)
    {
        config_temp = eeprom_data[i*2] | eeprom_data[i*2+1]<<8 ;
        config_array[i] = (int16_t)config_temp ;
        Serial.print(i);
        Serial.print(" ");
        Serial.println(config_array[i]);
    }
    Serial.println("read done");
}

// CAN /////////////////////////////////////////////////////////
int data1;
int data2;
int data3;
void IRAM_ATTR onReceive(int packetSize) {
  // received a packet
  char a;
  int16_t temp_data;
  //Serial.print("R ");

  int pack_id = CAN.packetId();
  //Serial.print(pack_id); 
  //Serial.print(" ");
  if  (pack_id == config_array[ID_OS])
  {
    if (CAN.available()) {
      temp_data = CAN.read()<<8;
      data1 = temp_data | (CAN.read());
      temp_data = CAN.read()<<8;
      data2 = temp_data | (CAN.read());
      temp_data = CAN.read()<<8;
      data3 = temp_data | (CAN.read());
    }
    //Serial.print(data1);
    //Serial.print(" ");
    //Serial.println(data2);
    if (data1==0)
    {
      Motor_on=0;
      GPIO.out_w1tc= ((uint32_t)1 << DRV_EN_GATE);
    }
    else if(data1==1)
    {
      Motor_on=1;
      Tar_Position = (int) data2;
      Maximum_PWM = data3;
      fault=1;
      GPIO.out_w1ts= ((uint32_t)1 << DRV_EN_GATE);
    }
    else if(data1==3)
    {
      Motor_Abs=0;
      Motor_on=0;
      GPIO.out_w1tc= ((uint32_t)1 << DRV_EN_GATE);
    }
      
  }
  while (CAN.available()) {
          a=CAN.read();}
}

void IRAM_ATTR CAN_Init(){
    CAN.setPins(CAN_RX, CAN_TX);
  Serial.println("CAN Sender");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    //while (1);
  }
    CAN.setPins(CAN_RX, CAN_TX);

      CAN.onReceive(onReceive);
}

void IRAM_ATTR setup() {
 delay(2000);
  Serial.begin(115200);

 pinMode(DRV_EN_GATE, OUTPUT);
 pinMode (DRV_nFAULT, INPUT_PULLUP);
  pinMode(TEST_IO2, OUTPUT);
  pinMode(TEST_IO4, OUTPUT);
  EepromInit();
delay(500);
eeprom_to_config();
    pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

pinMode (RXD0, INPUT_PULLUP);

  //
  //https://www.toptal.com/embedded/esp32-audio-sampling
  //analogInit();
   adcAttachPin(DRV_SO1);
   adcAttachPin(DRV_SO2);
   adcAttachPin(DRV_SO3);
   adcAttachPin(MOTOR_TEMP);
   adcAttachPin(FET_TEMP);
   adcAttachPin(VOLT_MON);


  GPIO.out_w1tc= ((uint32_t)1 << DRV_EN_GATE);
  Setup_Sin_array();

  SPIInit();
  setup_mcpwm();
  //EepromInit();
Serial.println("setup_mcpwm");

  int i;
  unsigned int temps;
  
  for (i=0; i<15;i++)
  {
    TMC6200_SPI_Read(i);
    Serial.print(i);
      Serial.print(" ");
    Serial.print(drv_spi_value[0]);
        Serial.print(" ");
    Serial.print(drv_spi_value[1]);
        Serial.print(" ");
            Serial.print(drv_spi_value[2]);
        Serial.print(" ");
            Serial.print(drv_spi_value[3]);
        Serial.print(" ");
    Serial.println(drv_spi_value[4]);
  }
  s1_nom = analogRead(DRV_SO1)+analogRead(DRV_SO1)+analogRead(DRV_SO1);
  s2_nom = analogRead(DRV_SO2)+analogRead(DRV_SO2)+analogRead(DRV_SO2);
  s3_nom = analogRead(DRV_SO3)+analogRead(DRV_SO3)+analogRead(DRV_SO3);
  
  CAN_Init();
  delay(100);
  Serial.println("TimerInit");
  delay(100);
  fault=1;
  TimerInit();
}

unsigned int TMC6200_SPI_Read(unsigned int reg)
{
    GPIO.out_w1tc = ((uint32_t)1 << DRV_nSCS);
    
    SPI2.data_buf[0] = (reg);
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[0] = SPI2.data_buf[0] ;  

    SPI2.data_buf[0] = 0;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[1] = SPI2.data_buf[0] ;  

    SPI2.data_buf[0] = 0;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[2] = SPI2.data_buf[0] ;  

    SPI2.data_buf[0] = 0;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[3] = SPI2.data_buf[0] ;  
    
    SPI2.data_buf[0] = 0;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[4] = SPI2.data_buf[0] ;  
    
    GPIO.out_w1ts = ((uint32_t)1 << DRV_nSCS);
    //return();
}

unsigned int TMC6200_SPI_Write(unsigned int reg, unsigned int data_write)
{
    GPIO.out_w1tc = ((uint32_t)1 << DRV_nSCS);
    
    SPI2.data_buf[0] = (reg)&0x80;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[0] = SPI2.data_buf[0] ;  

    SPI2.data_buf[0] = (data_write & 0xFF000000)>>24;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[1] = SPI2.data_buf[0] ;  

    SPI2.data_buf[0] = (data_write & 0x00FF0000)>>16;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[2] = SPI2.data_buf[0] ;  

    SPI2.data_buf[0] = (data_write & 0x0000FF00)>>8;
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[3] = SPI2.data_buf[0] ;  
    
    SPI2.data_buf[0] = (data_write & 0x000000FF);
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    drv_spi_value[4] = SPI2.data_buf[0] ;  
    
    GPIO.out_w1ts = ((uint32_t)1 << DRV_nSCS);
    //return();
}

int IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}
int voltage_value;


#define voltage_scale (1.0/18.16)
#define ANGLE_OFFSET -140

void IRAM_ATTR loop() {
  int i;
  static int adc_cnt; 
  unsigned int temp;
    char ser_read;
  count++;
  if (count>99)
    count=0;
GPIO.out_w1ts = ((uint32_t)1 << TEST_IO4); //TESTPINB
/*
      s1 = (analogRead(DRV_SO1)+analogRead(DRV_SO1)+analogRead(DRV_SO1))- s1_nom;
      s2 = (analogRead(DRV_SO2)+analogRead(DRV_SO2)+analogRead(DRV_SO2))- s2_nom;
      s3 = (analogRead(DRV_SO3)+analogRead(DRV_SO3)+analogRead(DRV_SO3))- s3_nom;

      cur_cur = (abs(s1) + abs(s2) + abs(s3));
      tt = analogRead(FET_TEMP);
      tm = analogRead(VOLT_MON);

      Serial.print (cur_cur);

      Serial.print(" tt ");
      Serial.print (tt);
      Serial.print(" tm ");
      Serial.print (tm);
      //Serial.print(" ");


      Serial.print(" T ");
    Serial.println(Tar_Position);
    //Serial.print(" ABS ");
    //Serial.print(Motor_Abs);
    //Serial.print(" max ");
    //Serial.println(Maximum_PWM);
*/
    //Serial.print(" max ");
    Serial.print(Maximum_PWM);
      Serial.print(" T ");
    Serial.println(Tar_Position);
    
    GPIO.out_w1tc = ((uint32_t)1 << TEST_IO4); //TESTPINB
    
if ( Serial.available() )
  {
    if (Serial.available())
    {
      ser_read = Serial.read();
      Serial.println(ser_read);
    }
    
    if (ser_read=='l')
    {
      Serial.println( config_array[MAG_OFFSET_OS]);
      config_array[MAG_OFFSET_OS]++;
    }    
    else if(ser_read=='k')
    {
      Serial.println( config_array[MAG_OFFSET_OS]);
      config_array[MAG_OFFSET_OS]--;
    }
    else if (ser_read=='n')
    {
      Serial.println(config_array[PHASE_OFFSET_OS]);
      config_array[PHASE_OFFSET_OS]++;
    }    
    else if(ser_read=='m')
    {
      Serial.println( config_array[PHASE_OFFSET_OS]);
      config_array[PHASE_OFFSET_OS]--;
    }
      else if (ser_read=='e')
    {
      edit_config();
    }
    
    else if(ser_read=='q')
    {
      Motor_on=0;
       GPIO.out_w1tc= ((uint32_t)1 << DRV_EN_GATE);
    }
    /*
    else if(ser_read=='w')
    {
      fault=0;
      delay(10);
      for (i=0; i<15;i++)
      {
        //temp=DRV8305_SPI_Read(i);
        Serial.print(i);
        Serial.print(" ");
        Serial.println(temp, HEX);
      }
      fault=1;
      Motor_on=1;
      GPIO.out_w1ts= ((uint32_t)1 << DRV_EN_GATE);
    }   
    */
    else if (ser_read=='f')
    {
      fault=0;
      delay(10);
  for (i=0; i<15;i++)
  {
    TMC6200_SPI_Read(i);
    Serial.print(i);
      Serial.print(" ");
    Serial.print(drv_spi_value[0]);
        Serial.print(" ");
    Serial.print(drv_spi_value[1]);
        Serial.print(" ");
            Serial.print(drv_spi_value[2]);
        Serial.print(" ");
            Serial.print(drv_spi_value[3]);
        Serial.print(" ");
    Serial.println(drv_spi_value[4]);
  }
      //Serial4_DRV_Write(7, 0x296);
      fault=1;
    }
    
  }
  
}
