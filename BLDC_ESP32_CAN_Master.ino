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

#include "BluetoothSerial.h"

#include <WiFi.h>
#include <esp_now.h>
#include <stdint.h>
#include <string.h>

#define WIFI_CHANNEL                        (1)
#define LED_PIN                             (2)

//////////////////////////// wifi

static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct __attribute__((packed)) esp_now_msg_t
{
  uint32_t address;
  int32_t now_data;
  // Can put lots of things here...
} esp_now_msg_t;

int32_t other_motor_pos;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//#include "wiring_private.h" // pinPeripheral() function

//  I2c OLED  ///////////////////////////////////////////////////////////
#include "ssd1306.h" // library by Alexey Dynda
char display_str[] = "1234567890";

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
/*
#define DRV_SO3 34
#define DRV_SO1 33
#define DRV_SO2 32
#define MOTOR_TEMP 37
#define FET_TEMP 36
#define VOLT_MON 39
*/
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
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); //AS5047
  //hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); //MA702
  //SPI2.mosi_dlen.usr_mosi_dbitlen = (1 * 16) - 1;
  //SPI2.miso_dlen.usr_miso_dbitlen = (1 * 16) - 1;
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
  //encoder_int = encoder_int & 0x3FFF ;
  /*
  if (even){
    GPIO.out_w1tc = ((uint32_t)1 << EXT_ENC_nCS);
    SPI2.data_buf[0] = 0xFFFF; //AS5147
    //SPI2.data_buf[0] = 0x0000; //MA702
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    GPIO.out_w1ts = ((uint32_t)1 << EXT_ENC_nCS);
    encoder_ext = SPI2.data_buf[0] & 0xFFFF;  
    if(!SPI2.ctrl.rd_bit_order){
          encoder_ext = (encoder_ext >> 8) | (encoder_ext << 8);
    }
    encoder_ext = encoder_ext & 0x3FFF ;
    even=0;
  }
  else
  {
    GPIO.out_w1tc = ((uint32_t)1 << DRV_nSCS);
    SPI2.data_buf[0] = 0xFFFF; //AS5147
    //SPI2.data_buf[0] = 0x0000; //MA702
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    GPIO.out_w1ts = ((uint32_t)1 << DRV_nSCS);
    drv_spi_value = SPI2.data_buf[0] & 0xFFFF;  
    if(!SPI2.ctrl.rd_bit_order){
          drv_spi_value = (drv_spi_value >> 8) | (drv_spi_value << 8);
    }
    //drv_spi_value
    even=1;
  }
  */
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
   //Motor_Error = (Joint_Position - other_motor_pos)/2;
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
    // Limit Voltage/PWM to motor
    //if (motor_output > Maximum_PWM) //config_array[TEMP_MAX_OS])
    //    motor_output = Maximum_PWM; //config_array[TEMP_MAX_OS];
    if (motor_output > config_array[TEMP_MAX_OS])
        motor_output = config_array[TEMP_MAX_OS];
    
    if (cur_cur>2000)
      motor_output = motor_output/2;
    else if (cur_cur>3000)
      motor_output = 0;
        
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
#define POSITIVE 0

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
        //phase_120
        /*
        Serial.print(step1);
        Serial.print(',');
        Serial.print(SinArray[step1]);
        Serial.println();
        delay(10);
        */
        
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
 * @brief  Puts one variable on the SSD1306 OLED display
 * @retval void
 */
//float current=0.9;
float voltage;
static void textDemo()
{
  static int dis_cnt=0;
  dis_cnt++;
  if (dis_cnt>4)
    dis_cnt =0;
  /*
  if (digitalRead(DRV_nFAULT))
  {
    sprintf(display_str, "e %5d OK", encoder_int );
  }
  else
  {
    sprintf(display_str, "e %5d FLT", encoder_int );
  }
*/
  if (dis_cnt==0)
  {
    if (digitalRead(DRV_nFAULT))
    {
      sprintf(display_str, "%2d.%1dV%2d.%1dA",(int)(voltage),((int)(voltage*10))%10,(int)(current),((int)(current*10))%10 );
    }
    else
    {
      sprintf(display_str, "%2d.%1dV FLT",(int)(voltage),((int)(voltage*10))%10); 
    }
    ssd1306_printFixed2x(0,  0, display_str, STYLE_NORMAL);
  }
  else if (dis_cnt==1)
  {
    sprintf(display_str, "%5d  %2dC",encoder_int , f_temp);
    //ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed2x(0,  24, display_str, STYLE_NORMAL);
  }
  else if (dis_cnt ==2)
  {
    sprintf(display_str, "%5d", encoder_ext);
    //ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_printFixed2x(0,  48, display_str, STYLE_NORMAL);
  } 

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

void Display_Init()
{
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_128x64_i2c_init();
    ssd1306_clearScreen();
    //ssd1306_setFixedFont(ssd1306xled_font6x8);
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

void IRAM_ATTR CAN_Send(int id, int stat, int motor_tar_pos, int Motor_PWM_Max){
  //CAN.beginPacket(0x12);
  CAN.beginPacket(id);
  
  CAN.write((int)(stat >>8) & 0x00FF);
  CAN.write(((int)(stat) & 0x00FF));
  CAN.write((int)(motor_tar_pos >>8) & 0x00FF);
  CAN.write(((int)(motor_tar_pos) & 0x00FF));
  CAN.write((int)(Motor_PWM_Max >>8) & 0x00FF);
  CAN.write(((int)(Motor_PWM_Max) & 0x00FF));

  CAN.endPacket();
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
SerialBT.begin("ESP32_one"); //Bluetooth device name
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

    network_setup();
  SPIInit();
  setup_mcpwm();
  //EepromInit();
Serial.println("setup_mcpwm");
    // I2C OLED
      delay(100);
  Display_Init();
        delay(3000);
  Display_Init();

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

  //TMC6200_SPI_Write(
 //hspi->endTransaction();
  //hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  
  /*
  //DRV8305_SPI_Write(7, 0x296 );
  DRV8305_SPI_Write(7, 0x29F); // sets 3x PWM 
  DRV8305_SPI_Write(5, 0x377); // 
  DRV8305_SPI_Write(6, 0x377); // 
  DRV8305_SPI_Write(9, 0x220); // disables the under voltage fault
  
  for (i=0; i<15;i++)
  {
    temps=DRV8305_SPI_Read(i);
    Serial.print(i);
    Serial.print(" ");
    Serial.println(temps, HEX);
  }
  */
  s1_nom = analogRead(DRV_SO1)+analogRead(DRV_SO1)+analogRead(DRV_SO1);
  s2_nom = analogRead(DRV_SO2)+analogRead(DRV_SO2)+analogRead(DRV_SO2);
  s3_nom = analogRead(DRV_SO3)+analogRead(DRV_SO3)+analogRead(DRV_SO3);
  
  CAN_Init();
  delay(100);
    //SerialBT.begin("ESP32test"); //Bluetooth device name
  //Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println (analogRead(DRV_SO1));
  Serial.println("TimerInit");
  delay(100);
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
/*
unsigned int DRV8305_SPI_Write(unsigned int reg, unsigned int data_write)
{
  //unsigned int drv_spi_value;
    GPIO.out_w1tc = ((uint32_t)1 << DRV_nSCS);
    SPI2.data_buf[0] = (reg<<3) + ((data_write & 0x1F00)>>8) + ((data_write & 0x00FF)<<8); 
    SPI2.cmd.usr = 1;
    while(SPI2.cmd.usr);
    GPIO.out_w1ts = ((uint32_t)1 << DRV_nSCS);
    drv_spi_value[0] = SPI2.data_buf[0] & 0xFFFF;  
    if(!SPI2.ctrl.rd_bit_order){
          drv_spi_value[0] = (drv_spi_value[0] >> 8) | (drv_spi_value[0] << 8);
    }
    return();
}
*/

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

static void handle_error(esp_err_t err)
{
  switch (err)
  {
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("Not init");
      break;

    case ESP_ERR_ESPNOW_ARG:
      Serial.println("Argument invalid");
      break;

    case ESP_ERR_ESPNOW_INTERNAL:
      Serial.println("Internal error");
      break;

    case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("Out of memory");
      break;

    case ESP_ERR_ESPNOW_NOT_FOUND:
      Serial.println("Peer is not found");
      break;

    case ESP_ERR_ESPNOW_IF:
      Serial.println("Current WiFi interface doesn't match that of peer");
      break;

    default:
      break;
  }
}

static void msg_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  //Serial.println("RX_MESSAGE");
  if (len == sizeof(esp_now_msg_t))
  {
    esp_now_msg_t msg;
    memcpy(&msg, data, len);
    other_motor_pos = msg.now_data;
    //Serial.print("Counter: ");
    //Serial.println(msg.counter);
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

static void msg_send_cb(const uint8_t* mac, esp_now_send_status_t sendStatus)
{

  switch (sendStatus)
  {
    case ESP_NOW_SEND_SUCCESS:
      Serial.println("S");
      break;

    case ESP_NOW_SEND_FAIL:
      Serial.println("F");
      break;

    default:
      break;
  }
}

static void send_msg(esp_now_msg_t * msg)
{
  // Pack
  uint16_t packet_size = sizeof(esp_now_msg_t);
  uint8_t msg_data[packet_size];
  memcpy(&msg_data[0], msg, sizeof(esp_now_msg_t));

  esp_err_t status = esp_now_send(broadcast_mac, msg_data, packet_size);
  if (ESP_OK != status)
  {
    Serial.println("Error sending message");
    handle_error(status);
  }
}
static void send_esp_now(void)
{
  static uint32_t counter = 0;
  esp_now_msg_t msg;
  msg.address = 0;
  msg.now_data = Motor_Abs;
  send_msg(&msg);
}
static void network_setup(void)
{
  //esp_wifi_internal_set_fix_rate(10);
  //Puts ESP in STATION MODE
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0)
  {
    Serial.println("esp_now_init Fail");
    return;
  }

  esp_now_peer_info_t peer_info;
  peer_info.channel = WIFI_CHANNEL;
  memcpy(peer_info.peer_addr, broadcast_mac, 6);
  peer_info.ifidx = ESP_IF_WIFI_STA;
  peer_info.encrypt = false;
  esp_err_t status = esp_now_add_peer(&peer_info);
  if (ESP_OK != status)
  {
    Serial.println("Could not add peer");
    handle_error(status);
  }

  // Set up callback
  status = esp_now_register_recv_cb(msg_recv_cb);
  if (ESP_OK != status)
  {
    Serial.println("Could not register callback");
    handle_error(status);
  }

  status = esp_now_register_send_cb(msg_send_cb);
  if (ESP_OK != status)
  {
    Serial.println("Could not register send callback");
    handle_error(status);
  }
}

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

//SPI_Read_Encoder();

Serial.print (motor_output);
Serial.print(" ");
Serial.print (insta_speed);
Serial.print(" ");
/*
Serial.print (encoder_int);
Serial.print(" ");
*/

//delay(100);


  //voltage_value = analogRead(VOLT_MON);
    //voltage = (float)voltage_value * voltage_scale;
  //textDemo();

  
  
/*
  //analogRead(VOLT_MON);
  if (SerialBT.hasClient())
    Serial.print ("BT ");
    else
    Serial.print ("OFF ");

  Serial.print (man_offset);
  Serial.print (" ");
  Serial.println (analogRead(VOLT_MON));
  */



      s1 = (analogRead(DRV_SO1)+analogRead(DRV_SO1)+analogRead(DRV_SO1))- s1_nom;
      s2 = (analogRead(DRV_SO2)+analogRead(DRV_SO2)+analogRead(DRV_SO2))- s2_nom;
      s3 = (analogRead(DRV_SO3)+analogRead(DRV_SO3)+analogRead(DRV_SO3))- s3_nom;

      cur_cur = (abs(s1) + abs(s2) + abs(s3));
      tt = analogRead(FET_TEMP);
      tm = analogRead(VOLT_MON);

      Serial.print (phase_forward);
      Serial.print(" ");
      Serial.print (cur_cur);
      Serial.print(" ");

      Serial.print (s1);
      Serial.print(" ");
      Serial.print (s2);
      Serial.print(" ");
      Serial.print (s3);
      Serial.print(" ");
      
      Serial.print (tt);
      Serial.print(" tm ");
      Serial.println (tm);
      //Serial.print(" ");

/*

    int tt = (analogRead(FET_TEMP)+analogRead(FET_TEMP)+analogRead(FET_TEMP)+analogRead(FET_TEMP))>>2;
    f_temp = (tt-600)/14;
  }
  else if (adc_cnt==2)
  {
    int tm = (analogRead(VOLT_MON)+analogRead(VOLT_MON)+analogRead(VOLT_MON)+analogRead(VOLT_MON))>>2;
    voltage = ((float)(tm+150))/30.0;
  }
  adc_cnt++;
  if (adc_cnt>4)
  adc_cnt=0;
  
  Serial.print (" "); //sprintf(display_str, "%2d.%1dV%2d.%1dA",,((int)(voltage*10))%10,(int)(current),((int)(current*10))%10 );

  Serial.print ((int)(voltage));
  Serial.print (".");
  Serial.print (((int)(voltage*10))%10);
  Serial.print ("V ");
  Serial.print ((int)(current));
  Serial.print (".");
  Serial.print (((int)(current*10))%10 );
  Serial.print ("A ");
  Serial.print (f_temp);
  Serial.println ("C");
  */
  GPIO.out_w1tc = ((uint32_t)1 << TEST_IO4); //TESTPINB
    //  Serial.print("MO ");
    //Serial.print(motor_output);
  //    Serial.print(" T ");
   // Serial.println(Tar_Position);
    //Serial.print(" ABS ");
    //Serial.print(Motor_Abs);
    //Serial.print(" max ");
    //Serial.println(Maximum_PWM);
    
  //Serial.println ("A");

  //GPIO.out_w1ts = ((uint32_t)1 << TEST_IO2); //TESTPINB

if (SerialBT.available() || Serial.available() )
  {
    //ser_read = SerialBT.read();
    if (SerialBT.available())
    {
      ser_read = SerialBT.read();
      SerialBT.println(ser_read);
    }
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
    else if (ser_read=='a')
    {
      Tar_Position=500000;
      //forward=0;
    }
    else if (ser_read=='s')
    {
      Tar_Position=0;
      //forward=1;
    }
    else if (ser_read=='d')
    {
      Tar_Position=-500000;
      //forward=1;
    }
    else if (ser_read=='z')
    {
      Motor_Abs=0;
      //forward=1;
    }
    else if (ser_read=='x')
    {    
      CAN_Send(30, 0, 0, 0);
      CAN_Send(31, 0, 0, 0);
    }
    else if (ser_read=='c')
    {    
      CAN_Send(30, 1, 5000, 100);
      CAN_Send(31, 1, 5000, 100);
    }
    else if (ser_read=='h')
    {    
      CAN_Send(30, 1, -5000, 50);
      CAN_Send(31, 1, -5000, 50);
    }
    else if (ser_read=='v')
    {    
      CAN_Send(30, 1, 0, 200);
      CAN_Send(31, 1, 0, 200);
    }
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
