#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
#include <CAN.h>
// CAN Pins (Default ???) can also be spare UART
#define CAN_RX 16
#define CAN_TX 17
// I2C Pins (Default)
#define I2C_DATA 21
#define I2C_CLK 22
// UART Pins Default
#define TXD0 1
#define RXD0 3
// Test Pins

#define TEST_IO2 23
#define TEST_IO4 25
#define VOLT_MON 39

#define buf_siz 60
char comms_buf[buf_siz];




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

      //CAN.onReceive(onReceive);
}

        double fls_as;
        double flh_as;
        double flk_as;
        double frs_as;
        double frh_as;
        double frk_as;


        int flsn = 12000;
        int fls9 = 0;
        int flhn = 12000;
        int flh9 = 0;
        int flkn = 17000;
        int flk9 = 7000;
        /// ////////////////////
        int frsn = 12000;
        int frs9 = 0;
        int frhn = 12000;
        int frh9 = 0;
        int frkn = 17000;
        int frk9 = 7000;

void  setup() {
  Serial.begin(115200);
  SerialBT.begin("Quad_BT"); //Bluetooth device name
  //SerialBT.setTimeout(2);
  //Serial.println("The device started, now you can pair it with bluetooth!");

   pinMode(TEST_IO2, OUTPUT);

            fls_as = (double)(flsn - fls9) / 90.0;
            flh_as = (double)(flhn - flh9) / 90.0;
            flk_as = (double)(flkn - flk9) / -90.0;
            frs_as = (double)(frsn - frs9) / 90.0;
            frh_as = (double)(frhn - frh9) / 90.0;
            frk_as = (double)(frkn - frk9) / -90.0;

     CAN_Init();
     delay(100);
     CAN_Send(10, 0, 0, 0);
     CAN_Send(11, 0, 0, 0);
     CAN_Send(12, 0, 0, 0);
     CAN_Send(20, 0, 0, 0);
     CAN_Send(21, 0, 0, 0);
     CAN_Send(22, 0, 0, 0);
     delay(100);
}
#define LC 4
        double HIP_LEN = 20.0;
        double THY_LEN = 110.0;
        double SHIN_LEN = 135;
        double rad_deg = 180.0 / 3.1415;
        double deg_rad = 3.1415 / 180.0;
        int hipx[LC]= { 300, 300, 500, 500 };
        int hipy[LC]= { 100, 300, 100, 300 };
        int hipz[LC];
        int kneex [LC];
        int kneey [LC];
        int kneez [LC];
        int footx [LC];
        int footy [LC];
        int footz [LC];



        double hip_foot_dist = 0;
        //double[] hip_foot_ang = new double[LC];
        double hip_foot_ang = 0;
        double hip_foot_ang_deg = 0;
        //double hip_footyz_ang = 0;
        double hip_footyz_ang [LC];
        //double hip_ang = 0;
        double hip_ang [LC];
        double hip_ang_deg = 0;
        //double knee_ang = 0;
        double knee_ang [LC];
        double knee_ang_deg = 0;
        double hip_knee_ang = 0;
        double hip_knee_ang_deg = 0;

        int s_ang [LC];
        int h_ang [LC];
        int k_ang [LC];

        int good_leg = 0;
        int started = 0;
        int wheel = 0;
        int counter = 0;
        //int speed = 0;

        byte walking = false;
        int walk_counter = 0;
        double walk_cnt_max =500.0;
        double leg_phase [LC];

        int sfootx [LC];
        int sfooty [LC];
        int sfootz[LC];

void Ang_To_Pos()
{
  /////// left left left left left
            int u = 0;
            s_ang[u] = ((int)((hip_footyz_ang[u] * rad_deg) * -fls_as) + flsn);

            h_ang[u] = ((int)((hip_ang[u] * rad_deg) * -flh_as) + flhn);

            k_ang[u] = ((int)(((180.0 - (knee_ang[u] * rad_deg)) *flk_as) + flkn));

            ///// right right right right right
            u = 1;
            s_ang[u] = -((int)((hip_footyz_ang[u] * rad_deg) * -frs_as) + frsn);

            h_ang[u] = -((int)((hip_ang[u] * rad_deg) * -frh_as) + frhn);
            
            k_ang[u] = -((int)(((180.0 - (knee_ang[u] * rad_deg)) * frk_as) + frkn));


}
void Cal_Leg3D(int u)
{
    double hip_foot_distyz = (sqrt((double)(sfootz[u] * sfootz[u] + sfooty[u] * sfooty[u])));
    hip_footyz_ang[u] = atan((double)sfootz[u] / (double)sfooty[u]);
    double hip_footyz_ang_deg = hip_footyz_ang[u] * rad_deg;
    //hip_side_lbl.Text = hip_footyz_ang_deg.ToString();
    double hip_foot_j_distyz = hip_foot_distyz - HIP_LEN;

    hip_foot_dist = (sqrt((double)(sfootx[u] * sfootx[u] + hip_foot_j_distyz * hip_foot_j_distyz)));
    if ((hip_foot_dist < (THY_LEN + SHIN_LEN)) && (hip_foot_dist > (SHIN_LEN - THY_LEN)))
    {
        hip_foot_ang = atan((double)sfootx[u] / (double)hip_foot_j_distyz);
        hip_foot_ang_deg = hip_foot_ang * rad_deg;
        //hip_foot_ang_lbl.Text = hip_foot_ang_deg.ToString();
        //hip_foot_dist_lbl.Text = hip_foot_dist.ToString();
        knee_ang[u] = acos((double)(SHIN_LEN * SHIN_LEN + THY_LEN * THY_LEN - hip_foot_dist * hip_foot_dist) / ((double)(2.0 * SHIN_LEN * THY_LEN)));
        knee_ang_deg = knee_ang[u] * rad_deg;
        //knee_ang_lbl.Text = knee_ang_deg.ToString();
        hip_knee_ang = acos((double)(THY_LEN * THY_LEN + hip_foot_dist * hip_foot_dist - SHIN_LEN * SHIN_LEN) / ((double)(2.0 * THY_LEN * hip_foot_dist)));
        hip_knee_ang_deg = hip_knee_ang * rad_deg;
        //hip_knee_ang_lbl.Text = hip_knee_ang_deg.ToString();
        hip_ang[u] = hip_foot_ang + hip_knee_ang;
        hip_ang_deg = hip_ang[u] * rad_deg;
        //hip_ang_lbl.Text = hip_ang_deg.ToString();
        kneex[u] = (int)(sin(hip_ang[u]) * THY_LEN);
        kneey[u] = (int)(cos(hip_ang[u]) * THY_LEN);
        good_leg = 1;
    }
    else
    {
        good_leg = 0;
    }
}
        
void walk()
{
    leg_phase[0] = (double) walk_counter;
    leg_phase[1] = (double)walk_counter + 0.5 * walk_cnt_max;
    leg_phase[2] = (double)walk_counter + 0.5 * walk_cnt_max;
    leg_phase[3] = (double)walk_counter - 0.0 * walk_cnt_max;
    int leg;
    for (leg = 0; leg < LC; leg++)
    {
        if (leg_phase[leg] > walk_cnt_max)
        {
            leg_phase[leg] -= walk_cnt_max;
        }
        else if (leg_phase[leg] < 0)
        {
            leg_phase[leg] += walk_cnt_max;
        }
    }

    double TOG = 0.3;
    double Base_Height = 200.0;
    double Foot_Off_Ground = 50.0;
    double Stride_length = 100.0;
    double Stride_offset = 0.0;

    double cycle;

    for (leg = 0; leg < LC; leg++)
    //leg = 0;
    {
        if ((leg_phase[leg] / (double)walk_cnt_max)< TOG )
        {
            cycle = ((double)leg_phase[leg] / ((double)walk_cnt_max*TOG)) * 3.1415;
            footx[leg] = (int)(Stride_length*0.5 * cos(cycle)) + (int)Stride_offset;
            footy[leg] = (int)Base_Height - (int)(Foot_Off_Ground * sin(cycle));
        }
        else
        {
            cycle = (((double)walk_cnt_max-(double)leg_phase[leg]) / ((double)walk_cnt_max * (1.0-TOG)));
            footx[leg] = (int)(-Stride_length  * (cycle-0.5)) + (int)Stride_offset;
           footy[leg] = (int)Base_Height;
        }

        footz[leg] = 40;
    }
    
}
        
#define PWM_SET 250
#define SLOW_PWM 70
int mov=0;
int tar;
int cur;
int sp=50;


void IRAM_ATTR loop() {
byte c;
int leg_cnt;

if (mov)
{
    walk_counter++;
    if (walk_counter > walk_cnt_max)
        walk_counter = 0;
        
       walk();

  for (leg_cnt = 0; leg_cnt<LC; leg_cnt++)
  {
    sfootx[leg_cnt] = footx [leg_cnt];
    sfooty [leg_cnt] = footy [leg_cnt];
    sfootz[leg_cnt] = footz [leg_cnt];
    Cal_Leg3D(leg_cnt);
  }
  Ang_To_Pos();
  CAN_Send(20, 1, -k_ang[0], PWM_SET);
  delayMicroseconds(1);
  CAN_Send(21, 1, -h_ang[0], PWM_SET);
  
  CAN_Send(10, 1, -k_ang[1], PWM_SET);
  delayMicroseconds(1);
  CAN_Send(11, 1, -h_ang[1], PWM_SET);
/*
  Serial.print((int)footx[0]);
  Serial.print(" ");
  Serial.print((int)footy[0]);
    Serial.print(" ");
  Serial.print((int)(knee_ang[0] * rad_deg));
    Serial.print(" ");
  Serial.print( (int)(hip_ang[0] * rad_deg));

    Serial.print(" ");
  Serial.print((int)h_ang[0] );
    Serial.print(" ");
  Serial.println((int)k_ang[0] );
*/
}

//delay(5);
/*
  while (Serial.available()) {
    c = Serial.read();
    */
      while (SerialBT.available()) {
        c = SerialBT.read();
        
    if (c=='1')
    {
      mov=0;
      CAN_Send(20, 1, 0, 0);
      Serial.println('1');
    }
    else if (c=='0')
    {
      mov=0;
      CAN_Send(20, 0, 0, 0);
      CAN_Send(21, 0, 0, 0);
      CAN_Send(22, 0, 0, 0);
      CAN_Send(10, 0, 0, 0);
      CAN_Send(11, 0, 0, 0);
      CAN_Send(12, 0, 0, 0);
      Serial.println('0');
    }
    else if (c=='2')
    {
      mov=0;
      CAN_Send(20, 2, 0, 0);
      CAN_Send(21, 2, 0, 0);
      CAN_Send(22, 2, 0, 0);
      CAN_Send(10, 2, 0, 0);
      CAN_Send(11, 2, 0, 0);
      CAN_Send(12, 2, 0, 0);
      Serial.println('2');
    }
    else if (c=='3')
    {
      mov=0;
      CAN_Send(20, 3, 0, 0);
      CAN_Send(21, 3, 0, 0);
      CAN_Send(22, 3, 0, 0);
      CAN_Send(10, 3, 0, 0);
      CAN_Send(11, 3, 0, 0);
      CAN_Send(12, 3, 0, 0);
      Serial.println('3');
    }
    else if (c=='m')
    {
      mov=1;
      Serial.println('m');
    }
    else if (c=='a')
    {
      mov=1;
      tar=-12000;
      Serial.println('a');
    }
    else if (c=='s')
    {
      //mov=1;
      CAN_Send(20, 1, -7000, SLOW_PWM);
      CAN_Send(21, 1, -7000, SLOW_PWM);
      CAN_Send(22, 1, -12000, SLOW_PWM);
      CAN_Send(10, 1, 7000, SLOW_PWM);
      CAN_Send(11, 1, 7000, SLOW_PWM);
      CAN_Send(12, 1, 12000, SLOW_PWM);
      Serial.println('s');
    }
        else if (c=='v')
    {
      //mov=1;
      CAN_Send(20, 1, -2000, PWM_SET);
      CAN_Send(21, 1, -10000, PWM_SET);
      CAN_Send(22, 1, -0, PWM_SET);
      CAN_Send(10, 1, 2000, PWM_SET);
      CAN_Send(11, 1, 10000, PWM_SET);
      CAN_Send(12, 1, 0, PWM_SET);
      Serial.println('v');
    }
        else if (c=='c')
    {
      //mov=1;
      CAN_Send(20, 1, -2000, PWM_SET);
      CAN_Send(21, 1, -10000, PWM_SET);
      CAN_Send(22, 1, -10000, PWM_SET);
      CAN_Send(10, 1, 2000, PWM_SET);
      CAN_Send(11, 1, 10000, PWM_SET);
      CAN_Send(12, 1, 10000, PWM_SET);
      Serial.println('c');
    }
    else if (c=='x')
    {
      //mov=1;
      CAN_Send(20, 1, -17000, PWM_SET);
      CAN_Send(21, 1, -12000, PWM_SET);
      CAN_Send(22, 1, -0, PWM_SET);
      CAN_Send(10, 1, 17000, PWM_SET);
      CAN_Send(11, 1, 12000, PWM_SET);
      CAN_Send(12, 1, 0, PWM_SET);
      Serial.println('x');
    }
    else if (c=='d')
    {
      mov=1;
      tar=12000;
      Serial.println('d');
    }
    else if (c==',')
    {
      sp--;
      if (sp<1)
        sp=1;
      Serial.println(sp);
    }
    else if (c=='.')
    {
      sp++;
      Serial.println(sp);
    }

  }
}
