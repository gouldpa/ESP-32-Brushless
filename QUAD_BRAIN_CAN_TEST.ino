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
        
        double rls_as;
        double rlh_as;
        double rlk_as;
        double rrs_as;
        double rrh_as;
        double rrk_as;


        double flsn = 12000;
        double fls9 = 0;
        double flhn = 12000;
        double flh9 = 0;
        double flkn = 17000;
        double flk9 = 7000;
        /// ////////////////////
        double frsn = 12000;
        double frs9 = 0;
        double frhn = 12000;
        double frh9 = 0;
        double frkn = 17000;
        double frk9 = 7000;

        double rlsn = 12000;
        double rls9 = 0;
        double rlhn = 12000;
        double rlh9 = 0;
        double rlkn = 17000;
        double rlk9 = 7000;
        /// ////////////////////
        double rrsn = 12000;
        double rrs9 = 0;
        double rrhn = 12000;
        double rrh9 = 0;
        double rrkn = 17000;
        double rrk9 = 7000;

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

            rls_as = (double)(rlsn - rls9) / 90.0;
            rlh_as = (double)(rlhn - rlh9) / 90.0;
            rlk_as = (double)(rlkn - rlk9) / -90.0;
            rrs_as = (double)(rrsn - rrs9) / 90.0;
            rrh_as = (double)(rrhn - rrh9) / 90.0;
            rrk_as = (double)(rrkn - rrk9) / -90.0;

     CAN_Init();
     delay(100);
     CAN_Send(10, 0, 0, 0);
     CAN_Send(11, 0, 0, 0);
     CAN_Send(12, 0, 0, 0);
     CAN_Send(20, 0, 0, 0);
     CAN_Send(21, 0, 0, 0);
     CAN_Send(22, 0, 0, 0);
     CAN_Send(30, 0, 0, 0);
     CAN_Send(31, 0, 0, 0);
     CAN_Send(32, 0, 0, 0);
     CAN_Send(40, 0, 0, 0);
     CAN_Send(41, 0, 0, 0);
     CAN_Send(42, 0, 0, 0);
     delay(100);


}
#define LC 4
        double HIP_LEN = 20.0;
        double THY_LEN = 110.0;
        double SHIN_LEN = 135.0;
        double rad_deg = 180.0 / 3.1415;
        double deg_rad = 3.1415 / 180.0;
        double hipx[LC]= { 300, 300, 500, 500 };
        double hipy[LC]= { 100, 300, 100, 300 };
        double hipz[LC];
        double kneex [LC];
        double kneey [LC];
        double kneez [LC];
        double footx [LC];
        double footy [LC];
        double footz [LC];



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

        double s_ang [LC];
        double h_ang [LC];
        double k_ang [LC];

        int good_leg = 0;
        int started = 0;
        int wheel = 0;
        int counter = 0;
        //int speed = 0;

        byte walking = false;
        int walk_counter = 0;
        double walk_cnt_max =500.0;
        double leg_phase [LC];

        double sfootx [LC];
        double sfooty [LC];
        double sfootz[LC];

void Ang_To_Pos()
{
  /////// left left left left left
            int u = 0;
            s_ang[u] = (((hip_footyz_ang[u] * rad_deg) * -fls_as) + flsn);

            h_ang[u] = (((hip_ang[u] * rad_deg) * -flh_as) + flhn);

            k_ang[u] = ((((180.0 - (knee_ang[u] * rad_deg)) *flk_as) + flkn));

            ///// right right right right right
            u = 1;
            s_ang[u] = -(((hip_footyz_ang[u] * rad_deg) * -frs_as) + frsn);

            h_ang[u] = -(((hip_ang[u] * rad_deg) * -frh_as) + frhn);
            
            k_ang[u] = -((((180.0 - (knee_ang[u] * rad_deg)) * frk_as) + frkn));
            
            //////////////////////////////////////////////////////////////////
            // rear 
            u = 2;
            s_ang[u] = (((hip_footyz_ang[u] * rad_deg) * -rls_as) + rlsn);

            h_ang[u] = (((hip_ang[u] * rad_deg) * -rlh_as) + rlhn);

            k_ang[u] = ((((180.0 - (knee_ang[u] * rad_deg)) *rlk_as) + rlkn));

            ///// right right right right right
            u = 3;
            s_ang[u] = -(((hip_footyz_ang[u] * rad_deg) * -rrs_as) + rrsn);

            h_ang[u] = -(((hip_ang[u] * rad_deg) * -rrh_as) + rrhn);
            
            k_ang[u] = -((((180.0 - (knee_ang[u] * rad_deg)) * rrk_as) + rrkn));


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
        kneex[u] = (sin(hip_ang[u]) * THY_LEN);
        kneey[u] = (cos(hip_ang[u]) * THY_LEN);
        good_leg = 1;
    }
    else
    {
        good_leg = 0;
    }
}

double TOG = 0.1;
double Base_Height = 215.0;
double Foot_Off_Ground = 50.0;
double Stride_length=70.0;
double l_r=0.0;
double Stride_length_a[LC];
double Stride_offset = 0.0;

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
  if (l_r<0.0)
  {
    Stride_length_a[0] = Stride_length*(1.0+l_r);
    Stride_length_a[1] = Stride_length*(1.0);
    Stride_length_a[2] = Stride_length*(1.0+l_r);
    Stride_length_a[3] = Stride_length*(1.0);
  }
  else
  {
    Stride_length_a[0] = Stride_length*(1.0);
    Stride_length_a[1] = Stride_length*(1.0-l_r);
    Stride_length_a[2] = Stride_length*(1.0);
    Stride_length_a[3] = Stride_length*(1.0-l_r);
  }

    double cycle;

    for (leg = 0; leg < LC; leg++)
    //leg = 0;
    {
        if ((leg_phase[leg] / (double)walk_cnt_max)< TOG )
        {
            cycle = ((double)leg_phase[leg] / ((double)walk_cnt_max*(double)TOG)) * 3.1415;
            footx[leg] = (Stride_length_a[leg]*0.5 * cos(cycle)) + Stride_offset;
            footy[leg] = Base_Height - (Foot_Off_Ground * sin(cycle));
        }
        else
        {
            cycle = (((double)walk_cnt_max-(double)leg_phase[leg]) / ((double)walk_cnt_max * (1.0-(double)TOG)));
            footx[leg] = (-Stride_length_a[leg]  * (cycle-0.5)) + Stride_offset;
           footy[leg] = Base_Height;
        }

        footz[leg] = 40;
    }
    
}
        
int pwm_set = 100;
#define SLOW_PWM 100
int mov=0;
int tar;
int cur;
int sp=500;
#define TILT 3000


void IRAM_ATTR loop() {
byte c;
int leg_cnt;

if (mov)
{
  walk_cnt_max = sp;
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
  CAN_Send(20, 1, -(int)k_ang[0], pwm_set);
  delayMicroseconds(1);
  CAN_Send(21, 1, -(int)h_ang[0], pwm_set);
  delayMicroseconds(1);
  CAN_Send(10, 1, -(int)k_ang[1], pwm_set);
  delayMicroseconds(1);
  CAN_Send(11, 1, -(int)h_ang[1], pwm_set);
  delayMicroseconds(1);
  CAN_Send(40, 1, -(int)k_ang[2], pwm_set);
  delayMicroseconds(1);
  CAN_Send(41, 1, (int)h_ang[2], pwm_set);
  delayMicroseconds(1);
  CAN_Send(30, 1, -(int)k_ang[3], pwm_set);
  delayMicroseconds(1);
  CAN_Send(31, 1, (int)h_ang[3], pwm_set);
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

  //while (Serial.available()) {
  //  c = Serial.read();
    
     while (SerialBT.available()) {
        c = SerialBT.read();
        //SerialBT.println(c);

    if (c=='0')
    {
      mov=0;
      CAN_Send(40, 0, 0, 0);
      CAN_Send(41, 0, 0, 0);
      CAN_Send(42, 0, 0, 0);
      CAN_Send(30, 0, 0, 0);
      CAN_Send(31, 0, 0, 0);
      CAN_Send(32, 0, 0, 0);
      CAN_Send(20, 0, 0, 0);
      CAN_Send(21, 0, 0, 0);
      CAN_Send(22, 0, 0, 0);
      CAN_Send(10, 0, 0, 0);
      CAN_Send(11, 0, 0, 0);
      CAN_Send(12, 0, 0, 0);
      SerialBT.println('0');
    }
    else if (c=='2')
    {
      mov=0;
      CAN_Send(40, 2, 0, 0);
      CAN_Send(41, 2, 0, 0);
      CAN_Send(42, 2, 0, 0);
      CAN_Send(30, 2, 0, 0);
      CAN_Send(31, 2, 0, 0);
      CAN_Send(32, 2, 0, 0);
      CAN_Send(20, 2, 0, 0);
      CAN_Send(21, 2, 0, 0);
      CAN_Send(22, 2, 0, 0);
      CAN_Send(10, 2, 0, 0);
      CAN_Send(11, 2, 0, 0);
      CAN_Send(12, 2, 0, 0);
      SerialBT.println('2');
    }
    else if (c=='3')
    {
      mov=0;
      CAN_Send(40, 3, 0, 0);
      CAN_Send(41, 3, 0, 0);
      CAN_Send(42, 3, 0, 0);
      CAN_Send(30, 3, 0, 0);
      CAN_Send(31, 3, 0, 0);
      CAN_Send(32, 3, 0, 0);
      CAN_Send(20, 3, 0, 0);
      CAN_Send(21, 3, 0, 0);
      CAN_Send(22, 3, 0, 0);
      CAN_Send(10, 3, 0, 0);
      CAN_Send(11, 3, 0, 0);
      CAN_Send(12, 3, 0, 0);
      SerialBT.println('3');
    }
    else if (c=='p')
    {
      //mov=1;
      CAN_Send(40, 1, 0, SLOW_PWM);
      CAN_Send(41, 1, 2000, SLOW_PWM);
      CAN_Send(42, 1, -12000-TILT, SLOW_PWM);
      CAN_Send(30, 1, 0, SLOW_PWM);
      CAN_Send(31, 1, -2000, SLOW_PWM);
      CAN_Send(32, 1, 12000-TILT, SLOW_PWM);
      CAN_Send(20, 1, 0, SLOW_PWM);
      CAN_Send(21, 1, -2000, SLOW_PWM);
      CAN_Send(22, 1, -12000-TILT, SLOW_PWM);
      CAN_Send(10, 1, 0, SLOW_PWM);
      CAN_Send(11, 1, 2000, SLOW_PWM);
      CAN_Send(12, 1, 12000-TILT, SLOW_PWM);
      SerialBT.println('p');
    }
        else if (c=='o')
    {
      //mov=1;
      CAN_Send(40, 1, 0, SLOW_PWM);
      CAN_Send(41, 1, -2000, SLOW_PWM);
      CAN_Send(42, 1, -12000, SLOW_PWM);
      CAN_Send(30, 1, 0, SLOW_PWM);
      CAN_Send(31, 1, 2000, SLOW_PWM);
      CAN_Send(32, 1, 12000, SLOW_PWM);
      CAN_Send(20, 1, 0, SLOW_PWM);
      CAN_Send(21, 1, 2000, SLOW_PWM);
      CAN_Send(22, 1, -12000, SLOW_PWM);
      CAN_Send(10, 1, 0, SLOW_PWM);
      CAN_Send(11, 1, -2000, SLOW_PWM);
      CAN_Send(12, 1, 12000, SLOW_PWM);
      SerialBT.println('o');
    }
    else if (c=='s')
    {
      //mov=1;
      CAN_Send(40, 1, -10000, SLOW_PWM);
      CAN_Send(41, 1, 7000, SLOW_PWM);
      CAN_Send(42, 1, -12000, SLOW_PWM);
      CAN_Send(30, 1, 10000, SLOW_PWM);
      CAN_Send(31, 1, -7000, SLOW_PWM);
      CAN_Send(32, 1, 12000, SLOW_PWM);
      CAN_Send(20, 1, -10000, SLOW_PWM);
      CAN_Send(21, 1, -7000, SLOW_PWM);
      CAN_Send(22, 1, -12000, SLOW_PWM);
      CAN_Send(10, 1, 10000, SLOW_PWM);
      CAN_Send(11, 1, 7000, SLOW_PWM);
      CAN_Send(12, 1, 12000, SLOW_PWM);
      SerialBT.println('s');
    }
        else if (c=='d')
    {
      //mov=1;
      CAN_Send(40, 1, 0, pwm_set);
      CAN_Send(41, 1, 0, pwm_set);
      CAN_Send(42, 1, 0, pwm_set);
      CAN_Send(30, 1, 0, pwm_set);
      CAN_Send(31, 1, 0, pwm_set);
      CAN_Send(32, 1, 0, pwm_set);
      CAN_Send(20, 1, 0, pwm_set);
      CAN_Send(21, 1, 0, pwm_set);
      CAN_Send(22, 1, 0, pwm_set);
      CAN_Send(10, 1, 0, pwm_set);
      CAN_Send(11, 1, 0, pwm_set);
      CAN_Send(12, 1, 0, pwm_set);
      SerialBT.println('v');
    }
    else if (c=='r')
    {
      mov=1;
      SerialBT.println("run");
    }
    else if (c=='e')
    {
      mov=0;
      SerialBT.println("pause");
    }
    else if (c==',')
    {
      sp=sp-10;
      if (sp<1)
        sp=1;
      SerialBT.print("sp ");
      SerialBT.println(sp);
    }
    else if (c=='.')
    {
      sp=sp+10;
      SerialBT.print("sp ");
      SerialBT.println(sp);
    }
    else if (c=='n')
    {
      pwm_set=pwm_set-10;
      if (pwm_set<1)
        pwm_set=1;
      SerialBT.print("pwm_set ");
      SerialBT.println(pwm_set);
    }
    else if (c=='m')
    {
      pwm_set=pwm_set+10;
      SerialBT.print("pwm_set ");
      SerialBT.println(pwm_set);
    }
    else if (c=='b')
    {
      Stride_length=Stride_length-10;
      if (Stride_length<0)
        Stride_length=0;
      SerialBT.print("Stride_length ");
      SerialBT.println(Stride_length);
    }
    else if (c=='v')
    {
      Stride_length=Stride_length+10;
      if (Stride_offset>50)
          Stride_offset=50;
      SerialBT.print("Stride_length ");
      SerialBT.println(Stride_length);
    }
    else if (c=='l')
    {
      Stride_offset=Stride_offset-2;
      if (Stride_offset<-50)
        Stride_offset=-50;
      SerialBT.print("Stride_offset ");
      SerialBT.println(Stride_offset);
    }
    else if (c=='k')
    {
      Stride_offset=Stride_offset+2;
      if (Stride_offset>50)
        Stride_offset=50;
      SerialBT.print("Stride_offset ");
      SerialBT.println(Stride_offset);
    }
        else if (c=='j')
    {
      TOG=TOG-0.05;
      if (TOG<0)
        TOG=0;
      SerialBT.print("TOG ");
      SerialBT.println(TOG);
    }
    else if (c=='h')
    {
      TOG=TOG+.05;
      if (TOG>1)
        TOG=1;
      SerialBT.print("TOG ");
      SerialBT.println(TOG);
    }
    else if (c=='g')
    {
      Base_Height=Base_Height-5;
      if (Base_Height<100)
        Base_Height=100;
      SerialBT.print("Base_Height ");
      SerialBT.println(Base_Height);
    }
    else if (c=='f')
    {
      Base_Height=Base_Height+5;
      if (Base_Height>250)
        Base_Height=250;
      SerialBT.print("Base_Height ");
      SerialBT.println(Base_Height);
    }
        else if (c=='i')
    {
      Foot_Off_Ground=Foot_Off_Ground-5;
      if (Foot_Off_Ground<0)
        Foot_Off_Ground=0;
      SerialBT.print("Foot_Off_Ground ");
      SerialBT.println(Foot_Off_Ground);
    }
    else if (c=='u')
    {
      Foot_Off_Ground=Foot_Off_Ground+5;
      if (Foot_Off_Ground>100)
        Foot_Off_Ground=100;
      SerialBT.print("Foot_Off_Ground ");
      SerialBT.println(Foot_Off_Ground);
    }
    else if (c=='z')
    {
      l_r=-0.5;
      SerialBT.println("left ");
      SerialBT.println(l_r);
    }
    else if (c=='x')
    {
      l_r=0;
      SerialBT.print("straight ");
      SerialBT.println(l_r);
    }
    else if (c=='c')
    {
      l_r=0.5;
      SerialBT.print("right ");
      SerialBT.println(l_r);
    }

  }
}
