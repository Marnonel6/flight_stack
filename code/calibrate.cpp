
#include <stdio.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <signal.h>

#define PWM_MAX 2000
#define frequency 25000000.0
#define LED0 0x6			
#define LED0_ON_L 0x6		
#define LED0_ON_H 0x7		
#define LED0_OFF_L 0x8		
#define LED0_OFF_H 0x9		
#define LED_MULTIPLYER 4	

int pwm;

//gcc -o keyboard keyboard.cpp -lwiringPi -lncurses -lm
void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
     // printf("I2C connected \n\r");
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}



void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}


void set_PWM( uint8_t channel, float time_on_us)
{
  if(1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  { 
    time_on_us=1000;
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}


int main()
{
    int key=0;
    int i=0;

    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);
    int state=0;


    while(1)
    {
        int num=0;

        printf("(re)starting calibration remove battery, press enter after\n");
        num= getchar();


        set_PWM(0,2000);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(1,2000);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(2,2000);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(3,2000);//speed between 1000 and PWM_MAX, motor 0-3
        printf("plug in battery, wait for tone to stop 10 seconds,  press enter after\n");

        num= getchar();

        set_PWM(0,1000);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(1,1000);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(2,1000);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(3,1000);//speed between 1000 and PWM_MAX, motor 0-3
        delay(1000);
        printf("calibration done press enter once beeps stop \n\r");
        num= getchar();


        int speed1=1100;
        printf("testing motors %d\n",speed1);
        set_PWM(0,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(1,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(2,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(3,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        delay(1000);
        delay(1000);
        delay(1000);
        speed1=1200;
        printf("testing motors %d\n",speed1);
        set_PWM(0,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(1,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(2,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(3,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        delay(1000);
        delay(1000);
        delay(1000);
        speed1=1300;
        printf("testing motors %d\n",speed1);
        set_PWM(0,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(1,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(2,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(3,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        delay(1000);
        delay(1000);
        delay(1000);
        speed1=1000;
        printf("turning off motors\n");
        set_PWM(0,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(1,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(2,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        set_PWM(3,speed1);//speed between 1000 and PWM_MAX, motor 0-3
        delay(1000);
        delay(1000);
        delay(1000);
    }

return 0;
}
