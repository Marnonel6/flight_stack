#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>
#include <stdlib.h>
#include "vive.h"

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define A                0.02  //Complemetary filter ratio
#define MAX_GYRO_RATE    1000  // deg/s
#define MAX_ROLL_ANGLE   45.0  // deg
#define MAX_PITCH_ANGLE  45.0  // deg
#define PWM_MAX          1800
#define PWM_OFF          1000
#define frequency        25000000.0
#define LED0             0x6
#define LED0_ON_L        0x6
#define LED0_ON_H        0x7
#define LED0_OFF_L       0x8
#define LED0_OFF_H       0x9
#define LED_MULTIPLYER   4
#define NEUTRAL_THRUST   1500
#define P_PITCH          9
#define D_PITCH          1.0
#define I_PITCH          0.02
#define MAX_PITCH_I      100
#define P_ROLL           15
#define D_ROLL           1.0
#define I_ROLL           0.04
#define MAX_ROLL_I       100
#define P_YAW            3
#define P_YAW_VIVE       600
#define P_Y_VIVE         0.3
#define D_Y_VIVE         0.1
#define P_X_VIVE         0.6
#define D_X_VIVE         0.1
#define PITCH_MAX        8 // Degrees
#define ROLL_MAX         8 // Degrees
#define YAW_MAX          100 // DPS
#define JOY_NEUTRAL      128
#define JOY_HIGH         240
#define JOY_LOW          16
#define VIVE_MAX_XY      1000
#define VIVE_MAX_YAW     1.57079 // 90 Degrees

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// Joystick
struct Keyboard {
  int keypress;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num; // heartbeat
};

//declare global struct for vive position
Position local_p;

// Function prototypes
int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void setup_keyboard();
void trap(int signal);
void safety_check(Keyboard keyboard);
void joystick_control(Keyboard keyboard);
void get_joystick(Keyboard keyboard);
void init_pwm();
void init_motor(uint8_t channel);
void set_PWM( uint8_t channel, float time_on_us);
void pid_update();
void safety_check_vive(Position local_p);
void vive_control(Position local_p);

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6];
long time_curr;
long time_prev;
struct timespec te;
struct timespec t_heartbeat;
struct timespec te_vive;
struct timespec t_heartbeat_vive;
long time_curr_heartbeat;
long time_prev_heartbeat = 0.0;
int hearbeat_prev = 0;
long time_curr_heartbeat_vive;
long time_prev_heartbeat_vive = 0.0;
int hearbeat_prev_vive = 0;
float yaw=0.0;
float pitch_angle=0.0;
float roll_angle=0.0;
//intitialising variables used to average data for calibration
float pi = 3.14159;
float filtered_pitch = 0.0;
float filtered_roll = 0.0;
float roll_gyro_delta=0.0;
float pitch_gyro_delta = 0.0;
FILE *file_p;
int pwm;
int pwm_0 = 1000;
int pwm_1 = 1000;
int pwm_2 = 1000;
int pwm_3 = 1000;
float pitch_error;
float pitch_error_I = 0.0;
float roll_error;
float roll_error_I = 0.0;
float desired_thrust = NEUTRAL_THRUST;
int prev_version = 1000; //  For keyboard control
float desired_pitch = 0.0;
float desired_roll = 0.0;
float desired_yaw_velocity = 0.0;
float yaw_error_velocity;
// Joystick variables
int joy_pitch = 0;
int joy_roll = 0;
int joy_yaw = 0;
int joy_thrust = 0;
int joy_keypress = 0;
int sequence_num = 0;
// Vive
float vive_x_desired = 0.0;
float vive_y_desired = 0.0;
float vive_yaw_desired = 0.0;
float vive_y_estimated = 0.0;
float y_vive_prev = 0.0;
float desired_pitch_vive = 0.0;
float vive_x_estimated = 0.0;
float x_vive_prev = 0.0;
float desired_roll_vive = 0.0;
double dt_vive_ = 0.0;
double vive_D_error_y = 0.0;
double vive_D_error_x = 0.0;
// Flags
int Flag_pause = 1;
int Flag_set_vive_desired = 1;
// Shared data
Keyboard* shared_memory;
int run_program=1;

int main (int argc, char *argv[])
{
    // Init motors
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);

    // Setup IMU and Keyboard
    setup_imu();
    calibrate_imu();
    setup_keyboard();
    signal(SIGINT, &trap);
    delay(1000);

    // Get shared memory values
    Keyboard keyboard=*shared_memory;
    init_shared_memory();

    // Save values to CSV
    file_p = fopen("desired_vs_actual_angles.csv", "w+");

    while(run_program==1)
    {

      // Refresh vive position data
      local_p=*position;

      if (Flag_set_vive_desired == 1)
      {
          // Save desired X,Y position as current vive X,Y position
          vive_x_desired = local_p.x;
          vive_y_desired = local_p.y;
          vive_yaw_desired = local_p.yaw;
          Flag_set_vive_desired = 0;
      }

      // Get sensor data
      read_imu();
      update_filter();

      printf(" Vive x: %10.5f  y: %10.5f  z: %10.5f  yaw: %10.5f  desired pitch: %10.5f desired roll: %10.5f\n", local_p.x - vive_x_desired, local_p.y - vive_y_desired, local_p.z, local_p.yaw - vive_yaw_desired, desired_pitch, desired_roll);
      // Save values to CSV
      fprintf(file_p, "%10.5f, %10.5f, %10.5f, %10.5f,\n", desired_pitch, filtered_pitch, desired_roll, filtered_roll); // Desired vs actual angles

      // Refresh values from shared memory first
      Keyboard keyboard=*shared_memory;
      safety_check(keyboard);
      safety_check_vive(local_p);

      // Joystick controls
      joystick_control(keyboard);
      get_joystick(keyboard);

      if (Flag_pause == 0) // Pause motors
      {
        // Update motor speeds
        vive_control(local_p);
        pid_update();
      }

    }

    // Save values to CSV
    fclose(file_p);

    // Kill all motors
    printf("\n Killing Motors! \n"); 
    set_PWM(0,PWM_OFF);
    set_PWM(1,PWM_OFF); 
    set_PWM(2,PWM_OFF);
    set_PWM(3,PWM_OFF);
    delay(1000);
    printf("\n Motors dead \n");

    return 0;
}

void calibrate_imu()
{
  // initialize sum variables locally
  float x_gyro_sum=0.0;
  float y_gyro_sum=0.0;
  float z_gyro_sum=0.0;
  float roll_sum=0.0;
  float pitch_sum=0.0;
  float accel_z_sum=0.0;

  // loop that runs to sum data over a 1000 iterations
  for (int i=0; i<1000; i++){
    read_imu();

    x_gyro_sum += imu_data[0];
    y_gyro_sum += imu_data[1];
    z_gyro_sum += imu_data[2];
    roll_sum += roll_angle;
    pitch_sum += pitch_angle;
    accel_z_sum += imu_data[5];
  }

  // Add negetive to eliminate drift
  // Averaging data using the summation variables
  x_gyro_calibration = -x_gyro_sum/1000.0;
  y_gyro_calibration = -y_gyro_sum/1000.0;
  z_gyro_calibration = -z_gyro_sum/1000.0;
  roll_calibration = -roll_sum/1000.0;
  pitch_calibration = -pitch_sum/1000.0;
  accel_z_calibration = -accel_z_sum/1000.0;

  printf("\n calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}

void read_imu()
{
  int address=59;// set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;
  
  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[3]=vw*(-2.0-2.0)/(-32768.0-32767.0);// convert vw from raw values to "g's"
  
  
  address=61;// set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=vw*(-2.0-2.0)/(-32768.0-32767.0);// convert vw from raw values to "g's"
  
  
  address=63;// set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=vw*(-2.0-2.0)/(-32768.0-32767.0);// convert vw from raw values to g's
  
  
  address=67;// set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]= -(x_gyro_calibration + vw*(-500.0-500.0)/(-32768.0-32767.0));// convert vw from raw values to degrees/second 
  
  address=69;// set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
 imu_data[1]=y_gyro_calibration + vw*(-500.0-500.0)/(-32768.0-32767.0);// convert vw from raw values to degrees/second     
  
  address=71;// set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=z_gyro_calibration + vw*(-500.0-500.0)/(-32768.0-32767.0);// convert vw from raw values to degrees/second
  
  // Calculate Roll, Pitch and Yaw
  pitch_angle = (atan2(imu_data[4],-imu_data[5])*180/pi) + pitch_calibration;
  roll_angle = (atan2(imu_data[3],-imu_data[5])*180/pi) + roll_calibration;
}

void update_filter()
{
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;

  //comp. filter for roll, pitch here: 
  roll_gyro_delta = imu_data[1]*imu_diff; // Gyro roll angle calculation
  filtered_roll = roll_angle*A + (1-A)*(roll_gyro_delta+filtered_roll);

  pitch_gyro_delta = imu_data[0]*imu_diff; // Gyro pitch angle calculation
  filtered_pitch = pitch_angle*A + (1-A)*(pitch_gyro_delta+filtered_pitch);
}

int setup_imu()
{
  wiringPiSetup ();

  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address

  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}


void setup_keyboard()
{
  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;

  /* Allocate a shared memory segment.  */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size  =               shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
}

// when cntrl+c pressed, kill motors
void trap(int signal)
{
  printf("\n ending program - control C was pressed - Kill Motors \n\r");
  // Kill all motors
  set_PWM(0,PWM_OFF);
  set_PWM(1,PWM_OFF); 
  set_PWM(2,PWM_OFF);
  set_PWM(3,PWM_OFF);
  run_program=0;
}

void vive_control(Position local_p)
{
/*
    Get 3D position from Vove
*/

    //get current time in nanoseconds
    timespec_get(&t_heartbeat_vive,TIME_UTC);
    time_curr_heartbeat_vive = t_heartbeat_vive.tv_nsec;
    //compute time since last execution
    double passed_time_vive = time_curr_heartbeat_vive-time_prev_heartbeat_vive;

    //check for rollover
    if(passed_time_vive<=0.0)
    {
    passed_time_vive+=1000000000.0;
    }

    //convert to seconds
    passed_time_vive=passed_time_vive/1000000000.0;
    dt_vive_ = passed_time_vive;

    // Exponential filter to filter out vive noise
    vive_y_estimated = vive_y_estimated*0.6 + local_p.y*0.4;
    vive_x_estimated = vive_x_estimated*0.8 + local_p.x*0.2;

    // P_Y_VIVE
    if (hearbeat_prev_vive != local_p.version)
    {
      y_vive_prev = vive_y_estimated;
      x_vive_prev = vive_x_estimated;
      time_prev_heartbeat_vive = time_curr_heartbeat_vive;
      vive_D_error_y = (vive_y_estimated - y_vive_prev)*(1/dt_vive_);
      vive_D_error_x = (vive_x_estimated - x_vive_prev)*(1/dt_vive_);
    }
}


void safety_check_vive(Position local_p) // Vive
{
/*
  Safety checks that stops the student program when any of the following cases are violated/detected:
  – X !> +-500
  – Y !> +-500
  – Yaw !> +-90Deg (Radians 1.57079)
  – Vive timeout - .version (Heart beat does not update in 0.5 seconds)
*/

  //get current time in nanoseconds
  timespec_get(&t_heartbeat_vive,TIME_UTC);
  time_curr_heartbeat_vive = t_heartbeat_vive.tv_nsec;
  //compute time since last execution
  double passed_time_vive = time_curr_heartbeat_vive-time_prev_heartbeat_vive;

  //check for rollover
  if(passed_time_vive<=0.0)
  {
    passed_time_vive+=1000000000.0;
  }

  //convert to seconds
  passed_time_vive=passed_time_vive/1000000000.0;

  if (hearbeat_prev_vive != local_p.version)
  { // Reset previous heartbeat time stamp if a new heartbeat is detected.
    hearbeat_prev_vive = local_p.version;
    time_prev_heartbeat_vive = time_curr_heartbeat_vive;
  }
  else if (passed_time_vive>0.5)
  { // If the previous heartbeat is the same as the current heartbeat and 0.25s has passed
    // Stop the student from executing.
    printf("Vive timedout! (Heartbeat)");
    printf("Heartbeat_prev  %d  Current version %d  passed_time_vive %f", hearbeat_prev_vive, local_p.version, passed_time_vive);
    run_program=0;
  }
  else if (abs(local_p.x - vive_x_desired)> VIVE_MAX_XY)
  {
    printf("Vive X location to far %10.5f", local_p.x);
    run_program=0;
  }
  else if (abs(local_p.y - vive_y_desired)> VIVE_MAX_XY)
  {
    printf("Vive Y location to far %10.5f", local_p.y);
    run_program=0;
  }
  else if (abs(local_p.yaw - vive_yaw_desired)> VIVE_MAX_YAW)
  {
    printf("Vive YAW location greater than max %10.5f", local_p.yaw);
    run_program=0;
  }
}

void safety_check(Keyboard keyboard) // Joystick
{
/*
  Safety checks that stops the student program when any of the following cases are violated/detected:
  – Any gyro rate > 300 degrees/sec
  – Roll angle > 45 or <-45
  – Pitch angle >45 or <-45
  – Keyboard press of space
  – Keyboard timeout (Heart beat does not update in 0.25 seconds)
*/

  //get current time in nanoseconds
  timespec_get(&t_heartbeat,TIME_UTC);
  time_curr_heartbeat = t_heartbeat.tv_nsec;
  //compute time since last execution
  double passed_time = time_curr_heartbeat-time_prev_heartbeat;

  //check for rollover
  if(passed_time<=0.0)
  {
    passed_time+=1000000000.0;
  }

  //convert to seconds
  passed_time=passed_time/1000000000.0;

  if (hearbeat_prev != keyboard.sequence_num)
  { // Reset previous heartbeat time stamp if a new heartbeat is detected.
    hearbeat_prev = keyboard.sequence_num;
    time_prev_heartbeat = time_curr_heartbeat;
  }
  else if (passed_time>0.5) // TODO was 0.25
  { // If the previous heartbeat is the same as the current heartbeat and 0.25s has passed
    // Stop the student from executing.
    printf("Keyboard timedout! (Heartbeat)");
    printf("Heartbeat_prev  %d  Current heart %d  passed_time %f", hearbeat_prev, keyboard.sequence_num, passed_time);
    run_program=0;
  }

  if (keyboard.keypress == 32)
  { // If the joystick A button is pressed, then stop the student code.
    printf("\n A button was pressed!");
    run_program=0;
  }
  else if (abs(filtered_pitch)>MAX_PITCH_ANGLE || abs(filtered_roll)>MAX_ROLL_ANGLE)
  { // If the pitch or roll angles are larger than the max allowable angle, then stop the student code.
    printf("\n Pitch or Roll angle exceeds maximum limit: Pitch: %10.5f  Roll: %10.5f", filtered_pitch, filtered_roll);
    run_program=0;
  }
  else if (abs(imu_data[0])>MAX_GYRO_RATE || abs(imu_data[1])>MAX_GYRO_RATE || abs(imu_data[2])>MAX_GYRO_RATE)
  { // If any of the 3 gyro rates are larger than the max allowable gyro rate, then stop the student code.
    printf("\n Gyro rate exceeds maximum limit: x: %10.5f  y: %10.5f  z: %10.5f", imu_data[0], imu_data[1], imu_data[2]);
    run_program=0;
  }
}

void pid_update()
{
  // Calculate pitch error
  desired_pitch_vive = (vive_y_desired - vive_y_estimated)*P_Y_VIVE - vive_D_error_y*D_Y_VIVE;
  desired_pitch = desired_pitch*0.5 - desired_pitch_vive*0.5; // Flip vive Y
  if (desired_pitch > 6)
  {
    desired_pitch = 6.0;
  }
  else if (desired_pitch < -6)
  {
    desired_pitch = -6.0;
  }
  pitch_error = desired_pitch - filtered_pitch;
  pitch_error_I += pitch_error*I_PITCH;

  // Calculate roll error
  desired_roll_vive = (vive_x_desired - vive_x_estimated)*P_X_VIVE + vive_D_error_x*D_X_VIVE;
  desired_roll = desired_roll*0.5 - desired_roll_vive*0.5;
  if (desired_roll > 6)
  {
    desired_roll = 6.0;
  }
  else if (desired_roll < -6)
  {
    desired_roll = -6.0;
  }
  roll_error = desired_roll - filtered_roll;
  roll_error_I += roll_error*I_ROLL;

  // Calculate yaw error
  desired_yaw_velocity = local_p.yaw*P_YAW_VIVE;
  yaw_error_velocity = desired_yaw_velocity - imu_data[2]; // Vive high

  // Limit pitch integral term
  if (pitch_error_I > MAX_PITCH_I)
  {
    pitch_error_I = MAX_PITCH_I;
  }
  else if (pitch_error_I < -MAX_PITCH_I)
  {
    pitch_error_I = -MAX_PITCH_I;
  }
  // Limit roll integral term
  if (roll_error_I > MAX_ROLL_I)
  {
    roll_error_I = MAX_ROLL_I;
  }
  else if (roll_error_I < -MAX_ROLL_I)
  {
    roll_error_I = -MAX_ROLL_I;
  }

  // PID - Controller for Pitch and Roll
  pwm_0 = desired_thrust - pitch_error*P_PITCH + imu_data[0]*D_PITCH - pitch_error_I + roll_error*P_ROLL - imu_data[1]*D_ROLL + roll_error_I + yaw_error_velocity*P_YAW;
  pwm_1 = desired_thrust + pitch_error*P_PITCH - imu_data[0]*D_PITCH + pitch_error_I + roll_error*P_ROLL - imu_data[1]*D_ROLL + roll_error_I - yaw_error_velocity*P_YAW;
  pwm_2 = desired_thrust + pitch_error*P_PITCH - imu_data[0]*D_PITCH + pitch_error_I - roll_error*P_ROLL + imu_data[1]*D_ROLL - roll_error_I + yaw_error_velocity*P_YAW;
  pwm_3 = desired_thrust - pitch_error*P_PITCH + imu_data[0]*D_PITCH - pitch_error_I - roll_error*P_ROLL + imu_data[1]*D_ROLL - roll_error_I - yaw_error_velocity*P_YAW;

  // Limit PWM signal at 1000 - 1300s
  if (pwm_0 > PWM_MAX)
  {
    pwm_0 = PWM_MAX;
  }
  else if (pwm_0 < PWM_OFF)
  {
    pwm_0 = PWM_OFF;
  }

  if (pwm_1 > PWM_MAX)
  {
    pwm_1 = PWM_MAX;
  }
  else if (pwm_1 < PWM_OFF)
  {
    pwm_1 = PWM_OFF;
  }

  if (pwm_2 > PWM_MAX)
  {
    pwm_2 = PWM_MAX;
  }
  else if (pwm_2 < PWM_OFF)
  {
    pwm_2 = PWM_OFF;
  }

  if (pwm_3 > PWM_MAX)
  {
    pwm_3 = PWM_MAX;
  }
  else if (pwm_3 < PWM_OFF)
  {
    pwm_3 = PWM_OFF;
  }

  // Set motor speed - Motor number and speed - 1000->1300
  set_PWM(0,pwm_0);
  set_PWM(1,pwm_1);
  set_PWM(2,pwm_2);
  set_PWM(3,pwm_3);
}

void joystick_control(Keyboard keyboard)
{
/*
  Safety checks that stops the student program when any of the following cases are violated/detected:
  - u (34) - Start the motors
  - p (33) - Pause the motors
  - c (35) - Calibrate 
*/
  if (keyboard.keypress == 33)
  {
    Flag_pause = 1;
    printf("\n Pause motors \n\r");
    // Kill all motors
    set_PWM(0,PWM_OFF);
    set_PWM(1,PWM_OFF); 
    set_PWM(2,PWM_OFF);
    set_PWM(3,PWM_OFF);
    // prev_version = keyboard.version;
  }
  else if (keyboard.keypress == 35)
  {
    printf("\n Calibrate IMU \n\r");
    // Kill all motors
    set_PWM(0,PWM_OFF);
    set_PWM(1,PWM_OFF); 
    set_PWM(2,PWM_OFF);
    set_PWM(3,PWM_OFF);
    delay(100);
    // Reset all calibration values
    x_gyro_calibration = 0.0;
    y_gyro_calibration = 0.0;
    z_gyro_calibration = 0.0;
    roll_calibration = 0.0;
    pitch_calibration = 0.0;
    accel_z_calibration = 0.0;
    // Calibrate IMU
    calibrate_imu();
    // Set integral windup back to 0
    pitch_error_I = 0.0;
    roll_error_I = 0.0;
    desired_pitch = 0.0;
    desired_roll = 0.0;
    desired_thrust = NEUTRAL_THRUST;

    // Save desired X,Y position as current vive X,Y position
    vive_x_desired = local_p.x;
    vive_y_desired = local_p.y;
    vive_yaw_desired = local_p.yaw;

    printf("\n Done calibrating IMU \n\r");
    printf("\n Done saving desired Vive X,Y position \n\r");
  }
  else if (keyboard.keypress == 34)
  {
    printf("\n Unpause motors \n\r");
    Flag_pause = 0;
    pitch_error_I =0.0;
    roll_error_I =0.0;
  }
}

void get_joystick(Keyboard keyboard)
{
    /*
    Update global variables for the joystick
    */
    // Update joystick values
    joy_pitch = keyboard.pitch;
    joy_roll = keyboard.roll;
    joy_yaw = keyboard.yaw;
    joy_thrust = keyboard.thrust;
    joy_keypress = keyboard.keypress;
    sequence_num = keyboard.sequence_num;

    desired_pitch = ((float)(2.0 * PITCH_MAX)/(JOY_HIGH - JOY_LOW))*(joy_pitch-128);
    desired_roll = -((float)(2.0 * ROLL_MAX)/(JOY_HIGH - JOY_LOW))*(joy_roll-128);
    desired_yaw_velocity = ((float)(2.0 * YAW_MAX)/(JOY_HIGH - JOY_LOW))*(joy_yaw-128);
    desired_thrust = ((float)(PWM_MAX - PWM_OFF)/(JOY_HIGH - JOY_LOW))*(joy_thrust-128) + NEUTRAL_THRUST;
}

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

