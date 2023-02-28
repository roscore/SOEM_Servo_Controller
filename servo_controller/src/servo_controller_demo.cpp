#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>

#include <pwd.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sor/output.h>
#include <bits/stdc++.h>

#include "ft_sensor_msgs/ForceTorque.h"
#include "sensor_msgs/Joy.h"
#include "ethercat.h"
#include "get_time.h"

#include "pdo_def.h"
#include "servo_def.h"

#define EC_TIMEOUTMON 2000
#define NUMOFSERVO_DRIVE 6

//#define _WRITE_MODEOP_SDO TRUE

#define Control_Word 0x6040
#define Status_Word 0x6041
#define mod_op 0x6060
#define sync_ch_2 0x1C12
#define sync_ch_3 0x1C13
#define Tx_PDO1 0x1A00
#define Tx_PDO2 0x1A01
#define Tx_PDO3 0x1A02
#define Tx_PDO4 0x1A03
#define Rx_PDO1 0x1600
#define Rx_PDO2 0x1601
#define Rx_PDO3 0x1602
#define Rx_PDO4 0x1603

MAXPOS_Drive_pt	maxpos_drive_pt[NUMOFSERVO_DRIVE];

using namespace std;
using namespace GetTime;
using namespace boost::filesystem;
using namespace boost::lambda;

double torque_command[24] = {0,0,0,0,0,0};

const int storage_size = 500;

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;

int run=1;
int sys_ready=0;
bool init_flag = false;

int started[NUMOFSERVO_DRIVE]={0}, ServoState=0;
uint8 servo_ready=0, servo_prestate=0;
int32_t zeropos[NUMOFSERVO_DRIVE]={0};
double gt=0;

double sine_amp=11500, f=0.2, period;
int recv_fail_cnt=0;

uint8 currentgroup = 0;

int ServoOnGetCtrlWrd(uint16_t status_word, uint16_t *control_word);

void WriteLog(std::ofstream &log_file, float32 data[1][6]);
void SendCommand(uint8_t *data, uint16_t *buf, int buf_length);
void PrintValues_2(uint8_t *data);
void GetSensorValue(float32 new_data[6], uint8_t *data);
void SetRosMsg(ft_sensor_msgs::ForceTorque *ft_msg, float32 data[][6]);

OSAL_THREAD_FUNC ec_check(void *ptr);
int CountFiles(path the_path);

//variables for pdo re-mapping (sdo write)
int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;
boolean inOP;

static int ServoWrite8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int ServoWrite16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int ServoWrite32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int ServoSetup(uint16 slave)
{
  int wkc = 0;

  printf ("HERoEHS Servo Drive Setup\n");

  wkc += ServoWrite8 (slave, 0x1C12, 0, 0);
  wkc += ServoWrite8 (slave, 0x1C13, 0, 0);

  wkc += ServoWrite8  (slave, 0x1A00, 0, 0);
  wkc += ServoWrite32 (slave, 0x1A00, 1, 0x60410010);
  wkc += ServoWrite32 (slave, 0x1A00, 2, 0x60640020);
  wkc += ServoWrite8  (slave, 0x1A00, 0, 2);

  wkc += ServoWrite8  (slave, 0x1600, 0, 0);
  wkc += ServoWrite32 (slave, 0x1600, 1, 0x60400010);
  wkc += ServoWrite32 (slave, 0x1600, 2, 0x607A0020);
  wkc += ServoWrite8  (slave, 0x1600, 0, 2);

  wkc += ServoWrite16 (slave, 0x1C12, 1, 0x1600);
  wkc += ServoWrite16 (slave, 0x1C12, 0, 1);

  wkc += ServoWrite16 (slave, 0x1C13, 1, 0x1A00);
  wkc += ServoWrite16 (slave, 0x1C13, 0, 1);

  /* Explicitly set flags that are (probably) invalid in EEPROM */
  //  ec_slave[slave].SM[2].SMflags = 0x10024;

  /* Explicitly disable sync managers that are activated by EEPROM */
  //  ec_slave[slave].SM[4].StartAddr = 0;
  //  ec_slave[slave].SM[5].StartAddr = 0;

  /* Set a slave name */
  strncpy (ec_slave[slave].name, "SERVO", EC_MAXNAME);

  printf("ServoSetup: %d\n", wkc);


  if (wkc != 14)
  {
    printf ("Servo Setup Failed\n");
    return -1;
  }

  return 0;
}



void Homing(void)
{
  int i, oloop, iloop, k, wkc_count;	

  for(int k=0; k < NUMOFSERVO_DRIVE; ++k)
  {
    os=sizeof(ob2); ob2 = 0x06;	//Shutdown
    wkc_count=ec_SDOwrite(k+1, 0x6040,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
    printf("Control Word: %d\n", ob2);

    os=sizeof(ob2); ob2 = 0x0f;	//Set Enable Operation
    wkc_count=ec_SDOwrite(k+1, 0x6040,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
    printf("Control Word: %d\n", ob2);


    os=sizeof(ob2); ob2 = 0x06;	//Set HM mode
    wkc_count=ec_SDOwrite(k+1, 0x6060,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
    printf("Control Word: %d\n", ob2);


    os=sizeof(ob2); ob2 = 0x1f;	//Start HM
    wkc_count=ec_SDOwrite(k+1, 0x6040,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave operation mode
    printf("Control Word: %d\n", ob2);
  }

    init_flag = true;

}

boolean ecat_init(void)
{
  std::string lan_port_name_;
  lan_port_name_ = "enp2s0";
  int n = lan_port_name_.length();

  char ecat_ifname[n + 1];

  strcpy(ecat_ifname, lan_port_name_.c_str());

  int i, oloop, iloop, k, wkc_count;	
  needlf = FALSE;
  inOP = FALSE;

  printf("Starting simple test\n");
	
  if (ec_init(ecat_ifname))
  {
    printf("ec_init on %s succeeded.\n", ecat_ifname); //ifname
    /* find and auto-config slaves */

    if ( ec_config_init(FALSE) > 0 )
    {
      printf("%d slaves found and configured.\n",ec_slavecount);
      
      //ServoSetup(1);

      //PDO re-mapping****************************************************************************************************
      // for (k=0; k<NUMOFSERVO_DRIVE; ++k)
      // {
      //   if (( ec_slavecount >= 1 ) && strcmp(ec_slave[k+1].name, "iServo_EtherCAT(LAN9252)_V1") == 0) //change name for other drives
      //   {
      //     printf("Re mapping for HERoEHS Linear Actuator...\n");
      //     os=sizeof(ob2); ob2 = 0x1600;	//RxPDO, check MAXPOS ESI
      //     //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
      //     //wkc_count=ec_SDOwrite(k+1, 0x1c12,01,TRUE,os, &ob2,EC_TIMEOUTRXM);	//change slave position (k+1) if needed
      //     wkc_count=ec_SDOwrite(k+1, 0x1c12,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM);	//change slave position (k+1) if needed

      //     if (wkc_count==0)
      //     {
      //       printf("RxPDO assignment error\n");
      //       return FALSE;
      //     }
        
      //     os=sizeof(ob2); ob2 = 0x1a00;	//TxPDO, check MAXPOS ESI
      //     //0x1c13 is Index of Sync Manager 3 PDO Assignment (input TxPDO), CA (Complete Access) must be TRUE
      //     //wkc_count=ec_SDOwrite(k+1, 0x1c13,01,TRUE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
      //     wkc_count=ec_SDOwrite(k+1, 0x1c13,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
      //     if (wkc_count==0)
      //     {
      //       printf("TxPDO assignment error\n");
      //       return FALSE;
      //     }

      //     os=sizeof(ob2); ob2 = 0x02;	//TxPDO, check MAXPOS ESI
      //     wkc_count=ec_SDOwrite(k+1, 0x1600,0x00,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
      //     os=sizeof(ob2); ob2 = 0x6040;	//TxPDO, check MAXPOS ESI
      //     wkc_count=ec_SDOwrite(k+1, 0x1600,0x01,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
      //     os=sizeof(ob2); ob2 = 0x607A;	//TxPDO, check MAXPOS ESI
      //     wkc_count=ec_SDOwrite(k+1, 0x1600,0x02,FALSE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
         
      //   }
      // }
      //PDO re-mapping****************************************************************************************************

      ec_config_map(&IOmap);
      
      printf("Slaves mapped, state to SAFE_OP.\n");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_INIT,  EC_TIMEOUTSTATE * 4); ///////////////////////////////////////////

      oloop = ec_slave[0].Obytes;
      if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
      //if (oloop > 8) oloop = 8;
      iloop = ec_slave[0].Ibytes;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
      //if (iloop > 8) iloop = 8;

      printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

      printf("Request operational state for all slaves\n");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */

      // ec_writestate(0);
      // ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP

      if (ec_slave[0].state == EC_STATE_OPERATIONAL )
      {
        printf("Operational state reached for all slaves.\n");
        wkc_count = 0;

        // for (k=0; k<NUMOFSERVO_DRIVE; ++k)
        // {
        //   maxpos_drive_pt[k].ptOutParam=(MAXPOS_DRIVE_RxPDO_t*)  		ec_slave[k+1].outputs;
        //   maxpos_drive_pt[k].ptInParam= (MAXPOS_DRIVE_TxPDO_t*)  		ec_slave[k+1].inputs;
        //   maxpos_drive_pt[k].ptOutParam->ModeOfOperation=OP_MODE_CYCLIC_SYNC_POSITION;
        // }
        inOP = TRUE;
            }
      else
      {
        printf("Not all slaves reached operational state.\n");
        // ec_readstate();
        // for(i = 1; i<=ec_slavecount ; i++)
        // {
        //   if(ec_slave[i].state != EC_STATE_OPERATIONAL)
        //   {
        //     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
        //     i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        //   }
        // }
        // for (i=0; i<NUMOFSERVO_DRIVE; ++i)
        //   ec_dcsync01(i+1, FALSE, 0, 0, 0); // SYNC0,1 
      }
    }
    else
    {
        printf("No slaves found!\n");
        inOP=FALSE;
    }

  }
  else
  {
    printf("No socket connection on %s\nExcecute as root\n", ecat_ifname);
    return FALSE;
  }

  return inOP;
}

void TorqueCommandCallback(const sor::output::ConstPtr& msg)
{
  torque_command[0] = round(msg->motor1 / 75559 * 1000 * 1000 / 4.86);
  torque_command[1] = round(msg->motor2 / 75559 * 1000 * 1000 / 4.86);
  torque_command[2] = round(msg->motor3 / 75559 * 1000 * 1000 / 4.86);
  torque_command[3] = round(msg->motor4 / 75559 * 1000 * 1000 / 4.86);
  torque_command[4] = round(msg->motor5 / 75559 * 1000 * 1000 / 4.86);
  torque_command[5] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);

  torque_command[6] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[7] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[8] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[9] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[10] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[11] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);

  torque_command[12] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[13] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[14] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[15] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[16] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[17] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);

  torque_command[18] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[19] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[20] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[21] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[22] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
  torque_command[23] = round(msg->motor6 / 75559 * 1000 * 1000 / 4.86);
}

void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    ROS_INFO_STREAM("Up  : "    << msg->buttons[1]);
    ROS_INFO_STREAM("Down  : "  << msg->buttons[2]);
}

int main(int argc, char *argv[])
{

  std::string lan_port_name_;
  lan_port_name_ = "enp2s0";
  int n = lan_port_name_.length();

  char ecat_ifname[n + 1];

  strcpy(ecat_ifname, lan_port_name_.c_str());

  std::cout << "[HERoHES Linear Actuator] EtherCAT port name :" << ecat_ifname << std::endl;

  std::string ss_user_name;
  ss_user_name = "heroehs";
 
  string file_string = "/home/"+ss_user_name+"/log/";

  char file_location[100] ;
  strcpy(file_location,file_string.c_str());
  
  std::ofstream log_file;
  
  //char file_location[80] = "/home";

  char date[40];

  sprintf(date, "log_%04d.txt", CountFiles(path(file_location)) + 1);

  strcat(file_location, date);

  log_file.open(file_location);

  needlf = FALSE;

  ros::init(argc, argv, "linear_actuator_communication_node");
  ros::NodeHandle nh;
  ros::Publisher ec_data = nh.advertise<ft_sensor_msgs::ForceTorque>("/linear_actuator_data", 1);
  ros::Publisher linear_actuator_length_pub = nh.advertise<std_msgs::Float64MultiArray>("/linear_actuator_length", 1);

  ros::Subscriber RecieveTorqueCommand = nh.subscribe("/Torque", 10, TorqueCommandCallback);
  ros::Subscriber RecieveJoyCommand    = nh.subscribe("/joy",1000, JoyCallback);

  std_msgs::Float64MultiArray linear_actuator_length;

  ecat_init();

  bool homing_flag = false;
  bool check_param = false;
  int check_param_result = 0;
  bool run_flag = false;
  
  for(int iter = 0; iter < NUMOFSERVO_DRIVE; iter++)
  {
    uint16_t kind_of_motor;
    os=sizeof(kind_of_motor); kind_of_motor = 0x00;
    ec_SDOread(iter+1, 0x6402,0x00, FALSE, &os, &kind_of_motor, EC_TIMEOUTRXM); //read status of driver

    if(kind_of_motor == 2)  check_param_result++;
    else                    printf("%d Moter Status is wrong!!! \n", iter+1);
  }

  if(check_param_result == NUMOFSERVO_DRIVE)  check_param = true; 

  if(check_param == true) homing_flag = true;

  if(homing_flag == true)
  {
    ROS_WARN("Homing will Start in 5 seconds");
    ros::Duration(5.00).sleep();

    Homing();
    ROS_INFO("Homing Start");
    ros::Duration(20.0).sleep();
    ROS_INFO("Homing Done");

    ROS_INFO("Start Moving in 5 seconds");
    ros::Duration(5.00).sleep();

    run_flag = true;  
  }


  int play_count = 0;
  int count = 0;
  linear_actuator_length.data.clear();
    
  while (init_flag == true && ros::ok())
  {
    if(run_flag == true)
    {
      for(int iter = 0; iter < NUMOFSERVO_DRIVE; iter++)
      {
        uint16_t mode_of_driver;
        uint profile_velocity;
        int target_position;
        int target_position_read;
        int actual_position;
        int demand_position;
        uint16_t op_cmd;
        int8_t op_mode;

        if(play_count == 0)
        {
          os=sizeof(op_mode); op_mode = 1;	//Set Profile Position Mode
          ec_SDOwrite(iter+1, 0x6060,0x00,FALSE,os, &op_mode,EC_TIMEOUTRXM); //change slave operation mode
          printf("Set Profile Position Mode: %d\n", op_mode);

          uint16_t status_of_driver;
          os=sizeof(status_of_driver); status_of_driver = 0x00;
          ec_SDOread(iter+1, 0x6061,0x00, FALSE, &os, &status_of_driver, EC_TIMEOUTRXM); //read status of driver
          printf("Status of driver: %d\n", status_of_driver);
          ros::Duration(1.00).sleep();
          ROS_INFO("start Position control mode");
        }
        else if(play_count == 1)
        {
          os=sizeof(mode_of_driver); mode_of_driver = 0x00;
          ec_SDOread(iter+1, 0x6041,0x00, FALSE, &os, &mode_of_driver, EC_TIMEOUTRXM); //read status of driver
          printf("Mode of driver: %d\n", mode_of_driver);

          os=sizeof(op_cmd); op_cmd = 0x0f;	//Enable
          ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
          printf("set OP mod");

          os=sizeof(profile_velocity); profile_velocity = 0x1f4;	// pre state
          ec_SDOwrite(iter+1, 0x6081,0x00,FALSE,os, &profile_velocity,EC_TIMEOUTRXM); //change slave operation mode
          printf("profile veloicty: %d\n", profile_velocity);

          os=sizeof(target_position);
          if((iter+1 == 1) || (iter+1 == 6) || (iter+1 == 7) || (iter+1 == 12) || (iter+1 == 13) || (iter+1 == 18) || (iter+1 == 19) || (iter+1 == 24))
          {
            target_position = 60491;
          }
          else if((iter+1 == 2) || (iter+1 == 5) || (iter+1 == 8) || (iter+1 == 11) || (iter+1 == 14) || (iter+1 == 17) || (iter+1 == 20) || (iter+1 == 23))
          {
            target_position = 58028;
          }
          else if((iter+1 == 3) || (iter+1 == 4) || (iter+1 == 9) || (iter+1 == 10) || (iter+1 == 15) || (iter+1 == 16) || (iter+1 == 21) || (iter+1 == 22))
          {
            target_position = 80086;
          }

          ec_SDOwrite(iter+1, 0x607A,0x00, FALSE, os, &target_position, EC_TIMEOUTRXM); //read status of driver
          printf("Target Position(write) of driver: %d\n", target_position);

          os=sizeof(op_cmd); op_cmd = 0x3f;	// pre state
          ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
          printf("set PP mod");

          os=sizeof(target_position_read); target_position_read = 0x00;
          ec_SDOread(iter+1, 0x607A,0x00, FALSE, &os, &target_position_read, EC_TIMEOUTRXM); //read status of driver
          printf("Target Position(read) of driver: %d\n", target_position_read);

          os=sizeof(actual_position); actual_position = 0x00;
          ec_SDOread(iter+1, 0x6064,0x00, FALSE, &os, &actual_position, EC_TIMEOUTRXM); //read status of driver
          printf("Actual Position of driver: %d\n", actual_position);

          os=sizeof(demand_position); demand_position = 0x00;
          ec_SDOread(iter+1, 0x6062,0x00, FALSE, &os, &demand_position, EC_TIMEOUTRXM); //read status of driver
          printf("Demand Position of driver: %d\n", demand_position);

          ROS_INFO("moving to target position");
        }
        /*else
        {
          count ++;
          ROS_INFO("count : %d", count);  
          
          os=sizeof(op_mode); op_mode = 4;	//Set Profile Torque Mode
          ec_SDOwrite(iter+1, 0x6060,0x00,FALSE,os, &op_mode,EC_TIMEOUTRXM); //change slave operation mode
          printf("Set Profile Torque Mode: %d\n", op_mode);

          os=sizeof(mode_of_driver); mode_of_driver = 0x00;
          ec_SDOread(iter+1, 0x6041,0x00, FALSE, &os, &mode_of_driver, EC_TIMEOUTRXM); //read status of driver
          //printf("Mode of driver: %d\n", mode_of_driver);

          uint target_torque = 0;
          os=sizeof(target_torque);
          target_torque = torque_command[iter];
          ec_SDOwrite(iter+1, 0x6071,0x00, FALSE, os, &target_torque, EC_TIMEOUTRXM); //read status of driver
          printf("Target torque(write) of driver: %d\n", target_torque);

          os=sizeof(op_cmd); op_cmd = 0xf;	// pre state
          ec_SDOwrite(iter+1, 0x6040,0x00,FALSE,os, &op_cmd,EC_TIMEOUTRXM); //change slave operation mode
          printf("set PT mod\n");

          int16_t actual_torque;
          os=sizeof(actual_torque); actual_torque=0;
          ec_SDOread(1, 0x6077, 0x00, FALSE, &os, &actual_torque, EC_TIMEOUTRXM);
          printf("Actual Torque of driver: %d\n", actual_torque);

          uint target_torque_read = 0;
          os=sizeof(target_torque_read); target_torque_read = 0x00;
          ec_SDOread(iter+1, 0x6071,0x00, FALSE, &os, &target_torque_read, EC_TIMEOUTRXM); //read status of driver
          //printf("Target Position(read) of driver: %d\n", target_torque_read);

          os=sizeof(actual_position); actual_position = 0x00;
          ec_SDOread(iter+1, 0x6064,0x00, FALSE, &os, &actual_position, EC_TIMEOUTRXM); //read status of driver

            ROS_WARN("calc actual position!!!");
            linear_actuator_length.data.push_back(float(actual_position));
            ROS_WARN("pub actual position!!!");
            
          os=sizeof(demand_position); demand_position = 0x00;
          ec_SDOread(iter+1, 0x6062,0x00, FALSE, &os, &demand_position, EC_TIMEOUTRXM); //read status of driver
          //printf("Demand Position of driver: %d\n", demand_position);
        }*/

      }
      
      linear_actuator_length_pub.publish(linear_actuator_length);
      linear_actuator_length.data.clear();

      if(play_count == 1)
      {
        ROS_INFO("start torque control mode");
        ros::Duration(10.00).sleep();
      }
      /*else if(play_count == 2)
      {
        ROS_WARN("Change OP Mode in 10 seconds");
        ros::Duration(10.00).sleep();
      }*/

      play_count++;
      

    }
    ros::spinOnce();
    //usleep(1000);
  }

  ec_close();
  log_file.close();

}

int ServoOnGetCtrlWrd(uint16_t status_word, uint16_t *control_word)
{
  int _enable = 0;

  if(bit_is_clear(status_word, STATUSWORD_OPERATION_ENABLE_BIT))          // Not Enabled Yet
  {
    if(bit_is_clear(status_word, STATUSWORD_SWITCHED_ON_BIT))            // Not Switched On Yet
    {
      if(bit_is_clear(status_word, STATUSWORD_READY_TO_SWITCH_ON_BIT))    // Not Ready to Switch On Yet
      {
        if(bit_is_clear(status_word, STATUSWORD_FAULT_BIT))               // Not Fault Yet
        {
          (*control_word) = 0x80;                                         // Fault Reset command
        }
        else
        {
          (*control_word) = 0x06;                                         // Shutdown command
        }
      }
      else
      {
        (*control_word) = 0x07;                                           // Switch On command
      }
    }
    else
    {
      (*control_word) = 0x0F;                                             // Enable command
      _enable = 1;
    }
  }
  else                                                                    // has been Enabled
  {
    (*control_word) = 0x0F;                                               // maintain Operation state
    _enable = 1;
  }

  return _enable;
}

void WriteLog(std::ofstream &log_file, float32 data[1][6])
{
  log_file << fixed << setprecision(3) << setw(10) << GetMillis() << " |"
           << setw(10) << data[0][0] << ", " << setw(10) << data[0][1] << ", " << setw(10) << data[0][2] << " |"
           << setw(10) << data[0][3] << ", " << setw(10) << data[0][4] << ", " << setw(10) << data[0][5] << " |"
           << setw(10) << data[1][0] << ", " << setw(10) << data[1][1] << ", " << setw(10) << data[1][2] << " |"
           << setw(10) << data[1][3] << ", " << setw(10) << data[1][4] << ", " << setw(10) << data[1][5] << " |"
           << endl;
}


int CountFiles(path the_path)
{
  int cnt = count_if(
      directory_iterator(the_path),
      directory_iterator(),
      static_cast<bool (*)(const path &)>(is_regular_file));
  return cnt;
}
