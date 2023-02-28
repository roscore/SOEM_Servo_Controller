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
#include <bits/stdc++.h>

#include "ft_sensor_msgs/ForceTorque.h"
#include "ethercat.h"
#include "get_time.h"

#include "pdo_def.h"
#include "servo_def.h"

#define EC_TIMEOUTMON 2000
#define NUMOFSERVO_DRIVE 1

#define x_USE_DC

MAXPOS_Drive_pt	maxpos_drive_pt[NUMOFSERVO_DRIVE];

using namespace std;
using namespace GetTime;
using namespace boost::filesystem;
using namespace boost::lambda;

const int storage_size = 500;
double data_storage[1][6][storage_size];
double data_offset[1][6];

char IOmap[4096];
boolean needlf;

uint8 currentgroup = 0;

int ServoOnGetCtrlWrd(uint16_t status_word, uint16_t *control_word);

void WriteLog(std::ofstream &log_file, float32 data[1][6]);
void SendCommand(uint8_t *data, uint16_t *buf, int buf_length);
void PrintValues_2(uint8_t *data);
void GetSensorValue(float32 new_data[6], uint8_t *data);
void SetRosMsg(ft_sensor_msgs::ForceTorque *ft_msg, float32 data[][6]);

OSAL_THREAD_FUNC ec_check(void *ptr);
void SetupFT(int addr);
void InitFT(int addr);
int SetupMotor(int addr);
void InitMotor(int addr);
int CountFiles(path the_path);

int main(int argc, char *argv[])
{
  std::string lan_port_name_;
  lan_port_name_ = "enp2s0";
  int n = lan_port_name_.length();

  char ifname[n + 1];

  strcpy(ifname, lan_port_name_.c_str());

  std::cout << "[HERoHES Linear Actuator] EtherCAT port name :" << ifname << std::endl;

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

  double data_sum[1][6];

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < storage_size; j++)
    {
      data_storage[0][i][j] = 0;
      // data_storage[1][i][j] = 0;
    }
    data_sum[0][i] = 0;
    // data_sum[1][i] = 0;
  }

  needlf = FALSE;

  ros::init(argc, argv, "linear_actuator_communication_node");
  ros::NodeHandle nh;
  ros::Publisher ec_data = nh.advertise<ft_sensor_msgs::ForceTorque>("/linear_actuator_data", 1);

  //variables for pdo re-mapping (sdo write)
  int os;
  uint32_t ob;
  uint16_t ob2;
  uint8_t  ob3;

	int i, oloop, iloop, k, wkc_count;

  int expectedWKC;

  ft_sensor_msgs::ForceTorque ft_msg;

  printf("Starting EtherCAT master...\n");

  if (ec_init(ifname))
  {
    if (ec_config_init(FALSE) > 0)
    {
      printf("%d slaves found and configured.\n",ec_slavecount);

      for(int iter = 0; iter < NUMOFSERVO_DRIVE; ++iter)
      {
        if (ec_slavecount >= 1) //change name for other drives
        {
          printf("Re mapping for HERoEHS Servo...\n");
          os = sizeof(ob2); ob2 = 0x1600;	//RxPDO, check ESI file
          //printf("Addr : %d Size : %d \n", ob2, os);
          
          //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
          //wkc_count = ec_SDOwrite(iter + 1, 0x1c12, 01, TRUE, os, &ob2, EC_TIMEOUTRXM);	//change slave position (k+1) if needed
          wkc_count = ec_SDOwrite(iter + 1, 0x1c12, 00, FALSE, os, &ob2, EC_TIMEOUTRXM);	//change slave position (k+1) if needed
          //printf("Set wkc || count : %d \n", wkc_count);

          if (wkc_count == 0)
          {
            printf("RxPDO assignment error\n");
          }

          os = sizeof(ob2); ob2 = 0x1a00;	//TxPDO, check ESI file
          //0x1c13 is Index of Sync Manager 3 PDO Assignment (input TxPDO), CA (Complete Access) must be TRUE
          //wkc_count = ec_SDOwrite(iter + 1, 0x1c13, 01, TRUE, os, &ob2, EC_TIMEOUTRXM); //change slave position (k+1) if needed
          wkc_count = ec_SDOwrite(iter + 1, 0x1c13, 00, FALSE, os, &ob2, EC_TIMEOUTRXM); //change slave position (k+1) if needed

          if (wkc_count == 0)
          {
            printf("TxPDO assignment error\n");
          }
        }
      }

#ifdef _WRITE_MODEOP_SDO
			uint8 modeOp = OP_MODE_CYCLIC_SYNC_POSITION;
			// write mode of operation by SDO
			for (i = 0; i < NUMOFSERVO_DRIVE; ++i)
			{
				os = sizeof(modeOp);
				ec_SDOwrite(i + 1, 0x6060, 0x0, FALSE, os, &modeOp, EC_TIMEOUTRXM); //set mode of operation by SDO
			}
#endif

			ec_config_map(&IOmap);

#ifdef _USE_DC
			ec_configdc();
#endif
			printf("Slaves mapped, state to SAFE_OP.\n");
			/* wait for all slaves to reach SAFE_OP state */
			//ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      ec_statecheck(0, 0x08, EC_TIMEOUTSTATE * 4);
#ifdef _USE_DC
			//FOR DC----
			/* configure DC options for every DC capable slave found in the list */
			printf("DC capable : %d\n", ec_configdc());
			//---------------
#endif


			oloop = ec_slave[0].Obytes;
			if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
			// if (oloop > 8) oloop = 8;
			iloop = ec_slave[0].Ibytes;
			if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
			// if (iloop > 8) iloop = 8;

			printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

			printf("Request operational state for all slaves\n");
			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			printf("Calculated workcounter %d\n", expectedWKC);
			// ec_slave[0].state = EC_STATE_OPERATIONAL;
      ec_slave[0].state = 0x08;

			/* send one valid process data to make outputs in slaves happy*/

			ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);

      ec_writestate(0);
			ec_statecheck(0, 0x08, EC_TIMEOUTSTATE); //wait for OP

			// ec_receive_processdata(EC_TIMEOUTRET);
			// /* request OP state for all slaves */
			// ec_writestate(0);

      printf("Operation Mode : %d \n", ec_slave[0].state);
    
			if (ec_slave[0].state == 0x08)
			{
				printf("Operational state reached for all slaves.\n");
				wkc_count = 0;

				for (k = 0; k < NUMOFSERVO_DRIVE; ++k)
				{
					maxpos_drive_pt[k].ptOutParam = (MAXPOS_DRIVE_RxPDO_t*)ec_slave[k + 1].outputs;
					maxpos_drive_pt[k].ptInParam = (MAXPOS_DRIVE_TxPDO_t*)ec_slave[k + 1].inputs;
					maxpos_drive_pt[k].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_POSITION;
          maxpos_drive_pt[k].ptOutParam->ControlWord = 0x0f;
          printf("ControlWord : %d\n", maxpos_drive_pt[k].ptOutParam->ControlWord);

          // int result__ = ec_SDOwrite(1, 0x6040, 00, FALSE, sizeof(maxpos_drive_pt[k].ptOutParam->ControlWord), &(maxpos_drive_pt[k].ptOutParam->ControlWord), EC_TIMEOUTRXM);	//change slave position (k+1) if needed

          printf("WKC result : %d\n", result__);

          // int point_input = sizeof(maxpos_drive_pt[k].ptInParam->StatusWord);
          // int actual_state = ec_SDOread(1, 0x6041, 00, FALSE, &point_input, &maxpos_drive_pt[k].ptInParam->StatusWord, EC_TIMEOUTRXM);

          // printf("Actual ControlWord : %d\n", actual_state);
        }
			}
			else
			{
				printf("Not all slaves reached operational state.\n");
				ec_readstate();
				for (i = 1; i <= ec_slavecount; i++)
				{
					if (ec_slave[i].state != EC_STATE_OPERATIONAL)
					{
						printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
							i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}
				for (i = 0; i < NUMOFSERVO_DRIVE; ++i)
					ec_dcsync01(i + 1, FALSE, 0, 0, 0); // SYNC0,1 
			}


      // ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      // ec_slave[0].state = EC_STATE_OPERATIONAL;
      // ec_send_processdata();
      // ec_receive_processdata(EC_TIMEOUTRET);
      // ec_writestate(0);

      // int chk = 40;
      // do
      // {
      //   ec_send_processdata();
      //   ec_receive_processdata(EC_TIMEOUTRET);
      //   ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      // } while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);

      // log_file << "      time |"
      //          << "     Fx_l ,      Fy_l ,      Fz_l  |     Tx_l ,      Ty_l ,      Tz_l  |"
      //          << "     Fx_r ,      Fy_r ,      Fz_r  |     Tx_r ,      Ty_r ,      Tz_r  |"
      //          << endl;

      while (ec_slave[0].state == EC_STATE_OPERATIONAL && ros::ok())
      {
        printf("in loop \n");
        ros::spinOnce();
        usleep(1000);
      }

      // printf("Not all slaves reached operational state.\n");

      // ec_readstate();

      for (int i = 1; i <= ec_slavecount; i++)
      {
        
        if (ec_slave[i].state != EC_STATE_OPERATIONAL)
        {
          printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                 i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
        
      }

      //printf("\nRequest init state for all slaves\n");
      //ec_slave[0].state = EC_STATE_INIT;
      /* request INIT state for all slaves */
      //ec_writestate(0);
    }
    else
    {
      printf("No slaves found!\n");
    }
    ec_close();
    log_file.close();
  }
  else
  {
    printf("No socket connection on %s\nExcecute as root\n", ifname);
  }
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

void SendCommand(uint8_t *data, uint16_t *buf, int buf_length)
{
  for (int i = 0; i < buf_length; i++)
  {
    *data++ = (buf[buf_length - i - 1] >> 0) & 0xFF;
    *data++ = (buf[buf_length - i - 1] >> 8) & 0xFF;
  }
  ec_send_processdata();
}

void GetSensorValue(float32 new_data[6], uint8_t *data)
{
  memcpy(new_data, ec_slave[1].inputs, 24);
}

void SetRosMsg(ft_sensor_msgs::ForceTorque *ft_msg, float32 data[][6])
{
  ft_msg->force_x_raw = data[0][0];
  ft_msg->force_y_raw = data[0][1];
  ft_msg->force_z_raw = data[0][2];
  ft_msg->torque_x_raw = data[0][3];
  ft_msg->torque_y_raw = data[0][4];
  ft_msg->torque_z_raw = data[0][5];
}

void PrintValues_2(uint8_t *data)
{
  // CanRx1_id (2 Bytes)
  uint16_t can_rx1_id = *data | (*(data + 1) >> 8);
  data += 2;

  // CanRx1_len (2 Bytes)
  uint16_t can_rx1_len = *data | (*(data + 1) >> 8);
  data += 2;

  // CanRx1_data_d1 ~ d8 (1 Byte * 8)
  uint8_t can_rx1_data[8];
  for (int i = 0; i < 8; i++)
    can_rx1_data[i] = *data++;

  // CanRx2_id (2 Bytes)
  uint16_t can_rx2_id = *data | (*(data + 1) >> 8);
  data += 2;

  // CanRx2_len (2 Bytes)
  uint16_t can_rx2_len = *data | (*(data + 1) >> 8);
  data += 2;

  // CanRx2_data_d1 ~ d8 (1 Byte * 8)
  uint8_t can_rx2_data[8];
  for (int i = 0; i < 8; i++)
    can_rx2_data[i] = *data++;

  int16_t raw_force[3];
  // Raw_Fx (2 Bytes)
  raw_force[0] = (int16_t)(*data | (*(data + 1) >> 8));
  data += 2;
  // Raw_Fy (2 Bytes)
  raw_force[1] = (int16_t)(*data | (*(data + 1) >> 8));
  data += 2;
  // Raw_Fz (2 Bytes)
  raw_force[2] = (int16_t)(*data | (*(data + 1) >> 8));
  data += 2;

  int16_t raw_torque[3];
  // Raw_Tx (2 Bytes)
  raw_torque[0] = (int16_t)(*data | (*(data + 1) >> 8));
  data += 2;
  // Raw_Ty (2 Bytes)
  raw_torque[1] = (int16_t)(*data | (*(data + 1) >> 8));
  data += 2;
  // Raw_Tz (2 Bytes)
  raw_torque[2] = (int16_t)(*data | (*(data + 1) >> 8));
  data += 2;

  // OverloadStatus (1 Byte)
  uint8_t overload_status = *data++;

  // ErrorFlag (1 Byte)
  uint8_t error_flag = *data++;

  printf("CanRx1_id  : %d \n", can_rx1_id);
  printf("CanRx1_len : %d \n", can_rx1_len);
  printf("CanRx1_data_d1~d8 : ");
  for (int i = 0; i < 8; i++)
    printf("%3d ", can_rx1_data[i]);
  printf("\n");
  printf("CanRx2_id  : %d \n", can_rx2_id);
  printf("CanRx2_len : %d \n", can_rx2_len);
  printf("CanRx2_data_d1~d8 : ");
  for (int i = 0; i < 8; i++)
    printf("%3d ", can_rx2_data[i]);
  printf("\n");
  printf("Raw_Force  : Fx= %4d, Fy= %4d, Fz= %4d \n", raw_force[0], raw_force[1], raw_force[2]);
  printf("Raw_Torque : Tx= %4d, Ty= %4d, Tz= %4d \n", raw_torque[0], raw_torque[1], raw_torque[2]);
  printf("OverloadStatus : %2d \n", overload_status);
  printf("ErrorFlag  : %2d \n\n\n", error_flag);
}

void InitMotor(int addr)
{
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
}

int SetupMotor(int addr)
{
  int retval;

  uint16 u16val;

  retval = 0;

  /* Map PDO assignment via Complete Access */

  InitMotor(addr);
}

void SetupFT(int addr)
{
  int buf_size = 4;
  uint16_t buf[buf_size];

  //stop command (for setup the f/t sensor)
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x0C;
  //SendCommand(ec_slave[addr].outputs, buf, buf_size);

  //set cutoff frequency to 150Hz
  buf[0] = 0x00;
  buf[1] = 0x04;
  buf[2] = 0x01;
  buf[3] = 0x08;
  //SendCommand(ec_slave[addr].outputs, buf, buf_size);
  //ec_receive_processdata(EC_TIMEOUTRET);

  //start command
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x0B;
  //SendCommand(ec_slave[addr].outputs, buf, buf_size);

  InitFT(addr);
}

void InitFT(int addr)
{
  for (int j = 0; j < 6; j++)
  {
    data_offset[addr - 1][j] = 0;
  }

  float data[6];
  for (int i = 0; i < storage_size; i++)
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    GetSensorValue(data, ec_slave[addr].inputs);
    for (int j = 0; j < 6; j++)
    {
      data_storage[addr - 1][j][i] = data[j];
      data_offset[addr - 1][j] += data[j];
    }
  }

  for (int j = 0; j < 6; j++)
  {
    data_offset[addr - 1][j] /= storage_size;
  }
}

int CountFiles(path the_path)
{
  int cnt = count_if(
      directory_iterator(the_path),
      directory_iterator(),
      static_cast<bool (*)(const path &)>(is_regular_file));
  return cnt;
}
