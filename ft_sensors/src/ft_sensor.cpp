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

#define EC_TIMEOUTMON 500

using namespace std;
using namespace GetTime;
using namespace boost::filesystem;
using namespace boost::lambda;

const int storage_size = 500;
double data_storage[1][6][storage_size];
double data_offset[1][6];

char IOmap[4096];
boolean needlf;
std_msgs::Bool ft_init_done;
uint8 currentgroup = 0;

void WriteLog(std::ofstream &log_file, float32 data[1][6]);
void SendCommand(uint8_t *data, uint16_t *buf, int buf_length);
void PrintValues_2(uint8_t *data);
void PrintValues_3(uint8_t *data);
void GetSensorValue(float32 new_data[6], uint8_t *data);
void SetRosMsg(ft_sensor_msgs::ForceTorque *ft_msg, float32 data[][6]);
double Kalman(double *input, double *data, int length);
OSAL_THREAD_FUNC ec_check(void *ptr);
void SetupFT(int addr);
void InitFT(int addr);
int CountFiles(path the_path);

void InitCallback(const std_msgs::Bool msg)
{
  if (msg.data == TRUE)
  {
    ft_init_done.data = FALSE;
    InitFT(1);
    ft_init_done.data = TRUE;
  }
}

int main(int argc, char *argv[])
{
  std::string lan_port_name_;
  lan_port_name_ = "enp2s0";
  int n = lan_port_name_.length();

  char ifname[n + 1];

  strcpy(ifname, lan_port_name_.c_str());

  std::cout << "[FT Sensor] lan port name :" << ifname << std::endl;

  std::string ss_user_name;
  ss_user_name = "heroehs";
 
  string file_string = "/home/"+ss_user_name+"/log_ft/";

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

  ros::init(argc, argv, "ft_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_init = nh.subscribe("/ft_init", 1, InitCallback);
  ros::Publisher ec_data = nh.advertise<ft_sensor_msgs::ForceTorque>("/force_torque_data", 1);
  ros::Publisher init_done = nh.advertise<std_msgs::Bool>("/ft_init_done", 1);

  ft_sensor_msgs::ForceTorque ft_msg;

  if (ec_init(ifname))
  {
    if (ec_config_init(FALSE) > 0)
    {
      ec_config_map(&IOmap);
      ec_configdc();

      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_writestate(0);

      int chk = 40;
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      } while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);

      log_file << "      time |"
               << "     Fx_l ,      Fy_l ,      Fz_l  |     Tx_l ,      Ty_l ,      Tz_l  |"
               << "     Fx_r ,      Fy_r ,      Fz_r  |     Tx_r ,      Ty_r ,      Tz_r  |"
               << endl;

      while (ec_slave[0].state == EC_STATE_OPERATIONAL && ros::ok())
      {
        if (!needlf)
        {
          SetupFT(1);
          needlf = TRUE;
        }

        // update ethercat slave....
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        float data[1][6];
        for (int n = 0; n < ec_slavecount; n++)
        {
          //PrintValues_2(ec_slave[n+1].inputs);
          GetSensorValue(data[n], ec_slave[n + 1].inputs);
          //PrintValues_3(ec_slave[n + 1].inputs);
          // for (int i = 0; i < 6; i++)
          // {
          //   Kalman(data[n] + i, data_storage[n][i], storage_size);
          //   data[n][i] -= data_offset[n][i];
          // }

          /*
             printf("\n");
             printf("Force  : Fx= %10.2f, Fy= %10.2f, Fz= %10.2f \n",
             data[n][0], data[n][1], data[n][2]);
             printf("Torque : Tx= %10.2f, Ty= %10.2f, Tz= %10.2f \n",
             data[n][3], data[n][4], data[n][5]);
           */
        }

        static double pub_timer = GetMillis();
        static double log_timer = GetMillis();
        static int count = 0;
        count++;
        for (int i = 0; i < 1; i++)
          for (int j = 0; j < 6; j++)
            data_sum[i][j] += data[i][j];

        double cur_time = GetMillis();
        if (pub_timer + 0.008f < cur_time)
        {
          pub_timer += 0.008f;

          for (int i = 0; i < 1; i++)
          {
            for (int j = 0; j < 6; j++)
            {
              data[i][j] = data_sum[i][j] / count;
              data_sum[i][j] = 0;
            }
          }

          count = 0;

          if (log_timer + 0.1f < cur_time)
          {
            log_timer = 0.1f;
            WriteLog(log_file, data);
          }

          SetRosMsg(&ft_msg, data);
          ec_data.publish(ft_msg);

          init_done.publish(ft_init_done);
          ros::spinOnce();
        }

        usleep(1000);
      }

      printf("Not all slaves reached operational state.\n");
      ec_readstate();
      for (int i = 1; i <= ec_slavecount; i++)
      {
        if (ec_slave[i].state != EC_STATE_OPERATIONAL)
        {
          printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                 i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
      }
      printf("\nRequest init state for all slaves\n");
      ec_slave[0].state = EC_STATE_INIT;
      /* request INIT state for all slaves */
      ec_writestate(0);
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

  // printf("fx : %f\n", force_torque_data[0]);
  // printf("fy : %f\n", force_torque_data[1]);
  // printf("fz : %f\n", force_torque_data[2]);
  // printf("tx : %f\n", force_torque_data[3]);
  // printf("tx : %f\n", force_torque_data[4]);
  // printf("tx : %f\n", force_torque_data[5]);

  /*
     printf("\n");
     printf("Raw_Force  : Fx= %10.2f, Fy= %10.2f, Fz= %10.2f \n",
     force[0], force[1], force[2]);
     printf("Raw_Torque : Tx= %10.2f, Ty= %10.2f, Tz= %10.2f \n",
     torque[0], torque[1], torque[2]);
   */
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

double Kalman(double *input, double *data, int length)
{
  double result = (*input / length);

  for (int i = 1; i < length; i++)
  {
    result += (data[i] / length);
    data[i - 1] = data[i];
  }
  data[length - 1] = *input;

  *input = result;
  return result;
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

void PrintValues_3(uint8_t *data)
{
  uint8_t df_data[50];
  for (int i = 0; i < 50; i++)
    df_data[i] = *data++;

  int16_t raw_force[3];
  // Raw_Fx (2 Bytes)
  raw_force[0] = (int16_t)(*data | (*(data + 1) << 8));
  data += 2;
  // Raw_Fy (2 Bytes)
  raw_force[1] = (int16_t)(*data | (*(data + 1) << 8));
  data += 2;
  // Raw_Fz (2 Bytes)
  raw_force[2] = (int16_t)(*data | (*(data + 1) << 8));
  data += 2;

  int16_t raw_torque[3];
  // Raw_Tx (2 Bytes)
  raw_torque[0] = (int16_t)(*data | (*(data + 1) << 8));
  data += 2;
  // Raw_Ty (2 Bytes)
  raw_torque[1] = (int16_t)(*data | (*(data + 1) << 8));
  data += 2;
  // Raw_Tz (2 Bytes)
  raw_torque[2] = (int16_t)(*data | (*(data + 1) << 8));
  data += 2;

  // OverloadStatus (1 Byte)
  uint8_t overload_status = *data++;

  // ErrorFlag (1 Byte)
  uint8_t error_flag = *data++;

  double ft[6];
  for (int i = 0; i < 3; i++)
  {
    ft[i] = (double)raw_force[i] / 50.0;
    ft[i + 3] = (double)raw_torque[i] / 1000.0;
  }

  //printf("DF_data_1~50 : ");
  system("clear");
  printf("\r data %d : %3d", 0, df_data[0]);
  printf("\n data %d : %3d", 1, df_data[1]);
  printf("\n data %d : %3d", 2, df_data[2]);
  printf("\n data %d : %3d", 3, df_data[3]);
  printf("\n data %d : %3d", 4, df_data[4]);
  printf("\n data %d : %3d", 5, df_data[5]);
  printf("\n data %d : %3d", 6, df_data[6]);
  printf("\n data %d : %3d", 7, df_data[7]);
  printf("\n data %d : %3d", 8, df_data[8]);
  printf("\n data %d : %3d", 9, df_data[9]);
  printf("\n data %d : %3d", 10, df_data[10]);
  printf("\n data %d : %3d", 11, df_data[11]);
  printf("\n data %d : %3d", 12, df_data[12]);
  printf("\n data %d : %3d", 13, df_data[13]);
  printf("\n data %d : %3d", 14, df_data[14]);
  printf("\n data %d : %3d", 15, df_data[15]);
  printf("\n data %d : %3d", 16, df_data[16]);
  printf("\n data %d : %3d", 17, df_data[17]);
  printf("\n data %d : %3d", 18, df_data[18]);
  printf("\n data %d : %3d", 19, df_data[19]);
  printf("\n data %d : %3d", 20, df_data[20]);
  printf("\n data %d : %3d", 21, df_data[21]);
  printf("\n data %d : %3d", 22, df_data[22]);
  printf("\n data %d : %3d", 23, df_data[23]);
  printf("\n data %d : %3d", 24, df_data[24]);
  printf("\n data %d : %3d", 25, df_data[25]);
  printf("\n data %d : %3d", 26, df_data[26]);
  printf("\n data %d : %3d", 27, df_data[27]);
  printf("\n data %d : %3d", 28, df_data[28]);
  printf("\n data %d : %3d", 29, df_data[29]);
  printf("\n data %d : %3d", 30, df_data[30]);
  printf("\n data %d : %3d", 31, df_data[31]);
  printf("\n data %d : %3d", 32, df_data[32]);
  printf("\n data %d : %3d", 33, df_data[33]);
  printf("\n data %d : %3d", 34, df_data[34]);
  printf("\n data %d : %3d", 35, df_data[35]);
  printf("\n data %d : %3d", 36, df_data[36]);
  printf("\n data %d : %3d", 37, df_data[37]);
  printf("\n data %d : %3d", 38, df_data[38]);
  printf("\n data %d : %3d", 39, df_data[39]);
  printf("\n data %d : %3d", 40, df_data[40]);
  printf("\n data %d : %3d", 41, df_data[41]);
  printf("\n data %d : %3d", 42, df_data[42]);
  printf("\n data %d : %3d", 43, df_data[43]);
  printf("\n data %d : %3d", 44, df_data[44]);
  printf("\n data %d : %3d", 45, df_data[45]);

  for (int i = 0; i < 16; i++)
  {
    //    printf(" %3d ", df_data[i]);
  }
      
  // printf("\n");
  // printf("Raw_Force  : Fx= %4d, Fy= %4d, Fz= %4d \n", raw_force[0], raw_force[1], raw_force[2]);
  // printf("Raw_Torque : Tx= %4d, Ty= %4d, Tz= %4d \n", raw_torque[0], raw_torque[1], raw_torque[2]);
  // printf("Raw_Force  : Fx= %4f, Fy= %4f, Fz= %4f \n", ft[0], ft[1], ft[2]);
  // printf("Raw_Torque : Tx= %4f, Ty= %4f, Tz= %4f \n", ft[3], ft[4], ft[5]);
  // printf("OverloadStatus : %2d \n", overload_status);
  // printf("ErrorFlag  : %2d \n\n\n", error_flag);
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
