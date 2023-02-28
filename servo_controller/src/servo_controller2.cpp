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

#define NSEC_PER_SEC 			1000000000

#define x_USE_DC

MAXPOS_Drive_pt	maxpos_drive_pt[NUMOFSERVO_DRIVE];

using namespace std;
using namespace GetTime;
using namespace boost::filesystem;
using namespace boost::lambda;

unsigned int cycle_ns = 1000000; /* 1 ms */

char IOmap[4096];
//char IOmap[1024];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

long ethercat_time_send, ethercat_time_read=0;
long ethercat_time=0, worst_time=0;

std::string lan_port_name_;
lan_port_name_ = "enp2s0";
int n = lan_port_name_.length();
char ecat_ifname[n + 1];
strcpy(ecat_ifname, lan_port_name_.c_str());

std::cout << "[HERoHES Linear Actuator] EtherCAT port name :" << ifname << std::endl;

int run=1;
int sys_ready=0;

int started[NUMOFSERVO_DRIVE]={0}, ServoState=0;
uint8 servo_ready=0, servo_prestate=0;
int32_t zeropos[NUMOFSERVO_DRIVE]={0};
double gt=0;

double sine_amp=11500, f=0.2, period;
int recv_fail_cnt=0;

//variables for pdo re-mapping (sdo write)
int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;

boolean ecat_init(void)
{
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
			 
			 //PDO re-mapping****************************************************************************************************
			 for (k=0; k<NUMOFSERVO_DRIVE; ++k)
			 {
				 //if (( ec_slavecount >= 1 ) && (strcmp(ec_slave[k+1].name,"MAXPOS") == 0)) //change name for other drives
				 if ( ec_slavecount >= 1 ) //change name for other drives
				 {
					  printf("Re mapping for MAXPOS...\n");
            os=sizeof(ob2); ob2 = 0x1600;	//RxPDO, check MAXPOS ESI
            //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
            wkc_count=ec_SDOwrite(k+1, 0x1c12,01,TRUE,os, &ob2,EC_TIMEOUTRXM);	//change slave position (k+1) if needed

            if (wkc_count==0)
            {
              printf("RxPDO assignment error\n");
              return FALSE;
            }
					
            os=sizeof(ob2); ob2 = 0x1a00;	//TxPDO, check MAXPOS ESI
            //0x1c13 is Index of Sync Manager 3 PDO Assignment (input TxPDO), CA (Complete Access) must be TRUE
            wkc_count=ec_SDOwrite(k+1, 0x1c13,01,TRUE,os, &ob2,EC_TIMEOUTRXM); //change slave position (k+1) if needed
            if (wkc_count==0)
            {
              printf("TxPDO assignment error\n");
              return FALSE;
            }
				 }
			 }
			 //PDO re-mapping*************************rt_prt_printfrintf***************************************************************************

#ifdef _WRITE_MODEOP_SDO
			 uint8 modeOp=OP_MODE_CYCLIC_SYNC_POSITION;
			 // write mode of operation by SDO
			  for (i=0; i<NUMOFSERVO_DRIVE; ++i)
			  {
				  os=sizeof(modeOp);
				  ec_SDOwrite(i+1, 0x6060, 0x0, FALSE, os, &modeOp, EC_TIMEOUTRXM); //set mode of operation by SDO
			  }
#endif

			 ec_config_map(&IOmap);
			 
#ifdef _USE_DC
			 ec_configdc();
#endif
			 printf("Slaves mapped, state to SAFE_OP.\n");
			 /* wait for all slaves to reach SAFE_OP state */
			 ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
#ifdef _USE_DC
			 //FOR DC----
			 /* configure DC options for every DC capable slave found in the list */
			 printf("DC capable : %d\n",ec_configdc());
			 //---------------
#endif
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

			 ec_writestate(0);
			 ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP

			if (ec_slave[0].state == EC_STATE_OPERATIONAL )
			{
				printf("Operational state reached for all slaves.\n");
				wkc_count = 0;

				for (k=0; k<NUMOFSERVO_DRIVE; ++k)
				{
					maxpos_drive_pt[k].ptOutParam=(MAXPOS_DRIVE_RxPDO_t*)  		ec_slave[k+1].outputs;
					maxpos_drive_pt[k].ptInParam= (MAXPOS_DRIVE_TxPDO_t*)  		ec_slave[k+1].inputs;
					maxpos_drive_pt[k].ptOutParam->ModeOfOperation=OP_MODE_CYCLIC_SYNC_POSITION;
				}
				inOP = TRUE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
				for (i=0; i<NUMOFSERVO_DRIVE; ++i)
					ec_dcsync01(i+1, FALSE, 0, 0, 0); // SYNC0,1 
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


// main task
void demo_run(void *arg)
{
	RTIME now, previous;
	unsigned long ready_cnt=0;
	uint16_t controlword=0;
	int ival=0, i;

	if (ecat_init()==FALSE)
	{
		run =0;
		printf("fail\n");
		return;	//all initialization stuffs here
	}	
	rt_task_sleep(1e6);
	
#ifdef _USE_DC
	//for dc computation
	long  long toff;
	long long cur_DCtime=0, max_DCtime=0;
	unsigned long long  cur_dc32=0, pre_dc32=0;
	int32_t shift_time=380000; //dc event shifted compared to master reference clock
	long long  diff_dc32;

	for (i=0; i<NUMOFSERVO_DRIVE; ++i)
		ec_dcsync0(1+i, TRUE, cycle_ns, 0); // SYNC0,1 on slave 1

	RTIME cycletime=cycle_ns, cur_time=0;
	RTIME cur_cycle_cnt=0, cycle_time;
	RTIME remain_time, dc_remain_time;
	toff = 0;
	
	RTIME rt_ts;
	//get DC time for first time
	ec_send_processdata();
	
	cur_time=rt_timer_read();			//get current master time
	cur_cycle_cnt=cur_time/cycle_ns;	//calcualte number of cycles has passed
	cycle_time=cur_cycle_cnt*cycle_ns;	
	remain_time = cur_time%cycle_ns;	//remain time to next cycle, test only
	
	printf("cycle_cnt=%lld\n", cur_cycle_cnt);
	printf("remain_time=%lld\n", remain_time);
	
	wkc = ec_receive_processdata(EC_TIMEOUTRET); 	//get reference DC time
	cur_dc32= (uint32_t) (ec_DCtime & 0xffffffff);	//only consider first 32-bit
	dc_remain_time=cur_dc32%cycletime;				//remain time to next cycle of REF clock, update to master
	rt_ts=cycle_time+dc_remain_time;					//update master time to REF clock
	
	printf("dc remain_time=%lld\n", dc_remain_time);
	
	rt_task_sleep_until(rt_ts);

#else  //nonDC
	ec_send_processdata();
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
#endif
	while (run)
	{
	   //wait for next cycle	
#ifdef _USE_DC		   
		rt_ts+=(RTIME) (cycle_ns + toff);
		rt_task_sleep_until(rt_ts);
#else
		rt_task_wait_period(NULL); 
#endif
	   previous = rt_timer_read();

	   ec_send_processdata();
	   wkc = ec_receive_processdata(EC_TIMEOUTRET);
	   if (wkc<3*(NUMOFSERVO_DRIVE)) 
		   recv_fail_cnt++;
	   now = rt_timer_read();
	   ethercat_time = (long) (now - previous);

#ifdef _USE_DC	   
	   cur_dc32= (uint32_t) (ec_DCtime & 0xffffffff); 	//use 32-bit only
	   if (cur_dc32>pre_dc32)							//normal case
		   diff_dc32=cur_dc32-pre_dc32;
	   else												//32-bit data overflow
		   diff_dc32=(0xffffffff-pre_dc32)+cur_dc32;
	   pre_dc32=cur_dc32;
	   cur_DCtime+=diff_dc32;

	   toff=dc_pi_sync(cur_DCtime, cycletime, shift_time);
	 
	   if (cur_DCtime>max_DCtime) max_DCtime=cur_DCtime;
#endif	
	  
	   //servo-on	
	   for (i=0; i<NUMOFSERVO_DRIVE; ++i)
	   {
		   controlword=0;
		   started[i]=ServoOn_GetCtrlWrd(maxpos_drive_pt[i].ptInParam->StatusWord, &controlword);
		   maxpos_drive_pt[i].ptOutParam->ControlWord=controlword;
		   if (started[i]) ServoState |= (1<<i);
	   }

	   if (ServoState == (1<<NUMOFSERVO_DRIVE)-1) //all servos are in ON state
	   {
		   if (servo_ready==0) 
			servo_ready=1;
	   }
		if (servo_ready) ready_cnt++;
		if (ready_cnt>=3000) //wait for 3s after servo-on
		{
			ready_cnt=10000;
			sys_ready=1;
		}

		if (sys_ready)
		{
			ival=(int) (sine_amp*(sin(PI2*f*gt)));
			for (i=0; i<NUMOFSERVO_DRIVE; ++i)
			{
				if (i==0)
					maxpos_drive_pt[i].ptOutParam->TargetPosition=ival + zeropos[i];
				else
					maxpos_drive_pt[i].ptOutParam->TargetPosition=-ival + zeropos[i];
			}
			gt+=period;
		}
		else
		{
			for (i=0; i<NUMOFSERVO_DRIVE; ++i)
			{
				zeropos[i]=maxpos_drive_pt[i].ptInParam->PositionActualValue;
				maxpos_drive_pt[i].ptOutParam->TargetPosition=zeropos[i];
			}
		}
		
		
	   if (sys_ready)
		if (worst_time<ethercat_time) worst_time=ethercat_time;
	}

	rt_task_sleep(cycle_ns);
#ifdef _USE_DC
	for (i=0; i<NUMOFSERVO_DRIVE; ++i)
		ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1 on slave 1
#endif

	//Servo OFF
	for (i=0; i<NUMOFSERVO_DRIVE; ++i)
	{
		maxpos_drive_pt[i].ptOutParam->ControlWord=0; //Servo OFF (Disable voltage, transition#9)
	}
	ec_send_processdata();
	wkc = ec_receive_processdata(EC_TIMEOUTRET);
	
	rt_task_sleep(cycle_ns);
	
	printf("End simple test, close socket\n");
	/* stop SOEM, close socket */
	 printf("Request safe operational state for all slaves\n");
	 ec_slave[0].state = EC_STATE_SAFE_OP;
	 /* request SAFE_OP state for all slaves */
	 ec_writestate(0);
	 /* wait for all slaves to reach state */
	 ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
	 ec_slave[0].state = EC_STATE_PRE_OP;
	 /* request SAFE_OP state for all slaves */
	 ec_writestate(0);
	 /* wait for all slaves to reach state */
	 ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

	ec_close();
}

void print_run(void *arg)
{
	int i;
	unsigned long itime=0;
	long stick=0;
	rt_task_set_periodic(NULL, TM_NOW, 1e8);

	while (run)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle
		if (inOP==TRUE)
		{
			if (!sys_ready)
			{
				if (stick==0)
					printf("waiting for system ready...\n");
				if (stick%10==0)
					printf("%i\n", stick/10);
				stick++;
			}
			else
			{
				itime++;
				printf("Time=%06d.%01d, \e[32;1m fail=%ld\e[0m, ecat_T=%ld, maxT=%ld\n", itime/10, itime%10, recv_fail_cnt,  ethercat_time/1000, worst_time/1000);
				for(i=0; i<NUMOFSERVO_DRIVE; ++i)
				{
					printf("MAXPOS_Drive#%i\n", i+1);
					printf("Status word = 0x%X\n", maxpos_drive_pt[i].ptInParam->StatusWord);
					printf("Actual Position = %i / %i\n", maxpos_drive_pt[i].ptInParam->PositionActualValue, maxpos_drive_pt[i].ptOutParam->TargetPosition);
					printf("\n");
				}
				printf("\n");

			}

		}
	}
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "linear_actuator_communication_node");
  ros::NodeHandle nh;
  ros::Publisher ec_data = nh.advertise<ft_sensor_msgs::ForceTorque>("/linear_actuator_data", 1);

  printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
	mlockall(MCL_CURRENT | MCL_FUTURE);

	cycle_ns=1000000; // nanosecond
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	if (argc > 1)
	{
		sine_amp=atoi(argv[1]);
	}

	printf("use default adapter %s\n", ecat_ifname);
	
	demo_run();
  print_run();

  while (run)
	{
		usleep(100000);
	}


   printf("End program\n");
   return (0);
}
