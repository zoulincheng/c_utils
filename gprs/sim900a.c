#include "contiki.h"
#include "xprintf.h"
#include "basictype.h"
#include "sim900a.h"
#include "iodef.h"
#include "string.h"
#include "lib/ringbuf.h"
#include "sysprintf.h"
#include "gprsProtocol.h"
#include "hwgg.h"
#include "dev_info.h"

PROCESS(sim900a_hard_init, "sim900a_hard_init");
PROCESS(sim900a_check_process, "sim900a_check");
PROCESS(sim900a_cfggprs_process, "sim900a_cfggprs");
PROCESS(sim900a_tcpudp_con_process, "sim900a_tcpcon");
PROCESS(sim900a_app_process, "sim900a_app");
PROCESS(sim900a_send_process, "sim900a_send");
PROCESS(gprs_resp_process, "sim900a_resp");


//receive data from sim900a
process_event_t sim900_event_resp;
process_event_t	sim900_event_send_data;
process_event_t sim900_event_send_data_wait;
process_event_t	sim900_event_send_cmd;
process_event_t	sim900_event_heart;
process_event_t sim900_event_fire_warn;


#define TIME_CHECK_GPRS_AT		100
#define TIME_CLOSE_GPRS_ECHO	200
#define TIME_CHECK_GPRS_SIM		200
static struct etimer et_gprs;
static struct etimer et_gprs_status;
static struct etimer et_gprs_heart_periodic;
static u_short uwSeq = 0;

static SIM900A_MSG sim900_rx;
static SIM900A_MSG sim900_tx;
static SIM900A_MSG sim900_app;
const u_char serverIp[]="119.29.155.148";
const u_char serverPort[]="9000";
static u_char dataSrcFlag = SIM900A_DATA_NONE;
static enum GPRS_STATE gprsState = SIM900A_ERROR;

const u_char routermac[GPRS_F_MAC_LEN]={0x12,0x21,0x33,0x44};

struct process *gprs_process = NULL;

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
const u_char  *modetbl[2]={"TCP","UDP"};//连接模式


/**
 * 
 * @param presp pointer response data from gprs
 * @return none
 */
void sim_at_response(const char * presp)
{
	xprintf(presp);
}


/**
 * 
 * @param pcTarget The pointer to target string
 * @param pcFindStr The pointer to the find str
 * @return if the  pcFindStr string  is a substring of the pcTarget string
 * return the addr the pcFindStr first position,else return null
 */
u_char* sim900a_check_cmd(const char *pcTarget, const char* pcFindStr)
{
	char *strx = NULL;
	strx = (char *)strstr(pcTarget,pcFindStr);
	return (u_char*)strx;
}


/**
 * 
 * @param  cmd send num to gprs
 * @return 
 * note this function need to check grps ack in app layer
 */
u_char sim900a_send_cmd_num(u_char cmd)
{
	uart4_send_char(cmd);
	return 0;
}

/**
 * 
 * @param  cmd cmd str to gprs, not add '\r\n'
 * @return 
 * note this function need to check grps ack in app layer
 */
u_char sim900a_send_cmd(const u_char *cmd)
{
	 u_char ubaBuf[64] = {0};
	xsprintf(ubaBuf, "%s\n", cmd);
	MEM_DUMP(10, "CMD", ubaBuf, strlen(ubaBuf));
	//xfprintf(uart4_send_char, "%s\n", cmd);
	uart4_send_bytes(ubaBuf, strlen(ubaBuf));
	return 0;
} 





/**
 * 
 * @param  chr char
 * @return hex value of the chr if vilad,else return 0
 */
u_char sim900a_chr2hex(u_char chr)
{
	if(chr>='0'&&chr<='9')
		return chr-'0';
	if(chr>='A'&&chr<='F')
		return (chr-'A'+10);
	if(chr>='a'&&chr<='f')
		return (chr-'a'+10); 
	return 0;
}


//将1个16进制数字转换为字符
//hex:16进制数字,0~15;
//返回值:字符
u_char sim900a_hex2chr(u_char hex)
{
	if(hex<=9)
		return hex+'0';
	if(hex>=10&&hex<=15)
		return (hex-10+'A'); 
	return '0';
}


void sim900a_tcpudp_connect(u_char *pbuf,u_char mode,u_char *ipaddr, u_char *port)
{
	xsprintf((char*)pbuf,"AT+CIPSTART=\"%s\",\"%s\",\"%s\"",modetbl[mode],ipaddr,port);
	sim900a_send_cmd(pbuf);
}







//app layer
/******************************************************************/
PROCESS_THREAD(sim900a_hard_init, ev, data)
{
	PROCESS_BEGIN( );
	//gprs init
	//RST
	XPRINTF((0, "sim900a_hard_init\r\n"));
	GPRS_SRST(0);
	GPRS_STATUS(0);
	GPRS_PWRKEY(0);
	etimer_set(&et_gprs, 5000);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_gprs));

	XPRINTF((0, "sim900a_hard_init2\r\n"));
	process_start(&sim900a_check_process, NULL);

	process_exit(&sim900a_hard_init);
	GPRS_SRST(1);
	GPRS_STATUS(1);
	GPRS_PWRKEY(1);
	XPRINTF((0, "sim900a_hard_init2\r\n"));
	PROCESS_END( );
}


void sim900a_send_cmd_wait_ack(const u_char *cmd, struct etimer *pet, clock_time_t waitTime)
{
	sim900a_send_cmd(cmd);
	if (waitTime > 0 && pet != NULL)
	{
		etimer_set(pet, waitTime);
	}
}


void sim900a_handler(process_event_t ev, process_data_t data)
{
	static u_char ubCheckCount = 0;
	static u_char ubSim900aState = SIM900A_CHECK_AT; //init sim900a state

	if (ev == PROCESS_EVENT_TIMER && data == &et_gprs)
	{
		//
		if (ubSim900aState == SIM900A_CHECK_AT)
		{
			sim900a_send_cmd_wait_ack("AT", &et_gprs,TIME_CHECK_GPRS_AT);
			ubCheckCount++;
			if (ubCheckCount == 10)//restart grps moduel
			{
				//process_exit(&sim900a_rev_process);
				//process_start(sim900a_process, NULL);
			}
		}
		//
		else if (ubSim900aState == SIM900A_CLOSE_ECHO)
		{
			sim900a_send_cmd_wait_ack("ATE0", &et_gprs,TIME_CLOSE_GPRS_ECHO);
		}

		//
		else if (ubSim900aState == SIM900A_CHECK_SIM)
		{
			
		}
		
	}

	//data
	if (ev == sim900_event_resp && data != NULL)
	{
		//开机启动后，发送AT指令同步波特率，同时检查模块
		if (ubSim900aState == SIM900A_CHECK_AT)
		{
			u_char *presp = NULL;
			presp = sim900a_check_cmd((const char*)data,"OK");
			
			if (presp == NULL)//continue 
			{
				sim900a_send_cmd_wait_ack("AT", &et_gprs,TIME_CHECK_GPRS_AT);
			}
			else //gprs model normal
			{
				//关闭回显
				etimer_stop(&et_gprs);
				ubCheckCount = 0;
				ubSim900aState = SIM900A_CLOSE_ECHO;
				sim900a_send_cmd_wait_ack("ATE0", &et_gprs,TIME_CLOSE_GPRS_ECHO);
			}
		}

		//完成AT指令检查后，关闭回显
		else if (ubSim900aState == TIME_CLOSE_GPRS_ECHO)
		{
			u_char *presp = NULL;
			presp = sim900a_check_cmd((const char*)data,"OK");	
			
			if (presp == NULL)//continue 
			{
				sim900a_send_cmd_wait_ack("ATE0", &et_gprs,TIME_CLOSE_GPRS_ECHO);
			}
			else //关闭成功
			{
				//检查SIM卡是否在
				etimer_stop(&et_gprs);
				ubSim900aState = SIM900A_CHECK_SIM;
				sim900a_send_cmd_wait_ack("AT+CPIN?", &et_gprs,TIME_CHECK_GPRS_SIM);
			}			
		}

		//检查SIM卡
		else if (ubSim900aState == TIME_CHECK_GPRS_SIM)
		{
			u_char *presp = NULL;
			presp = sim900a_check_cmd((const char*)data,"OK");	
			
			if (presp == NULL)//continue 
			{
				sim900a_send_cmd_wait_ack("AT+CPIN?", &et_gprs,TIME_CHECK_GPRS_SIM);
			}
			else //
			{
				//检查SIM卡是否在
				etimer_stop(&et_gprs);
				ubSim900aState = SIM900A_TCPUDP;
				sim900a_send_cmd_wait_ack("AT+CPIN?", &et_gprs,TIME_CLOSE_GPRS_ECHO);
			}			
		}
	}
}



void sim900a_clear_rx(void)
{
	memset(&sim900_rx, 0, sizeof(SIM900A_MSG));
}

void sim900a_update_rx(const u_char *pcdata, u_short uwLen)
{
	sim900_rx.nLen = uwLen;
	memcpy(sim900_rx.ubamsg, pcdata, uwLen);
}


#define ATCMD_MAX_REPEAT_NUMS		5


static int check_gprs_moduel(u_char atcmdCount)
{
	if (atcmdCount == ATCMD_MAX_REPEAT_NUMS)
	{
		//process_exit(&sim900a_init_process);
		//process_start(&sim900a_rst_process, NULL);
		return 1;
	}
	return 0;
}


const char * gprs_check[]={
	"AT",
	"ATE0",
	"AT+CPIN?",
	"AT+COPS?"
};

const char * gprs_check_resp[]={
	"OK",
	"OK",
	"OK",
	"OK"
};

//common check grps and sim
PROCESS_THREAD(sim900a_check_process, ev, data)
{
	static u_char i = 0;
	static struct etimer et_check;
	u_char *presp = NULL;
	u_char *pcmd = NULL;
	
	PROCESS_BEGIN( );
	gprs_process = &sim900a_check_process;
	i = 0;
	XPRINTF((0, "sim900a_check_process\r\n"));

#if 0
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT");
		etimer_set(&et_check, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_check) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			etimer_stop(&et_check);
			MEM_DUMP(10, "AT", data, strlen(data));
			if (presp != NULL)
			{
				gprsState = SIM900A_AT_OK;
				break;
			}
		}
	}
	
	//close grps echo function
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("ATE0");
		etimer_set(&et_check, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_check) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			etimer_stop(&et_check);
			MEM_DUMP(10, "ATE0", data, strlen(data));
			if (presp != NULL)
				break;
		}
	}	

	//查询SIM卡是否在位 
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT+CPIN?");
		etimer_set(&et_check, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_check) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			pcmd = sim900a_check_cmd((const char*)data,"CPIN");
			etimer_stop(&et_check);
			MEM_DUMP(10, "CPIN", data, strlen(data));
			if (presp != NULL && pcmd != NULL)
			{
				gprsState = SIM900A_SIM_OK;
				break;
			}
		}		
	}
	
	//查询运营商名字
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT+COPS?");
		etimer_set(&et_check, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_check) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			pcmd = sim900a_check_cmd((const char*)data,"COPS");
			etimer_stop(&et_check);
			MEM_DUMP(10, "COPS", data, strlen(data));
			if (presp != NULL && pcmd != NULL)
				break;
		}	
	}
	#else
	etimer_set(&et_check, 5*1000); //5s
	sim900a_send_cmd(gprs_check[i]);
	while(1)
	{
		PROCESS_YIELD( );
		if (ev == PROCESS_EVENT_TIMER && data == &et_check)
		{
			etimer_set(&et_check, 5*1000); //5s
			sim900a_send_cmd(gprs_check[i]);
		}
		else if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,gprs_check_resp[i]);
			if (presp!=NULL) //next cmd
			{
				i++;
				if (i == sizeof(gprs_check)/sizeof(gprs_check[0]))
				{
					etimer_stop(&et_check);
					XPRINTF((10,"check out\n"));
					break;
				}
			}
			etimer_stop(&et_check);
			etimer_set(&et_check, 100); 
		}
	}
	#endif
	gprsState = SIM900A_SIM_OK;
	process_start(&sim900a_cfggprs_process,NULL);
	process_exit(&sim900a_check_process );
	PROCESS_END();
}

//gprs model param cfg
/*
sim900a_send_cmd("AT+CIPCLOSE=1","CLOSE OK",100);	//关闭连接
sim900a_send_cmd("AT+CIPSHUT","SHUT OK",100);		//关闭移动场景 
if(sim900a_send_cmd("AT+CGCLASS=\"B\"","OK",1000))return 1;				//设置GPRS移动台类别为B,支持包交换和数据交换 
if(sim900a_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",1000))return 2;//设置PDP上下文,互联网接协议,接入点等信息
if(sim900a_send_cmd("AT+CGATT=1","OK",500))return 3;					//附着GPRS业务
if(sim900a_send_cmd("AT+CIPCSGP=1,\"CMNET\"","OK",500))return 4;	 	//设置为GPRS连接模式
if(sim900a_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//设置接收数据显示IP头(方便判断数据来源)
*/
//sim900a_cfggprs_process
const char * gprs_cfg[]={
	"AT+CIPCLOSE=1",						//关闭连接
	"AT+CIPSHUT=1",							//关闭移动场景 
	"AT+CGCLASS=\"B\"",						//设置GPRS移动台类别为B,支持包交换和数据交换 					
	"AT+CGDCONT=1,\"IP\",\"CMNET\"",		//设置PDP上下文,互联网接协议,接入点等信息	
	"AT+CGATT=1",							//附着GPRS业务							
	"AT+CIPCSGP=1,\"CMNET\"",				//设置为GPRS连接模式
	"AT+CLPORT=\"TCP\",\"2000\""			//
};

const char * gprs_cfg_resp[]={
	"OK",
	"OK",
	"OK",
	"OK",
	"OK",
	"OK",
	"OK"
};

PROCESS_THREAD(sim900a_cfggprs_process, ev, data)
{
	static u_char i = 0;
	static struct etimer et_cfggprs;
	u_char *presp = NULL;
	PROCESS_BEGIN( );
	i = 0;
	gprs_process = &sim900a_cfggprs_process;


	//if (gprsState == SIM900A_TCPUDP_OK)
	#if 1
	{
		//关闭连接
		for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
		{
			sim900a_send_cmd("AT+CIPCLOSE=1");//"CLOSE OK"
			etimer_set(&et_cfggprs, 5*1000); //5s
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_cfggprs) || ev == sim900_event_resp);
			if (ev == sim900_event_resp && data != NULL)
			{
				presp = sim900a_check_cmd((const char*)data,"OK");
				etimer_stop(&et_cfggprs);
				MEM_DUMP(10, "LOSE", data, strlen(data));
				if (presp != NULL)
					break;
			}
		}

		//关闭移动场景 
		for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
		{
			sim900a_send_cmd("AT+CIPSHUT=1");//"SHUT OK"
			etimer_set(&et_cfggprs, 5*1000); //5s
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_cfggprs) || ev == sim900_event_resp);
			if (ev == sim900_event_resp && data != NULL)
			{
				presp = sim900a_check_cmd((const char*)data,"OK");
				etimer_stop(&et_cfggprs);
				MEM_DUMP(10, "SHUT", data, strlen(data));
				if (presp != NULL)
				{
					gprsState = SIM900A_TCPUDP_CLOSE;
					break;
				}
			}
		}
	}

	//设置GPRS移动台类别为B,支持包交换和数据交换 
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT+CGCLASS=\"B\"");//"OK"
		etimer_set(&et_cfggprs, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_cfggprs) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			etimer_stop(&et_cfggprs);
			MEM_DUMP(10, "LASS", data, strlen(data));
			if (presp != NULL)
			{
				break;
			}
		}
	}

	//设置PDP上下文,互联网接协议,接入点等信息
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"");//"OK"
		etimer_set(&et_cfggprs, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_cfggprs) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			etimer_stop(&et_cfggprs);
			MEM_DUMP(10, "CGDC", data, strlen(data));
			if (presp != NULL)
			{
				break;
			}
		}		
	}

	//附着GPRS业务
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT+CGATT=1");//"OK"
		etimer_set(&et_cfggprs, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_cfggprs) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			etimer_stop(&et_cfggprs);
			MEM_DUMP(10, "CGAT", data, strlen(data));
			if (presp != NULL)
			{
				break;
			}
		}		
	}	

	//设置为GPRS连接模式
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT+CIPCSGP=1,\"CMNET\"");//"OK"
		etimer_set(&et_cfggprs, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_cfggprs) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			etimer_stop(&et_cfggprs);
			MEM_DUMP(10, "CSGP", data, strlen(data));
			if (presp != NULL)
			{
				break;
			}
		}	
	}


	//设置接收数据显示IP头(方便判断数据来源)
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd("AT+CIPHEAD=1");//"OK"
		etimer_set(&et_cfggprs, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_cfggprs) || ev == sim900_event_resp);
		if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,"OK");
			etimer_stop(&et_cfggprs);
			MEM_DUMP(10, "PHEAD", data, strlen(data));
			if (presp != NULL)
			{
				break;
			}
		}		
	}
	#else
	etimer_set(&et_cfggprs, 5*1000); //5s
	sim900a_send_cmd(gprs_cfg[i]);
	while(1)
	{
		PROCESS_YIELD( );
		if (ev == PROCESS_EVENT_TIMER && data == &et_cfggprs)
		{
			etimer_set(&et_cfggprs, 5*1000); //5s
			sim900a_send_cmd(gprs_cfg[i]);
		}
		else if (ev == sim900_event_resp && data != NULL)
		{
			presp = sim900a_check_cmd((const char*)data,gprs_cfg_resp[i]);
			if (presp!=NULL) //next cmd
			{
				i++;
				if (i == sizeof(gprs_cfg)/sizeof(gprs_cfg[0]))
				{
					etimer_stop(&et_cfggprs);
					XPRINTF((10,"check out\n"));
					break;
				}
			}
			etimer_stop(&et_cfggprs);
			etimer_set(&et_cfggprs, 100); 
		}
	}
	#endif
	
	process_start(&sim900a_tcpudp_con_process,NULL);
	process_exit(&sim900a_cfggprs_process);
	
	PROCESS_END( );
}

PROCESS_THREAD(sim900a_tcpudp_con_process, ev, data)
{
	static u_char i = 0;
	static u_char ubcount = 0;
	static struct etimer et_tcpudp;
	static u_char baBuf[128] = {0x00};
	u_char *presp = NULL;
    u_char *pcon_ok=NULL;
	u_char *pcon_true = NULL;
	PROCESS_BEGIN( );
	gprs_process = &sim900a_tcpudp_con_process;

//	if (gprsState == SIM900A_TCPUDP_OK)
	{
		//关闭连接
		for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
		{
			sim900a_send_cmd("AT+CIPCLOSE=1");//"CLOSE OK"
			etimer_set(&et_tcpudp, 5*1000); //5s
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_tcpudp) || ev == sim900_event_resp);
			if (ev == sim900_event_resp && data != NULL)
			{
				presp = sim900a_check_cmd((const char*)data,"OK");
				etimer_stop(&et_tcpudp);
				MEM_DUMP(10, "LOSE", data, strlen(data));
				if (presp != NULL)
					break;
			}
		}

		//关闭移动场景 
		for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
		{
			sim900a_send_cmd("AT+CIPSHUT=1");//"SHUT OK"
			etimer_set(&et_tcpudp, 5*1000); //5s
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_tcpudp) || ev == sim900_event_resp);
			if (ev == sim900_event_resp && data != NULL)
			{
				presp = sim900a_check_cmd((const char*)data,"OK");
				etimer_stop(&et_tcpudp);
				MEM_DUMP(10, "SHUT", data, strlen(data));
				if (presp != NULL)
				{
					gprsState = SIM900A_TCPUDP_CLOSE;
					break;
				}
			}
		}	
	}
	//	sprintf((char*)p,"AT+CIPSTART=\"%s\",\"%s\",\"%s\"",modetbl[mode],ipaddr,port);
	//	if(sim900a_send_cmd(p,"OK",500))return;		//发起连接
	//AT+CIPSTART="TCP","180.120.52.222","8086"	
	//connect serverip 
	xsprintf(baBuf, "AT+CIPSTART=\"%s\",\"%s\",\"%s\"", modetbl[1],serverIp, serverPort);

	/*
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		sim900a_send_cmd(baBuf);
		etimer_set(&et_tcpudp, 5*1000); //5s
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_tcpudp) || ev == sim900_event_resp);
		presp = sim900a_check_cmd((const char*)data,"OK");
		pcon_true = sim900a_check_cmd((const char*)data,"ALREADY");
		pcon_ok = sim900a_check_cmd((const char*)data,"CONNECT OK");
		MEM_DUMP(10, "IPST", data, strlen(data));
		if (pcon_ok != NULL)
		{
			gprsState = SIM900A_TCPUDP_CONNECT;
			break;
		}

		if (pcon_true != NULL)
		{
			gprsState = SIM900A_TCPUDP_CONNECT;
			break;		
		}
		
	}
	*/
	
	sim900a_send_cmd(baBuf);
	ubcount = 0;
	etimer_set(&et_tcpudp, 10*1000); //5s
	while(1)
	{
		PROCESS_YIELD( );
		if (ev == PROCESS_EVENT_TIMER && data == &et_tcpudp)
		{
			sim900a_send_cmd(baBuf);
			etimer_set(&et_tcpudp, 10*1000); //5s			
		}
		else if (ev == sim900_event_resp && data != NULL)
		{
			pcon_ok = sim900a_check_cmd((const char*)data,"CONNECT OK");
			//pcon_true = sim900a_check_cmd((const char*)data,"ALREADY CONNECT");
			if (pcon_ok != NULL)
			{
				gprsState = SIM900A_TCPUDP_CONNECT;
				gprs_process = &sim900a_app_process;
				etimer_stop(&et_tcpudp);
				process_post(&sim900a_app_process, sim900_event_heart, NULL);
				MEM_DUMP(10, "IPST", data, strlen(data));
				break;
			}
		}
		if (ubcount == 10)
		{
			ubcount= 0;
			process_start(&sim900a_hard_init, NULL);
			gprs_process = &sim900a_hard_init;
			process_exit(&sim900a_tcpudp_con_process);
		}
		ubcount ++;
	}
	//gprs_process = &sim900a_app_process;
	XPRINTF((10, "sim900a_tcpudp_con_process exit\r\n"));
	process_exit(&sim900a_tcpudp_con_process);
	PROCESS_END( );
}



void gprsProtocolRxProcess(const u_char *pcFrame, u_short uwSendSeq , struct etimer *petwait)
{
	const GPRS_PROTOCOL * pGprs = (const GPRS_PROTOCOL *)pcFrame;
	u_short uwSeq = pGprs->ubSeqL | (pGprs->ubSeqH << 8);

	if (pGprs->ubCmd == GPRS_F_CMD_ACK)
	{
		if (uwSendSeq == uwSeq)
		{
			etimer_stop(petwait);
		}
	}
	else if (pGprs->ubCmd == GPRS_F_CMD_DATA_SYNC)
	{
		SIM900A_MSG *ptxMsg = &sim900_tx;
		static FIRE_NODE_INFO stFireNode;
		const FIRE_NODE_INFO *pFireNodeInfo; 
		NODE_ADDR_INFO *paddrInfo = (NODE_ADDR_INFO *)extgdbdevGetDeviceSettingInfoSt(LABLE_ADDR_INFO);
		int nFramL = -1;

		memset(&stFireNode, 0, sizeof(FIRE_NODE_INFO));
		pFireNodeInfo = (const FIRE_NODE_INFO *)pGprs->ubaData;
		if (pFireNodeInfo->node_num > 0)
		{
			stFireNode.node_num = pFireNodeInfo->node_num;
			memcpy(stFireNode.nodeArray, pFireNodeInfo->nodeArray, stFireNode.node_num*4);
			extgdbdevSetDeviceSettingInfoSt(LABLE_FIRE_NODE_INFO, 0, (const void *)&stFireNode, sizeof(FIRE_NODE_INFO));
		}
		nFramL = gprsProtocolFrameFill(ptxMsg->ubamsg, GPRS_F_CMD_ACK, uwSeq, paddrInfo->ubaNodeAddr, NULL, 0);
		if (nFramL > 0 && gprsState == SIM900A_TCPUDP_CONNECT)
		{
			ptxMsg->nLen = nFramL;
			process_post(&sim900a_send_process, sim900_event_send_data,ptxMsg);
		}
	}
}




void fillNotNetNodeInfo(FIRE_NODE_INFO *pFireInfo)
{
	const FIRE_NODE_INFO *pfireNodeInfo = (const FIRE_NODE_INFO *)extgdbdevGetDeviceSettingInfoSt(LABLE_FIRE_NODE_INFO);
	int i = 0;
	NODE_INFO *pnode = NULL;


	//clear fire node info
	if (pFireInfo != NULL)
	{
		memset(pFireInfo, 0, sizeof(FIRE_NODE_INFO));
	}
	
	if (pfireNodeInfo->node_num == 0)
	{
		return;
	}
	
	for(pnode = (NODE_INFO *)endNodeListHead(); pnode != NULL; pnode = (NODE_INFO *)endNodeListNext(pnode)) 
	{
		if (pnode->nodeNetState == HWGG_NODE_OUT_NET)
		{
			memcpy(pFireInfo->nodeArray[i++].ubaNodeAddr, pnode->ubaHWGGMacAddr, HWGG_NODE_MAC_LEN);
		}
	}

	pFireInfo->node_num = i;
}





void sim900a_app_handler(process_event_t ev, process_data_t data)
{
	static u_char ubGprsRespType = SIM900A_RESP_STATUS;
	static u_char ubaFireWarnBuf[32] = {0x00};
	static u_char ubSendCmd;
	static u_char ubSendState = SIM900A_SEND_NONE;
	static struct etimer et_heart;
	static struct etimer et_wait_ack;
	static FIRE_NODE_INFO stFireNodeInfo;
	static u_short uwCurrentSeq = 0;
	//XPRINTF((10, "grps app\r\n"));

	//revceive data process
	if (ev == sim900_event_resp && data != NULL)
	{
		if(sim900a_check_cmd((const char*)data,"CLOSED"))
		{
			gprs_process = &sim900a_check_process;
			gprsState = SIM900A_TCPUDP_CLOSE_T;
			sim900a_send_cmd("AT+CPOWD=1");
			process_start(&sim900a_check_process, NULL);
			XPRINTF((8, "ERROR\n"));
		}
		else if (sim900a_check_cmd((const char*)data,"ERROR"))
		{
			gprs_process = &sim900a_check_process;
			gprsState = SIM900A_TCPUDP_CLOSE_T;
			sim900a_send_cmd("AT+CPOWD=1");
			process_start(&sim900a_check_process, NULL);
			XPRINTF((8, "ERROR\n"));
		}
		else if (sim900a_check_cmd(((const char*)data),"SEND FAIL"))
		{
			gprs_process = &sim900a_check_process;
			gprsState = SIM900A_TCPUDP_CLOSE_T;
			sim900a_send_cmd("AT+CPOWD=1");
			process_start(&sim900a_check_process, NULL);
			XPRINTF((8, "ERROR\n"));			
		}
		else if (sim900a_check_cmd((const char*)data,"+IPD"))
		{
			u_char *pDataStar = NULL;
			u_short ubDataLen = 0;
			u_char ubassicNum[8] = {0x00};
			u_char *pdata = sim900a_check_cmd((const char*)data,"+IPD,");
			u_char *pdataEnd = sim900a_check_cmd((const char*)data,":");
			
			pDataStar = pdata + strlen("+IPD,");
			memcpy(ubassicNum, pDataStar, pdataEnd-pDataStar);
			//MEM_DUMP(10, "LEN", ubassicNum, pdataEnd-pDataStar);
			ubDataLen = Asc2Int((const u_char *)ubassicNum);
			XPRINTF((10, "len = %d\r\n", ubDataLen));
			memset(&sim900_app, 0, sizeof(SIM900A_MSG));
			sim900_app.nLen = ubDataLen;
			memcpy(sim900_app.ubamsg, pdataEnd+1, ubDataLen);
			
			if (gprsProtocolCheck((const u_char *) sim900_app.ubamsg))
			{
				MEM_DUMP(7, "<-rx", sim900_app.ubamsg, ubDataLen);
				gprsProtocolRxProcess((const u_char *) sim900_app.ubamsg,uwCurrentSeq, &et_wait_ack);
			}
		}
		//> send data start flag
		else if (sim900a_check_cmd((const char*)data,">"))
		{
			process_post(&sim900a_send_process, sim900_event_send_data_wait, data);
		}
		//finish send data
		else if (sim900a_check_cmd((const char*)data,"SEND OK"))
		{
			process_post(&sim900a_send_process, sim900_event_send_data_wait, data);
			ubSendState = SIM900A_SEND_FINISH;
		}
		else 
		{
			const u_char *pHead = NULL;
			pHead = (const u_char *)gprsProtocolFindHead((const u_char *)data);
			if (pHead != NULL)
			{
				if (gprsProtocolCheck(pHead))
				{
					const GPRS_PROTOCOL *pGprs = (const GPRS_PROTOCOL *)pHead;
					u_short uwLen = pGprs->ubDataLenL | (pGprs->ubDataLenH<<8) + 10;
					memset(&sim900_app, 0, sizeof(SIM900A_MSG));
					sim900_app.nLen = uwLen;
					memcpy(sim900_app.ubamsg, pHead, uwLen);
					MEM_DUMP(7, "<-Rx", sim900_app.ubamsg, uwLen);
					gprsProtocolRxProcess((const u_char *) sim900_app.ubamsg,uwCurrentSeq, &et_wait_ack);
				}
			}
		}
	}

	else if (ev == sim900_event_heart)
	{
		SIM900A_MSG *ptxMsg = &sim900_tx;
		NODE_ADDR_INFO *paddrInfo = (NODE_ADDR_INFO *)extgdbdevGetDeviceSettingInfoSt(LABLE_ADDR_INFO);
		int nFramL = -1;
		//XPRINTF((10, "heart1"));
		fillNotNetNodeInfo(&stFireNodeInfo);
		if (stFireNodeInfo.node_num > 0)
		{
			nFramL = gprsProtocolFrameFill(ptxMsg->ubamsg, GPRS_F_CMD_HEART, uwSeq, paddrInfo->ubaNodeAddr, (const u_char *)&stFireNodeInfo, stFireNodeInfo.node_num*4+2);
		}
		else
		{
			nFramL = gprsProtocolFrameFill(ptxMsg->ubamsg, GPRS_F_CMD_HEART, uwSeq, paddrInfo->ubaNodeAddr, NULL, 0);
		}
		if (nFramL > 0 && gprsState == SIM900A_TCPUDP_CONNECT)
		{
			ptxMsg->nLen = nFramL;
			uwCurrentSeq = uwSeq;
			ubSendCmd = GPRS_F_CMD_HEART;
			process_post(&sim900a_send_process, sim900_event_send_data,ptxMsg);
			etimer_set(&et_wait_ack, 10*1000);
			ubSendState = SIM900A_SEND_START;
		}
		//1 send heart frame
		//2 reset etimer
		etimer_set(&et_heart, 30*1000);
	}

	//send fire warn data to pc
	else if (ev == sim900_event_fire_warn && data != NULL)
	{
		SIM900A_MSG *ptxMsg = &sim900_tx;
		NODE_ADDR_INFO *paddrInfo = (NODE_ADDR_INFO *)extgdbdevGetDeviceSettingInfoSt(LABLE_ADDR_INFO);
		u_char ubaWarn[5] = 0;
		int nFramL = -1;
		const FIRE_NODE * pFireNode = (const FIRE_NODE *)data;
		if (((u_long)ubaFireWarnBuf)!=((u_long)data))
		{
			MEM_DUMP(7, "warn", data, pFireNode->ubLen);
			memcpy(ubaFireWarnBuf, data, pFireNode->ubLen);
		}
		memcpy(ubaWarn, pFireNode->ubaSrcMac, HWGG_HEAD_END_CRC_LEN);
		ubaWarn[4] = pFireNode->ubCmd;
		uwSeq++;
		nFramL = gprsProtocolFrameFill(ptxMsg->ubamsg, GPRS_F_CMD_WARN, uwSeq, paddrInfo->ubaNodeAddr, ubaWarn, 5);
		if (nFramL > 0 && gprsState == SIM900A_TCPUDP_CONNECT)
		{
			ptxMsg->nLen = nFramL;
			uwCurrentSeq = uwSeq;
			ubSendCmd = GPRS_F_CMD_WARN;
			process_post(&sim900a_send_process, sim900_event_send_data,ptxMsg);
			etimer_set(&et_wait_ack, 10*1000);
			ubSendState = SIM900A_SEND_START;
		}
	}
	else if (ev == PROCESS_EVENT_TIMER && data == &et_heart)
	{
		process_post(&sim900a_app_process, sim900_event_heart, NULL);
		uwSeq++;
	}

	else if (ev == PROCESS_EVENT_TIMER && data == &et_wait_ack)
	{
		XPRINTF((10, "ack time out\n"));
	}	
	
}

//sim900a_app_process
PROCESS_THREAD(sim900a_app_process, ev, data)
{
	PROCESS_BEGIN( );
	//sim900a_send_cmd_wait_ack("AT+CIPSTATUS",&et_gprs_status,500);
	while(1)
	{
		PROCESS_YIELD( );
		sim900a_app_handler(ev, data);
	}
	PROCESS_END( );
}

void sim900a_send_handler(process_event_t ev, process_data_t data)
{
	static const SIM900A_MSG *psendMsg = NULL;
	static u_char sendState = SIM900A_SEND_NONE;
	static struct etimer et_send;

	if (ev == sim900_event_send_data && data != NULL)
	{
		psendMsg = (const SIM900A_MSG *)data;
		sim900a_send_cmd("AT+CIPSEND");
		etimer_set(&et_send, 2*1000);
	}
	else if (ev == sim900_event_send_data_wait && data != NULL)
	{
		if (sim900a_check_cmd((const char*)data,">"))
		{
			if (psendMsg != NULL)
			{
				MEM_DUMP(7,"->TX", psendMsg->ubamsg, psendMsg->nLen);
				uart4_send_bytes(psendMsg->ubamsg, psendMsg->nLen);
				uart4_send_char(0x1a);
				sendState = SIM900A_SEND_START;
			}
		}
		if (sim900a_check_cmd((const char*)data,"SEND OK"))
		{
			etimer_stop(&et_send);
			psendMsg = NULL;
			sendState = SIM900A_SEND_FINISH;
		}
	}
	else if (ev == PROCESS_EVENT_TIMER && data == &et_send)
	{
		XPRINTF((12, "UDP SEND TIME OUT\n"));
	}
}
//PROCESS(sim900a_send_process, "sim900a_send");
PROCESS_THREAD(sim900a_send_process, ev, data)
{

	PROCESS_BEGIN( );
	
	while(1)
	{
		PROCESS_YIELD( );
		sim900a_send_handler(ev, data);
	}
	PROCESS_END( );
}


static struct ringbuf ringuartbuf;
static uint8_t uartbuf[128];

/*---------------------------------------------------------------------------*/
int gprs_uart_input_byte(unsigned char c)
{
	static uint8_t overflow = 0; /* Buffer overflow: ignore until END */
	if(!overflow) 
	{
		/* Add character */
		if(ringbuf_put(&ringuartbuf, c) == 0) 
		{
			/* Buffer overflow: ignore the rest of the line */
			overflow = 1;
		}
	} 
	else 
	{
		/* Buffer overflowed:
		* Only (try to) add terminator characters, otherwise skip */
		if(ringbuf_put(&ringuartbuf, c) != 0) 
		{
			overflow = 0;
		}
	}
	/* Wake up consumer process */
	process_poll(&gprs_resp_process);	
	//XPRINTF((10, "gprs\n"));
	return 1;
}

void gprs_uart_init(void)
{
	ringbuf_init(&ringuartbuf, uartbuf, sizeof(uartbuf));
	//process_start(&hwfs_uart_rev_process, NULL);
	Uart_GprsSetInput(gprs_uart_input_byte);
}


//sim900a_app_process
PROCESS_THREAD(gprs_resp_process, ev, data)
{
	static char buf[SIMG900A_MAX_TCPUDP_DATA_LEN];
	static struct etimer et_rev_timeout;
	static int ptr;
	int c;
	
	PROCESS_BEGIN();
	sim900_event_resp = process_alloc_event( );
	sim900_event_send_cmd = process_alloc_event( );
	sim900_event_send_data = process_alloc_event( );
	sim900_event_heart = process_alloc_event( );
	sim900_event_send_data_wait = process_alloc_event( );
	sim900_event_fire_warn = process_alloc_event( );
	XPRINTF((10, "sim900a_resp_process\r\n"));
	
	while(1) 
	{
		c = ringbuf_get(&ringuartbuf);
		if ((ev == PROCESS_EVENT_TIMER)&&(etimer_expired(&et_rev_timeout)))
		{
			XPRINTF((9, "receive finish prt= %d\r\n", ptr));
			memset(&sim900_rx, 0, sizeof(SIM900A_MSG));
			sim900_rx.nLen = ptr;
			memcpy(sim900_rx.ubamsg, buf, ptr);
			MEM_DUMP(8, "grev", buf, ptr);
			process_post(gprs_process, sim900_event_resp, sim900_rx.ubamsg);
			
			memset(buf, 0 ,sizeof(buf));
			ptr = 0;
		}

		if(c == -1) 
		{
			/* Buffer empty, wait for poll */
			PROCESS_YIELD();
		} 
		else 
		{
			if (ptr < SIMG900A_MAX_TCPUDP_DATA_LEN)
			{
				buf[ptr++] = (uint8_t)c;
				etimer_set(&et_rev_timeout, 50);
				//XPRINTF((6, "%02x \r\n", c));
			}
			else
			{
				ptr = 0;
			}
		}
	}
	PROCESS_END();
}






void sim900a_init(void)
{
	gprs_uart_init( );
	process_start(&sim900a_hard_init, NULL);
	process_start(&gprs_resp_process, NULL);
	process_start(&sim900a_app_process, NULL);
	process_start(&sim900a_send_process, NULL);
}


/*This function is used to test*/
/*
test gprs ack
*/
void testGprsAck(void)
{
	int nFrameL = -1;
	NODE_ADDR_INFO *paddrInfo = (NODE_ADDR_INFO *)extgdbdevGetDeviceSettingInfoSt(LABLE_ADDR_INFO);
	nFrameL = gprsProtocolFrameFill(sim900_tx.ubamsg, GPRS_F_CMD_ACK, uwSeq, paddrInfo->ubaNodeAddr, NULL, 0);
	process_post(gprs_process, sim900_event_resp, sim900_tx.ubamsg);
}

/*
test sync from pc
*/
typedef struct node_sync{
	u_short uwNums;
	u_char ubaBuf[160];
}NODE_SYNC;

NODE_SYNC nodeSync={
	40,
	{\
	0x32,0x50,0x0E,0x33,\
	0x01,0x00,0x00,0x00,\
	0x02,0x00,0x00,0x00,\
	0x03,0x00,0x00,0x00,\
	0x04,0x00,0x00,0x00,\
	0x05,0x00,0x00,0x00,\
	0x06,0x00,0x00,0x00,\
	0x07,0x00,0x00,0x00,\
	0x08,0x00,0x00,0x00,\
	0x09,0x00,0x00,0x00,\
	0x10,0x00,0x00,0x00,\
	0x11,0x00,0x00,0x00,\
	0x12,0x00,0x00,0x00,\
	0x13,0x00,0x00,0x00,\
	0x14,0x00,0x00,0x00,\
	0x15,0x00,0x00,0x00,\
	0x16,0x00,0x00,0x00,\
	0x17,0x00,0x00,0x00,\
	0x18,0x00,0x00,0x00,\
	0x19,0x00,0x00,0x00,\
	0x20,0x00,0x00,0x00,\
	0x21,0x00,0x00,0x00,\
	0x22,0x00,0x00,0x00,\
	0x23,0x00,0x00,0x00,\
	0x24,0x00,0x00,0x00,\
	0x25,0x00,0x00,0x00,\
	0x26,0x00,0x00,0x00,\
	0x27,0x00,0x00,0x00,\
	0x28,0x00,0x00,0x00,\
	0x29,0x00,0x00,0x00,\
    0x30,0x00,0x00,0x00,\
	0x31,0x00,0x00,0x00,\
	0x32,0x00,0x00,0x00,\
	0x33,0x00,0x00,0x00,\
	0x34,0x00,0x00,0x00,\
	0x35,0x00,0x00,0x00,\
	0x36,0x00,0x00,0x00,\
	0x37,0x00,0x00,0x00,\
	0x38,0x00,0x00,0x00,\
	0x39,0x00,0x00,0x00}
};

void testGprsSync(void)
{
	int nFrameL = -1;
	NODE_ADDR_INFO *paddrInfo = (NODE_ADDR_INFO *)extgdbdevGetDeviceSettingInfoSt(LABLE_ADDR_INFO);
	nFrameL = gprsProtocolFrameFill(sim900_tx.ubamsg, GPRS_F_CMD_DATA_SYNC, uwSeq, paddrInfo->ubaNodeAddr, (const u_char *)&nodeSync, nodeSync.uwNums*4+2);
	process_post(gprs_process, sim900_event_resp, sim900_tx.ubamsg);
}


void testNodeInfo(void)
{
	const FIRE_NODE_INFO *pFireNode = (const FIRE_NODE_INFO *)extgdbdevGetDeviceSettingInfoSt(LABLE_FIRE_NODE_INFO);
	MEM_DUMP(0, "fnod", pFireNode, sizeof(FIRE_NODE_INFO));
}

