#ifndef _SIM900A_H
#define _SIM900A_H

#define SIM900A_CHECK_AT		0x00		//开机后检查模块AT
#define SIM900A_CLOSE_ECHO		0x01		//关闭回显
#define SIM900A_CHECK_SIM		0x02		//检查sim卡是否在
#define SIM900A_TCPUDP			0x03		//tcp



#define SIM900A_INIT_ERROR			0x00
#define SIM900A_CHECK_AND_SIM		0x01
#define SIM900A_CFG_GPRS_PARAM		0x02
#define SIM900A_CFG_TCPUDP_PARAM	0x03
#define SIM900A_NOT_SURPORT_B		0x04		//设置GPRS移动台类别为B,支持包交换和数据交换错误
#define SIM900A_PDP_ERROR			0x05		//设置PDP上下文,互联网接协议,接入点等信息出错
#define SIM900A_GPRS_ERROR			0x06		//附着GPRS出错
#define SIM900A_GPRS_CONNECT_ERROR	0x07	//设置为GPRS连接模式
#define SIM900A_IP_HEAD_ERROR		0x08		//设置接收数据显示IP头(方便判断数据来源)
#define SIM900A_TCPUDP_CLOSE_T		0x09
#define SIM900A_TCPUDP_CONNECT		0x0a


#define SIM900A_DATA_TCP			0x01
#define SIM900A_DATA_RESP			0x02
#define SIM900A_DATA_NONE			0x04

#define SIM900A_RESP_STATUS			0x01
#define SIM900A_RESP_HEART			0x02

#define SIM900A_SEND_START			0x01
#define SIM900A_SEND_FINISH			0x02
#define SIM900A_SEND_NONE			0x03






enum GPRS_STATE {
  SIM900A_AT_OK = 0,		//gprs model ok
  SIM900A_SIM_OK = 1,		//sim card ok
  SIM900A_TCPUDP_OK = 2,	//tcpudp connect ok
  SIM900A_TCPUDP_CLOSE = 3,	//tcpudp close
  SIM900A_ERROR		= 4		//error, can not communicattion
};

#define SIMG900A_MAX_TCPUDP_DATA_LEN	1352

typedef struct _sim900a_msg
{
	int  nLen;									//数据长度
	struct etimer *petimer;
	u_char ubsrc;								//数据来源
	u_char ubamsg[SIMG900A_MAX_TCPUDP_DATA_LEN];
}SIM900A_MSG;

void uart4_send_char(u_char ch);


#endif
