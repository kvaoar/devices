#include "usbd_cdc_if.h"
#include "terminal.h"
#include "string.h" 

const uint8_t bufsize = 255;
char rx_buf[bufsize];
uint8_t rx_fill = 0;


//number cmd in list
const uint8_t cmd_size = 1;
//list text-function
TDictonary d[cmd_size] = {
{"help\0",help}
};

//reset terminal
void init_term(){
	rx_fill = 0;
		memset(rx_buf, 0, bufsize);
}

void dbdigit(uint32_t d){
char buf[20];
	memset(buf,0,20);
sprintf(buf,"%d\n", d);
	HAL_Delay(100);
CDC_Transmit_FS((uint8_t*)buf,strlen(buf));
		HAL_Delay(100);
}

// command1
void help (int len, char* param){

	char* p0 = param;
	char* p = strchr(p0,' ');
	static char hbuf[20];
//	memset(hbuf,0,20);
 //int i = 0;
 //int l = (20< len)?20:len;
			CDC_Transmit_FS((uint8_t*) "\'",1);
		HAL_Delay(100);
	CDC_Transmit_FS((uint8_t*) param,len);
	HAL_Delay(100);
			CDC_Transmit_FS((uint8_t*) "\'",1);
		HAL_Delay(100);
	
	while((p != NULL)&&(((p+1)-param)<=len)){
		int plen = p - p0;
		if(plen == 0 ) { 
				p0++; 
				p = strchr(p0,' '); 
				continue;
			};
		memset(hbuf,0,20);
		memcpy(hbuf,p0,plen);
		//*p = '\n';
		CDC_Transmit_FS((uint8_t*) "\'",1);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) hbuf,plen);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) "\'",1);
			HAL_Delay(100);
			
			p0 = p+1;
			p = strchr(p0,' ');
		
	}
	
};



void t_print(const char* str){
CDC_Transmit_FS((uint8_t*)str, strlen(str));
}


void t_addstr(char* str, uint32_t len){
CDC_Transmit_FS((uint8_t*) str,len);
if(len > (bufsize-rx_fill)) 
	{
	init_term();
	return;
	}
memcpy(&rx_buf[rx_fill], str, len);
rx_fill += len;
};

void t_exe(){
char* p = strchr(rx_buf,'\r');

if(p != NULL)
	{
		*p = '\0';
		uint8_t len = p-rx_buf; //command with parameters lenght
	
		for(uint8_t i = 0; i < cmd_size; i++) 
		{
			int cmd_len = strlen(d[i].cmd);
			if(memcmp(rx_buf,d[i].cmd,cmd_len)==0) 
			{
				char* ppar = &rx_buf[strlen(d[i].cmd)];
				int plen = len - strlen(d[i].cmd);
				d[i].exe(plen, ppar);
				init_term();
			}
		}
	}
}
