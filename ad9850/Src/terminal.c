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


int param_parser(int len, char* param,TParam* plist, int max_par_cnt , char space){
  int par_cnt = 0;
	for(int start = 0; start < len-1; start++){
	if((param[start] == space)&&(param[(start+1)] != ' '))
		{
			int end = start+1; 
			while(end <= len)
			{
				if((param[end] == space)||(end == len ))
					{
					if(par_cnt < max_par_cnt)
						{
						plist[par_cnt].p = &param[start+1];
						plist[par_cnt].l = (end-1)-start;
						par_cnt++;
						}
					start = end-1;
					break;
					}
			end++;
			}
		}
	}
	return par_cnt;
}


// command1
void help (int len, char* param){
	const int max_par_cnt = 3;
	TParam plist[max_par_cnt] = {{0,0},{0,0},{0,0}};
 
//	int par_cnt = 0;
	static char hbuf[20];


	/*
			CDC_Transmit_FS((uint8_t*) "\'",1);
		HAL_Delay(100);
	CDC_Transmit_FS((uint8_t*) param,len);
	HAL_Delay(100);
			CDC_Transmit_FS((uint8_t*) "\'",1);
		HAL_Delay(100);
	*/
	/*
	for(int start = 0; start < len-1; start++){
	if((param[start] == ' ')&&(param[(start+1)] != ' '))
		{
			int end = 1+start; 
			while(end <= len)
			{
				
				if((param[end] == ' ')||(end == len ))
					{
					if(par_cnt < max_par_cnt)
						{
						plist[par_cnt].p = &param[start+1];
						plist[par_cnt].l = (end-1)-start;
						par_cnt++;
						}
					start = end-1;
					break;
					}
			end++;
			}
		}
	}
	*/
int par_cnt = param_parser(len,param,plist,max_par_cnt,' ');
	for(int i = 0; i < par_cnt; i++){
		memset(hbuf,0,20);
		memcpy(hbuf,plist[i].p,plist[i].l);
		//*p = '\n';
		CDC_Transmit_FS((uint8_t*) "=",1);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) hbuf,plist[i].l);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) "\n",1);
			HAL_Delay(100);
	}
	
};



void t_print(const char* str){
CDC_Transmit_FS((uint8_t*)str, strlen(str));
}


void t_addstr(char* str, uint32_t len){
//CDC_Transmit_FS((uint8_t*) str,len);
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
