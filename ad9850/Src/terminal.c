#include "usbd_cdc_if.h"
#include "terminal.h"
#include "string.h" 
#include "ad9850.h"
#include "main.h"

const uint8_t bufsize = 255;
char rx_buf[bufsize];
uint8_t rx_fill = 0;


//number of commands in command list
const uint8_t cmdlist_size = 2;

//list of availible commands
TDictonary cmdlist[cmdlist_size] = {
{{"help\0",0},help},
{{"set\0",0},fset}
};

//reset terminal
void term_init(){
for(int i = 0; i < cmdlist_size; i++) cmdlist[i].cmd.l = strlen(cmdlist[i].cmd.p);
term_reset();
}

void term_reset(){
	rx_fill = 0;
		memset(rx_buf, 0, bufsize);
	term_print("\n#>");
}

void dbdigit(uint32_t d){
char buf[20];
	memset(buf,0,20);
sprintf(buf,"%d\n", d);
	HAL_Delay(100);
CDC_Transmit_FS((uint8_t*)buf,strlen(buf));
		HAL_Delay(100);
}

/*
* Parse parameter from string to param list

*/

int param_parser(TString param,TString* plist, int max_par_cnt , char space){
  int par_cnt = 0;
	for(int start = 0; start < param.l-1; start++){
	if((param.p[start] == space)&&(param.p[(start+1)] != ' '))
		{
			int end = start+1; 
			while(end <= param.l)
			{
				if((param.p[end] == space)||(end == param.l ))
					{
					if(par_cnt < max_par_cnt)
						{
						plist[par_cnt].p = &param.p[start+1];
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





void term_print(const char* str){
CDC_Transmit_FS((uint8_t*)str, strlen(str));
}


void term_addstr(char* str, uint32_t len){
//CDC_Transmit_FS((uint8_t*) str,len);
if(len > (bufsize-rx_fill)) 
	{
	term_init();
	return;
	}
memcpy(&rx_buf[rx_fill], str, len);
rx_fill += len;
};

void term_exe(){
TString cmd = {0,0};
TString param = {0,0};

cmd.p = rx_buf; // start search from buffer start
char* p = strchr(cmd.p,'\r'); // search command spacer
if(p != NULL) // if command spacer found in buffer
	{
		*p = '\0'; //it replaced to end of string
		cmd.l = p-rx_buf; //command with parameters lenght
		for(uint8_t i = 0; i < cmdlist_size; i++)  // for all comands in command list
		{
			if(memcmp(cmd.p,cmdlist[i].cmd.p,cmdlist[i].cmd.l)==0) // if first symbols in command equal first symbols in command list
			{
				if(cmd.p[cmdlist[i].cmd.l] == ' ') // if next symbol is spacer
					{
					param.p = &rx_buf[cmdlist[i].cmd.l]; // from first param spacer next to [command name] and to [command spacer] - parameter string
					param.l = cmd.l - cmdlist[i].cmd.l;  // lengt of parameter string eq total length minus command lengt (param spacer - to param)
					cmdlist[i].exe(param); // run assigned function witn parameter string
					term_reset(); // terminal reset (for one command with one 255 char buffer)
						return;
					}
			}
		}
		term_reset(); // terminal reset (for one command with one 255 char buffer)
	}
}

// command1
void help (TString param){
	const int max_par_cnt = 3;
	TString plist[max_par_cnt] = {{0,0},{0,0},{0,0}};

	static char hbuf[20];
		CDC_Transmit_FS((uint8_t*) "\n",1);
			HAL_Delay(100);
int par_cnt = param_parser(param,plist,max_par_cnt,' ');
	for(int i = 0; i < par_cnt; i++){
		memset(hbuf,0,20);
		memcpy(hbuf,plist[i].p,plist[i].l);
		//*p = '\n';
		CDC_Transmit_FS((uint8_t*) ">",1);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) hbuf,plist[i].l);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) "\n",1);
			HAL_Delay(100);
	}
	
};

// command1
void fset (TString param){
TString sfreq = {0,0};
static char hbuf[20];
int par_cnt = param_parser(param,&sfreq,1,' ');
		memset(hbuf,0,20);
		memcpy(hbuf,sfreq.p,sfreq.l);
		CDC_Transmit_FS((uint8_t*) "\nfreq=",6);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) hbuf,sfreq.l);
		HAL_Delay(100);
		CDC_Transmit_FS((uint8_t*) "\n",1);
			HAL_Delay(100);
uint32_t f = atoi(sfreq.p);
freq = f;
};

