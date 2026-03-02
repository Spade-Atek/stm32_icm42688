#include "eeprom.h"

struct data_map Config;	//配置信息
void load_config(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	int16_t *temp_addr = (int16_t *)PAGE_Config;
	FLASH_Unlock();
	for(i=0 ; i< sizeof(Config)/2;i++){
		*ptr = *temp_addr;
		temp_addr++;
		ptr++;
	}
	FLASH_Lock();
	if(Config.is_good != (int16_t)0xA55A||Config.end_is_good != (int16_t)0xA55A){ //数据无效 ，此时需要装载默认值。
		load_backconfig();
		Write_config();	 //将默认值写入flash
	}
}
void load_backconfig(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	int16_t *temp_addr = (int16_t *)PAGE_BackConfig;
	FLASH_Unlock();
	for(i=0 ; i< sizeof(Config)/2;i++){
		*ptr = *temp_addr;
		temp_addr++;
		ptr++;
	}
	FLASH_Lock();
	if(Config.is_good != (int16_t)0xA55A||Config.end_is_good != (int16_t)0xA55A){ //数据无效 ，此时需要装载默认值。
		Config.is_good = 0xA55A;	
		
		Config.dGx_offset = 0;
		Config.dGy_offset = 0;
		Config.dGz_offset = 0;
		
		Config.dAx_offset = 0;
		Config.dAy_offset = 0;
		
		Config.Acx_offset = 0;
		Config.Acy_offset = 0;
		Config.Acz_offset = 0;
		
		Config.end_is_good=0xA55A;
		Write_config();	 //将默认值写入flash
	}
}
//将当前配置写入flash61
void Write_config(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	uint32_t ptemp_addr = PAGE_Config;
	FLASH_Unlock();
 	FLASH_ErasePage(PAGE_Config); //擦 页
	for(i=0;i<sizeof(Config)/2;i++){
	 	FLASH_ProgramHalfWord(ptemp_addr,ptr[i]);
	 	ptemp_addr+=2;
	}
	FLASH_Lock();
}
//将当前配置写入flash62
void Write_backconfig(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	uint32_t ptemp_addr = PAGE_BackConfig;
	FLASH_Unlock();
 	FLASH_ErasePage(PAGE_BackConfig); //擦 页
	for(i=0;i<sizeof(Config)/2;i++){
	 	FLASH_ProgramHalfWord(ptemp_addr,ptr[i]);
	 	ptemp_addr+=2;
	}
	FLASH_Lock();
}

//------------------End of File----------------------------

