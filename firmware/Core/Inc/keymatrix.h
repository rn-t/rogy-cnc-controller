/*
class keymatrix{
private:
	uint16_t inpin[4] = {6,7,8,9};
	uint16_t outpin[6] = {0,1,2,3,4,5};
public:
    keymatrix(){
        refresh();
    }
    bool status[4][6]={};

    void refresh(){ //keymatrixの状態を取得
        for(int i=0;i<6;i++){
            for(int j=0;j<6;j++){
                if(j==i)HAL_GPIO_WritePin(GPIOA,outpin[j],GPIO_PIN_SET);
                else HAL_GPIO_WritePin(GPIOA,outpin[j],GPIO_PIN_RESET);
            }
            for(int j=0;j<4;j++)status[j][i] = HAL_GPIO_ReadPin(GPIOA,inpin[j]);
        }
    }
};
*/

#include <stdbool.h>

uint16_t inpin[4] = {6,7,8,9};
uint16_t outpin[6] = {0,1,2,3,4,5};

bool keymatrixStatus[4][6]={};
void keymatrixRefresh(){
	for(int i=0;i<6;i++){
	    for(int j=0;j<6;j++){
	        if(j==i)HAL_GPIO_WritePin(GPIOA,outpin[j],GPIO_PIN_SET);
	        else HAL_GPIO_WritePin(GPIOA,outpin[j],GPIO_PIN_RESET);
	    }
	    for(int j=0;j<4;j++)keymatrixStatus[j][i] = HAL_GPIO_ReadPin(GPIOA,inpin[j]);
	}
}
