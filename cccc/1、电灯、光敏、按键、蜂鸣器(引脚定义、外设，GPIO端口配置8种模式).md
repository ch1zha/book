# 电灯、光敏、按键、蜂鸣器（引脚定义、外设，GPIO端口配置8种模式）

## 一、引脚定义、外设，GPIO端口配置8种模式

<img src="https://s1.ax1x.com/2023/05/30/p9jLAcq.png" alt="p9jLAcq.png" style="zoom:150%;" />

![p9jHB3q.md.png](https://s1.ax1x.com/2023/05/30/p9jHB3q.md.png)

![p9jH2E4.png](https://s1.ax1x.com/2023/05/30/p9jH2E4.png)

- 开漏 高电平没有驱动能力

- 推挽 高低电平都有驱动能力

- 第二个GPIO_InitStructure.GPIO_Pin 的选择

- int在51单片机中是16位的，在STM32中32位的，如果要用16位的数据要用short来表示

- float和double都是带符号的，没有不带符号的

- 枚举enum的使用，类似于struct结构体，只是赋值且引用是有范围限制的

  

## 二、点灯程序<Light.c>

```
#include "stm32f10x.h"   // Device header
#include "Delay.h"


void Led_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_SetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure1;
    GPIO_InitStructure1.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure1.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure1);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}    

void Led1_ON(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void Led2_ON(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void Led1_Off(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void Led2_Off(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void Led1_Turn(void)
{
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0) == 0){
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
    }
    else{
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    }
}
/* GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) 默认是高电平，也就按下键盘先执行else后面函数 */
void Led2_Turn(void)
{
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) == 0){
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
    }
    else{
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    }

}
```



## 三、按键程序<key.c>

```c
#include "stm32f10x.h"
#include "Delay.h"

void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

}

uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0;//局部变量
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
    {
        Delay_ms(20);
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
        Delay_ms(20);
        KeyNum = 1;
    }

    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 0)
    {
        Delay_ms(20);
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
        Delay_ms(20);
        KeyNum = 2;
    }
    return KeyNum;
}
```



##四、 蜂鸣器<Buzzer.c>

```c
#include "stm32f10x.h"   // Device header
#include "Delay.h"


void Buzzer_Init(void)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB,&GPIO_InitStructure);

        GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void Buzzer_ON(void)
{
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET );
 /*       Delay_ms(500);
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET );
        Delay_ms(500);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        Delay_ms(100);
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        Delay_ms(100);
*/
}

void Buzzer_Off(void)
{
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
 /*       Delay_ms(500);
        GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET );
        Delay_ms(500);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        Delay_ms(100);
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        Delay_ms(100);
*/
}

/* GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12) 默认是高电平，也就按下键盘先执行else后面函数 */
void Buzzer_Turn(void)
{
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12) == 0){
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
    }
    else{
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    }
}
```



## 五、光敏传感器代码模块<LightSensor.c>

```c
#include "stm32f10x.h"   // Device header

void LightSensor(void)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB,&GPIO_InitStructure);

        GPIO_SetBits(GPIOB, GPIO_Pin_13);

}

uint8_t LightSensor_Get(void)
{
    return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13);
}

```

## 六、主函数

```c
#include "stm32f10x.h"
#include "Delay.h"
#include "Led.h"
#include "Key.h"
#include "Buzzer.h"
#include "LightSensor.h"

uint8_t KeyNum;//全局变量
uint8_t LightSensor_Gets;

int main(void)
{
        Led_init();
        Key_Init();
        Buzzer_Init();
        LightSensor();

        while(1)
        {
                KeyNum = Key_GetNum();
                LightSensor_Gets = LightSensor_Get();
                if(LightSensor_Gets == 1)
                {
                        Buzzer_ON();
                        Led1_ON();
                }
                else
                {
                        Buzzer_Off();
                        Led1_Off();
                }

                if(KeyNum == 1)
                {
                        Led1_Turn();
                        Buzzer_Turn();
                }
                if(KeyNum == 2)
                {
                        Led2_Turn();
                        Buzzer_Off();
                }
        }
}
```

