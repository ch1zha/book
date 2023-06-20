# PWM（呼吸灯、舵机、直流电机、输入捕获、输出比较）

# 一、PWM简介



![p9v2UmR.md.png](https://s1.ax1x.com/2023/05/31/p9v2UmR.md.png)

电机相关比较重要。

OC Output Compare（IC 是[输入捕获](https://so.csdn.net/so/search?q=输入捕获&spm=1001.2101.3001.7020)，CC代指这两个单元），用于输出一定频率和占空比的PWM波形。

<img src="https://s1.ax1x.com/2023/05/31/p9vRg8U.md.png" alt="p9vRg8U.md.png" style="zoom:150%;" />

右下角四个就是CCR。只有通用计时器和高级计时器有，共用一个cnt计数器，高级计数器的前三个ccr寄存器还有死区比较和互补输出功能，可以驱动三相电机。

PWM（Pulse Width Modulation）脉冲宽度调制，在具有惯性的系统中，可以通过对一系列脉冲的宽度进行调制，来等效地获得所需要的模拟参量，常应用于电机控速等领域。

<img src="https://s1.ax1x.com/2023/05/31/p9vRHPK.md.png" alt="p9vRHPK.md.png" style="zoom:150%;" />

按一定频率置0置1，可以改变电机综合速度。LED也是，我们人眼看着就觉得灯有亮度，实际上就是按一定频率闪烁就会呈现不同的亮度。

周期Ts，占空比Ton/Ts（置1的时间占总周期比例），频率=周期倒数，分辨率是占空比变化步距。

![p9vWpIP.png](https://s1.ax1x.com/2023/05/31/p9vWpIP.png)

输入模式：

| **模式**         | **描述**                                                     |
| ---------------- | ------------------------------------------------------------ |
| 冻结             | CNT=CCR时，REF保持为原状态                                   |
| 匹配时置有效电平 | CNT=CCR时，REF置有效电平                                     |
| 匹配时置无效电平 | CNT=CCR时，REF置无效电平                                     |
| 匹配时电平翻转   | CNT=CCR时，REF电平翻转                                       |
| 强制为无效电平   | CNT与CCR无效，REF强制为无效电平                              |
| 强制为有效电平   | CNT与CCR无效，REF强制为有效电平                              |
| PWM模式1         | 向上计数：CNT<CCR时，REF置有效电平，CNT≥CCR时，REF置无效电平 向下计数：CNT>CCR时，REF置无效电平，CNT≤CCR时，REF置有效电平 |
| PWM模式2         | 向上计数：CNT<CCR时，REF置无效电平，CNT≥CCR时，REF置有效电平 向下计数：CNT>CCR时，REF置有效电平，CNT≤CCR时，REF置无效电平 |

强制模式比如断开输入的时候。

TIMx_CCER里也可以设置极性。

整体处理逻辑：

![p9vW8sJ.png](https://s1.ax1x.com/2023/05/31/p9vW8sJ.png)

![p9vWoLj.png](https://s1.ax1x.com/2023/05/31/p9vWoLj.png)

频率周期和普通定时器一样，占空比也很好理解。

分辨率就是arr的最小值的倒数。

至于高级计时器，暂时简单了解其区别即可：

![p9vWHwn.png](https://s1.ax1x.com/2023/05/31/p9vWHwn.png)

两个互补输出可以接到推挽电路上，死区生成电路使得两管切换有一定延迟。

舵机：输入一个角度，舵机停止在固定角度。周期20ms，高电平宽度0.5-2.5ms。

![p9vWboq.png](https://s1.ax1x.com/2023/05/31/p9vWboq.png)

舵机三个角，±极，信号线。（5V）信号线内部有驱动电路，所以可以直接接。

直流电机正向流正向转，反向电流反向转。无法直接驱动，需要依靠 TB6612 芯片驱动。

由两个推挽电路构成，一个H型电路，中间是电机。电流从左上到右下和右上到左下正好流经中间是相反的。

![p9vWXWT.png](https://s1.ax1x.com/2023/05/31/p9vWXWT.png)

![p9vfCwR.png](https://s1.ax1x.com/2023/05/31/p9vfCwR.png)

VM：驱动电路。

VCC：控制电路，比如我们32的3.3v。

PWMA PWMB是两个信号端，另外两个引脚可以接任意GPIO口。PWM 控制频率的改变，IN一直保持一个状态即可。

STNDBY 待机控制引脚，接地待机，接VCC工作。



# 二、呼吸灯代码

```c
/*******************************************************************************************/
这四个函数是配置输出比较的（重要）
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct); 
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
/******************************************************************************************/
这四个函数是强制输出模式的，用的不多
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction); 
/*******************************************************************************************/
这四个函数是配置CCR寄存器预装功能（影子寄存器）要在更新时间才会生效，一般不用
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
/*******************************************************************************************/
这四个函数是配置快速使能，用的不多
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
/*******************************************************************************************/
这四个函数清楚REF信号，用的也不多
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
/*******************************************************************************************/
单独设置输出比较极性，这里带有N的就是高级定时器互补通道配置
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
/*******************************************************************************************/
用来单独修改输出使能参数
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);
/*******************************************************************************************/
用来单独更改输出比较模式的函数
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
/*******************************************************************************************/
这四个是用来单独更改CCR寄存器值的函数（重要）
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4);
/*******************************************************************************************/
这个仅高级定时器使用，使用高级定时器输出PWM，需要调用这个函数使能主输出，否则PWM不能正常输出
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
/*******************************************************************************************/

```

```c
void PWM_Init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;/* 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period                = 100-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler             = 720-1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter     = 0;

    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    /*TIM_OCInitStructure.TIM_OCIdleState      = ;高级定时器配置*/
    TIM_OCStructInit(&TIM_OCInitStructure);/* 初始值 */
    TIM_OCInitStructure.TIM_OCMode           = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity       = TIM_OCPolarity_High;//引性高电平
    TIM_OCInitStructure.TIM_OutputState      = TIM_OutputState_Enable;//使能
    TIM_OCInitStructure.TIM_Pulse            = 0;//CCR的值
    /*TIM_OCInitStructure.TIM_OCNIdleState = ;  带有N的是高级定时器*/
    TIM_OC1Init(TIM2,&TIM_OCInitStructure);

    TIM_Cmd(TIM2,ENABLE);
}

void PWM_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2,Compare);
}
```

设置周期100，ccr的值/100就是占空比，当前设置为50%亮度。设置10就是10%亮度。

```c
while(1)
{
    OLED_ShowNum(2,5,Num2,5);
    OLED_ShowNum(3,5,TIM_GetCounter(TIM2),5);
    for(i = 0; i <= 100;i++)
    {
  	    PWM_SetCompare1(i);
  	    Delay_ms(10);
    }
    for(i = 0; i <= 100;i++)
    {
    	PWM_SetCompare1(100-i);
   	    Delay_ms(10);
    }
}
```

可以通过 TIM_SetCompare1(TIM2,i); 改变ccr值。

# 三、 引脚重映射（呼吸灯重映射代码）

![p9v4ryF.png](https://s1.ax1x.com/2023/05/31/p9v4ryF.png)

如图，TIM2_CH1可以重映射到PA15上。

首先需要开启 AFIO， RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

前面介绍过 AFIO 重映射函数：void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState) ，第一个参数选取什么呢？

数据手册里说：
![p9v4fW6.png](https://s1.ax1x.com/2023/05/31/p9v4fW6.png)

所以我们选择部分重映像1或完全重映像都行。

然后我们需要取消PA15原来的功能：

![p9v4goR.png](https://s1.ax1x.com/2023/05/31/p9v4goR.png)

第一个：只取消NoJTRST调试的，也就是PB4.

第二个：取消PA15，PB3，PB4这三个JTAG调试端口。

第三个：解除JTAG和SWJ的端口，千万千万千万千万不能用，因为一旦烧录了，我们的板子就再也没法通过stlink下载了。需要用串口处理了。

代码上只是改了GPIO Pin，增加了AFIO重映射。

```c
#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
    
     //PA0对应定时器2，oc channel1.
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//防止漏赋初值出错，而且再更改想改的赋值简单一些
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}
```



四、舵机代码

## 1、PWM代码

```c
#include "stm32f10x.h"   // Device header

void PWM_Init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;/* 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period             = 20000-1;/* ARP */
    TIM_TimeBaseInitStructure.TIM_Prescaler          = 72-1;/* PSC */
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter     = 0;
/*******************************Freq=CK_PSC/(PSC+1)/(ARR+1) =50 HZ*************************************/
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    /*TIM_OCInitStructure.TIM_OCIdleState      = ;高级定时器配置*/
    TIM_OCStructInit(&TIM_OCInitStructure);/* 初始值 */
    TIM_OCInitStructure.TIM_OCMode           = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity       = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState      = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse            = 0;//CCR的值
    /*TIM_OCInitStructure.TIM_OCNIdleState = ;  带有N的是高级定时器*/
    TIM_OC2Init(TIM2,&TIM_OCInitStructure);

    TIM_Cmd(TIM2,ENABLE);
}

void PWM_SetCompare2(uint16_t Compare)
{
    TIM_SetCompare2(TIM2,Compare);
}
```

## 2、舵机代码

```c
#include "stm32f10x.h"   // Device header
#include "PWM.h"

void Servo_Init()
{
    PWM_Init();
}

void Servo_SetAngle(float Angle)
{
    PWM_SetCompare2(Angle / 180 * 2000 + 500);
}

```

## 3、主函数

```c
#include "stm32f10x.h"
#include "Delay.h"
#include "Led.h"
#include "Key.h"
#include "Buzzer.h"
#include "LightSensor.h"
#include "OLED.h"
#include "CountSensor.h"
#include "Encoder.h"
#include "timer.h"
#include "PWM.h"
#include "Servo.h"

uint8_t KeyNum;//全局变量
uint8_t LightSensor_Gets;
int16_t Num;
uint16_t Num2;
uint8_t i;
float Angle;

int main(void)
{
    OLED_Init();
    /*Encoder_Init();
    CountSensor_Init();
    Timer_Init();*/
    Key_Init();
    Servo_Init();
    Servo_SetAngle(90);
    Angle =90;

    OLED_ShowString(1,3,"Hellod world");
    OLED_ShowString(2,1,"Angle:");
    while(1)
    {
        KeyNum = Key_GetNum();
        if(KeyNum == 2)
        {
            Angle += 30;
        }
        if(Angle >= 180)
        {
            Angle = 0;
        }
        Servo_SetAngle(Angle);
        OLED_ShowNum(2,7,Angle,3);
    }
}
```

# 四、直流电机

需要用到电机驱动模块。

![p9vqhsP.png](https://s1.ax1x.com/2023/05/31/p9vqhsP.png)

AN控制方向，随便接GPIO，PWMA接PWM输出。

和之前的输出比较没啥区别，还是我们给定一个ccr参数控制速度。AN的两个GPIO脚要初始化，初始化后一个SetBits一个ResetBits，来控制转向。

### 1、直流电机代码

```c
#include "stm32f10x.h"   // Device header
#include "PWM.h"

void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;/* 推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    

    PWM_Init();
}

void Motor_Speed(int8_t Speed)
{
    if(Speed >= 0)
    {
        GPIO_SetBits(GPIOA,GPIO_Pin_4);
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);
        PWM_SetCompare3(Speed);
    }
    else
    {
        GPIO_ResetBits(GPIOA,GPIO_Pin_4);
        GPIO_SetBits(GPIOA,GPIO_Pin_5);
        PWM_SetCompare3(-Speed);
    }
}
```

转动时有蜂鸣器般的噪声，可以通过调大频率解决，也就是prescaler和period参数调小点，频率就大了（超过人耳的频率）。