void SystemInit()
{};
   
//*******************     System Clock Enable      ************************* //
#define SYSCTL_RCGCGPIO_R    (*((volatile unsigned long*) 0x400FE608)) 
   
//********************** PORT B Initializations ***********************//
#define GPIO_PORTB_DATA_R      (*((volatile unsigned long*) 0x400053FC))
#define GPIO_PORTB_DIR_R      (*((volatile unsigned long*) 0x40005400))
#define GPIO_PORTB_DEN_R      (*((volatile unsigned long*) 0x4000551C))

//********* Defining all Macros ***********//
#define PORTB_CLK_EN       0x02
#define LEDs                 0x0E
   
//***************      Timer Initialization          ********************//
#define TM_BASE 0x40031000
   
// Peripheral clock enabling for timer and GPIO
#define RCGC_TIMER_R *( volatile unsigned long *)0x400FE604
#define RCGC2_GPIO_R *( volatile unsigned long *)0x400FE108   
#define CLOCK_GPIOF 0x00000020
#define SYS_CLOCK_FREQUENCY 16000000

// General Purpose Timer Register Definitions
#define GPTM_CONFIG_R *( volatile long *)( TM_BASE + 0x000 )
#define GPTM_TA_MODE_R *( volatile long *)( TM_BASE + 0x004 )
#define GPTM_TB_MODE_R *( volatile long *)( TM_BASE + 0x008  )
#define GPTM_CONTROL_R *( volatile long *)( TM_BASE + 0x00C )
#define GPTM_INT_MASK_R *( volatile long *)( TM_BASE + 0x018 )
#define GPTM_INT_CLEAR_R *( volatile long *)( TM_BASE + 0x024)
#define GPTM_TA_IL_R *( volatile long *)( TM_BASE + 0x028 )
#define GPTM_TB_IL_R *( volatile long *)( TM_BASE + 0x02C )
#define GPTM_TA_MATCH_R *( volatile long *)( TM_BASE + 0x030 )
#define GPTM_TB_MATCH_R *( volatile long *)( TM_BASE + 0x034 )

// GPIO PF2 alternate functionality configuration
#define GPIO_PORTF_AFSEL_R *(( volatile unsigned long *)0x40025420 )
#define GPIO_PORTF_PCTL_R *(( volatile unsigned long *)0x4002552C )
#define GPIO_PORTF_DEN_R *(( volatile unsigned long *)0x4002551C )

// Timer Configuration and Mode Bit Field Definitions
#define TIM_16_BIT_CONFIG 0x00000004
#define TIM_PERIODIC_MODE 0x00000002
#define TIM_AB_ENABLE 0x00000101
#define TIM_PWM_MODE_A 0x0000000A
#define TIM_PWM_MODE_B 0x0000000A
#define TIM_CAPTURE_MODE 0x00000004

// Timer Reload Value for 1kHz PWM Frequency
#define TIM_A_INTERVAL 16000
#define TIM_B_INTERVAL 16000
   
//********************* Sensors Configuration ***********************//
#define INT_PB0               0x01
#define INT_PB1               0x02
#define INT_PB2               0x04
#define INT_PB3               0x08
#define INT_PB4               0x10
#define PORTA_DIR                       0xFF
#define PORTA_DEN                 0xFF

// Delay Function
void Delay( unsigned long cycles)
{
    while ( ( cycles -- )!= 0);
}

////////*********************timer funtion************//////////////
void Timer1A_Init ( void )
{
   GPTM_CONFIG_R |= TIM_16_BIT_CONFIG ;
   GPTM_TA_MODE_R |= TIM_PWM_MODE_A ;
   GPTM_TA_MODE_R &= ~( TIM_CAPTURE_MODE );
   GPTM_TA_IL_R = TIM_A_INTERVAL ;
}
void Timer1B_Init ( void )
{
   GPTM_CONFIG_R |= TIM_16_BIT_CONFIG ;
   GPTM_TB_MODE_R |= TIM_PWM_MODE_B ;
   GPTM_TB_MODE_R &= ~( TIM_CAPTURE_MODE );
   GPTM_TB_IL_R = TIM_B_INTERVAL;
}

//************ Main function **************//
int __main (void)
{
   //Constants intializations
   int Kp=1225;
   int Ki=0;                                                                                                          
   int Kd=330;
   int min=15999 ,max=4000;
   signed int error, P, I, D, PIDvalue;
   signed int previousError;
   int leftMotorSpeed;
   int rightMotorSpeed;
   
   volatile unsigned delay_clk;
   
   SYSCTL_RCGCGPIO_R |=  CLOCK_GPIOF + PORTB_CLK_EN;  
   Delay(6);
   GPIO_PORTB_DEN_R    |= 0xFF;
   GPIO_PORTB_DIR_R    &= ~( 0xFF);
  RCGC_TIMER_R |= 0x02;
  GPIO_PORTF_AFSEL_R|= 0x0000000c;
  GPIO_PORTF_PCTL_R|= 0x00007700;
  GPIO_PORTF_DEN_R|= 0x0000000C;
   GPTM_CONTROL_R &= ~( TIM_AB_ENABLE );
   Timer1A_Init ();
   Timer1B_Init ();
   GPTM_CONTROL_R |= TIM_AB_ENABLE ;
   Delay(8);
   
   //********** Infinite Loop ***********//
   while(1)
   {
      if(GPIO_PORTB_DATA_R==0x11)
      {
         error = 0;
         
      }
      
      if(GPIO_PORTB_DATA_R==0x18)
      {
         error = 1;
      } 
      if(GPIO_PORTB_DATA_R==0x1C)
      {
         error = 2;
      }
      if(GPIO_PORTB_DATA_R==0x1E)
      {
         error = 4;
      }
      
      if(GPIO_PORTB_DATA_R==0x03) 
      {
         error = -1;
      }
      if(GPIO_PORTB_DATA_R==0x07)
      {
         error = -2;
      }
      
      if(GPIO_PORTB_DATA_R==0x0F)
      {
         error = -4;
         
      }
      
      // Calculating PID
      P = error;
      I = I + error;
      D = error - previousError;
      PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
      

      // Base Speed + (or -) PID Value
      rightMotorSpeed = 11000 + PIDvalue;
      leftMotorSpeed = 10050 - PIDvalue;
      
      //Limiting Maximum and Minimum Speed
      if(leftMotorSpeed >min)
      {
         leftMotorSpeed=min;
      }
      if(leftMotorSpeed <max)
      {
         leftMotorSpeed=max;
      }
    
      if(rightMotorSpeed >min)
      {
         rightMotorSpeed=min;
      }
      if(rightMotorSpeed <max)
      {
         rightMotorSpeed=max;
      }
   
           GPTM_TB_MATCH_R = rightMotorSpeed;
      GPTM_TA_MATCH_R =leftMotorSpeed;
                previousError = error;
  
   }
}

