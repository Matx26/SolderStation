/*
 * File:   main_v110.c
 * Author: Electronica
 * main_v010: Esta version se vasa en el programa "Lesson11HelloLCD.X" para
 *            mostrar en el LCD un menu de la estacion soldadora JBC
 * main_V110: Esta Version se basa en v010 y se le implementa la lectura 
 *            analogica atraves del puerto AN0, tambien se implementa la salida
 *            PWM en la salida RB12.
 *            Detalles:  La senal PWM tiene una frecuencia de 1Khz. El muestreo se
 *            realiza a los 990us del flanco de subida de cada senal PWM, de los 
 *            10us que quedan se usan 4.5us para hacer la conversion A/D y se deja   
 *            una pausa de 5.5us.
 * 
 * Como el PIC24FJ64GA002 se alimenta de 3.3v y el display necesita para el 
 * contraste casi 5 volt, se alimentara con 3.3 en vdd del lcd y en VEE con una
 * tension negativa para obtener la diferencia de potencia necesario para el 
 * contraste.
 * Created on 9 de junio de 2021, 15:47
 */


#include "xc.h"
#include "ConfigBits.h"
#define FCY 16000000UL      //definimos la velocidad a 16MIPS, UL=40bits (unsigned long)
                            //si el clock trabaja a 32Mhz y como cada siclo de instrucion
                            //se realiza cada 2 clocks la frecuencia de las instrucciones es de 16Mips 

#include <libpic30.h>       //incluimos esta libreria que tiene las definiciones de los
                            //reardos y usa la definicion de FCY para ello

#include "lcd_16x4.h"       //se debe incluir esta libreria despues de definir FCY e incluir
                            //<libpic30.h> ya que la libreria lcd_16x4.h hace uso 
                            // de las funciones __delay_ms y __delay_us las cuales 
                            //se vason en esas definiciones

#include <stdio.h>          //se incluye esta libreria para poder escribir 
                            //variables en el LCD


/******************************************************************************/
/*Definiciones particulares de Mnemonicos                                     */
#define B_Down PORTBbits.RB6    //defino boton down
#define B_Menu PORTBbits.RB7    //defino boton menu/enter
#define B_Up PORTBbits.RB8      //defino boton up
#define Sensor_Mag PORTBbits.RB9    //defino la entrada del sensor Magnetico

/******************************************************************************/
/*Variables globales que se usan en el control PDI   */
float temp=0, PWMControl=0, a=0.1243, b=0.0062, c=0.6215, SetPoint=350, LimMaxTemp=500, LimMinTemp=0;
float rT=0, eT=0, iT=0, dT=0, yT=0, uT=0, iT0=0, eT0=0, iT_1=0, eT_1=0;
/*temp          -> almacena la temperatura leida por el ADC
 *PWMControl    -> parametro que define la salida
 *a, b, c, d    -> constante del sitema
 *SetPoint      -> temperatura de trabajo
 * LimMaxTemp   -> limite maximo para el anti-windup
 * LimMinTemp   -> limite minimo para el anti-windup
 * rT           -> es el setpoint 
 * eT           -> es el error entre el setpoint y el valor medido
 * iT           -> calculo del termino integral
 * dT           -> calculo del termino derivativo
 * yT           -> es la señal medida
 * uT           -> Salida del contrl¿olador PID
 * iT0,eT0      -> son los valores de la medicion anterior 
 */


/*Variables Globales que se usan en los menus                                 */
int count = 0, ADCValue = 0, *ADC16Ptr;
unsigned char menu = 0;   //se usa para indicar que pantalla (de 1 a 8) esta visible
unsigned char hibernation=0;    //indica si esta habilitada la hibernacion
char s[20];     //para escribir en el LCD
unsigned int Temp=248, Set_Temp=250, Max_Temp=400, Min_Temp=90, Beep=0, Sleep_Delay=3, Sleep_Temp=100, Hib_Delay=5, Temp_Adj=0; 

unsigned char menu3=1, menu4=1, menu5=0, menu6=1, menu7=0, menu8=0; //se usa para indicar que opcion esta selecionada de cada menu

unsigned char set_menu4_1 = 0;      //indica si se puede editar el valor MaxTemp
unsigned char set_menu4_2 = 0;      //indica si se puede editar el valor MinTemp
unsigned char set_menu4_3 = 0;      //indica si se puede editar el valor Beep

unsigned char set_menu5_0 = 0;      //indica si se puede editar el valor MaxTemp
unsigned char set_menu5_1 = 0;      //indica si se puede editar el valor MinTemp
unsigned char set_menu5_2 = 0;      //indica si se puede editar el valor Beep

unsigned char set_menu6_1 = 0;      //indica si se puede editar el valor SleepDelay
unsigned char set_menu6_2 = 0;      //indica si se puede editar el valor SleepTemp
unsigned char set_menu6_3 = 0;      //indica si se puede editar el valor HibernacionDelay

unsigned char set_menu7_0 = 0;      //indica si se puede editar el valor SleepDelay
unsigned char set_menu7_1 = 0;      //indica si se puede editar el valor SleepTemp
unsigned char set_menu7_2 = 0;      //indica si se puede editar el valor HibernacionDelay
unsigned char set_menu7_3 = 0;      //indica si se puede editar el valor TempAdj

unsigned char set_menu8_0 = 0;      //indica si se puede editar el valor SleepTemp
unsigned char set_menu8_1 = 0;      //indica si se puede editar el valor HibernacionDelay
unsigned char set_menu8_2 = 0;      //indica si se puede editar el valor TempAdj


/***************************Config_Oscillator**********************************/
/* Esta funcion configura el oscilador a 32Mhz                                */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Config_Oscillator(void)
{
    OSCCONbits.COSC = 0b001;    // FCR fast RC oscillator with PLL
    OSCCONbits.NOSC = 0b001;    // New osc in case clock switch is FRC with PLL
    OSCCONbits.CLKLOCK = 1;   // Clock and PLL are lock
    OSCCONbits.IOLOCK = 1;    //IO PPS lock enable
    OSCCONbits.SOSCEN = 0;      //secondary oscillator is disable
    
    CLKDIVbits.DOZEN = 0;         // peripheral clock ratio is 1:1
    CLKDIVbits.RCDIV = 0b000;       //FRC postscaler divideby 1
}


/***************************Config_Ports_MCU***********************************/
/* Funcion: configura los puertos B(0-5) como digital sin OPEN DRAIN, los     */
/*          puertos B(6,7 y 8) como entradas para el teclado, AN0 como int AD */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Config_Port_Mcu(void)
{
    AD1PCFG = 0xFFFE;   //AN0 como analigo, todos los puertos A/D como digital IO
    ODCB = 0x0000;      //Open drain disable
    
    LATB = 0x0000;      //all off
    LATA = 0x0000;      //all off
    
    TRISBbits.TRISB0 = 0;   //RB0 como salida EN del LCD
    TRISBbits.TRISB1 = 0;   //RB1 como salida RS del LCD
    TRISBbits.TRISB2 = 0;   //RB2 como salida D4 del LCD
    TRISBbits.TRISB3 = 0;   //RB3 como salida D5 del LCD
    TRISBbits.TRISB4 = 0;   //RB4 como salida D6 del LCD
    TRISBbits.TRISB5 = 0;   //RB5 como salida D7 del LCD
    
    TRISBbits.TRISB6 = 1;   //RB6 como entrada del Switch Down
    TRISBbits.TRISB7 = 1;   //RB7 como entrada del Switch Down
    TRISBbits.TRISB8 = 1;   //RB8 como entrada del Switch Down
    TRISBbits.TRISB9 = 1;   //RB9 como entrada del Switch Down
    
    TRISBbits.TRISB12 = 0;   //RB12 como salida para PWM
    
    TRISBbits.TRISB13 = 0;     //se usa para medir el tiempo entre conversiones del modulo AD
    TRISBbits.TRISB14 = 0;     //se usa para medir el tiempo entre conversiones del modulo AD
    
    TRISAbits.TRISA0 = 1;   //AN0 como entrada para uso anologico
    
}

/*********************************Config_PPS++*********************************/
/* Funcion: configura el puerto de salida para la senal PWM (Output compare 1)*/
/*      (peripheral pin select)Le asignamos la  salida OC1 al RPn = RP0 (pin4)*/
/* Argumentos: no tiene                                                       */
/* Retorna: nada   */
/*****************************************************************************/
void Config_PPS(void)
{
    //iniciamos la secuencia de desbloqueo de IOLOCK
    OSCCON = 0x46;
    OSCCON = 0x57;
    //ahora ya se puede desbloquear IOLOCK
    
    OSCCONbits.IOLOCK = 0; //IOLOCK desbloqueado, ahora ya se puede escribir los
                           //registros     
                                
    RPOR6bits.RP12R = 18;    //Asignamos la salida OC1(PWM1) al RPn = RP12 = RB12(pin 23)
                            //es igual a 18 segun tabla 10-2 (pagina 109 de la hoja de datos)
    
    OSCCONbits.IOLOCK = 1;  //bloqueamos por las dudas IOLOCK, aunque ya no se odria comabiar de nuevo 
                            //en el resto del programa.
}


/***************************At_Button_Down*************************************/
/* Esta funcion se ejecuta cada ves que se pulse el boton down.               */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void At_Button_Down(void)
{
    switch(menu)
    {
        case 0:
            if(Set_Temp > Sleep_Temp && Set_Temp > Min_Temp)
            {
                Set_Temp = Set_Temp - 1; 
                //ahora actualizamos el LCD
                sprintf(s,"%d c  ",Set_Temp);
                Lcd_Set_Cursor(11,1);        // position cursor X=1,Y=1
                Lcd_Write_String(s);
            }
            break; 
            
        case 1:
            //pregunto que no sea menor que el menor numero permitido
            if(Set_Temp > Sleep_Temp && Set_Temp > Min_Temp)
            {
                Set_Temp = Set_Temp - 1; 
                //ahora actualizamos el LCD
                sprintf(s," SeT Temp %d c  ",Set_Temp);
                Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
                Lcd_Write_String(s);
                
                __delay_ms(2000);
                Menu1();
            }
            break;
            
        case 2:
            hibernation = 0; //deshabilito la hibernacion
            //pregunto que no sea menor que el menor numero permitido
            if(Set_Temp > Sleep_Temp && Set_Temp > Min_Temp)
            {
                Set_Temp = Set_Temp - 1; 
                //ahora actualizamos el LCD
                sprintf(s," SeT Temp %d c  ",Set_Temp);
                Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
                Lcd_Write_String(s);
                
                __delay_ms(2000);
                Menu1();
            }
            break;
            
        case 3:
            if(menu3 == 1)
            {
                menu3=2;
                //Borramos el guion en L1
                Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                Lcd_Write_String(" "); 
                //guion en L2
                Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                Lcd_Write_String("-");        
            }
            else if(menu3 == 2)
            {
                menu3=3;
                //Borramos el guion en L2
                Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                Lcd_Write_String(" ");
                //guion en L3
                Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            else if(menu3 == 3)
            {
                //aca no se hace nada
            }
            break;
            
        case 4:
            if(menu4==1)
            {
                if(set_menu4_1 == 1) //pregunto si esta en modo edicion maxima termperatura
                {
                    Max_Temp = Max_Temp - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%dc ",Max_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu4=2;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu4==2)
            {
                if(set_menu4_2 == 1) //pregunto si esta en modo edicion minima temperatura
                {
                    Min_Temp = Min_Temp - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dc ",Min_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu4=3;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu4==3)
            {
                if(set_menu4_3 == 1) //pregunto si esta en modo edicion Beep
                {
                    if(Beep == 1)   //pregunto si Beep esta habilitado
                    {
                        Beep = 0;
                        Lcd_Set_Cursor(13,4);        // position cursor X=1,Y=1
                        Lcd_Write_String("OFF ");
                    }
                    else
                    {
                        Beep = 1;
                        Lcd_Set_Cursor(13,4);        // position cursor X=1,Y=1
                        Lcd_Write_String("ON  ");
                    } 
                }
                else    // aca entra si esta en modo menu.
                {
                    menu = 5;       //se pasa al menu numero 5
                    menu5 = 2;
                    Menu5();        //imprimimos en pantalla el menu 5
                    //guion en L3
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            break;
            
        case 5:
            if(menu5==0)
            {
                if(set_menu5_0 == 1) //pregunto si esta en modo edicion maxima termperatura
                {
                    Max_Temp = Max_Temp - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,1);
                    sprintf(s,"%dc ",Max_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu5=1;
                    //Borramos el guion en L0
                    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu5==1)
            {
                if(set_menu5_1 == 1) //pregunto si esta en modo edicion minima temperatura
                {
                    Min_Temp = Min_Temp - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%dc ",Min_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu5=2;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu5==2)
            {
                if(set_menu5_2 == 1) //pregunto si esta en modo edicion Beep
                {
                    if(Beep == 1)   //pregunto si Beep esta habilitado
                    {
                        Beep = 0;
                        Lcd_Set_Cursor(13,3);        // position cursor X=1,Y=1
                        Lcd_Write_String("OFF ");
                    }
                    else
                    {
                        Beep = 1;
                        Lcd_Set_Cursor(13,3);        // position cursor X=1,Y=1
                        Lcd_Write_String("ON  ");
                    } 
                }
                else    // aca entra si esta en modo menu.
                {
                    menu5 = 3;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu5==3)
            {
                //aca no se hace nada porque esta en back
            }
            break;
            
        case 6:
            if(menu6==1)
            {
                if(set_menu6_1 == 1) //pregunto si esta en modo edicion de Sleep Delay
                {
                    Sleep_Delay = Sleep_Delay - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%ds ",Sleep_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu6=2;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu6==2)
            {
                if(set_menu6_2 == 1) //pregunto si esta en modo edicion Sleep Temp
                {
                    Sleep_Temp = Sleep_Temp - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dc ",Sleep_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu6=3;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu6==3)
            {
                if(set_menu6_3 == 1) //pregunto si esta en modo Hibernacion Delay
                {
                    Hib_Delay = Hib_Delay - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,4);
                    sprintf(s,"%dmin ",Hib_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu=7;     //pasamos al menu 7 
                    menu7= 2;   //opcion 3 del menu 7
                    Menu7();    //imprimo en pantalla el menu 7
                    //guion en L3
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            break;
            
        case 7:
            if(menu7==0)
            {
                if(set_menu7_0 == 1) //pregunto si esta en modo edicion de Sleep Delay
                {
                    Sleep_Delay = Sleep_Delay - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,1);
                    sprintf(s,"%ds ",Sleep_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu7=1;
                    //Borramos el guion en L0
                    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu7==1)
            {
                if(set_menu7_1 == 1) //pregunto si esta en modo edicion Sleep Temp
                {
                    Sleep_Temp = Sleep_Temp - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%dc ",Sleep_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu7=2;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu7==2)
            {
                if(set_menu7_2 == 1) //pregunto si esta en modo edicion Hiber Dalay
                {
                    Hib_Delay = Hib_Delay - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dmin ",Hib_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu7=3;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu7==3)
            {
                if(set_menu7_3 == 1) //pregunto si esta en modo Temp Adj
                {
                    Temp_Adj = Temp_Adj - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,4);
                    sprintf(s,"%dc ",Temp_Adj);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu=8;     //pasamos al menu 8 
                    menu8= 2;   //opcion 2 del menu 8
                    Menu8();    // imprimo en pantalla el menu 8
                    //guion en L3
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            break;
            
        case 8:
            if(menu8==0)
            {
                if(set_menu8_0 == 1) //pregunto si esta en modo edicion Sleep Temp
                {
                    Sleep_Temp = Sleep_Temp - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,1);
                    sprintf(s,"%dc ",Sleep_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu8=1;
                    //Borramos el guion en L0
                    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu8==1)
            {
                if(set_menu8_1 == 1) //pregunto si esta en modo edicion Hiber Dalay
                {
                    Hib_Delay = Hib_Delay - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%dmin ",Hib_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu8=2;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu8==2)
            {
                if(set_menu8_2 == 1) //pregunto si esta en modo Temp Adj
                {
                    Temp_Adj = Temp_Adj - 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dc ",Temp_Adj);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu8= 3;   //opcion 3 del menu 8
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            else if(menu8==3)
            {
                //aca no se hace nada porque esta en back, el final del menu
            }
            break;
    }
                
            
}

/***************************At_Button_Menu*************************************/
/* Esta funcion se ejecuta cada ves que se pulse el boton Menu/Enter.         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void At_Button_Menu(void)
{
    switch(menu)
    {
        case 0: //estando en menu 0 si se oprime Menu se va al menu3
            menu = 3;  
            menu3 = 1;  //foco en L1
            Menu3();    //imprimo el menu 3.
            //guion en L1
            Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
            Lcd_Write_String("-");
            break;
        
        case 1:
            menu = 3;  
            menu3 = 1;  //foco en L1
            Menu3();    //imprimo el menu 3.
            //guion en L1
            Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
            Lcd_Write_String("-");
            break;
            
        case 2:
            menu = 3;  
            menu3 = 1;  //foco en L1
            Menu3();    //imprimo el menu 3.
            //guion en L1
            Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
            Lcd_Write_String("-");
            break;
        
        case 3:
            if(menu3 == 1)      //pregunto si se seleccione "Station Settings" en el menu 3
            {
                menu = 4;  
                menu4 = 1;  //foco en L1
                Menu4();    //imprimo el menu 4 (station setting).
                //guion en L1
                Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            
            else if(menu3 == 2)  //pregunto si se seleccione "Tool Settings" en el menu 3
            {
                menu = 6;  
                menu6 = 1;  //foco en L1
                Menu6();    //imprimo el menu 6 (tool setting).
                //guion en L1
                Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            
            else if(menu3 == 3)  //pregunto si se seleccione "Back" en el menu 3
            {
                if(Sensor_Mag == 1) //en funcion del estado del sonsor magnetico se regresa al menu 0 o 1 
                {
                    menu = 1;  
                    Menu1();    //imprimo el menu 1 (Menu Sleep).
                }
                else
                {
                    menu = 0;  
                    Menu0();    //imprimo el menu 0 (Menu Narmal Operation).
                }
            }
            break; 
            
        case 4:
            if(menu4 == 1) //pregunto si el menu esta en Temp_Max
            {
                if(set_menu4_1 == 1) //pregunto si Temp max esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Temp Max

                    //Borramos arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu4_1 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu4_1 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu4 == 2) //pregunto si el menu esta en Min_Temp
            {
                if(set_menu4_2 == 1) //pregunto si Min_Temp esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Min Temp

                    //Borramos arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu4_2 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu4_2 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu4 == 3) //pregunto si el menu esta en Beep
            {
                if(set_menu4_3 == 1) //pregunto si Beep esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Beep

                    //Borramos arrow en L3
                    Lcd_Set_Cursor(2,4);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu4_3 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L3
                    Lcd_Set_Cursor(2,4);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu4_3 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            break;
            
        case 5:
        
            if(menu5 == 0) //pregunto si el menu esta en Temp_Max
            {
                if(set_menu5_0 == 1) //pregunto si Temp max esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Temp Max

                    //Borramos arrow en L0
                    Lcd_Set_Cursor(2,1);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu5_0 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L0
                    Lcd_Set_Cursor(2,1);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu5_0 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu5 == 1) //pregunto si el menu esta en Min_Temp
            {
                if(set_menu5_1 == 1) //pregunto si Min_Temp esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Min Temp

                    //Borramos arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu5_1 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu5_1 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu5 == 2) //pregunto si el menu esta en Beep
            {
                if(set_menu5_2 == 1) //pregunto si Beep esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Beep

                    //Borramos arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu5_2 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu5_2 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu5 == 3) //pregunto si el menu esta en Back
            {
                menu = 3;  
                menu3 = 1;  //foco en L1
                Menu3();    //imprimo el menu 3 (main menu).
                //guion en L1
                Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            break;
            
        case 6:
            
            if(menu6 == 1) //pregunto si el menu esta en Sleep_Delay
            {
                if(set_menu6_1 == 1) //pregunto si Sleep_delay esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Temp Max

                    //Borramos arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu6_1 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu6_1 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu6 == 2) //pregunto si el menu esta en Sleep_Temp
            {
                if(set_menu6_2 == 1) //pregunto si Sleep_Temp esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Min Temp

                    //Borramos arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu6_2 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu6_2 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu6 == 3) //pregunto si el menu esta en Hibernacion_Delay
            {
                if(set_menu6_3 == 1) //pregunto si Hib_delay esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Beep

                    //Borramos arrow en L3
                    Lcd_Set_Cursor(2,4);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu6_3 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L3
                    Lcd_Set_Cursor(2,4);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu6_3 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            break;
        
        case 7:
            
            if(menu7 == 0) //pregunto si el menu esta en Sleep_Delay
            {
                if(set_menu7_0 == 1) //pregunto si Sleep_delay esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Temp Max

                    //Borramos arrow en L0
                    Lcd_Set_Cursor(2,1);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu7_0 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L0
                    Lcd_Set_Cursor(2,1);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu7_0 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu7 == 1) //pregunto si el menu esta en Sleep_Temp
            {
                if(set_menu7_1 == 1) //pregunto si Sleep_Temp esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Min Temp

                    //Borramos arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu7_1 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu7_1 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu7 == 2) //pregunto si el menu esta en Hibernacion_Delay
            {
                if(set_menu7_2 == 1) //pregunto si Hib_delay esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Beep

                    //Borramos arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu7_2 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu7_2 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu7 == 3) //pregunto si el menu esta en Temp_Adj
            {
                if(set_menu7_3 == 1) //pregunto si Temp_Adj esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Beep

                    //Borramos arrow en L3
                    Lcd_Set_Cursor(2,4);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu7_3 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L3
                    Lcd_Set_Cursor(2,4);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu7_3 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            break;
            
        case 8:
            if(menu8 == 0) //pregunto si el menu esta en Sleep_Temp
            {
                if(set_menu8_0 == 1) //pregunto si Sleep_Temp esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Min Temp

                    //Borramos arrow en L0
                    Lcd_Set_Cursor(2,1);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu8_0 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L0
                    Lcd_Set_Cursor(2,1);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu8_0 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu8 == 1) //pregunto si el menu esta en Hibernacion_Delay
            {
                if(set_menu8_1 == 1) //pregunto si Hib_delay esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Beep

                    //Borramos arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu8_1 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L1
                    Lcd_Set_Cursor(2,2);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu8_1 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu8 == 2) //pregunto si el menu esta en Temp_Adj
            {
                if(set_menu8_2 == 1) //pregunto si Temp_Adj esta en modo edicion o no?
                {
                    //aca debo poner una rutina para guardar en la EEpron el dato Beep

                    //Borramos arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(" ");
                    set_menu8_2 = 0;    //Deshabilitamos el modo edicion del menu
                }
                else    // aca entra si esta en modo menu.
                {
                    //Ponemos Arrow en L2
                    Lcd_Set_Cursor(2,3);        // position cursor X=1,Y=1
                    Lcd_Write_String(">");
                    set_menu8_2 = 1;    //Habilitamos el modo edicion del menu
                }
            }
            
            else if(menu8 == 3) //pregunto si el menu esta en Back
            {
                menu = 3;  
                menu3 = 2;  //foco en L2
                Menu3();    //imprimo el menu 3 (main menu).
                //guion en L0
                Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            break;
    }

}

/***************************At_Button_Up*************************************/
/* Esta funcion se ejecuta cada ves que se pulse el boton Up.               */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void At_Button_Up(void)
{
    switch(menu)
    {
        case 0:
            //pregunto que no sea mayor que el mayor numero permitido
            if(Set_Temp < Max_Temp)
            {
                Set_Temp = Set_Temp + 1; 
                //ahora actualizamos el LCD
                sprintf(s,"%d c  ",Set_Temp);
                Lcd_Set_Cursor(11,1);        // position cursor X=1,Y=1
                Lcd_Write_String(s);
            }
            break; 
            
        case 1:
            //pregunto que no sea mayor que el mayor numero permitido
            if(Set_Temp < Max_Temp)
            {
                Set_Temp = Set_Temp + 1; 
                //ahora actualizamos el LCD
                sprintf(s," SeT Temp %d c  ",Set_Temp);
                Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
                Lcd_Write_String(s);
                
                __delay_ms(2000);
                Menu1();
            }
            break;
            
        case 2:
            hibernation = 0; //deshabilito la hibernacion
            //pregunto que no sea mayor que el mayor numero permitido
            if(Set_Temp > Max_Temp)
            {
                Set_Temp = Set_Temp + 1; 
                //ahora actualizamos el LCD
                sprintf(s," SeT Temp %d c  ",Set_Temp);
                Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
                Lcd_Write_String(s);
                
                __delay_ms(2000);
                Menu1();
            }
            break;
            
        case 3:
            if(menu3 == 1)
            {
                //aca no se hace nada.        
            }
            else if(menu3 == 2)
            {
                menu3=1;
                //Borramos el guion en L2
                Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                Lcd_Write_String(" ");
                //guion en L2
                Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            else if(menu3 == 3)
            {
                menu3=2;
                //Borramos el guion en L3
                Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=3
                Lcd_Write_String(" ");
                //guion en L2
                Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            break;
            
        case 4:
            if(menu4==1)
            {
                if(set_menu4_1 == 1) //pregunto si esta en modo edicion maxima termperatura
                {
                    Max_Temp = Max_Temp + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%dc ",Max_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    //aca no hace nada porque esta ya esta en la parte alta del menu
                }
            }
            
            else if(menu4==2)
            {
                if(set_menu4_2 == 1) //pregunto si esta en modo edicion minima temperatura
                {
                    Min_Temp = Min_Temp + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dc ",Min_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu4=1;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu4==3)
            {
                if(set_menu4_3 == 1) //pregunto si esta en modo edicion Beep
                {
                    if(Beep == 1)   //pregunto si Beep esta habilitado
                    {
                        Beep = 0;
                        Lcd_Set_Cursor(13,4);        // position cursor X=1,Y=1
                        Lcd_Write_String("OFF ");
                    }
                    else
                    {
                        Beep = 1;
                        Lcd_Set_Cursor(13,4);        // position cursor X=1,Y=1
                        Lcd_Write_String("ON  ");
                    } 
                }
                else    // aca entra si esta en modo menu.
                {
                    menu4 = 2;
                    //Borramos el guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            break;
            
        case 5:
            if(menu5==0)
            {
                if(set_menu5_0 == 1) //pregunto si esta en modo edicion maxima termperatura
                {
                    Max_Temp = Max_Temp + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,1);
                    sprintf(s,"%dc ",Max_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu = 4;   //nos vamos a menu 4
                    menu4=1;
                    Menu4();    //imprimimos en pantalla el menu 4
                    
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu5==1)
            {
                if(set_menu5_1 == 1) //pregunto si esta en modo edicion minima temperatura
                {
                    Min_Temp = Min_Temp + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%dc ",Min_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu5=0;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L0
                    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu5==2)
            {
                if(set_menu5_2 == 1) //pregunto si esta en modo edicion Beep
                {
                    if(Beep == 1)   //pregunto si Beep esta habilitado
                    {
                        Beep = 0;
                        Lcd_Set_Cursor(13,3);        // position cursor X=1,Y=1
                        Lcd_Write_String("OFF ");
                    }
                    else
                    {
                        Beep = 1;
                        Lcd_Set_Cursor(13,3);        // position cursor X=1,Y=1
                        Lcd_Write_String("ON  ");
                    } 
                }
                else    // aca entra si esta en modo menu.
                {
                    menu5 = 1;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu5==3)
            {
                menu5 = 2;
                //Borramos el guion en L3
                Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=3
                Lcd_Write_String(" ");
                //guion en L2
                Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            break;
            
        case 6:
            if(menu6==1)
            {
                if(set_menu6_1 == 1) //pregunto si esta en modo edicion de Sleep Delay
                {
                    Sleep_Delay = Sleep_Delay + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%ds ",Sleep_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    //aca no hace nada porque esta ya esta en la parte alta del menu
                }
            }
            
            else if(menu6==2)
            {
                if(set_menu6_2 == 1) //pregunto si esta en modo edicion Sleep Temp
                {
                    Sleep_Temp = Sleep_Temp + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dc ",Sleep_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu6=1;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu6==3)
            {
                if(set_menu6_3 == 1) //pregunto si esta en modo Hibernacion Delay
                {
                    Hib_Delay = Hib_Delay + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,4);
                    sprintf(s,"%dmin ",Hib_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu6= 2;   //opcion 3 del menu 7
                    //Borramos el guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            break;
            
        case 7:
            if(menu7==0)
            {
                if(set_menu7_0 == 1) //pregunto si esta en modo edicion de Sleep Delay
                {
                    Sleep_Delay = Sleep_Delay + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,1);
                    sprintf(s,"%ds ",Sleep_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu = 6;   //entro al menu anterior
                    menu6=1;
                    Menu6();    //imprimimos en pantalla el menu 6
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu7==1)
            {
                if(set_menu7_1 == 1) //pregunto si esta en modo edicion Sleep Temp
                {
                    Sleep_Temp = Sleep_Temp + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%ds ",Sleep_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu7=0;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L0
                    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu7==2)
            {
                if(set_menu7_2 == 1) //pregunto si esta en modo edicion Hiber Dalay
                {
                    Hib_Delay = Hib_Delay + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dmin ",Hib_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu7=1;
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu7==3)
            {
                if(set_menu7_3 == 1) //pregunto si esta en modo Temp Adj
                {
                    Temp_Adj = Temp_Adj + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,4);
                    sprintf(s,"%dc ",Temp_Adj);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu7= 2;   //opcion 3 del menu 8
                    //Borramos el guion en L3
                    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            break;
            
        case 8:
            if(menu8==0)
            {
                if(set_menu8_0 == 1) //pregunto si esta en modo edicion Sleep Temp
                {
                    Sleep_Temp = Sleep_Temp + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,1);
                    sprintf(s,"%dc ",Sleep_Temp);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu = 7;       //entro al menu anterior
                    menu7=1;
                    Menu7();    //imprimimos en pantalla el menu 7
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu8==1)
            {
                if(set_menu8_1 == 1) //pregunto si esta en modo edicion Hiber Dalay
                {
                    Hib_Delay = Hib_Delay + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,2);
                    sprintf(s,"%dmin ",Hib_Delay);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu8=0;
                    //Borramos el guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L0
                    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            
            else if(menu8==2)
            {
                if(set_menu8_2 == 1) //pregunto si esta en modo Temp Adj
                {
                    Temp_Adj = Temp_Adj + 1;
                    //actualizamos el LCD
                    Lcd_Set_Cursor(13,3);
                    sprintf(s,"%dc ",Temp_Adj);
                    Lcd_Write_String(s);
                }
                else    // aca entra si esta en modo menu.
                {
                    menu8= 1;   //opcion 3 del menu 8
                    //Borramos el guion en L2
                    Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=3
                    Lcd_Write_String(" ");
                    //guion en L1
                    Lcd_Set_Cursor(1,2);        // position cursor X=1,Y=1
                    Lcd_Write_String("-");
                }
            }
            else if(menu8==3)
            {
                menu8= 2;   //opcion 3 del menu 8
                //Borramos el guion en L3
                Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=3
                Lcd_Write_String(" ");
                //guion en L2
                Lcd_Set_Cursor(1,3);        // position cursor X=1,Y=1
                Lcd_Write_String("-");
            }
            break;                   
    }
}

/***************************Menu0**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu0                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu0(void)
{
    //Menu 0 en pantalla LCD
    sprintf(s," SeT Temp %d c   ",Set_Temp);
    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"     %d c       ",Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    sprintf(s,"                ",Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,4);
    sprintf(s,"Power %d%%       ",80);
    Lcd_Write_String(s);
}


/***************************Menu1**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu1                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu1(void)
{
    //Menu 2 en pantalla LCD
    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
    Lcd_Write_String("     Sleep      ");
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"     %d c      ",Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    sprintf(s,"                ",Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,4);
    sprintf(s,"Sleep Temp %d c",Sleep_Temp);
    Lcd_Write_String(s);
}


/***************************Menu2**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu2                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu2(void)
{
    //Menu 2 en pantalla LCD
    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
    Lcd_Write_String("  Hibernation   ");
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"     %d c      ",Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    sprintf(s,"                ",Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,4);
    Lcd_Write_String("    No Heat     ");
}


/***************************Menu3**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu3                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu3(void)
{
    //Menu 3 en pantalla LCD
    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
    Lcd_Write_String("   Main Menu    ");
    
    Lcd_Set_Cursor(1,2);
    Lcd_Write_String("   Station Setti");
    
    Lcd_Set_Cursor(1,3);
    Lcd_Write_String("   Tool Settings");
    
    Lcd_Set_Cursor(1,4);
    Lcd_Write_String("      Back      ");
}


/***************************Menu4**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu4                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu4(void)
{
    //Menu 4 en pantalla LCD
    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
    Lcd_Write_String("Station Settings");
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"   Max temp %dc ",Max_Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    sprintf(s,"   Min temp %dc ",Min_Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,4);
    if(Beep == 1)    //pregunto si beep esta habilitado
    {
         Lcd_Write_String("   Beep     ON  ");
    }
    else
    {
         Lcd_Write_String("   Beep     OFF ");
    }
   
}


/***************************Menu5**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu5                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu5(void)
{
    //Menu 5 en pantalla LCD
    Lcd_Set_Cursor(1,1);
    sprintf(s,"   Max temp %dc ",Max_Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"   Min temp %dc ",Min_Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    if(Beep == 1)    //pregunto si beep esta habilitado
    {
         Lcd_Write_String("   Beep     ON  ");
    }
    else
    {
         Lcd_Write_String("   Beep     OFF ");
    }
    
    Lcd_Set_Cursor(1,4);        // position cursor X=1,Y=1
    Lcd_Write_String("      Back      ");
}




/***************************Menu6**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu0                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu6(void)
{
    //Menu 0 en pantalla LCD
    Lcd_Set_Cursor(1,1);        // position cursor X=1,Y=1
    Lcd_Write_String(" Tool Settings  ");
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"   SleepDel %ds  ",Sleep_Delay);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    sprintf(s,"   SleepTem %dc  ",Sleep_Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,4);
    sprintf(s,"   HibDelay %dmin ",Hib_Delay);
    Lcd_Write_String(s);
}


/***************************Menu7**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu0                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu7(void)
{
    //Menu 7 en pantalla LCD
    Lcd_Set_Cursor(1,1);
    sprintf(s,"   SleepDel %ds  ",Sleep_Delay);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"   SleepTem %dc  ",Sleep_Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    sprintf(s,"   HibDelay %dmin",Hib_Delay);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,4);
    sprintf(s,"   TempAdj  %dc  ",80);
    Lcd_Write_String(s);
}


/***************************Menu8**********************************************/
/* Esta funcion se ejecuta imprime en el LCD el Menu0                         */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Menu8(void)
{
    //Menu 8 en pantalla LCD
    Lcd_Set_Cursor(1,1);
    sprintf(s,"   SleepTem %dc  ",Sleep_Temp);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,2);
    sprintf(s,"   HibDelay %dmin",Hib_Delay);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,3);
    sprintf(s,"   TempAdj  %dc  ",80);
    Lcd_Write_String(s);
    
    Lcd_Set_Cursor(1,4);
    Lcd_Write_String("      Back      ");
}

/****************************Config_OuputCompare1******************************/
/* Funcion: Configura la senal PWM, La frecuencia sera de 1000Hz lo que      */
/*         implica que el periodo sera de 1ms, se elige ese valor de frec.   */
/*         porque es la mayor valor que me da una resolucion de casi 14 bits.*/
/*         El prescaler de los timer sera 1:1 por lo que para obtener el      */
/*         periodo de 1ms PR1 debe valer (PWM_Period/Tcy -1) = 15999          */
/* Argumentos: no tiene                                                       */
/* Retorna: nada  */
/******************************************************************************/
void Config_OutputCompare1(void)
{
    //configuro el output capture 1
    OC1CONbits.OCM = 0b000;     //primero deshabilitamos el output capture 1 antes de configurarlo
    OC1R = 8000;      //inicializamos el registro de duty cycle que se usa al inicial la senal PWM
    OC1RS = 8000;      //inicializamos el segundo registro duty cycled que se usa una ves iniciado el PWM
    OC1CONbits.OCSIDL = 0;  //output compare 1 continua funcionando en idle mode
    OC1CONbits.OCTSEL = 0;  //Timer2 es la fuente de clock del output capture 1
    OC1CONbits.OCM = 0b110;     //modo PWM en output capture 1, pin faul OCF1 is disable
    PR2 = 15999;     //especificamos el valor de PR2 (del TIMER2) para obtener un periodo de la señal pwm de 1ms
    
    //Configuro el timer2 como base de tiempo para Output capture 1
    T2CONbits.TSIDL = 0;        //timer2 will continue to operate on idle mode
    T2CONbits.TGATE = 0;        //no usare el gate timer( mide el tiempo en alto de un pulso)
    T2CONbits.TCKPS = 0b00;     //prescaler 1:1, con lo que el periodo mas largo que se puede medir es de 4.096ms, aca
                                //aca solo necesito medir 64us
    T2CONbits.T32 = 0;      //timer anidados no esta habilitado
    T2CONbits.TCS = 0;      //Timer2 como timer al contar los pulsos Fosc/2
    TMR2 = 0x0000;      // borramos el contenido del timer2
    //T2CONbits.TON = 1;  //habilitamos el timer2 y por ende al PWM1
}

/************************ConfigInterrupT2**************************************/
/* Funcion: Configura la interrupcion por Timer2 para indicar justo cuando es */
/*          el flanco de subida de la senal PWM de OC1                        */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/*****************************************************************************/
void ConfigInterrupT2(void)
{
    INTCON1bits.NSTDIS = 0;     //el anidamiento de las interrupcciones esta habilitado (defecto)
    INTCON2bits.ALTIVT = 0;     //se selecciona el tabla estandar de interrupciones (defecto)
    
    IFS0bits.T2IF = 0;        // reseteamos el flag por las dudas.
    IPC1bits.T2IP = 0b001;    //seleccionamos la prioridad como 1 (la mas alta)
    IEC0bits.T2IE = 1;        //habilitamos la interrupcion por AD conversation
}


/******************************_T2Interrupt***********************************/
/* Funcion: Atiende la interrupcion por Timer2 que indica justo cuando es     */
/*          el flanco de subida de la senal PWM de OC1, desde esta punto      */
/*          se habilita el timer3 por 990us para hacer la tectura de la termocupla*/
/*          justo en los ultimos 10us de la senal PWM*/
/******************************************************************************/
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
    //mdimos el tiempo entre las interrupciones
    T3CONbits.TON = 1;  //habilitamos el timer3, y despues de 990us iniciara una conversion A/D
    IFS0bits.T2IF = 0; //reseteamos la bandera
    
    if(PORTBbits.RB14 == 0)
    {
        LATBbits.LATB14 = 1;    //si RB14 esta en 0 se pone en 1
    }
    else
    {
        LATBbits.LATB14 = 0;    //si RB14 esta en 1 se pone en 0
    }
    
}


/***************************Config_Timer3**************************************/
/* Funcion: Configura el Timer3 para usarlo como fuente de disparo de las     */
/*          comversiones del modulo AD, el timer3 iniciara su cuenta cuando   */
/*          con el flanco de subida del PWM (esta senal se obtiene del TIME2) */
/*          y contar 990us alli iniciara la conversion AD  */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Config_Timer3(void)
{   
    T3CONbits.TON = 0;      //deshabilito el timer3
    T3CONbits.TSIDL = 0;        //timer2 will continue to operate on idle mode
    T3CONbits.TGATE = 0;        //no usare el gate timer( mide el tiempo en alto de un pulso)
    T3CONbits.TCKPS = 0b00;     //prescaler 1:1, con lo que el periodo mas largo que se puede medir es de 4.096ms, aca
                                //aca solo necesito medir 64us
    T3CONbits.TCS = 0;      //Timer3 como timer al contar los pulsos Fosc/2
    TMR3 = 0x0000;      // borramos el contenido del timer2
    PR3 = 15840;        //Periodo = 990us
    //T2CONbits.TON = 1;  //habilitamos el timer2 y por ende al PWM1
}

    
/***************************Config_AD******************************************/
/* Funcion: configura el modulo AD de modo que realice la interrupcion cada   */
/*          16 conversiones, esta interrupcion se genera cada 1,0096mS,       */
/*          Haciendo el promedio de las 16 muestras, se puede tomar como frec.*/
/*          de muestreo para el PID de 1Khz, no se usa Timer3 como trigger.   */
/* Argumentos: no tiene                                                       */
/* Retorna: nada                                                              */
/******************************************************************************/
void Config_AD(void)
{
   AD1CON1bits.ADON = 0;    //por ahora el modulo AD estara apagado hasta que se confugure
   AD1CON1bits.ADSIDL = 0;  //continua operativo en modo IDlE
   AD1CON1bits.FORM = 0b00; //fomato Integer (0000 00dd dddd dddd)
   AD1CON1bits.SSRC = 0b010;    //el inicio de la conversioin esta bajo el control del Timer3
   AD1CON1bits.ASAM = 0;    //el muestreo inicia cuando SAMP=1
   AD1CON1bits.SAMP = 0;    //  el sample and hold esta manteniendo
   
   AD1CON2bits.VCFG = 0b000;    //VR+ = AVDD y VR- = AVss
   AD1CON2bits.CSCNA = 0;   //no escanea las entradas
   AD1CON2bits.SMPI = 0b0000;     //genera una interrupcion por cada conversiones
   AD1CON2bits.BUFM = 0; //el buffer es configurado como uno de 16 words
   AD1CON2bits.ALTS = 0; //siempre se usa el MUX A
   
   AD1CON3bits.ADRC = 0; //clock from system clock
   AD1CON3bits.SAMC = 0b00101;   // Sanc = 3TAD + 0.750us = entre 4 y 5 TAD (se usa 5)
   AD1CON3bits.ADCS = 0b00000110;   //TDA = 6 * TCy = 6 * 2/32M = 6* 0.625uS = 375ns -> que el
                                    //tiempo de coonversion es 12 * TDA = 12*375ns = 4.5us
   
   AD1CHSbits.CH0NA = 0; //negative MuxA is VR-
   AD1CHSbits.CH0SA = 0; //positive MUXA is AN0
   
   AD1CSSL = 0; //scan no se usa.
   
   AD1CON1bits.ADON = 1;    //hailitamos el modulo AD
}

/*****************************************************************************/
//ConfigInterrupt: configarom la interrupcion por ADC
/*****************************************************************************/
void ConfigInterrupAD(void)
{
    INTCON1bits.NSTDIS = 0;     //el anidamiento de las interrupcciones esta habilitado (defecto)
    INTCON2bits.ALTIVT = 0;     //se selecciona el tabla estandar de interrupciones (defecto)
    
    //INTCON2bits.INT0EP = 0;     // seleccionamos el flanco de deteccion como de subida
    IFS0bits.AD1IF = 0;        // reseteamos el flag por las dudas.
    IPC3bits.AD1IP = 0b001;    //seleccionamos la prioridad como 1
    IEC0bits.AD1IE = 1;        //habilitamos la interrupcion por AD conversation
}

/*****************************************************************************/
//ConfigInterrupt: configarom la interrupcion por AD conversion
/*****************************************************************************/
void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void)
{
    //mdimos el tiempo que lleva hacer una conversion
    T3CONbits.TON = 0;  //deshabilitamos el timer3, y despues de 990us iniciara una conversion A/D
    IFS0bits.AD1IF = 0; //reseteamos la bandera
    
    LATBbits.LATB13 = 1;    //si RB13 esta en 0 se pone en 1
    __delay_us(1);      //retardo de 1us 
    LATBbits.LATB13 = 0;    //si RB13 esta en 1 se pone en 0
        
}

int main(void) 
{
    Config_Oscillator();
    Config_Port_Mcu();
    Config_PPS();
    //Config_AD();
    
    Config_OutputCompare1();
    ConfigInterrupT2();
    Config_Timer3();
    Config_AD();
    ConfigInterrupAD();
    Lcd_Init();        //inicializo al LCD
 
    //Presentamos la estacion en el LCD
    Lcd_Clear(); 
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("Estacion Soldado");
    Lcd_Set_Cursor(4,2);
    Lcd_Write_String("FENIX-140W");
    Lcd_Set_Cursor(6,4);
    Lcd_Write_String("V 0.1.0");
    //__delay_ms(4000);
    
  
    Menu0();    // llamo a la funcion que escribe en el LCD el menu0
    menu = 0;   //indico que estamos en el menu 0
   
    T2CONbits.TON = 1;  //habilitamos el timer2 y por ende al PWM1
    
    //ADC16Ptr = &ADC1BUF0;    //asignamos la direccion del pimer dato al puntero
    //AD1CON1bits.ASAM = 1;   //comiensa automaticamente el auto sampling
    
    /*
    Menu4();
    __delay_ms(4000);
    Menu5();
    __delay_ms(4000);
    Menu6();
    __delay_ms(4000);
    Menu7();
    __delay_ms(4000);
    Menu8();
    */
    
    while(1)
    {   
        /*
        ADCValue = 0;   //reseteamos la cuenta
        ADC16Ptr = &ADC1BUF0;    //asignamos la direccion del pimer dato al puntero
        IFS0bits.AD1IF = 0; //reseteamos la bandera
        AD1CON1bits.ASAM = 1;   //comiensa automaticamente el auto sampling
        
        while (!IFS0bits.AD1IF){}; //esperamos a que la conversion termine
        AD1CON1bits.ASAM = 0;  // yes then stop sample/convert
        ADCValue = *ADC16Ptr++;
        */
        
        /*
        //chequeamos que boton se oprime y se llama la funcion que lo atendera.
        if(Sensor_Mag == 1)
        {
            if(menu != 1)
            {
                __delay_ms(1000 * Sleep_Delay);     //aca esperamos el tiempo de sleep delay
                                                    //antes de entrar en modo sleep , Sleep_Delay se expresa en segundos
                Menu1();    // llamo a la funcion que escribe en el LCD el menu0
                menu = 1;   //indico que estamos en el menu 0
            }
        }
        else
        {
            if(menu == 1)   //si el menu es Ibernacion volvemos a la pantalla de normal (menu0)
            {
                Menu0();    // llamo a la funcion que escribe en el LCD el menu0
                menu = 0;   //indico que estamos en el menu 0
            }
            
        }
        
        if(B_Down == 1)
        {
            At_Button_Down();
            __delay_ms(500);
        }
        if(B_Menu == 1)
        {
            At_Button_Menu();
            __delay_ms(500);
        }
        if(B_Up == 1)
        {
            At_Button_Up();
            __delay_ms(500);
        }
         */
    }  
         
    
    return 0;
}

