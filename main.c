#include <wiringPi.h>
#include <pcf8574.h>
#include <lcd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <dirent.h>
#include <unistd.h>
#include <regex.h>   


#include <stdio.h>
#include <stdlib.h>

//I2C

#define        AF_BASE                  64
#define        AF_RS                (AF_BASE + 0)
#define        AF_RW                (AF_BASE + 1)
#define        AF_E                 (AF_BASE + 2)
#define        AF_LED               (AF_BASE + 3)
#define        AF_DB4               (AF_BASE + 4)
#define        AF_DB5               (AF_BASE + 5)
#define        AF_DB6               (AF_BASE + 6)
#define        AF_DB7               (AF_BASE + 7)   

//LES CONSTANTES
#define I2C_ADDRESS 0x27
#define  BUFSIZE  128
#define MAX 1024
//#define TEMPERATURE 25

//LES PIN
#define GPIO_MANUAL     23 //au pif
#define GPIO_AI         21 //au pif
#define GPIO_MENU       22 //au pif
#define TRIG            4  
#define ECHO            5  
#define PWM             18
#define A1	            17	
#define A2	            26
#define A3	            24
#define A4	            16
#define FORWARD         1
#define BACKWARD        2
#define RIGHT           3
#define LEFT            6

typedef enum {
    INIT,
    MENU,
    MANUAL,
    AI,
    END
} state_t;


//machine à etat
void DoState(state_t State);
state_t ChangeState(state_t StateDepart, state_t StateArrivee);
void ExitState(state_t State);
void EntryState(state_t State);
state_t Transition(state_t StateDepart);

//fct 
void i2cLcd(int i2cAddress);
void pinInit(void);
float getTemperature(void);
float getDistance(void);    
void displayData(void);
void fctManual(void);
void fctAI(void);
//char * getSensorName(void);

//fct motor
void allLow(void);
void forward(void);
void backward(void);
void turnRight(void);
void turnLeft(void);

static int lcdHandle;
static float temperature,distance;

int main(void) {  
    state_t State_courant;  
	//wiringPiSetup();
    //wiringPiSetupGpio();
    //pinInit();
    //i2cLcd(I2C_ADDRESS);
    printf("aa");
    getTemperature();
    EntryState(INIT);
    State_courant = INIT;
    while (1) {
        DoState(State_courant);
        State_courant = Transition(State_courant);
    }
    return 0;
}




void DoState(state_t State) {
    switch (State) {
        case INIT:
            break;
        case MENU:
            lcdClear(lcdHandle);
            lcdPosition(lcdHandle,0,0);
            lcdPrintf(lcdHandle,"MENU TO");
            cdPosition(lcdHandle,0,1);
            lcdPrintf(lcdHandle,"CHOOSE MODE");
            delay(500);
            break;
        case MANUAL:
            fctManual();
            displayData();
            break;
        case AI:
            fctAI();
            displayData();
            break;
        case END:
            break;
        default: 
            break;
    }   
}

state_t Transition(state_t StateDepart) {
    state_t StateArrivee = StateDepart;
        switch (StateDepart) {
        case INIT:
            if(digitalRead(GPIO_MENU)==1){
                StateArrivee = ChangeState(StateDepart,MENU);
            }
            break;
        case MENU:
            if(digitalRead(GPIO_MANUAL)==1){
                StateArrivee = ChangeState(StateDepart,MANUAL);
            }
            else if(digitalRead(GPIO_AI)==1){
                StateArrivee = ChangeState(StateDepart,AI);
            }
            break;
        case MANUAL:
            if(digitalRead(GPIO_MENU)==1){
                StateArrivee = ChangeState(StateDepart,MENU);
            }
            break;
        case AI:
            break;
        case END:
            break;
        default: 
            break;
        }
    return StateArrivee;
}

state_t ChangeState(state_t StateDepart, state_t StateArrivee) {
    ExitState(StateDepart);
    EntryState(StateArrivee);
    return StateArrivee;
}

void ExitState(state_t State) {
    switch (State) {
        case INIT:
            break;
        case MENU:
            digitalWrite(GPIO_MENU,LOW);
            break;
        case MANUAL:
            digitalWrite(GPIO_MANUAL,LOW);
            break;
        case AI:
            digitalWrite(GPIO_AI,LOW);
            break;
        case END:
            break;
        default: 
            break;
    }
}

void EntryState(state_t State) {
    switch (State) {
        case INIT:
            digitalWrite(GPIO_MENU,LOW);
            digitalWrite(GPIO_MANUAL,LOW);
            digitalWrite(GPIO_AI,LOW);
            lcdClear(lcdHandle);
            lcdPosition(lcdHandle,0,0);
            lcdPrintf(lcdHandle,"HELLO WORLD");
            printf("HELLO WORLD \n");
            delay(2000);
            break;
        case MENU:
            printf("MANUAL GPIO %d\n",GPIO_MANUAL);
            printf("AI GPIO %d\n",GPIO_AI);
            lcdClear(lcdHandle);
            lcdPosition(lcdHandle,0,0);
            lcdPrintf(lcdHandle,"MANUAL GPIO %d",GPIO_MANUAL);
            lcdPosition(lcdHandle,0,1);
            lcdPrintf(lcdHandle,"AI GPIO %d",GPIO_AI);
            delay(200);
            break;
        case MANUAL:
            for(int i=0;i<4;i++){
                lcdClear(lcdHandle);
                lcdPosition(lcdHandle,0,0);
                lcdPrintf(lcdHandle,"MANUAL");
                printf("MANUAL \n");
                delay(250);
            }
            break;
        case AI:
            for(int i=0;i<4;i++){
                lcdClear(lcdHandle);
                lcdPosition(lcdHandle,5,0);
                lcdPrintf(lcdHandle,"AI");
                printf("AI \n");
                delay(250);
            }
            break;
        case END:
            break;
        default: 
            break;
    }
}

void pinInit(void){  
    pinMode(FORWARD,INPUT);
	pinMode(BACKWARD,INPUT);
	pinMode(RIGHT,INPUT);
	pinMode(LEFT,INPUT);
	pinMode(ECHO, INPUT);  
	pinMode(TRIG, OUTPUT); 
    pinMode(A1,OUTPUT);
	pinMode(A2,OUTPUT);
	pinMode(A3,OUTPUT);
	pinMode(A4,OUTPUT);
	pinMode(PWM,PWM_OUTPUT);
    pinMode(GPIO_MENU,INPUT);
    pinMode(MANUEL,INPUT);
    pinMode(GPIO_AI,INPUT);
	allLow(); 
    for(int i=0;i<8;i++)
          pinMode(AF_BASE+i,OUTPUT);
} 



void displayData(void){
    temperature = getTemperature();
    distance=getDistance();
    if(temperature > ALERT_TEMP){
        //allumer led TEMPERATURE
        //allumer buzzer
        printf("temperartur alerte \n");
    }
    if(distance > ALERT_DISTANCE){
        printf("distance alert \n");
        //allumer led TEMPERATURE
        //allumer buzzer
    }
    printf("TEMPERATURE:%.2f",temperature);
    printf("DISTANCE:%.2f",distance);
    lcdPosition(lcdHandle,0,0);
    lcdPrintf(lcdHandle,"TEMPERATURE:%.2f",temperature);
    lcdPosition(lcdHandle,0,1);
    lcdPrintf(lcdHandle,"DISTANCE:%.2f",distance);
    delay(50);
}

void fctAI(void){
/*
    angle = getAngle();    
    if(angle == gauche){
        while(angle non ToutDroit){
            turnLeft();
        }
    }  
    else if (angle == droite){
        while(angle non ToutDroit){
            tourner à gauche
        }
    }
    else if (angle == ToutDroit){
        forward();
    }
*/
}

void fctManual(void){
    if(digitalRead(FORWARD)==1){
        digitalWrite(BACKWARD,LOW);
        digitalWrite(RIGHT,LOW);
        digitalWrite(LEFT,LOW);
        forward();
    }
    else if(digitalRead(BACKWARD)==1){
        digitalWrite(FORWARD,LOW);
        digitalWrite(RIGHT,LOW);
        digitalWrite(LEFT,LOW);
        backward();
    }
    else if(digitalRead(RIGHT)==1){
        digitalWrite(FORWARD,LOW);
        digitalWrite(BACKWARD,LOW);
        digitalWrite(LEFT,LOW);
        turnRight();
    }
    else if(digitalRead(LEFT)==1){
        digitalWrite(FORWARD,LOW);
        digitalWrite(RIGHT,LOW);
        digitalWrite(BACKWARD,LOW);
        turnLeft();
    }
}

void allLow(void){
	digitalWrite(A4,LOW);
	digitalWrite(A3,LOW);
	digitalWrite(A2,LOW);
	digitalWrite(A1,LOW);
}
	
void forward(void){
	digitalWrite(A1,HIGH);
	digitalWrite(A4,HIGH);
}	

void backward(void){
	digitalWrite(A2,HIGH);
	digitalWrite(A3,HIGH);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turnRight(void){
	
}

void turnLeft(void){

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float getTemperature(void){
	int fd, fd2, ret,temp;
	int i, j;
	char buf[BUFSIZE];
	char tempBuf[5];
    char *sensorName, *sensorPath;
	//pinMode(TEMPERATURE,INPUT);
    //sensorName = getSensorName();
    //sprintf(sensorPath, "/sys/bus/w1/devices/%s/w1_slave",sensorName);
    //printf("path : %s",sensorPath);
    fd2 = open("/sys/bus/w1/devices/28-062131f3f529/w1_slave", O_RDONLY); //trouver le bon device 

    if(-1 == fd2){
        perror("open device file error");
        return 1;
    }
    while(ret != 0){
        ret = read(fd2, buf, BUFSIZE);
        if(-1 == ret){
            if(errno == EINTR){
                continue;	
            }
            perror("read()");
            close(fd2);
            return 1;
        }
    }

    for(i=0;i<sizeof(buf);i++){
        if(buf[i] == 't'){
            for(j=0;j<sizeof(tempBuf);j++){
                tempBuf[j] = buf[i+2+j]; 	
            }
        }	
    }

    temp = (float)atoi(tempBuf) / 1000; // comment this line

    close(fd);
    //lcdClear(fd);
    //lcdPosition(fd, 0, 0);
    //lcdPrintf(fd,"%d C",temp);
    //delay(1000);
	
	
	return temp;
}


float getDistance(void)  
{  
	struct timeval tv1;  
	struct timeval tv2;  
	long start, stop;  
	float dis;  

	digitalWrite(TRIG, LOW);  
	delayMicroseconds(2);  

	digitalWrite(TRIG, HIGH);  //produce a pluse
	delayMicroseconds(10); 
	digitalWrite(TRIG, LOW);  

	while(!(digitalRead(ECHO) == 1));  
	gettimeofday(&tv1, NULL);           //current time 

	while(!(digitalRead(ECHO) == 0));  
	gettimeofday(&tv2, NULL);           //current time  

	start = tv1.tv_sec * 1000000 + tv1.tv_usec; 
	stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;  

	dis = (float)(stop - start) / 1000000 * 34000 / 2;  //count the distance 

	return dis;  
}  

void i2cLcd(int i2cAddress){

    pcf8574Setup(AF_BASE,i2cAddress); //pcf8574 I2C address
    
    lcdHandle = lcdInit (2, 16, 4, AF_RS, AF_E, AF_DB4,AF_DB5,AF_DB6,AF_DB7, 0,0,0,0) ;
    
    if (lcdHandle < 0)
    {
        fprintf (stderr, "lcdInit failed\n") ;
        exit (EXIT_FAILURE) ;
    }
    digitalWrite(AF_LED,1);
    digitalWrite(AF_RW,0);
}


char * getSensorName(void){
    regex_t regex;
    int reti;
    char *res;
    reti = regcomp(&regex, "^28-0..........9$", 0);
    if (reti) {
        fprintf(stderr, "Could not compile regex\n");
        exit(1);
    }

    struct dirent *dir;
    // opendir() renvoie un pointeur de type DIR. 
    DIR *d = opendir("../test_grep/dir_test"); 
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            reti = regexec(&regex, dir->d_name , 0, NULL, 0);
            if (!reti) {
                return dir->d_name;
            }
            else{
                res = "no directory";
            }
        }
        closedir(d);
    }
    return res;
}