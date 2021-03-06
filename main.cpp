/* The "Eight" Coursework
 * Guillaume Fray
 * 17008776
 */
 
#include "mbed.h"
#include "platform/mbed_thread.h"
#include "SRF05.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"


// Blinking rate in milliseconds
#define BLINKING_RATE_MS                                                    3000

DigitalIn button (PC_13); // PC_13 --> user button
// PB_0  Motor Phase A
// PA_4  Motor Phase B

// To turn left and right - RANGE: |1000-2000| max values left & right
PwmOut Drive_pin_A(PA_8);  //steering servo PWM output pin D7

// when C is high and D is low => Forward
// when C is low and D is high => Backward
// when C and D are both = 0 => Motor stops
PwmOut Drive_pin_C(PB_10); //Motor drive PWM output pin D6
PwmOut Drive_pin_D(PB_4);  //Motor drive PWM output pin D5

int Turn_right=2;
int Turn_left=0;
int Speed1=10;              //Half power
int Speed2=20;              //Full power
int PW_Period=20;


//Ultrasonic Sensors
SRF05 srf(PA_10,PB_5); //Right - Type HY-SRF05
SRF05 hc(PA_9,PC_7); //Left - Type HC-SR04
Serial pc(USBTX,USBRX); //tx, rx

//Time-of-Flight Sensor
//VL53L0X lidar(PB_9, PB_8); // old library









/* --- VL53L0X functions required (from library's example) --- */

    VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        uint8_t NewDatReady=0;
        uint32_t LoopNb;
    
        // Wait until it finished
        // use timeout to avoid deadlock
        if (Status == VL53L0X_ERROR_NONE) {
            LoopNb = 0;
            do {
                Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
                if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                    break;
                }
                LoopNb = LoopNb + 1;
                VL53L0X_PollingDelay(Dev);
            } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);
    
            if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
                Status = VL53L0X_ERROR_TIME_OUT;
            }
        }
    
        return Status;
    }
    
    VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        uint32_t StopCompleted=0;
        uint32_t LoopNb;
    
        // Wait until it finished
        // use timeout to avoid deadlock
        if (Status == VL53L0X_ERROR_NONE) {
            LoopNb = 0;
            do {
                Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
                if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                    break;
                }
                LoopNb = LoopNb + 1;
                VL53L0X_PollingDelay(Dev);
            } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);
    
            if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
                Status = VL53L0X_ERROR_TIME_OUT;
            }  
        }   
        return Status;
    }

/* --- END of VL53L0X functions --- */


/* --- Motion Control Functions --- */
        // Move Forward
        void forward() {   
            Drive_pin_D.pulsewidth_ms(0);
            Drive_pin_C.pulsewidth_ms(Speed1);    //half power    
        }

        // Move Backward
        void back() {  
            Drive_pin_D.pulsewidth_ms(Speed1);
            Drive_pin_C.pulsewidth_ms(0);    //half power 
        }

        // Stop
        void stop() {   
            Drive_pin_D.pulsewidth_ms(0);          
            Drive_pin_C.pulsewidth_ms(0);        //drive motor off
        } 
            
         // Turn RIGHT
        void right() {         
            Drive_pin_A.pulsewidth_us(1100);
        }
        
        // Turn LEFT
        void left() {
            Drive_pin_A.pulsewidth_us(1900);
        }
            
        // Neutral Steering
        // Put WHEELS back to normal --> STRAIGHT
        void neutral() {
            Drive_pin_A.pulsewidth_us(1500);
        }    
















    
    
    
    
    
    
    

int main()
{
    // Initialise the digital pin LED1 as an output
    //DigitalOut led(LED1); //Actually LED2 (LED1 is for uploading program)
    
    /* --- Setting the pulse width for the servos --- */   
        Drive_pin_A.period_ms(PW_Period);
        Drive_pin_C.period_ms(PW_Period);
        Drive_pin_D.period_ms(PW_Period);
    
    /* --- VL53L0X sensor set up --- */
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        VL53L0X_Dev_t ToF;
        VL53L0X_Dev_t *pToF = &ToF;
        VL53L0X_Version_t                   Version;
        VL53L0X_Version_t                  *pVersion   = &Version;
        VL53L0X_DeviceInfo_t                DeviceInfo;
        
        int32_t status_int;
        
        // Initialize Comms
        pToF->I2cDevAddr      = 0x52;
        pToF->comms_type      =  1;
        pToF->comms_speed_khz =  400;
    
        int addr;    
        addr = VL53L0X_scan();
        
        uint16_t osc_calibrate_val=0;
        Status = VL53L0X_RdWord(&ToF, VL53L0X_REG_OSC_CALIBRATE_VAL,&osc_calibrate_val);
        Status = VL53L0X_DataInit(&ToF);
        
        VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
        VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;
        Status = VL53L0X_ERROR_NONE;
        uint32_t refSpadCount;
        uint8_t isApertureSpads;
        uint8_t VhvSettings;
        uint8_t PhaseCal;
        
        Status = VL53L0X_StaticInit(pToF); // Device Initialization
        Status = VL53L0X_PerformRefCalibration(pToF,
        &VhvSettings, &PhaseCal); // Device Initialization
        
        Status = VL53L0X_PerformRefSpadManagement(pToF,
        &refSpadCount, &isApertureSpads); // Device Initialization
        
        Status = VL53L0X_SetDeviceMode(pToF, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        Status = VL53L0X_StartMeasurement(pToF);
        
        uint32_t measurement;
        Status = VL53L0X_StartMeasurement(pToF);
        Status = WaitMeasurementDataReady(pToF);
        Status = VL53L0X_GetRangingMeasurementData(pToF, pRangingMeasurementData);
        
        // ToF reading variable
        int tof_reading;
    
    

    while (true) {
        //led = !led;
        //thread_sleep_for(BLINKING_RATE_MS);
        tof_reading = pRangingMeasurementData->RangeMilliMeter;
   
        pc.printf("Right Sensor: %.1f cm\n", srf.read());
        pc.printf("Left Sensor: %.1f cm\n", hc.read());     
        printf("ToF: %d mm\n", tof_reading);
        // Get a new ToF measurement
        VL53L0X_GetRangingMeasurementData(pToF, pRangingMeasurementData);
        

            


         //pc.putc(pc.getc());// You gotta push the reset button every time when using this.
         
         
         
         wait(0.5);
         
         
         
        /*
         
        // Forward
        Drive_pin_D.period_ms(PW_Period);    
        Drive_pin_D.pulsewidth_ms(0);
        Drive_pin_C.period_ms(PW_Period);
        Drive_pin_C.pulsewidth_ms(Speed1);    //half power      
        wait (1.6);
        
        // Turn RIGHT 
        Drive_pin_A.period_ms(PW_Period);   //Steering Servo period
        Drive_pin_A.pulsewidth_us(1200);   
        wait (2);
        
        // Turn LEFT
        Drive_pin_A.period_ms(PW_Period);   //Steering Servo period
        Drive_pin_A.pulsewidth_us(1900);   
        wait (3);
        
        // Straight
        Drive_pin_A.period_ms(PW_Period);   //Steering Servo period
        Drive_pin_A.pulsewidth_us(1500);   
        wait (4);
        
        // Turn RIGHT 
        Drive_pin_A.period_ms(PW_Period);   //Steering Servo period
        Drive_pin_A.pulsewidth_us(1200);   
        wait (4);
        
        */
        
        
    }
}





/*         
        // Forward
        Drive_pin_D.period_ms(PW_Period);    
        Drive_pin_D.pulsewidth_ms(0);
        Drive_pin_C.period_ms(PW_Period);
        Drive_pin_C.pulsewidth_ms(Speed1);    //half power      
        wait (1.6);
        
        // Stop
        Drive_pin_D.period_ms(PW_Period);    
        Drive_pin_D.pulsewidth_ms(0);        
        Drive_pin_C.period_ms(PW_Period);    
        Drive_pin_C.pulsewidth_ms(0);        //drive motor off       
        wait (3);
        
        // Backward
        Drive_pin_D.period_ms(PW_Period);    
        Drive_pin_D.pulsewidth_ms(Speed1);
        Drive_pin_C.period_ms(PW_Period);
        Drive_pin_C.pulsewidth_ms(0);    //half power
        wait (1.3);
        
        // Stop
        Drive_pin_D.period_ms(PW_Period);    
        Drive_pin_D.pulsewidth_ms(0);        
        Drive_pin_C.period_ms(PW_Period);    
        Drive_pin_C.pulsewidth_ms(0);       
        wait (3);
        
        // Turn RIGHT - 1000 
        Drive_pin_A.period_ms(PW_Period);   //Steering Servo period
        Drive_pin_A.pulsewidth_us(1000);   // MAX Steering right
        wait (2);
        
        // Turn LEFT - 2000
        Drive_pin_A.period_ms(PW_Period);   //Steering Servo period
        Drive_pin_A.pulsewidth_us(2000);   // MAX Steering left
        wait (2);
        
        // Put WHEELS back to normal --> STRAIGHT
        Drive_pin_A.pulsewidth_us(1500); // Neutral steering
        wait (3);  
*/

