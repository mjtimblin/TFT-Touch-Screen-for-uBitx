// ----------------------------- Various displays -----------------------------
//#define elegoo923             		 // Joe's 923elegoo displ
//#define MCUF0x154             		 // VU2SPF's test display
//#define PL0x9341              		 // VU2SPF's Potential Lab display BUT syst
//#define VE0x7783              		 // Ventor Tech display SPB/ubitx
//#define REJI5408              		 // Rejimon's display from Robodo detected as 5408 but works as 9320
//#define IL9325                		 // Robodo 1/18
//#define Sa35_9486             		 // sarmas 3.5 inch
//#define pl0x2053		        		 // poten lab 0x9341 now shows as 2053??
#define elegoo9341	           		 // Elegoo 2.8 inch

// --------------------------------- Settings ---------------------------------
#define CALLSIGN           	  "XXXXXX"
#define SM_speed           	  5    // speed of updating S meter
#define SWR_speed          	  5	 // speed of updating SWR display
#define si5351correction   	  0    // IF THERE IS ANY (check using calibrate program in the etherkit Si5351 library examples)

// ----------------------------- Pin allocations ------------------------------
// uBitx digital interface
#define TX_RX_PIN               14   // Required
#define CW_TONE_PIN             15   // Required
#define TX_LPF_A_PIN            16   // Required
#define TX_LPF_B_PIN            17   // Required
#define TX_LPF_C_PIN            18   // Required
#define CW_KEY_PIN              19   // Required

// TFT analog pins
#define LCD_RD             	  A0   // Required
#define LCD_WR             	  A1   // Required
#define LCD_CD             	  A2   // Required
#define LCD_CS             	  A3   // Required
#define LCD_RESET          	  A4   // Required

#define CW_KEY_INPUT_PIN   	  A12  // Required
#define SWR_REVERSE_PIN	   	  A8
#define SWR_FORWARD_PIN	   	  A9
#define ENCODER_A_PIN      	  A14
#define ENCODER_B_PIN      	  A13 
#define S_METER_PIN        	  A15
#define PTT_PIN     				  26
#define BAND_SELECT_UP_PIN 	  53
#define BAND_SELECT_DOWN_PIN    31
#define MEMORY_UP_PIN           32
#define MEMORY_DOWN_PIN         33
#define SELECT_SIDE_BAND_PIN    34
#define VFO_PIN                 36
#define STEP_SIZE_UP_PIN        48
#define STEP_SIZE_DOWN_PIN      38
#define VFO_TO_MEMORY_PIN       39
#define MEMORY_TO_VFO_PIN       40
#define TOGGLE_SPLIT_MODE_PIN   43
