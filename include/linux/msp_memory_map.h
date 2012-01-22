#define ID                                              0x00
#define REV_ID                                          0x01

#define STEP_COUNT_ROBUSTNESS                           0x02
#define INETER_STEP_COUNT                               0x03
#define NORMAL_MODE_TIMEOUT                             0x04
#define MAGNETIC_DECLINATION                            0x05
#define GPS_MEMS_SWITCH_ACCURACY                        0x06
#define RESERVED                                        0x07

#define STEP_LENGTH_WALK                                0x0A
#define STEP_LENGTH_RUN                                 0x0B
#define HEIGHT                                          0x0C
#define WEIGHT                                          0x0D
#define AGE                                             0x0E
#define SEX                                             0x0F

#define AP_POSIX_TIME                                   0x10

#define PRESSURE_SEA_LEVEL                              0x14

#define DEVICE_ORIEN_HEAD                               0x19
#define DEVICE_ORIEN_PITCH                              0x1B
#define DEVICE_ORIEN_ROLL                               0x1D

#define MODULE_CONFIG                                   0x1F

#define ANY_MOTION_SLOPE                                0x20
#define ANY_MOTION_DURATION                             0x21

#define ACCEL_UPDATE_RATE                               0x22
#define MAG_UPDATE_RATE                                 0x23
#define POWER_MODE                                      0x24

#define GPS_LATITUDE                                    0x25
#define GPS_LONGITUDE                                   0x29
#define GPS_HEADING                                     0x2D
#define GPS_HOR_ACCURACY                                0x2F
#define GPS_ALTITUDE                                    0x30
#define GPS_VER_ACCURACY                                0x34
#define RESERVED_2                                      0x35
#define GPS_SPEED                                       0x36

#define INTERRUPT_MASK                                  0x37

#define INCLIN_ACCUM_TIME                               0x38

#define SPEED_CADENCE_ACCUM_TIME                        0x39

#define INTERRUPT_STATUS                                0x3A

#define ERROR_STATUS                                    0x3B

#define TOTAL_STEP_COUNT                                0x3C
#define STEP_COUNT_LAST_ACT                             0x3E
#define STEP_COUNT_CUURENT_ACT                          0x40
#define TIME_ACT_CHANGE                                 0x42
#define ACTIVITY_DETECTION                              0x46
#define SPEED                                           0x47
#define CADENCE                                         0x49
#define DISTANCE_LAST_ACT                               0x4A
#define DISTANCE_CURRENT_ACT                            0x4C

#define CURRENT_ALTITUDE                                0x4E
#define CURRENT_PRESSURE                                0x50
#define ASCENT_LAST_ACT                                 0x54
#define ASCENT_CURRENT_ACT                              0x56
#define DESCENT_LAST_ACT                                0x58
#define DESCENT_CURRENT_ACT                             0x5A
#define CURRENT_INCLIN                                  0x5C

#define WALK_DIRN_QUALITY                               0x5D
#define WALK_DIRN_HEAD                                  0x5E
#define DEVICE_HEADING                                  0x60
#define TAP_TAP_3D                                      0x62

#define TEMPERATURE_MSP430                              0x63
#define TEMPERATURE_BMP085                              0x65
#define TEMPERATURE_TMP112                              0x67

#define ACCEL_X                                         0x69
#define ACCEL_Y                                         0x6B
#define ACCEL_Z                                         0x6D

#define MAG_HX                                          0x6F
#define MAG_HY                                          0x71
#define MAG_HZ                                          0x73

#define MSP_LATITUDE                                    0x75
#define MSP_LONGITUDE                                   0x79
#define CAL_STATUS                                      0X7D

#define NO_OF_ACCCEL_DATA_SETS_TO_READ                  0x7E
#define NO_OF_MAGNETIC_DATA_SETS_TO_READ                0x80

#define RESET                                           0x82

#define READ_ACCEL_BUFFER                               0x83
#define READ_MAG_BUFFER                                 0x84
#define TOTAL_DISTANCE                                  0x85
#define EQUIPMENT_TYPE                                  0x89

/*#define MONITORME_10SEC_METS                            0x8e*/
#define MONITORME_60SEC_METS                            0x8f

#define ELLPTICAL_STRIDE_COUNT                          0x90
#define STAIRMASTER_STEP_COUNT                          0x92
#define MONITORME_10SEC_METS                            0x95

#define OMAP_OFF_MODE                                   0x99

#define CALIBRATION_SPEED_WALK                          0x9A
#define CALIBRATION_SPEED_RUN                           0x9C
#define CALIBRATION_SPEED_JOG                           0x9B
#define CALIBRATION_ON                                  0x9D
/* 12 bytes from this address */
#define CALIBRATION_TABLE                               0x9E
#define GPS_CAL_TABLE                                   0xBB // 22 bytes
#define USER_CAL_TABLE                                  0xD1 // 22 bytes (0 - 10 miles)
#define WORKOUT_MSP_DIST                                0xE7
#define WORKOUT_USER_DIST                               0xEB



/* Mask values */
#define M_ACTIVITY_CHANGE              0x01
#define M_CALIBRATION_COMPLETE_JOG     0x02
#define M_TAP_TAP                      0x04
#define M_ANY_MOTION                   0x08
#define M_MME_60                       0x10
#define M_ACTIVE_MODE                  0x20
#define M_CALIBRATION_COMPLETE_WALK    0x40
#define M_CALIBRATION_COMPLETE_RUN     0x80

#define M_TAP_SENSING                 0x01
#define M_PEDOMETER                   0x02
#define M_PRESSURE                     0x04
#define M_ECOMPASS                     0x08
#define M_RAW_ACCEL_DATA_BUFFER        0x10
#define M_TEMPERATURE                 0x20
#define M_GPS_ASSISTED_CALIBRATION    0x40
#define M_RAW_ECOMPASS_DATA_BUFFER     0x80
