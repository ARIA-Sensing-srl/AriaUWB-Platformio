/**
 * @file app.c
*
 *  Created on: Jan 23, 2024
 *      Author: ARIA Sensing
 *      Copyright: ARIA Sensing, 2023 - 
 */

#include <uart_app.h>
#include "app.h"
#include "main.h"
#include "stdint.h"
#include "stddef.h"
#include "hal_evt_ctl.h"
#include "hal_intc.h"
#include "ipc_commands.h"
#include "ipc.h"
#include "math_utils.h"
#include "comm_cb_types.h"
#include "comunication_task.h"
#include "radarcommands.h"
#include "models.h"
#include "version.h"
#include "processing.h"
#include "string.h"
#include "stdlib.h"
#include "comm_sspi.h"
#include "cv32e40_core.h"
#include "hal_clkmgr.h"


//** DEFINES START **//
/** @defgroup grpSwEvent co-processor SW events
 *  Mapping between software events and co-processor generation events\n
 *  The application processor send to the co-processor which event number to trigger when the corresponding co-processor event occurs
 *@{*/
/** @brief Overrun event ID configuration*/
#define DEF_OVERRUN_EVT_ID 		EVC_EVT_APBEVT0_ID
/** @brief Acquisition event ID */
#define DEF_ACQREADY_EVT_ID 	EVC_EVT_APBEVT1_ID
/** @brief Acquisition event ID */
#define DEF_PPBUF0RDY_EVT_ID 	EVC_EVT_APBEVT2_ID
/** @brief ping-pong buffer 0 event ID */
#define DEF_PPBUF1RDY_EVT_ID 	EVC_EVT_APBEVT3_ID
/** @brief Inter-processor communication timeout */
#define DEF_IPCIO_TOms 			500
/** @brief acquisition done flag */
#define ACQ_FLAG_DONE 		0
/** @brief buffer0 buffer ready flag */
#define ACQ_FLAG_BUF0_RDY 	1
/** @brief buffer1 buffer ready flag */
#define ACQ_FLAG_BUF1_RDY 	2
/** @brief Overrun error flag */
#define ACQ_FLAG_OVERRUN	3
/**@}*/

/* USER_ENTRY User could change parameters here*/
/** @defgroup grpRdrInit Radar initialization constants
 *  These are the default parameters loaded in startup
 * @{*/
/** @brief Frame rate in Hz */
#define DEF_FRAME_RATE			10
/** @brief De-clatter time constant (sec)*/
#define DEF_DECLUTTER_CONSTANT	5.0
/** @brief Max number of scan sequence supported
 * Used for internal array initialization */
#define DEF_MAX_SEQUENCES		16
/** @brief ADC scaling factor, for data normalization */
#define DEF_ADC_SCALING_F		(1.0/64.0)
/**  @brief Minimum acquisition range (meters)*/
#define DEF_RADAR_RANGE_MIN 	0
/** @brief Maximum acquisition range (meters)*/
#define DEF_RADAR_RANGE_MAX 	5
/**  @brief Range offset adjust (meters)*/
#define DEF_RADAR_OFFSET 		-1
/** @brief Default variable gain amplifier gain*/
#define DEF_RADAR_VGA_GAIN 		20
/** @brief Default power setup*/
#define DEF_RADAR_TX_POWER		HUTXPWR_HIGH
/** @brief Pulse bandwidth in MHz*/
#define DEF_RADAR_BW_MHZ		1000
/** @brief Carrier frequency in MHz*/
#define DEF_RADAR_CARRIER_MHZ	8064
/** @brief co-processor pre-processing options*/
#define DEF_RADAR_PREPROCESSING_OPT 	(IPC_CMD_ARG_POSTPROC_DCREM_MASK | IPC_CMD_ARG_POSTPROC_CORR_MASK )
/** @brief enable radar operation at startup if set to 1*/
#define DEF_RADAR_AUTOSTART		1
/** @brief power save mode selected*/
#define DEF_PWRSAVE_MODE 		HUPWR_0
/**@}*/
/* ~USER_ENTRY User could change parameters here~*/

//communication
/** @brief Variable for message handlers, set chuck size for streaming transmission */
#define DEF_COMM_MAX_STREAM_CHUNK_SIZE 1024


/** @defgroup grpAppPwrSave Application level power seving modes
 *@{*/
/** @brief No local power mode */
#define DEF_LPMODE_NONE 		0
/** @brief Power save mode 0
 * Device executes WFI instruction on top of the superloop*/
#define DEF_LPMODE_0			1
/** @brief Power save mode 1
 * Designed for clock switching (currently not implemented)*/
#define DEF_LPMODE_1			2
/**@}*/



//Options for co-processor assisted DMA
/**
 * @def OPT_COPR_ASSISTED_DMA
 * @brief enable co-processor assisted DMA operations
 * @def DATAMOVERS_NUM_HANDLERS
 * @brief number of DMA handlers (valid if OPT_COPR_ASSISTED_DMA is enable)
 */
#define OPT_COPR_ASSISTED_DMA 		1
#define DATAMOVERS_NUM_HANDLERS 	8

//Radar acquisition mode options
/**
 * @def OPT_BUFFER_MODE_PINGPONG
 * @brief Enable ping-pong acquisition mode
 * @def OPT_ACQ_STOP_IF_OVERRUN
 * @brief If set, stop radar operations if overrun error occurs
 */
#define OPT_BUFFER_MODE_PINGPONG 	1
#define OPT_ACQ_STOP_IF_OVERRUN 	0	//select the behavior if overrun is detected, if set acquisition is stopped

//Reconstruction options
/**
 * @def OPT_RECONSTRUCTION_ENABLE
 * @brief Enable 2D/3D processing
 * @def OPT_3D_MODE
 * @brief If set, 3D reconstruction mode is enable
 */
#define OPT_RECONSTRUCTION_ENABLE 	1	//enable the 2D/3D reconstruction
#define OPT_3D_MODE 				0	//enable 3D, if enabled, the reconstruction algorithm compute the volume

//Communication interfaces options
/**
 * @def OPT_ENABLE_SSPI
 * @brief Enable slave SPI communication channel support
 */
#define OPT_ENABLE_SSPI 			0	//enable slave SPI channel

#if (DEF_BOARD_MODEL == AHM3D) || (DEF_BOARD_MODEL == AHM3DSC_RC1) || (DEF_BOARD_MODEL == AHM3DSC_RC2)
#undef OPT_3D_MODE
#define OPT_3D_MODE 				1	//set 0 only for test
#endif



//this "include" section is out of order 'cause depends on some opt_ settings
#if OPT_3D_MODE
#include "reconstruction_3d.h"
#else
#include "reconstruction_2d.h"
#endif



//Communication interface IDs
/**
 * \defgroup grpCommIF Communication interface
 * Used by message handlers to route messages to proper interface
 */
/**@{*/
/** @brief UART channel ID */
#define INTERFACE_ID_COMM 	0
/** @brief SPI channel ID*/
#define INTERFACE_ID_SPI 	1


//data request handler constants
/** @brief data request FIFO mask */
#define DATAREQ_FIFO_MASK 	3
/** @brief data request FIFO size*/
#define DATAREQ_FIFO_SIZE 	(DATAREQ_FIFO_MASK+1)

//bit 0-3 type, bit 4-7 channel, 8-11 format
/** @brief RAW type data request ID */
#define DATAREQ_CODE_RAW	0
/** @brief De-clatter data request ID*/
#define DATAREQ_CODE_DECL	1	//follow the encoding of the elab command
/** @brief Processed data request ID*/
#define DATAREQ_CODE_OUT	2	//follow the encoding of the elab command
/** @brief Stream data request boundary (used in communication parameter verification)*/
#define DATAREQ_CODE_MAX	DATAREQ_CODE_OUT	//for parameter boundary verification
/** @brief Image/Volume data request*/
#define DATAREQ_CODE_IMG	3	//this code instruct the handler about the requested data type

/** @brief Channel specific data request, all channel code*/
#define DATAREQ_CH_ALL		0xF

/** @brief Q.7 data format request*/
#define DATAREQ_FMT_Q7		0
/** @brief Q.15 data format request*/
#define DATAREQ_FMT_Q15		1
/** @brief Q.31 data format request*/
#define DATAREQ_FMT_Q31		2
/** @brief F32 data format request*/
#define DATAREQ_FMT_F32		3
/** @brief Q16 data format request*/
#define DATAREQ_FMT_F16		4
/** @brief Q16 complex data format request (used for image)*/
#define DATAREQ_FMT_F16CPLX	5
/** @brief Data request coding delimiter*/
#define DATAREQ_FMT_MAX		DATAREQ_FMT_F16CPLX		//for parameter boundary verification
/**@}*/

//acq_flag







#if OPT_3D_MODE==0
//2D reconstruction defines
#define DEF_PHI_MIN_RAD 		(-45.0*M_PI/180.0)
#define DEF_PHI_MAX_RAD 		(45.0*M_PI/180.0)
#define DEF_PHI_STEP_RAD 		(5.0*M_PI/180.0)
#define DEF_RHO_STEP			0.05
#endif

//3D reconstruction defines
#if OPT_3D_MODE
#define DEF_EL_MIN_RAD 			(-45.0*M_PI/180.0)		//elevetion default paramters
#define DEF_EL_MAX_RAD 			(45.0*M_PI/180.0)
#define DEF_EL_STEP_RAD 		(7.5*M_PI/180.0)
#define DEF_THETA_MIN_RAD 		(-45.0*M_PI/180.0)		//azimuth default paramters
#define DEF_THETA_MAX_RAD 		(45.0*M_PI/180.0)
#define DEF_THETA_STEP_RAD 		(7.5*M_PI/180.0)
#define DEF_RHO_STEP			0.15
#endif

//** DEFINES END **//

//** MACRO START **//
/** @defgroup grpCommMacro Communication handling macros
 * This group of macros are used by communication routine to handle messages and requests
 *@{*/
/** @brief Data request encode
 * This function encode a data request, used before pushing the request into the FIFO */
#define mDATAREQ_encode(type, fmt, ch)	((type) | ((fmt) << 8) | ((ch & 0xF) << 4))
/** @brief Retrieve request type from encoded request (popped from FIFO)*/
#define mDATAREQ_get_type(x) ((x) & 0xF)
/** @brief Retrieve required data format from encoded format (popped from FIFO)*/
#define mDATAREQ_get_fmt(x) (((x)>>8) & 0xF)
/** @brief Retrieve required data channel from encoded format (popped from FIFO)*/
#define mDATAREQ_get_ch(x) (((x)>>4) & 0xF)


/**@}*/
/** @brief Auxiliary macro for mark unused variables */
#define UNUSED(x) x __attribute__((unused))

#if (OPT_3D_MODE)
#if (OPT_DETECTION_ALGO_ENABLE)
#error "3D mode doesn't support detection algorithm"
#endif
#if (OPT_TRAKING_ALGO_ENABLE)
#error "3D mode doesn't support tracking algorithm"
#endif
#endif
//** MACRO END **//

//** TYPES START **//
/**
 * Data request FIFO entry
 */
typedef struct _datareq_t{
	uint16_t code;			/**< Encoded request */
	uint16_t interfaceID;	/**< Interface ID */
	radar_command cmd;		/**< Command */
}datareq_t;

/**
 * Antenna structure array
 */
typedef struct _ant_t{
	float x;			/**< Antenna's X position */
	float y;			/**< Antenna's Y position */
	float delay_s;		/**< Antenna's delay */
	float ampl;			/**< Antenna's amplitude/polarity correction */
}ant_t;
//** TYPES END **//

//** PRIVATE VARIABLES START **//

static HydrUDriver_t * pprivDriver;
static uint8_t running=0;		//asserted when system is running
static uint8_t stop_req = 0;	//asserted from communication interface request for stop
static uint8_t start_req = 0;	//asserted from communication interface request for start
static uint8_t next_buffer_expected;	//ensure syncronization in pp mode, when acquisition is ready the routine check the expected buffer (active only in continous mode)
static uint8_t acqMode = IPC_CMD_ARG_ACQUISITION_MODE_SINGLE;
static uint8_t lpMode = 0;

/**
 * @brief Acquisition flags
 * This flag signals the application processor about acquisition state (set and reset into event's ISR's handler)
 */
static volatile struct _acq_flags{
	unsigned acqDone: 3;	/**< Acqusition done flag*/
	unsigned buf0rdy: 1;	/**< Buffer0 ready flag*/
	unsigned buf1rdy: 1;	/**< Buffer1 ready flag*/
	unsigned overrun: 1;	/**< Overrun error*/
}acq_flags;
static declattuer_constants_t declConst;

static HydrUDrv_acqHandler_t acqHandler[2];		//ping pong buffer
static HydrUDrv_channels_t acqChannelsList[DEF_MAX_SEQUENCES];
static uint32_t acqDataReadyBuffer[2];

//distorsion compensation
static float dist_alpha = 1;
static float dist_nu = 0;

//local memory for elaboration
static complex_float temp_raw[512]; //temporary storage for current data stream
static complex_float temp_clutter[512];
static complex_float temp_output[512];
static proc_stream_t temp_stream;

//contain processed data of each stream
/**
 * This structure include the reference to the current processed streams\n
 */
static struct _streams_container{
	proc_stream_t * pstreams; 	/**< pointer to processed streams array*/
	int numStreams;				/**< number of streams*/
}streams_container;

//corpocessor data mover handlers
static volatile ipc_cmd_datamover_t dataMoversHandler[DATAMOVERS_NUM_HANDLERS] __attribute__((section(".shared")));

//circular buffer for incoming data requests
/**
 * @breif data request FIFO
 * This FIFO buffers incoming data requests\n
 * These request must be processed when a new stream/image is ready, so the request coming from UART/SPI
 * is pushed inside this FIFO and processed right after the processing and generation of new data*/
static struct _datareqfifo{
	uint8_t rp;
	uint8_t wp;
	datareq_t fifo[DATAREQ_FIFO_SIZE];
}datareqfifo;


static uint8_t commdata_type = DATAREQ_CODE_RAW;
static uint8_t commch_req = DATAREQ_CH_ALL;
static uint8_t commfmt_req = DATAREQ_FMT_F32;

static uint32_t lastScanDone_ticks;
static uint32_t lastScanPeriod_ticks;


#if OPT_3D_MODE == 0
//2D
static recon_handler_t rec_h;
#else
static recon3d_handler_t rec3d_h;
#endif
static recon_stream_t rec_streams[DEF_MAX_SEQUENCES];
static int rec_streams_num = 0;	//number of reconstruction stream used
#if OPT_3D_MODE == 1
static recalgs_t selAlgo = RA_DAS;
#else
static recalgs_t selAlgo = RA_DMAS;
#endif
//static canvas_t * rec_canvas;
/**
 * This structure contain the pointer to the image's data buffer
 * */
static struct _canvas_struct_t{
	canvas_t * rec_canvas;	/**< Image/Volume buffer */
	bool iscomplex;			/**< Complex format masking */
}canvas_struct;
static bool exe_reconstruction; //this flag disable reconstruction
static bool user_exe_reconstruction = true; //user option, allow user to disable recontruction algo (set if direct manipulation of the data is required)

static float rec_fcarrier_norm;

//Antennas array definitions
/*
 * 2D, module lays on plane XY, propagation direction +Z
 * Azimuth = 0, Zenith span from -Steering; +Steering
 * */
/*
 * 3D, module lays on YZ plane, propagation direcion +X
 * Antenna is remapped as x2d -> y3d, y2d -> z3d
 * Azimuth = (-steering, +steering) Elevation (-Steering; + Steering)
 *
 * */

/**
 * @def ANT_ARRAY_SELECTED
 * @brief Defined when board model is selected
 * This is a compilation flag enabled when the board model is selected.
 * If the user doesn't select the board, an error is triggered
 * @def DEF_SCAN_SEQ_SIZE
 * @brief Default scan sequence items
 * This parameter is used during default radar parameters loading
 * @def DEF_ITERATIONS
 * @brief Default number of iterations, this value is the suggested iteration amount according to board model
 * */

#if DEF_BOARD_MODEL == AHM2D
#define ANT_ARRAY_SELECTED
static ant_t txAntennas[4]= {\
	{0, 0, 0, 0},
	{-9.9e-3, -14.3e-3, /*9.1145e-9*/0, 1},
	{ 9.9e-3, -14.3e-3, /*9.1145e-9*/0, 1},
	{0, 0, 0, 0}
};
static ant_t rxAntennas[4]= {\
	{0, 0, 0, 0},
	{-9.9e-3, 14.3e-3, /*9.1145e-9*/0, 1},
	{ 9.9e-3, 14.3e-3, /*9.1145e-9*/0, 1},
	{0, 0, 0, 0}
};
static const uint8_t defTxSequence[] = {2, 4, 4, 2};
static const uint8_t defRxSequence[] = {2, 2, 4, 4};
#define DEF_SCAN_SEQ_SIZE (sizeof(defTxSequence))
#define DEF_ITERATIONS 10000
#endif

#if DEF_BOARD_MODEL == AHM3D
#define ANT_ARRAY_SELECTED
static ant_t txAntennas[4]= {\
	{0, 0, 0, 0},
	{-1.150e-3, 9.9e-3, 26e-12, 1}, //90e-12
	{17.65e-3, 9.9e-3, 0, 1},
	{0, 0, 0, 0}
};
static ant_t rxAntennas[4]= {\
	{0, 0, 0, 0},
	{-17.65e-3, 9.9e-3, 0, 1},
	{-17.65e-3, -9.9e-3, 5e-12, -1}, //15
	{0, 0, 0, 0}
};
static const uint8_t defTxSequence[] = {2, 4, 4, 2};
static const uint8_t defRxSequence[] = {2, 2, 4, 4};
#define DEF_SCAN_SEQ_SIZE (sizeof(defTxSequence))
#define DEF_ITERATIONS 10000
#endif

#if DEF_BOARD_MODEL == AHM2DSC
#define ANT_ARRAY_SELECTED
static ant_t txAntennas[4]= {\
	{0, 0, 0, 0},
	{-12.735e-3, 0, 0, 1},
	{ 12.735e-3, 0, 57e-12, 1},
	{0, 0, 0, 0}
};
static ant_t rxAntennas[4]= {\
	{0, 0, 0, 0},
	{-12.735e-3, 0, 57e-12, 1},
	{ 12.735e-3, 0, 0, 1},
	{0, 0, 0, 0}
};
static const uint8_t defTxSequence[] = {2, 4, 4, 2};
static const uint8_t defRxSequence[] = {2, 2, 4, 4};
#define DEF_SCAN_SEQ_SIZE (sizeof(defTxSequence))
#define DEF_ITERATIONS 10000
#endif

#if DEF_BOARD_MODEL == AHM3DSC_RC2
#define ANT_ARRAY_SELECTED
static ant_t txAntennas[4]= {\
	{9.4e-3, -9.4e-3, 0, 1},
	{0, 0, 0, 0},
	{-9.4e-3, -9.4e-3, 225e-12, 1},		//shared direct
	{0, 0, 0, 0}
};
static ant_t rxAntennas[4]= {\
	{0, 0, 0, 0},
	{-9.4e-3, 9.4e-3, 108.9e-12, 1},
	{-9.4e-3, -9.4e-3, 250.3e-12, 1},	//shared direct
	{0, 0, 0, 0}
};
static const uint8_t defTxSequence[] = {1, 1, 4, 4};
static const uint8_t defRxSequence[] = {2, 4, 4, 2};
#define DEF_SCAN_SEQ_SIZE (sizeof(defTxSequence))
#define DEF_ITERATIONS 10000
#endif

#if DEF_BOARD_MODEL == AHM3DSC_RC1
#define ANT_ARRAY_SELECTED
static ant_t txAntennas[4]= {\
	{9.4e-3, -9.4e-3, 0, 1},
	{0, 0, 0, 0},
	{-9.4e-3, -9.4e-3, 235e-12, 1}, //shared wirh coupler
	{0, 0, 0, 0}
};
static ant_t rxAntennas[4]= {\
	{0, 0, 0, 0},
	{-9.4e-3, 9.4e-3, 108.9e-12, 1},
	{-9.4e-3, -9.4e-3, 280e-12, 1},	//shared with coupler
	{0, 0, 0, 0}
};

static const uint8_t defTxSequence[] = {1, 1, 4, 4};
static const uint8_t defRxSequence[] = {2, 4, 4, 2};
#define DEF_SCAN_SEQ_SIZE (sizeof(defTxSequence))
#define DEF_ITERATIONS 10000
#endif

#if DEF_BOARD_MODEL == AHM2DL
#define ANT_ARRAY_SELECTED
static ant_t txAntennas[4]= {\
	{0		, 0, 0			, 0	},
	{55.6e-3, 0, 223.2e-12	, -1},
	{18e-3	, 0, 0			, 1	},
	{36.8e-3, 0, 118.9e-12	, 1	}
};
static ant_t rxAntennas[4]= {\
	{0			, 0, 0			, 0	},
	{-55.6e-3	, 0, 223.2e-12	, -1},
	{-18e-3		, 0, 0			, 1	},
	{-36.8e-3	, 0, 118.9e-12	, 1	}
};
static const uint8_t defTxSequence[] = {2, 8, 2, 2, 8, 4, 8, 4, 4};
static const uint8_t defRxSequence[] = {8, 4, 4, 2, 8, 4, 2, 8, 2};
#define DEF_SCAN_SEQ_SIZE (sizeof(defTxSequence))
#define DEF_ITERATIONS 4000
#endif



#ifndef ANT_ARRAY_SELECTED
#error "Undefined board"
#endif

#if OPT_3D_MODE == 0
static struct _canvas_param_t{
	float rhoMin;
	float rhoMax;
	float rhoStep;
	float phiMin;
	float phiMax;
	float phiStep;
	float rhoMax_actual; //updated when 2D canvas is calculated
	float rhoMin_actual;
	bool rhoRangeAuto; //auto range
}canvas_param ={1, 9, DEF_RHO_STEP, DEF_PHI_MIN_RAD, DEF_PHI_MAX_RAD, DEF_PHI_STEP_RAD,0 ,0, false};
#else
static struct _canvas3d_param_t{
	float rhoMin;
	float rhoMax;
	float rhoStep;
	float elMin;
	float elMax;
	float elStep;
	float thetaMin;
	float thetaMax;
	float thetaStep;
	float rhoMax_actual; //updated when 2D canvas is calculated
	float rhoMin_actual;
	bool rhoRangeAuto; //auto range
}canvas3d_param ={1, 9, DEF_RHO_STEP, DEF_EL_MIN_RAD, DEF_EL_MAX_RAD, DEF_EL_STEP_RAD, \
			DEF_THETA_MIN_RAD, DEF_THETA_MAX_RAD, DEF_THETA_STEP_RAD ,0 ,0, false};
#endif


/** @ingroup grpCommIF
 * @brief Auxiliary buffer for response message generation */
uint8_t txauxbuffer0[DEF_COMM_MAX_STREAM_CHUNK_SIZE];
/** @ingroup grpCommIF
 * @brief Auxiliary buffer for response message generation */
uint8_t txauxbuffer1[DEF_COMM_MAX_STREAM_CHUNK_SIZE];

//** PRIVATE VARIABLES END **//

//** PUBLIC VARIABLES START **//

//** PUBLIC VARIABLES END **//

//** PRIVATE FUNCTIONS START **//
static void priv_evt_acqdone(void *);
static void priv_evt_overrun(void *);
static void priv_evt_ppbu0rdy(void *);
static void priv_evt_ppbu1rdy(void *);
static int priv_ipcio_blocking(HydrUDriver_t * pdrv, ipc_messagedata_t* pmsg, uint32_t command, uint32_t timeout);
static int priv_initialization(HydrUDriver_t * );
static int priv_comm_handling(HydrUDriver_t * , int interface);
static int priv_update_range(HydrUDriver_t * , float min, float max);
static int priv_update_sequence(HydrUDriver_t *, uint8_t * txSel, uint8_t * rxSel, int numSeq);
static int priv_update_acquisition_data(HydrUDriver_t *);
static int priv_update_rec(HydrUDriver_t *);
static int priv_free_acquisitions_buffers(HydrUDrv_acqHandler_t *);
static int priv_streams_container_free();
static int priv_streams_container_alloc(int samples, int stream);
static int priv_streams_container_cleanup();
static int priv_start_data_txfr(void* dest, const void* src, int size);
static int priv_wait_txfr_complete(int handlerIndex, bool releaseHandler);
static int priv_release_txfr_handler(int handlerIndex);
static int priv_get_message_buffer(comm_msgbuffer_t *, int interfaceID);
static int priv_send_message(comm_msgbuffer_t, int interfaceID);
static int priv_push_datareq(uint16_t code, uint16_t interfaceID, radar_command);
static int priv_pop_datareq(uint16_t* code, uint16_t* interfaceID, radar_command *);
static const struct _RadarCommandItem * priv_get_cmditem_by_radar_command(radar_command);
static int priv_handle_datarequests();
static void *priv_serialize(void* pdst, const void*, int);
static int priv_send_stream(uint8_t* pbuf, int size, bool isstart, bool isend, int interface);
static int priv_default_init();
static int priv_flush_clutter_data();
static bool priv_acqflag_check_clr(int acqFlag);	//check the flag status and reset
static int priv_elaborate_data(HydrUDriver_t *, HydrUDrv_acqHandler_t*);
#if OPT_3D_MODE==0
static void priv_canvas_struct_cplx2abs(recon_handler_t * , struct _canvas_struct_t *);
#else
static void priv_canvas_struct_cplx2abs(recon3d_handler_t * , struct _canvas_struct_t *);
#endif
//** PRIVATE FUNCTIONS END **//

//** PUBLIC FUNCTIONS START **//

//** PUBLIC FUNCTIONS END **//

/**
 * Application entry point
 * @param pdrv
 * @return ERROR
 */
int app_entry(HydrUDriver_t* pdrv){

#if OPT_3D_MODE==0
	rec2D_init(&rec_h, selAlgo);	//initialization
#else
	rec3D_init(&rec3d_h, selAlgo);
#endif
	if (DEF_PWRSAVE_MODE != HUPWR_OFF){
		HydrUDrv_set_pwrSave(pdrv, DEF_PWRSAVE_MODE);
		if (DEF_PWRSAVE_MODE != HUPWR_2)
			lpMode = DEF_LPMODE_0;
		else
			lpMode = DEF_LPMODE_1;
	}


	if (priv_initialization(pdrv))
		return -1;
	if (priv_default_init()){
		return -1;
	}

#if OPT_ENABLE_SSPI == 1
	comm_sspi_init();
#endif

#if DEF_RADAR_AUTOSTART==1
	start_req = 1;	//automatic start radar operation
#endif

	/*super-loop*/
	while(1){


		if (lpMode){
			//if lpMode (low power mode) is set, CPU enters wfi state to save power
			asm volatile("wfi"); //this lpMode
		}

		comm_task();		//UART channel comm task
#if OPT_ENABLE_SSPI == 1
		comm_sspi_task();	//SPI channel comm task
#endif
		priv_comm_handling(pdrv, INTERFACE_ID_COMM);	//UART message handler
#if OPT_ENABLE_SSPI == 1
		priv_comm_handling(pdrv, INTERFACE_ID_SPI);		//SPI message handler
#endif

		//acquisition handling loop
		if (running){
			start_req = 0;
			if (stop_req){
				//a stop request was issued, stop acquisition and exit
				stop_req = 0;
				HydrUDrv_stop_cont(pprivDriver);
				running = 0;
				continue;
			}
			//check overrun state
			if (acq_flags.overrun){
				//overrun detected, radar wasn't able to store a new frame (i.e. frame rate too high)
#if OPT_ACQ_STOP_IF_OVERRUN == 1
				HydrUDrv_stop_cont(pdrv);
				running = 0;
				continue;
#endif
			}
			//verify if there are streams ready
			if (priv_acqflag_check_clr(ACQ_FLAG_DONE)){
				//there are data available
				lastScanPeriod_ticks = HAL_getSysTick()- lastScanDone_ticks;
				lastScanDone_ticks =HAL_getSysTick();
				HydrUDrv_acqHandler_t * pacqHandler; //pointer to current data

				//set the acquisition handler
				if (acqMode == IPC_CMD_ARG_ACQUISITION_MODE_SINGLE)
					pacqHandler = acqHandler; //alwayes transferred on handler 0
				else{
#if OPT_BUFFER_MODE_PINGPONG == 0
					pacqHandler = acqHandler;
#else
					//verify current buffer (ping-pong mode)
					if (next_buffer_expected == 1 && acq_flags.buf1rdy){
						acq_flags.buf1rdy = 0;
						pacqHandler = acqHandler+1;
						next_buffer_expected = 0; //switch expected buffer
					}else if (next_buffer_expected == 0 && acq_flags.buf0rdy){
						pacqHandler = acqHandler;
						acq_flags.buf0rdy = 0;
						next_buffer_expected = 1; //switch expected buffer
					}else{
						//ready but not syncronized, stop
						HydrUDrv_stop_cont(pdrv);
						running = 0;
						continue;
					}
#endif
				}
/** [usercode] */
/*
 * USER_ENTRY
 * can add processing in this section
 * pacqHandler->pIbuffer 			I channel buffer (all the scan are stored sequentially) |I_scan0|I_scan1|I_scanN...|
 * pacqHandler->pQbuffer 			Q channel buffer (all the scan are stored sequentially) |Q_scan0|Q_scan1|Q_scanN...|
 * pacqHandler->samplePerScan		number of sample per each scan
 *
 * IQ buffer preprocessed according to DEF_RADAR_PREPROCESSING_OPT
 * Samples are stored into 32 bit buffers, samples could be normalized (-1, 1) following this code
 *
 *  uint16_t integrations;
 *	HydrUDrv_get_integrations(pdrv, &integrations);
 *	float scale = DEF_ADC_SCALING_F/((float)integrations);
 *	int samples = pacqHandler->samplePerScan;
 *	int numScan = pacqHandler->numScan;
 *  complex_float destBuffer[samples][numScan];
 *	proc_convert_int2complex_f(pacqHandler->pIbuffer, pacqHandler->pQbuffer, destBuffer, samples*numScan, scale);
 *
 * */

				priv_elaborate_data(pdrv, pacqHandler);
/** [usercode] */
				priv_handle_datarequests(); //to get data at the same pace of FPS, user could send a request and acquire data with correct pace
			}

			//data processing and image reconstruction complete, check if data request must be performed
			//priv_handle_datarequests();
		}else{
			//if not running verify if something occur and correct incorrect states
			stop_req = 0;
			if (acq_flags.acqDone || acq_flags.buf0rdy || acq_flags.buf1rdy || acq_flags.overrun){
				//strange behavious, clear all and set stop command
				HydrUDrv_stop_cont(pprivDriver);
				//cleanup
				acq_flags.acqDone = 0;
				acq_flags.buf0rdy = 0;
				acq_flags.buf1rdy = 0;
				acq_flags.overrun = 0;
			}
			priv_handle_datarequests();	//check for request and return a stop
			if (start_req){
				//start new endless acquisition cycle
				start_req = 0;
				priv_flush_clutter_data();
				acqDataReadyBuffer[0]=0;
				acqDataReadyBuffer[1]=0;
				next_buffer_expected = 0;
				//cleanup flags
				memset((void*)&acq_flags, 0, sizeof(acq_flags));
				if (HydrUDrv_start_cont(pdrv, acqHandler, OPT_BUFFER_MODE_PINGPONG==1 ? acqHandler+1:NULL)){
					continue; //startup failed
				}
				lastScanDone_ticks = HAL_getSysTick();
				lastScanPeriod_ticks = 0;
				priv_streams_container_cleanup();
				//startup executed succesfully
				running = 1;
				acqMode = IPC_CMD_ARG_ACQUISITION_MODE_CONT;
				//proceed execution
			}
		}
		//DO NOT ADD CODE, DUE TO THE USAGE OF "CONTINUE" THIS POINT COULD NOT BE REACHED
	}
}

/**
 * @brief data elaboration
 * The functions performs data normalization, decluttering, and image reconstruction
 * @param pdrv		Hydrogen driver
 * @param pacqHandler	current acquisition handler
 * @return SUCCESS/FAIL
 */
static int priv_elaborate_data(HydrUDriver_t *pdrv, HydrUDrv_acqHandler_t*pacqHandler){
	//start handler processing, transfer data
	temp_stream.pclutter = temp_clutter;
	temp_stream.pout = temp_output;
	temp_stream.praw = temp_raw;
	temp_stream.pdecl = &declConst;

	int samples = pacqHandler->samplePerScan;
	int numScan = pacqHandler->numScan;
	int txfrClut;
	int txfrRaw;
	int txfrOut;
	temp_stream.size = samples;
	uint16_t integrations;
	HydrUDrv_get_integrations(pdrv, &integrations);
	float scale = DEF_ADC_SCALING_F/((float)integrations); //apply scale in order to convert data in the range +-1
	for(int i = 0 ; i< numScan; i++){
		//transfer data
		int offset = i*samples;
		proc_stream_t * pDestStream = streams_container.pstreams+i;
		if (i != 0){
			priv_wait_txfr_complete(txfrClut, true);
		}
		//start co-processor assisted transfer
		txfrClut = priv_start_data_txfr((void*)temp_clutter, (const void*)pDestStream->pclutter, samples * sizeof(temp_clutter[0]));

		if (i != 0){
			priv_wait_txfr_complete(txfrRaw, true);
		}
		//copy data to local variables
		proc_convert_int2complex_f(pacqHandler->pIbuffer+offset, pacqHandler->pQbuffer+offset, temp_raw, samples, scale);
		if (i == numScan-1){
			//release buffer
			*(pacqHandler->pstatus) = 0;
		}
		priv_wait_txfr_complete(txfrClut, true);
		if (i != 0){
			priv_wait_txfr_complete(txfrOut, true);
		}
		temp_stream.clutter_empty = pDestStream->clutter_empty;
		proc_elaborate(&temp_stream, dist_alpha, dist_nu);
		pDestStream->clutter_empty = temp_stream.clutter_empty;
		//transfer data back
		txfrClut = priv_start_data_txfr((void*)pDestStream->pclutter, (const void*)temp_clutter, samples * sizeof(temp_clutter[0]));
		txfrRaw = priv_start_data_txfr((void*)pDestStream->praw, (const void*)temp_raw, samples * sizeof(temp_raw[0]));
		txfrOut = priv_start_data_txfr((void*)pDestStream->pout, (const void*)temp_output, samples * sizeof(temp_output[0]));

	}
	priv_wait_txfr_complete(txfrRaw, true);
	priv_wait_txfr_complete(txfrOut, true);
	priv_wait_txfr_complete(txfrClut, true);
	//all data are ready
	if (exe_reconstruction && OPT_RECONSTRUCTION_ENABLE && user_exe_reconstruction){
		//correct antenna polarity
		for(int s = 0 ; s < rec_streams_num; s++){
			float curScale =rec_streams[s].amplitude;
			if (curScale == 1.0)
				continue; //process only stream where scaling is necessary

			//scale the current stream
			for (int i = 0 ; i< rec_streams[s].size; i++){
				complex_float cur = rec_streams[s].pdata[i];
				cur.imag *= curScale;
				cur.real *= curScale;
				rec_streams[s].pdata[i] = cur;
			}
		}
#if OPT_3D_MODE == 0
		rec2D_reconstruct(&rec_h, canvas_struct.rec_canvas, rec_streams, rec_streams_num, rec_fcarrier_norm);
		canvas_struct.iscomplex = rec_h.outputcplx;
#else
		rec3D_reconstruct(&rec3d_h, canvas_struct.rec_canvas,rec_streams, rec_streams_num, rec_fcarrier_norm);
#endif
	}
	return 0;
}

/**
 * Private routine to set frame per second
 * @param pdrv
 * @param FPS
 * @return SUCCESS/FAIL
 */

static int priv_update_framerate(HydrUDriver_t* pdrv, int FPS){
	if (HydrUDrv_set_framerate(pdrv, FPS))
		return -1;
	return 0;
}

/**
 * @brief SW/HW initialization
 * This routine setup local HW (UART/SPI) and configures radar for continuous acquisition mode with
 * ping-pong buffers
 * @param pdrv
 * @return SUCCESS/FAIL
 */
static int priv_initialization(HydrUDriver_t* pdrv){
	pprivDriver = pdrv;
	//setup UART
	uart_init(APP_SERIAL_BAUD);
	//setup buffer status flag memory
	acqHandler[0].pstatus = acqDataReadyBuffer;
	acqHandler[1].pstatus = acqDataReadyBuffer+1;
	//setup event callback to local function
	hal_evt_ctl_register(DEF_OVERRUN_EVT_ID, priv_evt_overrun, NULL);
	hal_evt_ctl_register(DEF_ACQREADY_EVT_ID, priv_evt_acqdone, NULL);
	hal_evt_ctl_register(DEF_PPBUF0RDY_EVT_ID, priv_evt_ppbu0rdy, NULL);
	hal_evt_ctl_register(DEF_PPBUF1RDY_EVT_ID, priv_evt_ppbu1rdy, NULL);

	//enable event generation
	hal_evt_ctl_unmask(DEF_OVERRUN_EVT_ID);
	hal_evt_ctl_unmask(DEF_ACQREADY_EVT_ID);
	hal_evt_ctl_unmask(DEF_PPBUF0RDY_EVT_ID);
	hal_evt_ctl_unmask(DEF_PPBUF1RDY_EVT_ID);
	hal_intc_enable(ITC_EVT_FIFO_VALID_ID);

	//send to co-processor the mapping between radar event and SW event
	ipc_messagedata_t ipcmsg;
	ipcmsg.u32 = mIPCEvtEncode(DEF_OVERRUN_EVT_ID);
	int ret = priv_ipcio_blocking(pdrv, &ipcmsg, IPC_CMD_SET_OVERRUN_EVT, DEF_IPCIO_TOms);
	ipcmsg.u32 = mIPCEvtEncode(DEF_ACQREADY_EVT_ID);
	ret |= priv_ipcio_blocking(pdrv, &ipcmsg, IPC_CMD_SET_ACQ_COMPLETE_EVT, DEF_IPCIO_TOms);
	ipcmsg.u32 = mIPCEvtEncode(DEF_PPBUF0RDY_EVT_ID);
	ret |= priv_ipcio_blocking(pdrv, &ipcmsg, IPC_CMD_SET_PP_BUFFER0_RDY_EVT, DEF_IPCIO_TOms);
	ipcmsg.u32 = mIPCEvtEncode(DEF_PPBUF1RDY_EVT_ID);
	ret |= priv_ipcio_blocking(pdrv, &ipcmsg, IPC_CMD_SET_PP_BUFFER1_RDY_EVT, DEF_IPCIO_TOms);

	//setup co-processor post processing options
	ipcmsg.u32 = DEF_RADAR_PREPROCESSING_OPT;
	ret |= priv_ipcio_blocking(pdrv, &ipcmsg, IPC_CMD_SET_POSTPROC_OPT, DEF_IPCIO_TOms);
	if (ret)
		return -1;

	//setup local variables and functions
	if (priv_update_framerate(pdrv, DEF_FRAME_RATE))
		return -1;

	uint16_t fpsu16;
	HydrUDrv_get_framerate(pdrv, &fpsu16);
	float taunorm = DEF_DECLUTTER_CONSTANT/(float)fpsu16;
	proc_compute_declatter_const(&declConst, taunorm);

	memset((void*)dataMoversHandler, 0, sizeof(dataMoversHandler));

	return 0;
}

/**
 * Default radar configration
 * @return SUCCESS/FAIL
 */
static int priv_default_init(){
	uint8_t codeword = 1;
	//Set acquisition mode (continuous/signle)
	if (HydrUDrv_set_acqmode(pprivDriver, HUACQ_CONT))
		return -1;
	//set default carrier frequency
	if (HydrUDrv_set_carrier(pprivDriver, DEF_RADAR_CARRIER_MHZ))
		return -1;
	//set default bandwidth
	if (HydrUDrv_set_bandwidth(pprivDriver, DEF_RADAR_BW_MHZ))
		return -1;
	//set offset and range (follow the order, before offset, then range
	if (HydrUDrv_set_offset(pprivDriver, DEF_RADAR_OFFSET))
		return -1;
	//set range
	if (priv_update_range(pprivDriver, DEF_RADAR_RANGE_MIN, DEF_RADAR_RANGE_MAX))
		return -1;
	//set reciever gain
	if (HydrUDrv_set_gain(pprivDriver, DEF_RADAR_VGA_GAIN, DEF_RADAR_VGA_GAIN))
		return -1;
	//set codeword
	if (HydrUDrv_set_code(pprivDriver, &codeword, 1))
		return -1;
	//set Tx power
	if (HydrUDrv_set_pwr(pprivDriver, DEF_RADAR_TX_POWER))
		return -1;
	//set scan sequence
	if (priv_update_sequence(pprivDriver, (uint8_t*)&defTxSequence, (uint8_t*)&defRxSequence, DEF_SCAN_SEQ_SIZE))
		return -1;
	return 0;
}

/**
 * This function cleanup clutter data for each stream
 * @return SUCCESS
 */
static int priv_flush_clutter_data(){
	for (int i = 0; i< streams_container.numStreams; i++){
		streams_container.pstreams[i].clutter_empty = true;
	}
	return 0;
}

#if OPT_3D_MODE==0
static void priv_canvas_struct_cplx2abs(recon_handler_t * ph, struct _canvas_struct_t * pstruct){
#else
static void priv_canvas_struct_cplx2abs(recon3d_handler_t * ph, struct _canvas_struct_t * pstruct){
#endif
	if (pstruct->iscomplex)
		rec2D_complex2abs(ph, pstruct->rec_canvas, pstruct->rec_canvas);
	pstruct->iscomplex = false;
}

/**
 * @brief Check and clear radar generated events
 * This section check the status of the required flag and return if asserted or not\n
 * The flag status must be deasserted with interrupt disabled to keep coherence
 *
 *
 * @param acqFlag
 * @return true if the flag is set
 */
static bool priv_acqflag_check_clr(int acqFlag){
	//enter critical section
	int state = hal_spr_read_then_clr(CSR_MSTATUS, (1<<3)); //disable interrupt and save current state
	asm volatile("nop");
	asm volatile("nop");

	bool ret;
	switch(acqFlag){
		case ACQ_FLAG_BUF0_RDY:
			ret = (acq_flags.buf0rdy != 0);
			acq_flags.buf0rdy =0;
			break;
		case ACQ_FLAG_BUF1_RDY:
			ret = (acq_flags.buf1rdy != 0);
			acq_flags.buf1rdy =0;
			break;
		case ACQ_FLAG_DONE:
			//this is the most critical one
			ret = (acq_flags.acqDone>0);
			if (ret)
				acq_flags.acqDone--;
			break;
		case ACQ_FLAG_OVERRUN:
			ret = (acq_flags.overrun != 0);
			acq_flags.overrun = 0;
			break;
	}
	if (state & (1 << 3))
		hal_spr_read_then_set(CSR_MSTATUS, (1 << 3)); //exit critical section and enable interrupt
	return ret;
}
/**
 * Callback from event controller, this event is generated when a new frame is ready
 * @param parg
 */
static void priv_evt_acqdone(void * parg __attribute__((unused))){
	acq_flags.acqDone++; //could be called multiple time
}
/**
 * Callback from event controller, this event is generated when a overrun error occurred\n
 * i.e. the destination buffer is still busy and the radar is not able to transfer the frame
 * @param parg
 */
static void priv_evt_overrun(void * parg __attribute__((unused))){
	acq_flags.overrun= 1;
}
/**
 * Callback from event controller, generated when buffer 0 is ready
 * @param parg
 */
static void priv_evt_ppbu0rdy(void * parg __attribute__((unused))){
	acq_flags.buf0rdy= 1;
}
/**
 * Callback from event controller, generated when buffer 1 is ready
 * @param parg
 */
static void priv_evt_ppbu1rdy(void * parg __attribute__((unused))){
	acq_flags.buf1rdy = 1;
}

/**
 * Auxiliary function for IPC communication
 * @param pdrv
 * @param pmsg
 * @param command
 * @param timeout
 * @return SUCCESS/FAIL
 */
static int priv_ipcio_blocking(HydrUDriver_t * pdrv, ipc_messagedata_t* pmsg, uint32_t command, uint32_t timeout){
	int iHandler = ipc_send_message(*pmsg, command, true);
	if (iHandler == IPC_NO_SPACE){
		return -1;
	}
	int ret;
	ipc_item_t responseItem;

	uint32_t startTime = pdrv->pGetSysTick();
	while((ret = ipc_check_response(&responseItem, iHandler)) == IPC_NO_RESPONSE){
		if (timeout){
			if ((pdrv->pGetSysTick() - startTime) > timeout)
				break;
		}
	}
	//check reason of exit
	if (ret != IPC_SUCCESS)
		return -2; //timeout or error

	//ready

	*pmsg = responseItem.msgdata;
	//check if response is error
	uint32_t ctl = mIPCGetCtl(responseItem.cmdctl);
	if (ctl == IPC_CTL_READY_ERR){
		//the payload is the message error code
		return abs(responseItem.msgdata.i32);
	}
	//response is OK return 0
	return 0;
}

/**
 * @brief This function update the acquisition range
 * @param pdrv driver pointer
 * @param min minimum distance in meters
 * @param max maximum distance in meters
 * @return SUCCESS/FAIL
 */

static int priv_update_range(HydrUDriver_t * pdrv, float min, float max){
	uint8_t saved_running_state = running;
	if (running){
		HydrUDrv_stop_cont(pdrv);
		HAL_Delay(200);
	}
	//start editing
	int ret = HydrUDrv_set_range(pdrv, min, max);
	if (ret)
		return ret;
	ret = priv_update_acquisition_data(pdrv);

	if (ret)
		return ret;

	if (saved_running_state && (acqMode == IPC_CMD_ARG_ACQUISITION_MODE_CONT)){
		start_req = 1; //send a start request, the main loop will take care of all
	}
	return ret;
}

/**
 * @brief Change the scanning sequence of the antennas
 * This function updates radar scanning sequence and local variables\n
 * The function updates also destination buffer according to the required number of sequence
 * @param pdrv
 * @param txSelMask array with active Tx antennas mask
 * @param rxSelMask array with active Rx antennas mask
 * @param numSeq number of sequence
 * @return SUCCESS/FAIL
 */
static int priv_update_sequence(HydrUDriver_t * pdrv, uint8_t * txSelMask, uint8_t * rxSelMask, int numSeq){
	uint8_t saved_running_state = running;
	if (running){
		HydrUDrv_stop_cont(pdrv);
		HAL_Delay(200);
	}
	if (numSeq > DEF_MAX_SEQUENCES)
		return -1;
	//update
	for(int i = 0 ; i< numSeq; i++){
		acqChannelsList[i].txmask = txSelMask[i];
		acqChannelsList[i].rxmask = rxSelMask[i];
	}
	acqHandler[0].pScanSeq = acqChannelsList;
	acqHandler[0].numScan = numSeq;
	acqHandler[1].pScanSeq = acqChannelsList;
	acqHandler[1].numScan = numSeq;
	int ret = priv_update_acquisition_data(pdrv);
	if (ret)
		return ret;

	if (saved_running_state && (acqMode == IPC_CMD_ARG_ACQUISITION_MODE_CONT)){
		start_req = 1; //send a start request, the main loop will take care of all
	}
	return ret;
}

/**
 * This function updates local buffer (IO from radar and local elaboration) according to local settings
 * @param pdrv
 * @return SUCCESS/FAIL
 */
static int priv_update_acquisition_data(HydrUDriver_t * pdrv){
	uint32_t bins;
	HydrUDrv_get_framebins(pdrv, &bins);
	uint32_t acqBufferBins = acqHandler[0].numScan * bins;
	uint32_t acqBuffersBytes =  acqBufferBins * sizeof(uint32_t);

	//check size to avoid continuous reallocation
	bool skipallocation = true;
#if OPT_BUFFER_MODE_PINGPONG == 0
	for(int i = 0 ; i< 1; i++){
#else
	for(int i = 0 ; i< 2; i++){
#endif
		if (acqHandler[i].totBinsAllocated != acqBufferBins){
			skipallocation = false;
			break;
		}
		if ((acqHandler[i].pIbuffer == NULL) || (acqHandler[i].pQbuffer == NULL) ||
				(acqHandler[i].samplePerScan != bins)){
			skipallocation = false;
			break;
		}
	}

	if (skipallocation == false){
		priv_free_acquisitions_buffers(acqHandler);
#if OPT_BUFFER_MODE_PINGPONG==1
		priv_free_acquisitions_buffers(acqHandler+1);
#endif


#if OPT_BUFFER_MODE_PINGPONG == 0
		for(int i = 0 ; i< 1; i++){
#else
		for(int i = 0 ; i< 2; i++){
#endif
			uint32_t * pI, *pQ;
			pI = (uint32_t*)malloc(acqBuffersBytes);
			pQ = (uint32_t*)malloc(acqBuffersBytes);
			if (pI == NULL || pQ == NULL){
				return -1;
			}
			acqHandler[i].pIbuffer = pI;
			acqHandler[i].pQbuffer = pQ;
			acqHandler[i].samplePerScan = bins;
			acqHandler[i].totBinsAllocated = bins*acqHandler[0].numScan;
		}
	}
	int ret = priv_streams_container_alloc(bins, acqHandler[0].numScan);
	if (ret != 0)
		return ret;
	//update
	return priv_update_rec(pdrv);
}

/**
 * @brief 2D/3D reconstruction canvas update
 * This function updates the reconstruction canvas according to local parameters
 * Adapt the canvas according to acquisition range and required reconsutrcion area/volume
 * The routine verifies the antenna sequence (must be a valid sequence for the reconstruction, i.e. no multiple antenna active for each sequence)
 * @param pdrv
 * @return SUCCESS/FAIL
 */
static int priv_update_rec(HydrUDriver_t * pdrv){
#if OPT_3D_MODE == 0
	//Destroy previous canvas
	rec2D_destroy_canvas(&canvas_struct.rec_canvas);
	//Update algorithm settings
	rec2D_setAlgo(&rec_h, selAlgo);
#else
	//Destroy previous canvas
	rec3D_destroy_canvas(&canvas_struct.rec_canvas);
	rec3D_cleanup_ctx(&rec3d_h);
	//Update algorithm settings
	rec3D_setAlgo(&rec3d_h, selAlgo);
#endif
	//Check sequence, every antenna mask must contain only 1 antenna active
	exe_reconstruction = true;
	rec_streams_num = 0;
	for (int i =  0; i< acqHandler[0].numScan; i++){
		uint8_t rxselmask, txselmask;
		int rxOnes, txOnes;
		rxselmask = acqChannelsList[i].rxmask;
		txselmask = acqChannelsList[i].txmask;

		/*asm volatile("p.cnt %0, %1" : "=r"(rxOnes) : "r"(rxselmask));
		asm volatile("p.cnt %0, %1" : "=r"(txOnes) : "r"(txselmask));*/
		rxOnes = utils_cnt_ones(rxselmask);
		txOnes = utils_cnt_ones(txselmask);
		if (rxOnes > 1 || txOnes > 1){
			exe_reconstruction = false;
		}
	}
	if (!exe_reconstruction){
		return 0; //invalid scanning sequence, no reconstruction could be performed
	}

	//Antenna sequence is ok, setup stream container and antanna related paramters
	recon_stream_t * prec_stream;
	prec_stream = rec_streams; //rec_streams are pre-allocated, set local variable to first item

	float sample_frequency =1e6 * (float)(hal_clkmgh_get_pll_frequency_MHz());
	float xmin, xmax;
	HydrUDrv_get_range(pdrv, &xmin, &xmax);

	//Scan the scan sequence, get the active antenna and set for each stream the required antenna info
	for (int i =  0; i< acqHandler[0].numScan; i++){
		uint8_t txIndex, rxIndex;
		uint8_t rxselmask, txselmask;
		rxselmask = acqChannelsList[i].rxmask;
		txselmask = acqChannelsList[i].txmask;
		//use find first 1 instruciton (return position of the first bit set or 32 is input is 0)
		/*asm volatile("p.ff1 %0, %1" : "=r"(txIndex) : "r" (txselmask));
		asm volatile("p.ff1 %0, %1" : "=r"(rxIndex) : "r" (rxselmask));*/
		txIndex = utils_findfirst_one(txselmask);
		rxIndex = utils_findfirst_one(rxselmask);
		if (txIndex > 3 || rxIndex > 3)
			continue;
		//both valid setup stream for reconstruction
		prec_stream->antennas_dx = (txAntennas[txIndex].x + rxAntennas[rxIndex].x)/2;
		prec_stream->antennas_dy = (txAntennas[txIndex].y + rxAntennas[rxIndex].y)/2;
		prec_stream->amplitude = txAntennas[txIndex].ampl * rxAntennas[rxIndex].ampl;
		prec_stream->offset = -(txAntennas[txIndex].delay_s + rxAntennas[rxIndex].delay_s)*sample_frequency; //antenna offset compensation
		prec_stream->offset += xmin * (2*sample_frequency)/LIGHT_SPEED;
		prec_stream->pdata = streams_container.pstreams[i].pout;
		prec_stream->size = streams_container.pstreams[i].size;
		//increment
		prec_stream++;
		rec_streams_num++;
	}
	if (rec_streams_num == 0){
		exe_reconstruction = false;
		return 0; //no valid streams detected
	}

	//Setup desired setup canvas
	int ret;
#if OPT_3D_MODE==0
	if (!canvas_param.rhoRangeAuto){
		xmin = canvas_param.rhoMin;
		xmax = canvas_param.rhoMax;
	}

	ret = rec2D_set_rho(&rec_h, xmin, xmax, canvas_param.rhoStep, sample_frequency);
	ret |= rec2D_set_phi(&rec_h, canvas_param.phiMin, canvas_param.phiMax, canvas_param.phiStep, sample_frequency);
#else
	if (!canvas3d_param.rhoRangeAuto){
		xmin = canvas3d_param.rhoMin;
		xmax = canvas3d_param.rhoMax;
	}

	ret = rec3D_set_rho(&rec3d_h, xmin, xmax, canvas3d_param.rhoStep, sample_frequency);
	ret |= rec3D_set_angles(&rec3d_h, canvas3d_param.elMin, canvas3d_param.elMax, canvas3d_param.elStep, \
			canvas3d_param.thetaMin, canvas3d_param.thetaMax, canvas3d_param.thetaStep,sample_frequency);
#endif



	if (ret){
		//allocation error
		rec_streams_num = 0;
		exe_reconstruction = 0;
		return 0;
	}

	/*this section verify if the reconstruction algorithm doesn't point to stream indexes out-of-bound,
	 * if so, the routine squeeze the canvas (by reducing down-range limits)
	 * For speed reason, the actual function doesn't test the index limit.
	 * */
	int lower_oob, higher_oob;
	int numSamples = rec_streams[0].size;

#if OPT_3D_MODE == 0
	while(rec2D_reconstruct_dryrun(&rec_h, rec_streams, rec_streams_num, &lower_oob, &higher_oob) != 0){
#else
	while(rec3D_reconstruct_dryrun(&rec3d_h, rec_streams, rec_streams_num, &lower_oob, &higher_oob) != 0){
#endif
		//OOB condition detected
		if (lower_oob){
			xmin += (float)-lower_oob * LIGHT_SPEED/(2*sample_frequency);
		}
		if (higher_oob){
			xmax -= (float)((higher_oob+1)-numSamples) * LIGHT_SPEED/(2*sample_frequency);
		}
#if OPT_3D_MODE == 0
		//rebuild rho
		ret = rec2D_set_rho(&rec_h, xmin, xmax, canvas_param.rhoStep, sample_frequency);
#else
		ret = rec3D_set_rho(&rec3d_h, xmin, xmax, canvas3d_param.rhoStep, sample_frequency);
#endif
		if (ret){
			rec_streams_num = 0;
			exe_reconstruction = 0;
			return 0;
		}
	}

#if OPT_3D_MODE == 0
	//store final rho
	canvas_param.rhoMin_actual = xmin;
	canvas_param.rhoMax_actual = xmax;
#else
	canvas3d_param.rhoMin_actual = xmin;
	canvas3d_param.rhoMax_actual = xmax;
#endif

	//Allocate actual array for reconstruction
#if OPT_3D_MODE == 0
	canvas_struct.rec_canvas = rec2D_init_canvas(&rec_h);
#else
	canvas_struct.rec_canvas = rec3D_init_canvas(&rec3d_h);
#endif
	if (canvas_struct.rec_canvas == NULL){
		rec_streams_num = 0;
		exe_reconstruction = false;
		//Allocation error
		return 0;
	}
	//This function setup auxiliary variables for reconstruction
#if OPT_3D_MODE == 0
	if (rec2D_setup(&rec_h, rec_streams_num)){
#else
	if (rec3D_setup(&rec3d_h, rec_streams_num)){
#endif
		rec_streams_num = 0;
		exe_reconstruction = false;
#if OPT_3D_MODE == 0
		rec2D_destroy_canvas(&(canvas_struct.rec_canvas));
#else
		rec3D_destroy_canvas(&canvas_struct.rec_canvas);
#endif
		return 0;
	}

	uint32_t fcarrier;
	HydrUDrv_get_carrier(pdrv, &fcarrier);
	rec_fcarrier_norm = ((float)fcarrier)*1e6/sample_frequency;
	exe_reconstruction = true;
	return 0;
}

/**
 * Stream buffer cleanup function
 * @return SUCCESS
 */
static int priv_streams_container_cleanup(){
	for (int i = 0 ; i< streams_container.numStreams; i++){
		proc_stream_t *pstream;
		pstream = streams_container.pstreams+i;
		int bytesize = pstream->size * sizeof(pstream->pclutter[0]);
		memset((void*)pstream->pclutter, 0, bytesize);
		memset((void*)pstream->pout, 0, bytesize);
		pstream->clutter_empty = true;
	}
	return 0;
}

/**
 * Acquisition memory dealloc function
 * @param p
 * @return SUCCESS
 */
static int priv_free_acquisitions_buffers(HydrUDrv_acqHandler_t * p) {
	if (p->samplePerScan){
		p->samplePerScan = 0;
		if (p->pIbuffer != NULL){
			free((void*)p->pIbuffer);
		}
		if (p->pQbuffer != NULL){
			free((void*)p->pQbuffer);
		}
		p->pIbuffer = NULL;
		p->pQbuffer = NULL;
	}
	return 0;
}
/**
 * Stream buffer deallocation function
 * @return SUCCESS
 */
static int priv_streams_container_free(){
	if (streams_container.numStreams){
		for(int i = 0; i < streams_container.numStreams; i++){
			//dealloc data memory
			proc_stream_t * curStream = streams_container.pstreams+i;
			if (curStream->pclutter)
				free((void*)(curStream->pclutter));
			if (curStream->praw)
				free((void*)(curStream->praw));
			if (curStream->pout)
				free((void*)(curStream->pout));
			//pdeclutter is local
		}
		//dealloc stream handler
		free((void*)(streams_container.pstreams));
	}
	streams_container.numStreams = 0;
	streams_container.pstreams = NULL;
	return 0;
}

/**
 * Stream buffer allocation function
 * @param samples
 * @param numstreams
 * @return SUCCESS
 */
static int priv_streams_container_alloc(int samples, int numstreams){
	//check
	bool skipupdate = true;
	if (streams_container.numStreams == numstreams){
		//verify samples
		for (int i = 0 ; i < numstreams; i++){
			proc_stream_t * pstream = streams_container.pstreams+i;
			if (pstream->pclutter == NULL ||
				pstream->praw == NULL ||
				pstream->pout == NULL){
					skipupdate = false;
					break;
			}
			//all allocated, check number of samples
			if (pstream->size != samples){
				skipupdate = false;
				break;
			}
		}
	}else{
		skipupdate = false;
	}

	if (skipupdate)
		return 0;
	//free
	priv_streams_container_free();
	streams_container.pstreams = (proc_stream_t*)malloc(sizeof(proc_stream_t)*numstreams);
	if (streams_container.pstreams == NULL)
		return -1;
	streams_container.numStreams = numstreams;
	for (int i = 0 ; i< numstreams; i++){
		//alloc memory
		proc_stream_t * pstream = streams_container.pstreams+i;
		pstream->pclutter = (complex_float*)malloc(samples*sizeof(complex_float));
		pstream->praw = (complex_float*)malloc(samples*sizeof(complex_float));
		pstream->pout = (complex_float*)malloc(samples*sizeof(complex_float));
		if (	pstream->pclutter == NULL ||
				pstream->praw == NULL ||
				pstream->pout == NULL){
			//allocation problem
			priv_streams_container_free();
			return -1;
		}
		pstream->size = samples;
		pstream->pdecl = &declConst;
	}
	return 0;
}

/**
 * @brief co-processor assited data mover
 * This function instruct the co-processor to start a memory/memory data transfer
 * @param dest
 * @param src
 * @param size
 * @return SUCCESS/FAIL
 */
//send to the coprocessor a request to transfer data (act like a DMA)
static int priv_start_data_txfr(void* dest, const void* src, int size){
	int handlerCode = -1;
	for (int i = 0 ; i< DATAMOVERS_NUM_HANDLERS; i++){
		if (dataMoversHandler[i].busy == 0){
			handlerCode = i;
			break;
		}
	}
	if (handlerCode == -1)
		return handlerCode;
	dataMoversHandler[handlerCode].busy = 1;
	dataMoversHandler[handlerCode].pdest = dest;
	dataMoversHandler[handlerCode].psrc = src;
	dataMoversHandler[handlerCode].size = size;
	dataMoversHandler[handlerCode].transferComplete = 0;
#if OPT_COPR_ASSISTED_DMA == 1
	ipc_messagedata_t msg;
	msg.pdata = (void*) &(dataMoversHandler[handlerCode]);
	if (ipc_send_message(msg, IPC_CMD_DATA_MOVER, false) < 0)
		return -1;
#else
	//emulate DMA
	memcpy((void*)dest, (const void *)src, size);
	dataMoversHandler[handlerCode].transferComplete = 1;
#endif

	return handlerCode;

}
/**
 * Wait until a data-moving operation is completed
 * @param handlerIndex
 * @param releaseHandler
 * @return SUCCESS/FAIL
 */
static int priv_wait_txfr_complete(int handlerIndex, bool releaseHandler){
	while(dataMoversHandler[handlerIndex].transferComplete == 0){
		asm volatile("nop");
	}
	if (releaseHandler)
		return priv_release_txfr_handler(handlerIndex);
	return 0;
}
/**
 * Data mover handler, release buffer entry of a complete transfer
 * @param handlerIndex
 * @return SUCCESS
 */
static int priv_release_txfr_handler(int handlerIndex){
	dataMoversHandler[handlerIndex].busy = 0;
	return 0;
}

/**
 * This function handles incoming message and call the approrpiate message handlers
 * @param pdrv
 * @param interface (UART/SPI)
 * @return SUCCESS/FAIL
 */
static int priv_comm_handling(HydrUDriver_t * pdrv, int interface){
	//check new message
	radar_command cmd;
	radar_command_payload payload;
	comm_cb_args_t cbArgs;
	int ret;
	if (interface == INTERFACE_ID_COMM){
		ret = comm_getmessage(&cmd, &payload);
		if (ret == COMM_NO_MESSAGE)
			return 0; //nothing to process, continue
		if (ret != COMM_SUCCESS)
			return -1; //something unexpected happened
	}else if (interface == INTERFACE_ID_SPI){
		ret = comm_sspi_getmessage(&cmd, &payload);
		if (ret == COMM_SSPI_NO_MESSAGE)
			return 0; //nothing to process, continue
		if (ret != COMM_SSPI_SUCCESS)
			return -1; //something unexpected happened
	} else
		return -1; //unknown interface ID


	cbArgs.interfaceID = interface;
	cbArgs.pdrv = pdrv;
	cbArgs.numAcqHandlers = sizeof(acqHandler)/sizeof(acqHandler[0]);
	cbArgs.pacqHanlders = acqHandler;
#if OPT_3D_MODE==0
	cbArgs.prec = &rec_h;
#else
	cbArgs.prec = &rec3d_h;
#endif

	//data valid call the appropiate call back
	const struct _RadarCommandItem * ptbl= NULL;
	for (int i = 0 ; i< RDR_CMD_LIST_SIZE; i++){
		if (RadarCommandItem[i].cmd == cmd){
			ptbl = &RadarCommandItem[i];
			break;
		}
	}
	if (ptbl == NULL){
		cmd = RADARCOMM_UNKNOWN_COMMAND;
		for (int i = 0 ; i< RDR_CMD_LIST_SIZE; i++){
			if (RadarCommandItem[i].cmd == cmd){
				ptbl = &RadarCommandItem[i];
				break;
			}
		}
	}
	//process callback
	pcomm_cb pcb =  ptbl->queryDataFunc;
	ret = pcb(cmd, &payload, (void*)&cbArgs);
	if (ret == COMM_CB_SENT_CUSTOM){
		return 0; //nothing to do
	}
	if (ret == COMM_CB_SEND_REGULAR){
		//setup custom response and send back to interface
		void * ptxfr = (void*)&payload.data_uint8; //are all in a union, excepted to STRUCT, the memory position is the same
		int size;
		switch(ptbl->payloadType){
			case PT_U8:
				size = 1;
				break;
			case PT_U16:
				size = 2;
				break;
			case PT_U32:
			case PT_FLOAT:
				size = 4;
				break;
			case PT_STRUCT:
				ptxfr = (void*)payload.data_struct.buffer;
				size = payload.data_struct.size;
				break;
			default:
				ptxfr = NULL;
				break;
		}
		if (ptxfr == NULL){
			return -1; //undefined
		}
		comm_msgbuffer_t msgbuffer;
		msgbuffer.size = size+1;
		if (priv_get_message_buffer(&msgbuffer, cbArgs.interfaceID))
			return -1; //error
		uint8_t* pdata = (uint8_t*)msgbuffer.buffer;
		pdata[0] = (uint8_t)ptbl->cmdCode;
		memcpy((void*)pdata+1, (const void*)ptxfr, size);
		if (priv_send_message(msgbuffer, cbArgs.interfaceID)){
			return -1;
		}
		return 0;
	}
	//at this point there's error states, exit with error code
	return -1;

}

/**
 * Get a message from selected interface
 * @param p
 * @param interfaceID
 * @return SUCCESS/NO MESSAGE/FAIL
 */
static int priv_get_message_buffer(comm_msgbuffer_t * p, int interfaceID){
	if (interfaceID == INTERFACE_ID_COMM){
		return comm_getbuffer(p->size, p);
	} else if (interfaceID == INTERFACE_ID_SPI){
		return comm_sspi_getbuffer(p->size, p);
	}
	return -1; //unknown
}

/**
 * Send a response message to the selected interface
 * @param msgbuf (message buffer)
 * @param interfaceID (interface ID, UART/SPI)
 * @return SUCCESS/FAIL
 */
static int priv_send_message(comm_msgbuffer_t msgbuf, int interfaceID){
	if (interfaceID == INTERFACE_ID_COMM){
		return comm_send(msgbuf);
	}else if (interfaceID == INTERFACE_ID_SPI){
		return comm_sspi_send(msgbuf);
	}
	return -1; //unknown
}

/**
 * This function push a data request into a circular buffer, request are handled right after
 * a new stream is collected
 * @param code
 * @param interfaceID
 * @param cmd
 * @return SUCCESS/FAIL
 */
static int priv_push_datareq(uint16_t code, uint16_t interfaceID, radar_command cmd){
	int freespace = ((datareqfifo.rp-datareqfifo.wp) & DATAREQ_FIFO_MASK)-1;
	if (freespace){
		datareqfifo.fifo[datareqfifo.wp].code = code;
		datareqfifo.fifo[datareqfifo.wp].interfaceID = interfaceID;
		datareqfifo.fifo[datareqfifo.wp].cmd = cmd;
		datareqfifo.wp = (datareqfifo.wp+1) & DATAREQ_FIFO_MASK;
		return 0;
	}
	return -1;
}

/**
 * Pop a data request from circular buffer
 * @param pcode
 * @param pinterfaceID
 * @param pcmd
 * @return SUCCESS/FAIL
 */
static int priv_pop_datareq(uint16_t* pcode, uint16_t* pinterfaceID, radar_command * pcmd){
	if (datareqfifo.rp == datareqfifo.wp){
		return 1; //nothing to read
	}
	//pop
	*pcode = datareqfifo.fifo[datareqfifo.rp].code;
	*pinterfaceID = datareqfifo.fifo[datareqfifo.rp].interfaceID;
	*pcmd = datareqfifo.fifo[datareqfifo.rp].cmd;
	datareqfifo.rp = (datareqfifo.rp+1) & DATAREQ_FIFO_MASK;
	return 0;
}

/**
 * Auxiliary function, return a RadarCommandItem pointer related to specific command code\n
 * This function is used by message handler
 * @param cmd
 * @return _RadarCommandItem pointer
 */

static const struct _RadarCommandItem * priv_get_cmditem_by_radar_command(radar_command cmd){
	const struct _RadarCommandItem * pret = NULL;
	for (int i = 0; i< RDR_CMD_LIST_SIZE; i++){
		if (RadarCommandItem[i].cmd == cmd){
			pret = RadarCommandItem+i;
			break;
		}
	}
	return pret;
}


//comunication handlers
__attribute__ ((optimize("-Os")))
int comm_cb_frame_rate(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint16 != 0xFFFF){
		/*if (HydrUDrv_set_framerate(pdrv, (ppayload->data_uint16)))
			return COMM_CB_ERR_IO;*/
		if (priv_update_framerate(pdrv, (ppayload->data_uint16)))
			return COMM_CB_ERR_IO;
	}
	//return current value
	if (HydrUDrv_get_framerate(pdrv, &(ppayload->data_uint16)))
		return COMM_CB_ERR_IO;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_elab(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint8 != 0xFF){
		if (ppayload->data_uint8  <= DATAREQ_CODE_MAX){
			//set elaboration
			commdata_type = ppayload->data_uint8 ;
		}
	}
	ppayload->data_uint8 = commdata_type;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_optproc(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
#if OPT_3D_MODE==0
	recon_handler_t * prec = ((comm_cb_args_t *)pars)->prec;
#else
	recon3d_handler_t * prec = ((comm_cb_args_t *)pars)->prec;
#endif
	ipc_messagedata_t ipcmsg;
	if (ppayload->data_struct.size == 2){
		//this is a set request

		//first byte is low level processing option, mapping is redirect to coprocessor
		ipcmsg.u32 = (uint32_t)(ppayload->data_struct.buffer[0]) & IPC_CMD_ARG_POSTPROC_ALL;
		int ret = priv_ipcio_blocking(pdrv, &ipcmsg, IPC_CMD_SET_POSTPROC_OPT, DEF_IPCIO_TOms);
		if (ret)
			return COMM_CB_IGNORE; //not responce in case of error



#if OPT_3D_MODE==0
		if (ppayload->data_struct.buffer[1] & (1 << RDCOM_OPTPROC_BYTE1_CPLX_IMAGE_POS)){
			prec->outputcplx = true;
		}else{
			prec->outputcplx = false;
		}
#endif
		if (ppayload->data_struct.buffer[1] & (1 << RDCOM_OPTPROC_BYTE1_EXE_RECONSTRUCTION)){
			user_exe_reconstruction = true;
		}else{
			user_exe_reconstruction = false;
		}

	}
	//readout
	ipcmsg.u32 = 0;
	int ret = priv_ipcio_blocking(pdrv, &ipcmsg, IPC_CMD_GET_POSTPROC_OPT, DEF_IPCIO_TOms);
	if (ret)
		return COMM_CB_IGNORE; //not responce in case of error
	ppayload->data_struct.size = 2;
	ppayload->data_struct.buffer[0] = (uint8_t)(ipcmsg.u32 & IPC_CMD_ARG_POSTPROC_ALL);
	ppayload->data_struct.buffer[1] = 0;	//preset to zero

#if OPT_3D_MODE==0
	if (prec->outputcplx){
		ppayload->data_struct.buffer[1] |= (1 << RDCOM_OPTPROC_BYTE1_CPLX_IMAGE_POS);
	}
#endif
	if (user_exe_reconstruction){
		ppayload->data_struct.buffer[1] |= (1 << RDCOM_OPTPROC_BYTE1_EXE_RECONSTRUCTION);
	}


	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_samprate(radar_command UNUSED(cmd), radar_command_payload*ppayload, void *pars){

	//return sampling rate
	uint32_t adcfreq = hal_clkmgh_get_pll_frequency_MHz();
	ppayload->data_uint16 = (uint16_t)adcfreq;
	return COMM_CB_SEND_REGULAR;
}



__attribute__ ((optimize("-Os")))
int comm_cb_data_channel(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint8 != 0xFF){
		//set elaboration
		commch_req = ppayload->data_uint8 ;
	}
	ppayload->data_uint8 = commch_req;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_tx_power(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint8 != 0xFF){
		if (ppayload->data_uint8 <= (uint8_t)HUTXPWR_HIGH){
			if(HydrUDrv_set_pwr(pdrv, (HydrUDrv_pwrlvl_t)ppayload->data_uint8))
				return COMM_CB_ERR_IO;

		}
	}
	HydrUDrv_pwrlvl_t curPlvl;
	if (HydrUDrv_get_pwr(pdrv, &curPlvl))
		return COMM_CB_ERR_IO;
	ppayload->data_uint8 = (uint8_t)curPlvl;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_running(radar_command UNUSED(cmd), radar_command_payload* UNUSED(ppayload), void * pars){
	comm_cb_args_t * pcbargs = ((comm_cb_args_t *)pars);
	start_req = 1;
	//send response
	comm_msgbuffer_t msgbuffer;
	msgbuffer.size = 1; //set required size
	if (priv_get_message_buffer(&msgbuffer, pcbargs->interfaceID))
		return COMM_CB_ERR_COMUNICATION;
	msgbuffer.buffer[0] = (uint8_t)'r';
	if (priv_send_message(msgbuffer, pcbargs->interfaceID)){
		return COMM_CB_ERR_COMUNICATION;
	}
	return COMM_CB_SENT_CUSTOM;
}

__attribute__ ((optimize("-Os")))
int comm_cb_stop(radar_command cmd, radar_command_payload* UNUSED(ppayload), void * pars){
	comm_cb_args_t * pcbargs = ((comm_cb_args_t *)pars);
	stop_req = 1;
	//send response
	comm_msgbuffer_t msgbuffer;
	msgbuffer.size = 1; //set required size
	if (priv_get_message_buffer(&msgbuffer, pcbargs->interfaceID))
		return COMM_CB_ERR_COMUNICATION;

	const struct _RadarCommandItem * pitem = priv_get_cmditem_by_radar_command(cmd);

	msgbuffer.buffer[0] = pitem->cmdCode;
	if (priv_send_message(msgbuffer, pcbargs->interfaceID)){
		return COMM_CB_ERR_COMUNICATION;
	}
	return COMM_CB_SENT_CUSTOM;
}

__attribute__ ((optimize("-Os")))
int comm_cb_reset(radar_command cmd, radar_command_payload* ppayload, void * pars){
	return comm_cb_stop(cmd, ppayload, pars);
}

__attribute__ ((optimize("-Os")))
int comm_cb_unknown(radar_command UNUSED(cmd), radar_command_payload* UNUSED(ppayload), void * UNUSED(pars)){
	//no response
	return COMM_CB_SENT_CUSTOM;
}

__attribute__ ((optimize("-Os")))
int comm_cb_data(radar_command cmd, radar_command_payload* UNUSED(ppayload), void * pars){
	//push data request, main routine will provide response to this command
	comm_cb_args_t * pcbargs = ((comm_cb_args_t *)pars);
	uint16_t datareq_code = mDATAREQ_encode(commdata_type, commfmt_req, commch_req);
	if (priv_push_datareq(datareq_code, pcbargs->interfaceID, cmd))
		return COMM_CB_ERR_IO;
	return COMM_CB_SENT_CUSTOM;
}

__attribute__ ((optimize("-Os")))
int comm_cb_img(radar_command cmd, radar_command_payload* UNUSED(ppayload), void * pars){
	//push data request by specify the request type as image
	comm_cb_args_t * pcbargs = ((comm_cb_args_t *)pars);
	uint16_t datareq_code = mDATAREQ_encode(DATAREQ_CODE_IMG, commfmt_req, commch_req);
	if (priv_push_datareq(datareq_code, pcbargs->interfaceID, cmd))
		return COMM_CB_ERR_IO;
	return COMM_CB_SENT_CUSTOM;
}

__attribute__ ((optimize("-Os")))
int comm_cb_version(radar_command cmd, radar_command_payload* UNUSED(ppayload), void * pars){
	comm_cb_args_t * pcbargs = ((comm_cb_args_t *)pars);
	//return version code
	char versionCode[] = VERSION_BYTE_STREAM;
	comm_msgbuffer_t msgbuffer;
	msgbuffer.size = sizeof(versionCode)+1; //set required size
	if (priv_get_message_buffer(&msgbuffer, pcbargs->interfaceID))
		return COMM_CB_ERR_COMUNICATION;
	const struct _RadarCommandItem * pcmd = priv_get_cmditem_by_radar_command(cmd);
	msgbuffer.buffer[0] = pcmd->cmdCode;
	memcpy((void*)(msgbuffer.buffer+1), (const void*)versionCode, sizeof(versionCode));
	if (priv_send_message(msgbuffer, pcbargs->interfaceID))
		return COMM_CB_ERR_COMUNICATION;
	return COMM_CB_SENT_CUSTOM;
}




__attribute__ ((optimize("-Os")))
int comm_cb_data_mult(radar_command cmd, radar_command_payload* UNUSED(ppayload), void * pars){
	comm_cb_args_t * pcbargs = ((comm_cb_args_t *)pars);
	uint16_t datareq_code = mDATAREQ_encode(commdata_type, commfmt_req, DATAREQ_CH_ALL);
	if (priv_push_datareq(datareq_code, pcbargs->interfaceID, cmd))
		return COMM_CB_ERR_IO;
	return COMM_CB_SENT_CUSTOM;
}

__attribute__ ((optimize("-Os")))
int comm_cb_format(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint8 != 0xFF){
		if (ppayload->data_uint8 <= DATAREQ_FMT_MAX){
			commfmt_req = ppayload->data_uint8;
		}
	}
	ppayload->data_uint8 = commfmt_req;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_antarray(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pargs){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pargs)->pdrv;
	if (ppayload->data_struct.size != 0){
		//is a request, check if basic structure is ok;
		if ((ppayload->data_struct.size % 17) == 0){
			//valid size, check structure
			int numSettings = ppayload->data_struct.size/13;
			uint8_t * pdataf;
			bool valid = true;
			for(int i = 0; (i< numSettings) & valid; i++){
				//RISCV handle unaligned access
				pdataf = &(ppayload->data_struct.buffer[i*13+1]);
				//check index
				uint8_t curAntID = ppayload->data_struct.buffer[13*i];
				valid &= (curAntID < 8);
				//check distances
				valid &= !isnanf(pdataf[0]); //X
				valid &= !isnanf(pdataf[1]); //Y
				valid &= !isnanf(pdataf[2]); //del
				valid &= !isnanf(pdataf[3]); //ampl
			}
			if (valid){
				//valid structure proceed with setting and update canvas settings
				for(int i = 0; (i< numSettings) & valid; i++){
					pdataf = &(ppayload->data_struct.buffer[i*13+1]);
					uint8_t antID = ppayload->data_struct.buffer[13*i];
					float curX, curY, del, ampl;
					curX = pdataf[0];
					curY = pdataf[1];
					del = pdataf[2];
					ampl = pdataf[3];
					if (mRDRCOM_antarrayID_isRx(antID)){
						//set Rx
						antID = mRDRCOM_decode_antArrayID(antID);
						rxAntennas[antID].delay_s = del;
						rxAntennas[antID].x = curX;
						rxAntennas[antID].y = curY;
						rxAntennas[antID].ampl = ampl;
					}else{
						antID = mRDRCOM_decode_antArrayID(antID);
						txAntennas[antID].delay_s = del;
						txAntennas[antID].x = curX;
						txAntennas[antID].y = curY;
						txAntennas[antID].ampl = ampl;
					}
				}
				priv_update_rec(pdrv);
			}
		}
	}
	//setup answer (send it as regular) calleer function takes care
	ppayload->data_struct.size = 17*8; //antenna
	uint8_t * pdata;
	pdata = ppayload->data_struct.buffer;
	//RX
	for(int i = 0; i< 4; i++){
		*pdata = mRDRCOM_encode_antArrayID(i, 1);
		pdata++;
		*((float*)pdata) = rxAntennas[i].x;
		pdata += 4;
		*((float*)pdata) = rxAntennas[i].y;
		pdata += 4;
		*((float*)pdata) = rxAntennas[i].delay_s;
		pdata += 4;
		*((float*)pdata) = rxAntennas[i].ampl;
		pdata += 4;
	}
	//Tx
	for(int i = 0; i< 4; i++){
		*pdata = mRDRCOM_encode_antArrayID(i, 0);
		pdata++;
		*((float*)pdata) = txAntennas[i].x;
		pdata += 4;
		*((float*)pdata) = txAntennas[i].y;
		pdata += 4;
		*((float*)pdata) = txAntennas[i].delay_s;
		pdata += 4;
		*((float*)pdata) = txAntennas[i].ampl;
		pdata += 4;
	}
	return COMM_CB_SEND_REGULAR;
}


__attribute__ ((optimize("-Os")))
int comm_cb_canvas(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pargs){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pargs)->pdrv;
	if (ppayload->data_struct.size != 4){
		//set request
#if OPT_3D_MODE == 0
		if (ppayload->data_struct.size == 25){
#else
		if (ppayload->data_struct.size == 37){
#endif
			//valid request check
			uint8_t reqAlgo = ppayload->data_struct.buffer[0];
			float * pdata;
			pdata = (float*)(ppayload->data_struct.buffer+1);
			//check nan
			bool valid = !isnan(pdata[0]);
			for(int i = 1 ; (i < 6) & valid; i++){
				valid &= !isnan(pdata[i]);
			}
			//check range
			valid &= !(pdata[2] < pdata[0]); 	//! rhomax < rhomin
			valid &= !(pdata[1] <= 0);			//! rhoStep <=0;
			valid &= !(pdata[5] < pdata[3]); 	//! phiMax < phiMin;
			valid &= !(pdata[4] <= 0);			//! phiStep <= 0;

			if (valid){
				//udpate
				selAlgo = reqAlgo;
#if OPT_3D_MODE == 0
				canvas_param.rhoMin 	= pdata[0];
				canvas_param.rhoStep 	= pdata[1];
				canvas_param.rhoMax 	= pdata[2];
				canvas_param.phiMin 	= pdata[3];
				canvas_param.phiStep 	= pdata[4];
				canvas_param.phiMax		= pdata[5];
				//check if autorange
				if ((canvas_param.rhoMin < 0) || (canvas_param.rhoMax < 0)){
					canvas_param.rhoRangeAuto = true;
				}else{
					canvas_param.rhoRangeAuto = false;
				}
				//protocol transfer phi in degree, convert in radiants
				canvas_param.phiMin *= (M_PI / 180.0);
				canvas_param.phiStep *= (M_PI / 180.0);
				canvas_param.phiMax *= (M_PI / 180.0);
#else
				canvas3d_param.rhoMin 	= pdata[0];
				canvas3d_param.rhoStep 	= pdata[1];
				canvas3d_param.rhoMax 	= pdata[2];
				canvas3d_param.elMin 	= pdata[3];
				canvas3d_param.elStep 	= pdata[4];
				canvas3d_param.elMax	= pdata[5];
				canvas3d_param.thetaMin = pdata[6];
				canvas3d_param.thetaStep= pdata[7];
				canvas3d_param.thetaMax	= pdata[8];

				//check if autorange
				if ((canvas3d_param.rhoMin < 0) || (canvas3d_param.rhoMax < 0)){
					canvas3d_param.rhoRangeAuto = true;
				}else{
					canvas3d_param.rhoRangeAuto = false;
				}
				//protocol transfer phi in degree, convert in radiants
				canvas3d_param.elMin *= (M_PI / 180.0);
				canvas3d_param.elStep *= (M_PI / 180.0);
				canvas3d_param.elMax *= (M_PI / 180.0);
				canvas3d_param.thetaMin *= (M_PI / 180.0);
				canvas3d_param.thetaStep *= (M_PI / 180.0);
				canvas3d_param.thetaMax *= (M_PI / 180.0);
#endif
				priv_update_rec(pdrv);
			}

		}
	}
	//response, regular
#if OPT_3D_MODE==0
	ppayload->data_struct.size = 4*6+1;
#else
	ppayload->data_struct.size = 4*9+1;
#endif
	ppayload->data_struct.buffer[0] = selAlgo;
	float * pdata = (float*)(ppayload->data_struct.buffer+1);
	//fill
#if OPT_3D_MODE == 0
	pdata[0] = canvas_param.rhoMin_actual;
	pdata[1] = canvas_param.rhoStep;
	pdata[2] = canvas_param.rhoMax_actual;
	//convert radiants > degrees
	pdata[3] = canvas_param.phiMin*(180.0/M_PI);
	pdata[4] = canvas_param.phiStep*(180.0/M_PI);
	pdata[5] = canvas_param.phiMax*(180.0/M_PI);
#else
	pdata[0] = canvas3d_param.rhoMin_actual;
	pdata[1] = canvas3d_param.rhoStep;
	pdata[2] = canvas3d_param.rhoMax_actual;
	//convert radiants > degrees
	pdata[3] = canvas3d_param.elMin*(180.0/M_PI);
	pdata[4] = canvas3d_param.elStep*(180.0/M_PI);
	pdata[5] = canvas3d_param.elMax*(180.0/M_PI);
	pdata[6] = canvas3d_param.thetaMin*(180.0/M_PI);
	pdata[7] = canvas3d_param.thetaStep*(180.0/M_PI);
	pdata[8] = canvas3d_param.thetaMax*(180.0/M_PI);
#endif

	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_sequence(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_struct.size != 0xFF){
		if (ppayload->data_struct.size <= DEF_MAX_SEQUENCES){
			int seqSize = ppayload->data_struct.size;
			uint8_t tx[seqSize];
			uint8_t rx[seqSize];
			uint8_t * pdata = ppayload->data_struct.buffer;
			for(int i = 0; i< seqSize; i++){
				tx[i] = mRDRCOM_decode_sequence_tx(pdata[i]);
				rx[i] = mRDRCOM_decode_sequence_rx(pdata[i]);
			}
			if (priv_update_sequence(pdrv, tx, rx, seqSize)){
				return COMM_CB_ERR_IO;
			}
		}
	}
	int numScan = acqHandler[0].numScan;
	ppayload->data_struct.size = numScan;
	for(int i = 0; i< numScan; i++){
		uint8_t encoded;
		encoded = mRDRCOM_encode_sequence(acqChannelsList[i].txmask, acqChannelsList[i].rxmask);
		ppayload->data_struct.buffer[i]=encoded;
	}
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_freq_carrier(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint16 != 0xFFFF){
		uint32_t freqMHz = (uint32_t)ppayload->data_uint16;
		if (HydrUDrv_set_carrier(pdrv, freqMHz)){
			return COMM_CB_ERR_IO;
		}
	}
	uint32_t freqMHz;
	if (HydrUDrv_get_carrier(pdrv, &freqMHz)){
		return COMM_CB_ERR_IO;
	}
	ppayload->data_uint16 = (uint16_t)freqMHz;
	return COMM_CB_SEND_REGULAR;
}


__attribute__ ((optimize("-Os")))
int comm_cb_freq_bw(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint16 != 0xFFFF){
		uint32_t freqMHz = (uint32_t)ppayload->data_uint16;
		if (HydrUDrv_set_bandwidth(pdrv, freqMHz)){
			return COMM_CB_ERR_IO;
		}
	}
	uint32_t freqMHz;
	if (HydrUDrv_get_bandwidth(pdrv, &freqMHz)){
		return COMM_CB_ERR_IO;
	}
	ppayload->data_uint16 = (uint16_t)freqMHz;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_iterations(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint16 != 0xFFFF){
		if (HydrUDrv_set_integrations(pdrv, ppayload->data_uint16)){
			return COMM_CB_ERR_IO;
		}
	}
	if (HydrUDrv_get_integrations(pdrv, &(ppayload->data_uint16))){
		return COMM_CB_ERR_IO;
	}
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_code(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_struct.size != 0xFF){
		if (HydrUDrv_set_code(pdrv, (int8_t*)(ppayload->data_struct.buffer), ppayload->data_struct.size)){
			return COMM_CB_ERR_IO;
		}
	}
	uint8_t size = sizeof(ppayload->data_struct.buffer);
	if (HydrUDrv_get_code(pdrv, (int8_t*)(ppayload->data_struct.buffer), &size)){
		return COMM_CB_ERR_IO;
	}
	ppayload->data_struct.size = size;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_min_Xrange(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint32 != 0xFFFFFFFF){
		float xmin, xmax;
		if (HydrUDrv_get_range(pdrv, &xmin, &xmax)){
			return COMM_CB_ERR_IO;
		}
		float newXmin = ppayload->data_float;
		xmin= newXmin;
		if (newXmin > xmax){
			xmax = newXmin+1.0; //override setting
		}
		if (priv_update_range(pdrv, xmin, xmax)){
			return COMM_CB_ERR_IO;
		}
	}
	//done

	float xmin, xmax;
	if (HydrUDrv_get_range(pdrv, &xmin, &xmax)){
		return COMM_CB_ERR_IO;
	}
	ppayload->data_float = xmin;
	return COMM_CB_SEND_REGULAR;
}


__attribute__ ((optimize("-Os")))
int comm_cb_max_Xrange(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint32 != 0xFFFFFFFF){
		float xmin, xmax;
		if (HydrUDrv_get_range(pdrv, &xmin, &xmax)){
			return COMM_CB_ERR_IO;
		}
		float newXmax = ppayload->data_float;
		if (newXmax < 0.0){
			newXmax = 1.0;
		}
		xmax= newXmax;

		if (newXmax < xmin){
			xmin = newXmax-1.0;
			if (xmin < 0.0){
				xmin = 0.0;
			}
		}
		if (priv_update_range(pdrv, xmin, xmax)){
			return COMM_CB_ERR_IO;
		}
	}
	//done

	float xmin, xmax;
	if (HydrUDrv_get_range(pdrv, &xmin, &xmax)){
		return COMM_CB_ERR_IO;
	}
	ppayload->data_float = xmax;
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_gain(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_struct.size != 0xFF){
		if (ppayload->data_struct.size == 4){
			uint16_t Gains[2];
			memcpy((void*)Gains, (const void*)(ppayload->data_struct.buffer), 4);
			if (HydrUDrv_set_gain(pdrv, Gains[0], Gains[1]))
				return COMM_CB_ERR_IO;
		}
	}
	uint16_t Gains[2];
	if (HydrUDrv_get_gain(pdrv, Gains, Gains+1))
		return COMM_CB_ERR_IO;
	ppayload->data_struct.size = 4;
	memcpy((void*)ppayload->data_struct.buffer, (const void*)Gains, 4);
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_declu_lgth(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint16 != 0xFFFF){
		//convert to normalized tau
		//if (ppayload->data_uint16 > 0){
			//if set to 0, tau_n become negative == no declutter
			float tau_n = (float)(ppayload->data_uint16-1);
			proc_compute_declatter_const(&declConst, tau_n);
		//}
	}
	ppayload->data_uint16 = roundf(declConst.normTau+1.0);
	return COMM_CB_SEND_REGULAR;
}

__attribute__ ((optimize("-Os")))
int comm_cb_output_size(radar_command cmd, radar_command_payload* UNUSED(ppayload), void * pars){

	comm_cb_args_t * pcbagrs = ((comm_cb_args_t *)pars);
	comm_msgbuffer_t msgbuffer;
	msgbuffer.size = 5;
	if (priv_get_message_buffer(&msgbuffer, pcbagrs->interfaceID))
		return COMM_CB_ERR_COMUNICATION;
	const struct _RadarCommandItem * pitem = priv_get_cmditem_by_radar_command(cmd);
	msgbuffer.buffer[0] = pitem->cmdCode;
	msgbuffer.buffer[1] = (uint8_t)pcbagrs->pacqHanlders[0].numScan;
	msgbuffer.buffer[2] = (uint8_t)commfmt_req;
	uint16_t size;
	size = pcbagrs->pacqHanlders[0].samplePerScan;
	msgbuffer.buffer[3] = (uint8_t)((size) & 0xFF);
	msgbuffer.buffer[4] = (uint8_t)((size>>8) & 0xFF);
	if (priv_send_message(msgbuffer, pcbagrs->interfaceID)){
		return COMM_CB_ERR_COMUNICATION;
	}
	return COMM_CB_SENT_CUSTOM;
}

__attribute__ ((optimize("-Os")))
int comm_cb_offset(radar_command UNUSED(cmd), radar_command_payload* ppayload, void * pars){
	HydrUDriver_t * pdrv = ((comm_cb_args_t *)pars)->pdrv;
	if (ppayload->data_uint32 != 0xFFFFFFFF){
		//set offset
		if (HydrUDrv_set_offset(pdrv,ppayload->data_float)){
			return COMM_CB_ERR_IO;
		}
		//change offset
		float xmin, xmax;
		HydrUDrv_get_range(pdrv, &xmin, &xmax);
		if (priv_update_range(pdrv, xmin, xmax)){
			return COMM_CB_ERR_IO;
		}
	}
	if (HydrUDrv_get_offset(pdrv, &(ppayload->data_float))){
		return COMM_CB_ERR_IO;
	}
	return COMM_CB_SEND_REGULAR;
}


static int priv_handle_datarequests(){
	//called after acqusition complete
	uint16_t datareqcode;	//encode the data request format
	uint16_t interface;
	radar_command cmd;
	if (priv_pop_datareq(&datareqcode, &interface, &cmd))
		return 0; //no requests pending
	if (!running){
		//return stop command
		comm_msgbuffer_t msgbuffer;
		msgbuffer.size = 1;
		if (priv_get_message_buffer(&msgbuffer, interface))
			return COMM_CB_ERR_COMUNICATION;
		const struct _RadarCommandItem * pitem = priv_get_cmditem_by_radar_command(RADARCOMM_STOP);
		msgbuffer.buffer[0]=pitem->cmdCode;
		if (priv_send_message(msgbuffer, interface))
			return COMM_CB_ERR_COMUNICATION;
		return 0;
	}
	//system is running and request is pending, send data according to format

	//due to the large amount of data, the system use streamed mode
	uint8_t * pbuf0;
	uint8_t * pbuf1;
	uint8_t * pworkingBuf;
	bool bufAlternate;

	uint32_t curdatareq = mDATAREQ_get_type(datareqcode);
	if ((curdatareq == DATAREQ_CODE_IMG) && !(exe_reconstruction && OPT_RECONSTRUCTION_ENABLE && user_exe_reconstruction)){
		//image not available
		//return stop command
		comm_msgbuffer_t msgbuffer;
		msgbuffer.size = 1;
		if (priv_get_message_buffer(&msgbuffer, interface))
			return COMM_CB_ERR_COMUNICATION;
		const struct _RadarCommandItem * pitem = priv_get_cmditem_by_radar_command(RADARCOMM_STOP);
		msgbuffer.buffer[0]=pitem->cmdCode;
		if (priv_send_message(msgbuffer, interface))
			return COMM_CB_ERR_COMUNICATION;
		return 0;
	}

	pbuf0 = txauxbuffer0;
	pbuf1 = txauxbuffer1;


	//setup header
	const struct _RadarCommandItem * pitem;
	pitem = priv_get_cmditem_by_radar_command(cmd);
	pbuf0[0] = pitem->cmdCode;
	uint16_t etime;
	etime = mDATAREQ_get_fmt(datareqcode) << 12;
	uint8_t encodefmt = mDATAREQ_get_fmt(datareqcode);
	if (curdatareq == DATAREQ_CODE_IMG){
		if (canvas_struct.iscomplex){
			//format is set to complex float16 by
			etime = DATAREQ_FMT_F16CPLX << 12;
			encodefmt = DATAREQ_FMT_F16CPLX; //override
		}
	}
	if (lastScanPeriod_ticks > 0xFFF){
		etime |= 0xFFF;
	}else{
		etime |= lastScanPeriod_ticks & 0xFFF;
	}
	int samples;
	if (curdatareq != DATAREQ_CODE_IMG){
		samples = acqHandler[0].samplePerScan;
		uint16_t samples_u16 = (uint16_t)samples;
		uint16_t totsamples = samples*(mDATAREQ_get_ch(datareqcode) == DATAREQ_CH_ALL ? acqHandler[0].numScan : 1);
		pworkingBuf = priv_serialize((void*)pbuf0+1,(const void*)&samples_u16 , 2);			//number of samples for each scan
		pworkingBuf = priv_serialize((void*)pworkingBuf,(const void*)&etime , 2);			//etime: resused bits 15-12, store the format
		pworkingBuf = priv_serialize((void*)pworkingBuf,(const void*)&totsamples , 2);		//total number of samples expected
	}else{
		//requested image
#if OPT_3D_MODE == 0
		uint16_t column = rec_h.rho_samples;
		uint16_t row = rec_h.phi_samples;
#else
		uint16_t column = rec3d_h.rho_samples;
		//row is splitted (0-7 el, 8-15 azim)
		uint16_t row = (rec3d_h.el_samples & 0xFF) | ((rec3d_h.theta_samples & 0xFF) << 8);
#endif
		pworkingBuf = priv_serialize((void*)pbuf0+1,(const void*)&column , 2);			//image column
		pworkingBuf = priv_serialize((void*)pworkingBuf,(const void*)&etime , 2);		//etime: resused bits 15-12, store the format
		pworkingBuf = priv_serialize((void*)pworkingBuf,(const void*)&row , 2);			//image row
#if OPT_3D_MODE == 0
		samples = row * column;
#else
		samples = column * rec3d_h.el_samples*rec3d_h.theta_samples;
#endif
	}

	//start sending header
	int ret = priv_send_stream(pbuf0, 7, true, false, interface);
	bufAlternate = true;
	if (ret){

		/*free((void*)pbuf0);
		free((void*)pbuf1);*/
		return COMM_CB_ERR_COMUNICATION;
	}

	int startStream = (mDATAREQ_get_ch(datareqcode)  == DATAREQ_CH_ALL) ? 0 : commch_req;
	int stopStream = (mDATAREQ_get_ch(datareqcode)  == DATAREQ_CH_ALL) ? (acqHandler[0].numScan-1) : commch_req;


	int samplesPerTransfer;

	switch (encodefmt){
		case DATAREQ_FMT_Q7:
			samplesPerTransfer = DEF_COMM_MAX_STREAM_CHUNK_SIZE/2; //I/Q 8 bit
			break;
		case DATAREQ_FMT_Q15:
		case DATAREQ_FMT_F16:
		case DATAREQ_FMT_F16CPLX:
			samplesPerTransfer = DEF_COMM_MAX_STREAM_CHUNK_SIZE/4; //I/Q 16 bit
			break;
		case DATAREQ_FMT_Q31:
		case DATAREQ_FMT_F32:
			samplesPerTransfer = DEF_COMM_MAX_STREAM_CHUNK_SIZE/8; //I/Q 16 bit
			break;
	}
	if (curdatareq == DATAREQ_CODE_IMG){

		if (canvas_struct.iscomplex == false)
			samplesPerTransfer *= 2; //image is real, could send double

		float * pdata = (float*)canvas_struct.rec_canvas;
		int samplescnt;
		int byteSize;
		samplescnt = 0;
		do{
			pworkingBuf = bufAlternate ? pbuf1 : pbuf0;
			int txfr;
			txfr = samplescnt + samplesPerTransfer;
			if (txfr > samples)
				txfr = samples;
			switch(encodefmt){
				case DATAREQ_FMT_Q7:
					byteSize = (txfr-samplescnt);
					for (;samplescnt < txfr; samplescnt++){
						float cval;
						cval = pdata[samplescnt];
						*(pworkingBuf++)=(q7_t)roundf(cval*127.0);
					}
					break;
				case DATAREQ_FMT_Q15:
					byteSize = (txfr-samplescnt)* 2;
					for (;samplescnt < txfr; samplescnt++){
						float cval;
						cval = pdata[samplescnt];
						*((q15_t*)pworkingBuf) = (q15_t)roundf(cval*65535.0);
						pworkingBuf += sizeof(q15_t);
					}
					break;
				case DATAREQ_FMT_F16:
					byteSize = (txfr-samplescnt)* 2;
					utils_conv_float2half((float*)(pdata+samplescnt), (float16_t*)pworkingBuf, (txfr-samplescnt));
					samplescnt = txfr;
					break;
				case DATAREQ_FMT_Q31:
					byteSize = (txfr-samplescnt)* 4;
					for (;samplescnt < txfr; samplescnt++){
						float cval;
						cval = pdata[samplescnt];
						*((q31_t*)pworkingBuf) = (q31_t)roundf(cval*(1<<31));
						pworkingBuf += sizeof(q31_t);
					}
					break;
				case DATAREQ_FMT_F16CPLX:	//F16 complex or abs F32 have same data handling
				case DATAREQ_FMT_F32:
					byteSize = (txfr-samplescnt)* sizeof(float);
					memcpy((void*)pworkingBuf, (const void*)(pdata+samplescnt), byteSize);
					samplescnt = txfr;
					break;
			}
			bool isend = (samplescnt == samples);
			ret = priv_send_stream(bufAlternate ? pbuf1 : pbuf0, byteSize, false, isend, interface);
			bufAlternate = !bufAlternate;
			if (ret){
				/*free((void*)pbuf0);
				free((void*)pbuf1);*/
				return COMM_CB_ERR_COMUNICATION;
			}
		}while(samplescnt < samples);
	} else {

		for(int i = startStream; i <= stopStream; i++){
			proc_stream_t * pcurstream = streams_container.pstreams+i;
			complex_float * pdata;
			switch(mDATAREQ_get_type(datareqcode) ){
				case DATAREQ_CODE_OUT:
					pdata = pcurstream->pout;
					break;
				case DATAREQ_CODE_DECL:
					pdata = pcurstream->pout;
					break;
				case DATAREQ_CODE_RAW:
				default:
					pdata = pcurstream->praw;
					break;
			}

			int samplescnt;
			int byteSize;
			samplescnt = 0;

			do{
				pworkingBuf = bufAlternate ? pbuf1 : pbuf0;
				int txfr;
				txfr = samplescnt + samplesPerTransfer;
				if (txfr > samples)
					txfr = samples;
				switch(encodefmt){
					case DATAREQ_FMT_Q7:
						byteSize = (txfr-samplescnt)* 2;
						for (;samplescnt < txfr; samplescnt++){
							complex_float cval;
							cval = pdata[samplescnt];
							*(pworkingBuf++)=(q7_t)roundf(cval.real*127.0);
							*(pworkingBuf++)=(q7_t)roundf(cval.imag*127.0);
						}
						break;
					case DATAREQ_FMT_Q15:
						byteSize = (txfr-samplescnt)* 4;
						for (;samplescnt < txfr; samplescnt++){
							complex_float cval;
							cval = pdata[samplescnt];
							*((q15_t*)pworkingBuf) = (q15_t)roundf(cval.real*65535.0);
							*((q15_t*)(pworkingBuf+sizeof(q15_t))) = (q15_t)roundf(cval.imag*65535.0);
							pworkingBuf += 2*sizeof(q15_t);
						}
						break;
					case DATAREQ_FMT_F16:
						byteSize = (txfr-samplescnt)* 4;
						utils_conv_float2half((float*)(pdata+samplescnt), (float16_t*)pworkingBuf, (txfr-samplescnt)*2);
						samplescnt = txfr;
						break;
					case DATAREQ_FMT_Q31:
						byteSize = (txfr-samplescnt)* 8;
						for (;samplescnt < txfr; samplescnt++){
							complex_float cval;
							cval = pdata[samplescnt];
							*((q31_t*)pworkingBuf) = (q31_t)roundf(cval.real*(1<<31));
							*((q31_t*)(pworkingBuf+sizeof(q31_t))) = (q31_t)roundf(cval.imag*(1<<31));
							pworkingBuf += 2*sizeof(q31_t);
						}
						break;

					case DATAREQ_FMT_F32:
						byteSize = (txfr-samplescnt)* sizeof(complex_float);
						memcpy((void*)pworkingBuf, (const void*)(pdata+samplescnt), byteSize);
						samplescnt = txfr;
						break;
				}
				bool isend = (samplescnt == samples) && (i == stopStream);
				ret = priv_send_stream(bufAlternate ? pbuf1 : pbuf0, byteSize, false, isend, interface);
				bufAlternate = !bufAlternate;
				if (ret){
					/*free((void*)pbuf0);
					free((void*)pbuf1);*/
					return COMM_CB_ERR_COMUNICATION;
				}

			}while(samplescnt < samples);

		}
	}
	/*free((void*)pbuf0);
	free((void*)pbuf1);*/
	return 0;

}

/**
 * Auxiliary function, used by message handlers, to transfer large amount of data
 * @param pbuf
 * @param size
 * @param isstart
 * @param isend
 * @param interface
 * @return SUCCESS/FAIL
 */
static int priv_send_stream(uint8_t* pbuf, int size, bool isstart, bool isend, int interface){
	if (interface == INTERFACE_ID_COMM){
		return comm_send_stream(pbuf, size, isstart, isend, true);
	}else if (interface == INTERFACE_ID_SPI){
		return comm_sspi_send_stream(pbuf, size, isstart, isend, true);
	}
	return -1; //unknown interface number
}

/**
 * Auxiliary function for serialize data
 * @param pdst
 * @param psrc
 * @param size
 */
static void *priv_serialize(void* pdst, const void* psrc, int size){
	memcpy(pdst, psrc, size);
	return ((void*)((uint8_t*)pdst)+size);
}



/**
 * @page pgUserEP User entry points
 * \section User entry points
 * Section where is suggested to enter user code are marked are USER_ENTRY\n
 * Inside app.c search section marked as USER_ENTRY, section are commented in-site on how to edit the section\n
 * */
