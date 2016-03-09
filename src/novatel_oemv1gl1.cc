/** \file novatel_oemv1gl1.cc
 * \author Robotnik Automation S.L.L.
 * \version 3.0
 * \date    2016
 *
 * \brief class novatel_oemv1gl1 - Class for RTK DGPS sensor Novatel OEMV-1G-L1
 * (C) 2012 Robotnik Automation, SLL
*/
#include "novatel_oemv/novatel_oemv1gl1.h"
#include "novatel_oemv/SerialDevice.h"
#include <novatel_oemv/bestxyza.h>	// ros msg 

#include <time.h>
#include <pthread.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <ros/package.h>


// tokens=split(input,str_count+1);//Splitting the line
string *split(string line,int count){// Splitting a line string into substrings
	char *token;
	char *str=new char[line.length()];
	for(unsigned int i=0;i<line.length();i++){
		str[i]=line[i];//For strtok function, creating a char* instead of const char*
	        }
	string *element=new string[count];//the substring array
	int i=0;
	token=strtok(str,",");
	while(token !=NULL){//splitting with respect to comma
		element[i]=string(token);
		i++;
		token=strtok(NULL,",");
                }
	return element;
}


/*!	\fn novatel_oemv1gl1::novatel_oemv1gl1(const char *device, double hz)
 * 	\brief Public constructor
*/
novatel_oemv1gl1::novatel_oemv1gl1( const char *device, double hz, string mode ) : Component( hz ) {

    iErrorType = NOVATEL_OEMV1GL1_ERROR_NONE;

    sComponentName.assign("novatel_oemv1gl1");

    // Create serial port
    serial = new SerialDevice(device, NOVATEL_OEMV1GL1_DEFAULT_TRANSFERRATE,
		NOVATEL_OEMV1GL1_DEFAULT_PARITY, NOVATEL_OEMV1GL1_DEFAULT_DATA_SIZE, 
		NOVATEL_OEMV1GL1_DEFAULT_SERIAL_HZ ); //Creates serial device
	
	// Define operation mode 
	mode_ = mode;
}

/*!	\fn novatel_oemv1gl1::~novatel_oemv1gl1()
 * 	\brief Public destructor
*/
novatel_oemv1gl1::~novatel_oemv1gl1(){
//pthread_mutex_destroy(& mutex_record );
    delete serial;
}

/*!	\fn ReturnValue novatel_oemv1gl1::Open(char *dev)
 * 	\brief Open component ports and files
 * 	\returns ERROR
 * 	\returns OK
*/
ReturnValue novatel_oemv1gl1::Open(){
    // Sets the same Log port and IP
    this->serial->SetLogParameters(iLogPort, (char*)sLogIp.c_str());

    // Configure serial device
	this->serial->SetCanonicalInput(true);
	if (this->serial->Setup()!=OK) {
		ROS_ERROR("novatel_oemv1gl1::Open : Error calling serial device Setup");
		return ERROR;
	    }
	// Open serial device
	if(this->serial->Start()!=OK) {
		ROS_ERROR("novatel_oemv1gl1::Open: Error calling serial device Start");
		SwitchToState(EMERGENCY_STATE);
		iErrorType = NOVATEL_OEMV1GL1_ERROR_OPENING;
		return ERROR;
	}

    // Open all log files
    OpenLogFiles();

    SwitchToState(INIT_STATE);
    return OK;
}

/*!	\fn ReturnValue novatel_oemv1gl1::Close()
 * 	\brief Closes component serial port and log files
 * 	\returns ERROR
 * 	\returns OK
*/
ReturnValue novatel_oemv1gl1::Close(){

	ReturnValue ret = OK;

	// Close serial port
    if (serial!=NULL) {
        serial->Stop();
        serial->ShutDown();
    }

    // Close all log files
    CloseLogFiles();

	return ret;
}

/*!	\fn ReturnValue SerialDevice::SendMessage(char *msg)
	* Sends strings to the SerialDevice
*/
ReturnValue novatel_oemv1gl1::SendMessage(char *msg)
{
	int written_bytes=0;
	int length;
	char cAux[LOG_STRING_LENGTH]="\0";

	length = (int) strlen(msg);

    if (serial->WritePort(msg, &written_bytes, length)!=OK) {
        ROS_ERROR("novatel_oemv1gl1::SendMessage : Error writing bytes");
        return ERROR;
    } else {
        if (written_bytes!=length) {
            ROS_ERROR("novatel_oemv1gl1::SendMessage : Error written_bytes=%d  message_length=%d", written_bytes, length);
            return ERROR;
        }
    }

    return OK;
}

/*!	\fn void novatel_oemv1gl1::FReset()
 * 	\brief Performs a reset to factory defaults
*/
void novatel_oemv1gl1::FReset(){

    this->SendMessage((char*)"freset\r");
}

/*!	\fn void novatel_oemv1gl1::InitState()
 * 	\brief Actions in the initial state
 * 	First call-> Set IMU in continuous mode -> IF ERROR go to EMERGENCY_STATE
*											-> IF OK inits communication's timers
* 	Next call -> Reads messages from the sensor and compares timers	-> IF no responses go to EMERGENCY_STATE
* 																	-> IF OK go to READY_STATE
*/
void novatel_oemv1gl1::InitState(){
	//struct timespec	tNext, tNow;
	static bool bFirst = true;
	long int diff = 0;

	if(bFirst){	//Fist time -> send init commands
        // Start data logging process
        SendInitCommands();
		usleep(2000000);
		bFirst = false;
		clock_gettime(this->threadData.pthreadPar.clock, &tNext);
		tNext.tv_sec += NOVATEL_OEMV1GL1_TIMEOUT_COMM;
		tsnorm(&tNext);
		//tsnorm(&tNow);
		//diff = calcdiff(tNext, tNow);
		//printf("novatel_oemv1gl1::InitState: FIRST diff= %ld\n",diff);
    } else	{	//Next
		clock_gettime(this->threadData.pthreadPar.clock, &tNow);
		diff = calcdiff(tNext, tNow);
		//printf("novatel_oemv1gl1::InitState: diff= %ld\n",diff);

		if(diff <= 0){ //Fail in the communication, because any message has been received (see ProcessMsg())
			//printf("novatel_oemv1gl1::InitState: Timeout in communication\n");
			ROS_ERROR("novatel_oemv1gl1::InitState: Timeout in communication");
			iErrorType = NOVATEL_OEMV1GL1_ERROR_TIMEOUT;
			SwitchToState(EMERGENCY_STATE);
			bFirst = true;
		 } else {
			SwitchToState(READY_STATE);
           }
        }
}

/*!	\fn int novatel_oemv1gl1::SendInitCommands()
 * 	\brief Sends the initial commands to the configured serial port
 * 	\return OK if command sent successfully
 * 	\return ERROR otherwise
*/
int novatel_oemv1gl1::SendInitCommands()
{
    // USB to novatel USB - usleep neded
    if(mode_=="NORMAL"){
        ROS_INFO("novatel_oemv1gl1::SendInitCommands: Enable");
		this->SendMessage((char*)"log rxstatusa once \r");
		usleep(100000);
		this->SendMessage((char*)"log versiona once \r");
		usleep(100000);
		this->SendMessage((char*)"log usb1 ionutca onnew \r");
		usleep(100000);
		this->SendMessage((char*)"log usb1 bestposa ontime 5 0 hold \r");
		usleep(100000);
		this->SendMessage((char*)"log usb1 bestxyza ontime 5 0 hold \r");
		usleep(100000);
		}

    if(mode_=="EGNOS"){
        ROS_INFO("novatel_oemv1gl1::SendInitCommands: Egnos Enable");
        this->SendMessage((char*)"rtksource rtcm any \r");
        usleep(100000);
        this->SendMessage((char*)"psrdiffsource rtcm any \r");
        usleep(100000);
        this->SendMessage((char*)"sbascontrol enable auto \r");
        usleep(100000);
		this->SendMessage((char*)"log rxstatusa once \r");
		usleep(100000);
		this->SendMessage((char*)"log versiona once \r");
		usleep(100000);
		this->SendMessage((char*)"log usb1 ionutca onnew \r");
		usleep(100000);
		this->SendMessage((char*)"log usb1 bestposa ontime 5 0 hold \r");
		usleep(100000);
		this->SendMessage((char*)"log usb1 bestxyza ontime 5 0 hold \r");
		usleep(100000);
		}
        
    if(mode_ == "PDPFILTER"){
        ROS_INFO("novatel_oemv1gl1::SendInitCommands: PDP Filter Enable");
        this->SendMessage((char*)"sbascontrol enable auto \r");
        usleep(100000);
        this->SendMessage((char*)"pdpfilter enable \r");
        usleep(100000);
        this->SendMessage((char*)"pdpmode relative auto \r");
        usleep(100000);
        this->SendMessage((char*)"setionotype grid \r");
        usleep(100000);
		this->SendMessage((char*)"log rxstatusa once \r");
		usleep(100000);
		this->SendMessage((char*)"log versiona once \r");
		usleep(100000);
		this->SendMessage((char*)"log com1 ionutca onnew \r");
		usleep(100000);
		this->SendMessage((char*)"log com1 bestposa ontime 1 0 hold \r");
		usleep(100000);		
		this->SendMessage((char*)"log com1 bestxyza ontime 1 0 hold \r");
		usleep(100000);		
		this->SendMessage((char*)"log com1 pdpxyza ontime 1 0 hold \r");		
		// usleep(100000);		
		// this->SendMessage((char*)"log com1 pdpposa ontime 1 0 hold \r");		
		}    
		
    // For GL1DE (is a type of PDP filter)
    if(mode_ == "GL1DE") {		
		ROS_INFO("novatel_oemv1gl1::SendInitCommands: Configuration for GL1DE");
		this->SendMessage((char*)"sbascontrol enable auto \r");
		usleep(100000);
		this->SendMessage((char*)"pdpfilter enable \r");
		usleep(100000);
		this->SendMessage((char*)"setionotype grid \r");
		usleep(100000);
		this->SendMessage((char*)"pdpmode relative dynamic \r");
		usleep(100000);
		this->SendMessage((char*)"log com1 rxstatusa once \r");
		usleep(100000);
		this->SendMessage((char*)"log com1 versiona once \r");
		usleep(100000);
		this->SendMessage((char*)"log com1 pdpposa ontime 0.2 \r");	
		}

    return OK;
}

/*!	\fn int novatel_oemv1gl1::SendUnlogCommands()
 * 	\brief Send Stop logging commands
 * 	\return OK if command sent successfully
 * 	\return ERROR otherwise
*/
int novatel_oemv1gl1::SendUnlogCommands(void)
{
    this->SendMessage((char*)"unlog ionutca\r");
    usleep(100000);
    this->SendMessage((char*)"unlog bestposa\r");
    usleep(100000);
    this->SendMessage((char*)"unlog bestxyza\r");
    usleep(100000);
    this->SendMessage((char*)"unlog pdpxyza\r");
    usleep(100000);
    this->SendMessage((char*)"unlog pdpposa\r");
    usleep(100000);
	return OK;
}

/*!	\fn void novatel_oemv1gl1::ReadyState()
 * 	\brief Actions in Ready state
 * 	First time -> Gets current time
 * 	Next times -> Read from sensor and checks the timeout between responses
 -> IF TIMEOUT go to EMERGENCY_STATE
 -> IF OK resets communication's timer
*/
void novatel_oemv1gl1::ReadyState(){
	static bool bFirst = true;
	long diff;

	if(bFirst){	//Fist time -> saves the current time for controlling timeout in the communication
		bFirst = false;
		clock_gettime(this->threadData.pthreadPar.clock, &tNext);
		tNext.tv_sec+= NOVATEL_OEMV1GL1_TIMEOUT_COMM;
		tsnorm(&tNext);
		}
	else {	// Next times
		clock_gettime(this->threadData.pthreadPar.clock, &tNow);
		diff = calcdiff(tNext, tNow);

		if(diff <= 0){	//Fails in the communication, because any message has been received (see ProcessMsg())
			printf("novatel_oemv1gl1::ReadyState: Timeout in communication\n");
			iErrorType = NOVATEL_OEMV1GL1_ERROR_TIMEOUT;
			SwitchToState(EMERGENCY_STATE);
			bFirst = true;
		}else {
            if(ReadOEMV4Frame()==OK) {	//When READ -> Reset timer
				clock_gettime(this->threadData.pthreadPar.clock, &tNext);
				tNext.tv_sec+= NOVATEL_OEMV1GL1_TIMEOUT_COMM;
				tsnorm(&tNext);
			}else{
                ROS_ERROR("novatel_oemv1gl1: Error Reading FRAME");
			}
		}
	}
}

/*!	\fn void novatel_oemv1gl1::EmergencyState()
 * 	\brief Actions in Emergency State
*/
void novatel_oemv1gl1::EmergencyState(){
	int timer = 5000000; //useconds

	switch(iErrorType)	{

		case NOVATEL_OEMV1GL1_ERROR_OPENING: //Try to recover
			ROS_ERROR("novatel_oemv1gl1::Failure: Recovering from emergency state : ERROR_OPENING");
			Close();
			usleep(timer);
			if(Open()==OK)
				this->Start();
			break;

		case NOVATEL_OEMV1GL1_ERROR_SERIALCOMM:
			ROS_ERROR("novatel_oemv1gl1::Failure: Recovering from emergency state : ERROR_SERIALCOMM");
			Close();
			usleep(timer);
			Open();
			break;

		case NOVATEL_OEMV1GL1_ERROR_TIMEOUT:
			ROS_ERROR("novatel_oemv1gl1::Failure: Recovering from emergency state : ERROR_TIMEOUT");
			Close();
			usleep(timer);
			Open();
			break;
	}
}

/*!	\fn int novatel_oemv1gl1::ReadOEMV4Frame
 * 	\brief Poll the device for complete ASCII frame
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ReadOEMV4Frame()
{
	char response[360] = "\0";
	int read_bytes=0;
    	int ret;

	serial->ReadPort( response, &read_bytes, 350 );

    if (read_bytes>0) {
        // Only ascii mode
        //printf("novatel_oemv1gl1::ProcessOEMV4Frame - dwBytesRead %d\n", dwBytesRead);
        //printf("novatel_oemv1gl1::ProcessOEMV4Frame : %d bytes\n %s\n", read_bytes, response );

        // Log all received frames (before strtok)
        if ( full_ofstream.is_open() ) {
            full_ofstream << response;
        }

        // Process received frame
        if (read_bytes>10) ret = ProcessOEMV4Frame( response );
    }

	return OK;
}

/*!	\fn int novatel_oemv1gl1::ProcessOEMV4Frame
 * 	\brief  Processes and ASCII OEMV4 frame and updates object data structures
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessOEMV4Frame(char* str)
{
    char * msg;
    char * sptr;
    char aux[500];
    bool bFirst=true;
    unsigned long ulCrc=0;
    unsigned long ulCount;
    char sCrc[15];
    char cAux[LOG_STRING_LENGTH]="\0";

    // Before processing frame check CRC
    strcpy(aux, str);
    //ROS_INFO("aux = %s", aux );
    msg = strtok_r( aux, "#*", &sptr );
    
    //ROS_INFO("msg = %s", msg );

    do {
        if (bFirst) {
            ulCount = (unsigned long) strlen( msg );
            if (ulCount<10) return ERROR;
            ulCrc=CalculateBlockCRC32( ulCount, (unsigned char*) msg );
            bFirst=false;
        }
        else {
            // Bug 8/4/2010 - some crc values cause buffer overflow with sCrc[10]
            // "0x" + "FFFFFFFF" + "\r\n" (Max ulong is 0xFFFFFFFF (32 bits)) = 12 characters
            sprintf(sCrc,"%08x\r\n", (unsigned int)ulCrc );
            if (strcmp(sCrc,msg)!=0) {
                if (strlen(msg)>10) {
			ROS_ERROR("novatel_oemv1gl1::ProcessOEMV4Frame - strlen > 10");
			return ERROR;
			}
		ROS_ERROR("novatel_oemv1gl1::ProcessOEMV4Frame - CRC Error : Read:%s  Computed:%s", msg, sCrc);
                return ERROR;
            }
        }
    } while((  msg = strtok_r( NULL, "*", &sptr )));

    // Now process frame
    char * sptr1;
    char * sptr2;
    char * pch=(char*)"";
    bool bHeader=true;

    msg = strtok_r( aux, ";", &sptr1 );
    ROS_DEBUG("novatel_oemv1gl1::ProcessOEMV4Frame: Read:%s", msg);
    do
    {
       // Header processing
       if (bHeader) {
            pch = strtok_r(str,",", &sptr2); // split command name from header
            // According to cmd header, defines type
            // of message and how to process it

            if (strcmp(pch,"#BESTXYZA")==0){
               ProcessAsciiMsgHeader(msg, &bestxyza.amh );
                //  ROS_INFO("novatel_oemv1gl1::ProcessOEMV4Frame: BESTXYZA: gps_time_status (%d)", bestxyza.amh.gps_time_status);
            }
            if (strcmp(pch,"#PDPXYZA")==0) ProcessAsciiMsgHeader(msg, &pdpxyza.amh );
            else if (strcmp(pch,"#BESTPOSA")==0) ProcessAsciiMsgHeader(msg, &bestposa.amh );
            else if (strcmp(pch,"#IONUTCA")==0) ProcessAsciiMsgHeader(msg, &ionutca.amh );
            else if (strcmp(pch,"#VERSIONA")==0) ProcessAsciiMsgHeader(msg, &versiona.amh );
            else if (strcmp(pch,"#PDPPOSA")==0) ProcessAsciiMsgHeader(msg, &pdpposa.amh );
            //ROS_INFO("novatel_oemv1gl1::ProcessOEMV4Frame: Header true:%s", pch);
            bHeader=false;
        } else {
        // Data processing           
	    // if mode is OEMV1G-1.01-TT

         if (strcmp(pch,"#BESTXYZA")==0) ProcessFrameBestxyza(msg);
            else if (strcmp(pch,"#PDPXYZA")==0) ProcessFramePdpxyza(msg);
            else if (strcmp(pch,"#BESTPOSA")==0) ProcessFrameBestposa(msg);
            else if (strcmp(pch,"#IONUTCA")==0) ProcessFrameIonutca(msg);
            else if (strcmp(pch,"#VERSIONA")==0) ProcessFrameVersiona(msg);
            else if (strcmp(pch,"#PDPPOSA")==0) ProcessFramePdpposa(msg);
            // ROS_INFO("novatel_oemv1gl1::ProcessOEMV4Frame: Header false:%s", pch);
        }
    }
    while( (msg = strtok_r( NULL, ";", &sptr1 )) );

    return OK;
}



/*!	\fn enum gps_time_status_t novatel_oemv1gl1::StringToGpsStatus(char *val)
 * 	\brief  Processes the GPS' status string from the device and converts it to an integer value
 * 	\return Current gps_time_status_t
*/
enum gps_time_status_t novatel_oemv1gl1::StringToGpsStatus(char *val){
    if(!strcmp(val, "UNKNOWN"))  return _UNKNOWN;
    if(!strcmp(val, "APPROXIMATE"))  return _APPROXIMATE;
    if(!strcmp(val, "COARSEADJUSTING"))  return _COARSEADJUSTING;
    if(!strcmp(val, "COARSE"))  return _COARSE;
    if(!strcmp(val, "COARSESTEERING"))  return _COARSESTEERING;
    if(!strcmp(val, "FREEWHEELING"))  return _FREEWHEELING;
    if(!strcmp(val, "FINEADJUSTING"))  return _FINEADJUSTING;
    if(!strcmp(val, "FINE"))  return _FINE;
    if(!strcmp(val, "FINESTEERING"))  return _FINESTEERING;
    if(!strcmp(val, "SATTIME"))  return _SATTIME;

    return _UNKNOWN;
}

/*!	\fn pv_sol_status_t novatel_oemv1gl1::StringToSolStatus(char *val)
 * 	\brief  Processes the Solution status string from the device and converts it to an integer value
 * 	\return Current pv_sol_status_t
*/
enum pv_sol_status_t novatel_oemv1gl1::StringToSolStatus(char *val){
    if(!strcmp(val, "SOL_COMPUTED"))  return _SOL_COMPUTED;    if(!strcmp(val, "INSUFFICIENT_OBS"))  return _INSUFFICIENT_OBS;
    if(!strcmp(val, "NO_CONVERGENCE"))  return _NO_CONVERGENCE;    if(!strcmp(val, "SINGULARITY"))  return _SINGULARITY;
    if(!strcmp(val, "COV_TRACE"))  return _COV_TRACE;  if(!strcmp(val, "TEST_DIST"))  return _TEST_DIST;
    if(!strcmp(val, "COLD_START"))  return _COLD_START;    if(!strcmp(val, "_V_H_LIMIT"))  return _V_H_LIMIT;
    if(!strcmp(val, "VARIANCE"))  return _VARIANCE;    if(!strcmp(val, "RESIDUALS"))  return _RESIDUALS;
    if(!strcmp(val, "DELTA_POS"))  return _DELTA_POS;    if(!strcmp(val, "NEGATIVE_VAR"))  return _NEGATIVE_VAR;
    if(!strcmp(val, "RESERVED"))  return _RESERVED;    if(!strcmp(val, "INTEGRITY_WARNING"))  return _INTEGRITY_WARNING;
    if(!strcmp(val, "INS1"))  return _INS1;    if(!strcmp(val, "INS2"))  return _INS2;
    if(!strcmp(val, "INS3"))  return _INS3;    if(!strcmp(val, "INS4"))  return _INS4;
    if(!strcmp(val, "PENDING"))  return _PENDING;  if(!strcmp(val, "INVALID_FIX"))  return _INVALID_FIX;
    if(!strcmp(val, "UNAUTHORIZED"))  return _UNAUTHORIZED;    if(!strcmp(val, "ANTENNA_WARNING"))  return _ANTENNA_WARNING;

    return _INSUFFICIENT_OBS;
}

/*!	\fn pv_type_t novatel_oemv1gl1::StringToType(char *val)
 * 	\brief  Processes the type string from the device and converts it to an integer value
 * 	\return Current pv_type_t
*/
enum pv_type_t novatel_oemv1gl1::StringToType(char *val){
    if(!strcmp(val, "NONE"))  return _NONE;    if(!strcmp(val, "FIXEDPOS"))  return _FIXEDPOS;
    if(!strcmp(val, "FIXEDHEIGHT"))  return _FIXEDHEIGHT;    if(!strcmp(val, "DOPPLER_VELOCITY"))  return _DOPPLER_VELOCITY;
    if(!strcmp(val, "SINGLE"))  return _SINGLE;    if(!strcmp(val, "PSRDIFF"))  return _PSRDIFF;
    if(!strcmp(val, "WAAS"))  return _WAAS;    if(!strcmp(val, "PROPAGATED"))  return _PROPAGATED;
    if(!strcmp(val, "OMNISTAR"))  return _OMNISTAR;    if(!strcmp(val, "L1_FLOAT"))  return _L1_FLOAT;
    if(!strcmp(val, "IONOFREE_FLOAT"))  return _IONOFREE_FLOAT;    if(!strcmp(val, "NARROW_FLOAT"))  return _NARROW_FLOAT;
    if(!strcmp(val, "L1_INT"))  return _L1_INT;    if(!strcmp(val, "WIDE_INT"))  return _WIDE_INT;
    if(!strcmp(val, "NARROW_INT"))  return _NARROW_INT;    if(!strcmp(val, "RTK_DIRECT_INS"))  return _RTK_DIRECT_INS;
    if(!strcmp(val, "INS"))  return _INS;    if(!strcmp(val, "INS_PSRSP"))  return _INS_PSRSP;
    if(!strcmp(val, "INS_PSRDIFF"))  return _INS_PSRDIFF;    if(!strcmp(val, "INS_RTKFLOAT"))  return _INS_RTKFLOAT;
    if(!strcmp(val, "INS_RTKFIXED"))  return _INS_RTKFIXED;    if(!strcmp(val, "OMNISTAR_HP"))  return _OMNISTAR_HP;
    if(!strcmp(val, "OMNISTAR_XP"))  return _OMNISTAR_XP;    if(!strcmp(val, "CDGPS"))  return _CDGPS;

    return _NONE;
}

/*!	\fn int novatel_oemv1gl1::ProcessAsciiMsgHeader
 * 	\brief  Processes Msg Header from ASCII frame
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessAsciiMsgHeader(char* str, ascii_msg_header_t* _amh)
{
    char * pch;
    int ivariable;
    char cAux[LOG_STRING_LENGTH]="\0";
    pch = strtok(str,",");

    ivariable=1;
    while (pch != NULL)
    {
        // Move each value to its data structure field
        switch (ivariable) {
            case 1: strcpy(_amh->cmd, pch);                  // # + cmd name
                    break;
            case 2: strcpy(_amh->cmdPort, pch);
                    break;
            case 3: _amh->sequence=(unsigned long) atol(pch);
                    break;
            case 4: _amh->percent_idle_time=atof(pch);
                    break;
            case 5: _amh->gps_time_status = StringToGpsStatus(pch);//(gps_time_status_t) atoi(pch);
                    // ROS_INFO("novatel_oemv1gl1::ProcessAsciiMsgHeader: variable %d, %s (%d)", ivariable, pch,  _amh->gps_time_status);
                    break;
            case 6: _amh->gps_week_number = (unsigned long) atol(pch);
                    break;
            case 7: _amh->seconds=(float) atof(pch);
                    break;
            case 8: _amh->receiver_status=(unsigned long) atol(pch);
                    break;
                    break;
            case 9: _amh->reserved=(unsigned long) atol(pch);
                    break;
            case 10:_amh->receiver_sw_version=(unsigned long) atol(pch);
                    break;
            default: break;
        }

        pch = strtok (NULL, ",");
        ivariable++;
    }

 /* 
  // Debug
  ROS_INFO("novatel_oemv1gl1::ProcessAsciiMsgHeader");
  ROS_INFO("  command=%s", _amh->cmd);
  ROS_INFO("  cmdPort=%s", _amh->cmdPort);
  ROS_INFO("  sequence=%lu", _amh->sequence);
  ROS_INFO("  percent_idle_time=%f", _amh->percent_idle_time);
  ROS_INFO("  gps_time_status=%d", _amh->gps_time_status);
  ROS_INFO("  gps_week_number=%lu", _amh->gps_week_number);
  ROS_INFO("  seconds=%f", _amh->seconds);
  ROS_INFO("  receiver_status=%lu", _amh->receiver_status);
  ROS_INFO("  receiver_sw_version=%lu", _amh->receiver_sw_version);
  */

  return OK;
}

/*!	\fn int novatel_oemv1gl1::ProcessFrameBestxyza
 * 	\brief  Processes #BESTXYZA frame
 *  \brief  Best Available Cartesian Position and Velocity V_123
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessFrameBestxyza(char* str)
{
  char * pch;
  int ivariable;
  char cAux[LOG_STRING_LENGTH] = "\0";

  bestxyza.bReceived = true;

  pch = strtok(str,",");
  ivariable=2;
  while (pch != NULL)
  {
    // Move each value to its data structure field
    switch (ivariable) {
        case 1: break;                  // BESTXYZ Log header
        case 2: bestxyza.p_sol_status= StringToSolStatus(pch); // P-sol status Solution status, see Table 51, Solution Status
                //ROS_INFO("novatel_oemv1gl1::ProcessFrameBestxyza: sol status = %s (%d)", pch, bestxyza.p_sol_status);
                break;
        case 3: bestxyza.pos_type= StringToType(pch);       // Position type, see Table 50, Position or Velocity Enum
                // ROS_INFO("novatel_oemv1gl1::ProcessFrameBestxyza: pos type= %s (%d)", pch, bestxyza.pos_type);
                break;
        case 4: bestxyza.p.x=atof(pch);          // Position X-coordinate (m)
                break;
        case 5: bestxyza.p.y=atof(pch);          // Position Y-coordinate (m)
                break;
        case 6: bestxyza.p.z=atof(pch);          // Position Z-coordinate (m)
                break;
        case 7: bestxyza.p_std.x=(float) atof(pch);      // Standard deviation of P-X (m)
                break;
        case 8: bestxyza.p_std.y=(float) atof(pch);      // Standard deviation of P-Y (m)
                break;
        case 9: bestxyza.p_std.z=(float) atof(pch);      // Standard deviation of P-Z (m)
                break;
        case 10:bestxyza.v_sol_status= StringToSolStatus(pch);         // V-sol status Solution status, see Table 51, Solution Status
                break;
        case 11:bestxyza.vel_type= StringToType(pch);             // Velocity type, see Table 50, Position or Velocity Enum
                break;
        case 12:bestxyza.v.x=atof(pch);                  // Velocity vector along X-axis (m/s)
                break;
        case 13:bestxyza.v.y=atof(pch);                  // Velocity vector along Y-axis (m/s)
                break;
        case 14:bestxyza.v.z=atof(pch);                  // Velocity vector along Z-axis (m/s)
                break;
        case 15:bestxyza.v_std.x=(float) atof(pch);      // Standard deviation of V-X (m/s)
                break;
        case 16:bestxyza.v_std.y=(float) atof(pch);      // Standard deviation of V-Y (m/s)
                break;
        case 17:bestxyza.v_std.z=(float) atof(pch);      // Standard deviation of V-Z (m/s)
                break;
        case 18:// sprintf(bestxyza.stnID,"%s", pch);       // Base station identification
                // bug detected id comes with quotes "120" 
                break;
        case 19:bestxyza.v_latency=(float) atof(pch);    // A measure of the latency in the velocity time tag Float
                break;                                   // in seconds. It should be subtracted from the time to give improved results.
        case 20:bestxyza.diff_age=(float) atof(pch);     // Differential age in seconds
                break;
        case 21:bestxyza.sol_age=(float) atof(pch);      // Solution age in seconds
                break;
        case 22:bestxyza.SVs=(unsigned char) atoi(pch);  // Number of satellite vehicles tracked
                break;
        case 23:bestxyza.solnSVs=(unsigned char) atoi(pch);// Number of satellite vehicles used in solution
                break;
        case 24:bestxyza.ggL1=(unsigned char) atoi(pch);   // Number of gps plus glonass L1 used in solution
                break;
        case 25:bestxyza.ggL1L2=(unsigned char) atoi(pch); // Number of gps plus glonass L1 and L2 used in solution
                break;
        case 26://char reserved1;
                break;
        case 27:bestxyza.ext_sol_stat=(unsigned char) atoi(pch); // (hex) extended solution status
                break;
        case 28://char reserved2;
                break;
        case 29:bestxyza.sig_mask=(unsigned char) atoi(pch);  // (hex) signals used mask 0=unknown
                break;
        default:break;
        }

    pch = strtok (NULL, ",");
    ivariable++;
  }

  // Write this location in log file
  if (csv_ofstream.is_open()) {
    csv_ofstream << bestxyza.amh.seconds << "," << bestxyza.amh.sequence << ","
                 << bestxyza.p_sol_status << "," << bestxyza.pos_type << ","
                 << bestxyza.p.x << "," << bestxyza.p.y << "," << bestxyza.p.z << ","
                 << bestxyza.p_std.x << "," << bestxyza.p_std.y << "," << bestxyza.p_std.z << ","
                 << bestxyza.v_sol_status << "," << bestxyza.vel_type << ","
                 << bestxyza.v.x << "," << bestxyza.v.y << "," << bestxyza.v.z << ","
                 << bestxyza.v_std.x << "," << bestxyza.v_std.y << "," << bestxyza.v_std.z << ","
                 << bestxyza.stnID << "," << bestxyza.v_latency << "," << bestxyza.diff_age << "," << bestxyza.sol_age << ","
                 << (int) bestxyza.SVs << "," << (int) bestxyza.solnSVs << endl;

    csv_ofstream.flush();
    // ROS_INFO("novatel_oemv1gl1::ProcessFrameBestxyza: Logging into file");
    }

  return OK;
}

/*!	\fn int novatel_oemv1gl1::ProcessFramePdpxyza
 * 	\brief  Processes #PDPXYZA frame
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessFramePdpxyza(char* str)
{
    char * pch;
    int ivariable;
    char cAux[LOG_STRING_LENGTH] = "\0";
    pdpxyza.bReceived = true;

    pch = strtok(str,",");
    ivariable=2;
    while (pch != NULL)
    {
        // Move each value to its data structure field
        switch (ivariable) {
            case 1: break;                  // BESTXYZ Log header
            case 2: pdpxyza.p_sol_status = StringToSolStatus(pch); // P-sol status Solution status, see Table 51, Solution Status
               // ROS_INFO("novatel_oemv1gl1::ProcessPdpxya: sol status = %s (%d)", pch, pdpxyza.p_sol_status);
            break;
            case 3: pdpxyza.pos_type= StringToType(pch);       // Position type, see Table 50, Position or Velocity Enum
               // ROS_INFO("novatel_oemv1gl1::ProcessPdpxya: pos type = %s (%d)", pch, pdpxyza.pos_type);
            break;
            case 4: pdpxyza.p.x=atof(pch);          // Position X-coordinate (m)
                    break;
            case 5: pdpxyza.p.y=atof(pch);          // Position Y-coordinate (m)
                    break;
            case 6: pdpxyza.p.z=atof(pch);          // Position Z-coordinate (m)
                    break;
            case 7: pdpxyza.p_std.x=(float) atof(pch);      // Standard deviation of P-X (m)
                    break;
            case 8: pdpxyza.p_std.y=(float) atof(pch);      // Standard deviation of P-Y (m)
                    break;
            case 9: pdpxyza.p_std.z=(float) atof(pch);      // Standard deviation of P-Z (m)
                    break;
            case 10:pdpxyza.v_sol_status= StringToSolStatus(pch);         // V-sol status Solution status, see Table 51, Solution Status
                    break;
            case 11:pdpxyza.vel_type= StringToType(pch);             // Velocity type, see Table 50, Position or Velocity Enum
                    break;
            case 12:pdpxyza.v.x=atof(pch);                  // Velocity vector along X-axis (m/s)
                    break;
            case 13:pdpxyza.v.y=atof(pch);                  // Velocity vector along Y-axis (m/s)
                    break;
            case 14:pdpxyza.v.z=atof(pch);                  // Velocity vector along Z-axis (m/s)
                    break;
            case 15:pdpxyza.v_std.x=(float) atof(pch);      // Standard deviation of V-X (m/s)
                    break;
            case 16:pdpxyza.v_std.y=(float) atof(pch);      // Standard deviation of V-Y (m/s)
                    break;
            case 17:pdpxyza.v_std.z=(float) atof(pch);      // Standard deviation of V-Z (m/s)
                    break;
            case 18://sprintf(pdpxyza.stnID,"%s", pch);       // Base station identification
                    // bug detected stnID comes with quotes "120"
                    break;
            case 19:pdpxyza.v_latency=(float) atof(pch);    // A measure of the latency in the velocity time tag Float
                    break;                                  // in seconds. It should be subtracted from the time to give improved results.
            case 20:pdpxyza.diff_age=(float) atof(pch);     // Differential age in seconds
                    break;
            case 21:pdpxyza.sol_age=(float) atof(pch);      // Solution age in seconds
                    break;
            case 22:pdpxyza.SVs=(unsigned char) atoi(pch);  // Number of satellite vehicles tracked
                    break;
            case 23:pdpxyza.solnSVs=(unsigned char) atoi(pch); // Number of satellite vehicles used in solution
                    break;
            case 24:// reserved
            case 25:// reserved
            case 26:// reserved
            case 27:// reserved
            case 28:// reserved
            case 29:// reserved
                    break;
            default:break;
        }

        pch = strtok (NULL, ",");
        ivariable++;
    }

    // Write this location in log file
    if (pdpcsv_ofstream.is_open()) {
        pdpcsv_ofstream << pdpxyza.amh.seconds << "," << pdpxyza.amh.sequence << ","
             << pdpxyza.p_sol_status << "," << pdpxyza.pos_type << ","
             << pdpxyza.p.x << "," << pdpxyza.p.y << "," << pdpxyza.p.z << ","
             << pdpxyza.p_std.x << "," << pdpxyza.p_std.y << "," << pdpxyza.p_std.z << ","
             << pdpxyza.v_sol_status << "," << pdpxyza.vel_type << ","
             << pdpxyza.v.x << "," << pdpxyza.v.y << "," << pdpxyza.v.z << ","
             << pdpxyza.v_std.x << "," << pdpxyza.v_std.y << "," << pdpxyza.v_std.z << endl;
    }

  return OK;
}

/*!	\fn int novatel_oemv1gl1::ProcessFrameBestposa
 * 	\brief  Processes #BESTPOSA frame
 *  \brief  Contains the best available combined GPS and inertial navigation system (INS - if available)
 *  \brief  position (in metres) computed by the receiver. V_123
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessFrameBestposa(char* str)
{
  char * pch;
  int ivariable;

  bestposa.bReceived = true;

  pch = strtok(str,",");
  ivariable=2;
  while (pch != NULL)
  {
    //ROS_INFO("variable %d,  str %s", ivariable, pch);
    // Move each value to its data structure field
    switch (ivariable) {
        
        case 1: break;                  // BESTXYZ Log header
        case 2: bestposa.p_sol_status= StringToSolStatus(pch); // P-sol status Solution status, see Table 51, Solution Status
                break;
        case 3: //bestposa.pos_type= StringToType(pch);       // Position type, see Table 50, Position or Velocity Enum
                break;
        case 4: bestposa.lat = atof(pch);           // latitude
                break;
        case 5: bestposa.lon = atof(pch);           // longitude
                break;
        case 6: bestposa.hgt = atof(pch);           // height above mean sea level
                break;
        case 7: bestposa.undulation = (float) atof(pch);  // Undulation - the relationship between the geoid and
                break;                              // the ellipsoid (m) of the chosen datum a
        case 8: //enum datum_id_t datum_id;         // Datum ID number (see Chapter 2, Table 21,
                break;
        case 9: bestposa.lat_std= (float) atof(pch);    // Latitude standard deviation
                break;
        case 10: bestposa.lon_std = (float) atof(pch);  // Longitude standard deviation
                break;
        case 11: bestposa.hgt_std = (float) atof(pch);  // Height standard deviation
                break;
        case 12://sprintf(bestposa.stnID,"%s", pch);       // Base station identification
                // bug detected base station ID comes with quotes "120"
                break;
        case 13:bestposa.diff_age=(float) atof(pch);     // Differential age in seconds
                break;
        case 14:bestposa.sol_age=(float) atof(pch);      // Solution age in seconds
                break;
        case 15:bestposa.SVs=(unsigned char) atoi(pch);  // Number of satellite vehicles tracked
                break;
        case 16:bestposa.solnSVs=(unsigned char) atoi(pch); // Number of satellite vehicles used in solution
                break;
        case 17:bestposa.ggL1=(unsigned char) atoi(pch);   // Number of gps plus glonass L1 used in solution
                break;
        case 18:bestposa.ggL1L2=(unsigned char) atoi(pch); // Number of gps plus glonass L1 and L2 used in solution
                break;
        case 19://char reserved1;
                break;
        case 20:bestposa.ext_sol_stat=(unsigned char) atoi(pch); // (hex) extended solution status
                break;
        case 21://char reserved2;
                break;
        case 22:bestposa.sig_mask=(unsigned char) atoi(pch);  // (hex) signals used mask 0=unknown
                break;
        default:break;
        }

    pch = strtok (NULL, ",");
    ivariable++;
  }

  // Write this location in log file
  if (kml_ofstream.is_open()) {
    if (bestposa.lat!=0.0 && bestposa.lon!=0.0) {
        kml_ofstream << bestposa.lon << ","
                    << bestposa.lat << ","
                    << bestposa.hgt << endl;
        }
    }

  return OK;
}

/*!	\fn int novatel_oemv1gl1::ProcessFrameIonutca
 * 	\brief  Processes #IONUTCA frame
 *  \brief  The Ionospheric Model parameters (ION) and the Universal Time Coordinated parameters (UTC) are
 *  \brief  provided. V_123
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessFrameIonutca(char* str)
{
  char * pch;
  int ivariable;

  ionutca.bReceived = true;

  pch = strtok(str,",");
  ivariable=2;
  while (pch != NULL)
  {
    // Move each value to its data structure field
    switch (ivariable) {
        case 1: break;                  // Log header
        case 2: ionutca.a0 = atof(pch); // Alpha parameter constant term
                break;
        case 3: ionutca.a1 = atof(pch); // Alpha parameter 1st order term
                break;
        case 4: ionutca.a2 = atof(pch); // Alpha parameter 2nd order term
                break;
        case 5: ionutca.a3 = atof(pch); // Alpha parameter 3rd order term
                break;
        case 6: ionutca.b0 = atof(pch); // Beta parameter constant term
                break;
        case 7: ionutca.b1 = atof(pch); // Beta parameter 1st order term
                break;
        case 8: ionutca.b2 = atof(pch); // Beta parameter 2nd order term
                break;
        case 9: ionutca.b3 = atof(pch); // Beta parameter 3rd order term
                break;
        case 10: ionutca.utc_wn = (unsigned long) atol(pch);   // UTC reference week number
                break;
        case 11: ionutca.tot = (unsigned long) atol(pch);   // Reference time of UTC parameters
                break;
        case 12: ionutca.A0 = atof(pch);    // UTC constant term of polynomial
                break;
        case 13: ionutca.A1 = atof(pch);    // UTC 1st order term of polynomial
                break;
        case 14: ionutca.wn_lsf = (unsigned long) atol(pch); // Future week number
                break;
        case 15: ionutca.dn = (unsigned long) atol(pch);    // Day number (the range is 1 to 7 where
                break;                                      // Sunday = 1 and Saturday = 7)
        case 16: ionutca.deltat_ls = atol(pch);  // Delta time due to leap seconds
                break;
        case 17: ionutca.deltat_lsf = atol(pch);    // Future delta time due to leap seconds
                break;
        case 18: ionutca.deltat_utc = (unsigned long) atol(pch); // Time difference
                break;
        default:break;
        }

    pch = strtok (NULL, ",");
    ivariable++;
  }

  return OK;
}


/*!	\fn int novatel_oemv1gl1::ProcessFramePdpposa
 * 	\brief  Processes #PDPPOSA frame
 *  \brief  The PDPPOS log contains the pseudorange position computed by 
 *          the receiver with the PDP filter enabled
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessFramePdpposa(char* str)
{
  char * pch;
  int ivariable;

  pdpposa.bReceived = true;

  pch = strtok(str,",");
  ivariable=2;
  while (pch != NULL)
  {
    //ROS_INFO("variable %d,  str %s", ivariable, pch);
    // Move each value to its data structure field    
    switch (ivariable) {
        
        case 1: break;                  // PDPPOSA Log header
        case 2: pdpposa.p_sol_status= StringToSolStatus(pch); // P-sol status Solution status, see Table 51, Solution Status
                break;
        case 3: pdpposa.pos_type= StringToType(pch);       // Position type, see Table 50, Position or Velocity Enum
                break;
        case 4: pdpposa.lat = atof(pch);           // latitude
                break;
        case 5: pdpposa.lon = atof(pch);           // longitude
                break;
        case 6: pdpposa.hgt = atof(pch);           // height above mean sea level
                break;
        case 7: pdpposa.undulation = (float) atof(pch);  // Undulation - the relationship between the geoid and
                break;                              // the ellipsoid (m) of the chosen datum a
        case 8: //enum datum_id_t datum_id;         // Datum ID number (see Chapter 2, Table 21,
                break;
        case 9: pdpposa.lat_std= (float) atof(pch);    // Latitude standard deviation
                break;
        case 10: pdpposa.lon_std = (float) atof(pch);  // Longitude standard deviation
                break;
        case 11: pdpposa.hgt_std = (float) atof(pch);  // Height standard deviation
                break;
        case 12://sprintf(pdpposa.stnID,"%s", pch);       // Base station identification
                // bug detected base station ID comes with quotes "120"
                break;
        case 13:pdpposa.diff_age=(float) atof(pch);     // Differential age in seconds
                break;
        case 14:pdpposa.sol_age=(float) atof(pch);      // Solution age in seconds
                break;
        case 15:pdpposa.SVs=(unsigned char) atoi(pch);  // Number of satellite vehicles tracked
                break;
        case 16:pdpposa.solnSVs=(unsigned char) atoi(pch); // Number of satellite vehicles used in solution
                break;
        case 17:
        case 18:
        case 19:
                break;
        case 20:pdpposa.ext_sol_stat = (unsigned char) atoi(pch); // (hex) extended solution status
                break;
        case 21:pdpposa.galileo_beidou_sig_mask = (unsigned char) atoi(pch); // (hex) extended solution status
                break;
        case 22:pdpposa.gps_glonass_sig_mask = (unsigned char) atoi(pch); // (hex) extended solution status
                break;                
        default:break;
        }

    pch = strtok (NULL, ",");
    ivariable++;
  }

  // Write this location in log file
  if (kml_ofstream.is_open()) {
    if (pdpposa.lat!=0.0 && pdpposa.lon!=0.0) {
        kml_ofstream << pdpposa.lon << ","
                    << pdpposa.lat << ","
                    << pdpposa.hgt << endl;
        }
    }


  // Debug
  /*
  ROS_INFO("novatel_oemv1gl1::ProcessPdpposa");
  ROS_INFO("  Sol Status=%d", pdpposa.p_sol_status);
  ROS_INFO("  pos_type=%d", pdpposa.pos_type);
  ROS_INFO("  lat=%5.2f", pdpposa.lat);
  ROS_INFO("  lon=%5.2f", pdpposa.lon);
  ROS_INFO("  hgt=%5.2f", pdpposa.hgt);      
  ROS_INFO("  SVs=%d", pdpposa.SVs);
  */

  return OK;
}




/*!	\fn int novatel_oemv1gl1::ProcessFrameVersion
 * 	\brief  Processes #VERSIONA frame
 *  \brief  This log contains the version information for all components of a system. When using a standard
 *  \brief  receiver, there is only one component in the log. V_123
 * 	\return OK
 * 	\return ERROR
*/
int novatel_oemv1gl1::ProcessFrameVersiona(char* str)
{
  char * pch;
  int ivariable;

  versiona.bReceived = true;

  pch = strtok(str,",");
  ivariable=2;

  while (pch != NULL)
  {
    // Move each value to its data structure field
    switch (ivariable) {
        case 1: break;                      // Log header
        case 2: versiona.comp=atol(pch);    // Number of componentes
                break;
        case 3: strcpy(versiona.type, pch);          // OEMV family component type: GPSCARD, CONTROLLER, ENCLOSURE, IMUCARD
                break;
        case 4: strcpy(versiona.model, pch);         // A base model name plus designators L12,L1,N12,N1
                break;
        case 5: strcpy(versiona.psn, pch);           // Product serial number
                break;
        case 6: strcpy(versiona.hw_version,pch);     // Hardware version
                break;
        case 7: strcpy(versiona.sw_version,pch);      // Firmware software version
                break;
        case 8: strcpy(versiona.boot_version, pch);   // Boot code version
                break;
        case 9: strcpy(versiona.comp_date, pch);      // Firmware compile date
                break;
        case 10:strcpy(versiona.comp_time, pch);     // Firmware compile time
                break;
        default:break;
        }

    pch = strtok (NULL, ",");
    ivariable++;
  }

  return OK;
}

/*!	\fn unsigned long novatel_oemv1gl1::CRC32Value
 * 	\brief Calculate a CRC value to be used by CRC calculation functions.
 * 	\return
*/
unsigned long novatel_oemv1gl1::CRC32Value(int i)
{
   int j;
   unsigned long ulCRC;
   ulCRC = i;
   for ( j = 8 ; j > 0; j-- )
   {
      if ( ulCRC & 1 )
          ulCRC = ( ulCRC >> 1 ) ^ NOVATEL_CRC32_POLYNOMIAL;
      else
          ulCRC >>= 1;
   }
   return ulCRC;
}

/*!	\fn unsigned long novatel_oemv1gl1::CalculateBlockCRC32
 * 	\brief Calculates the CRC-32 of a block of data all at once
 *  \param Number of bytes in the data block
 *  \param Data block
 * 	\return Computed crc value
*/
unsigned long novatel_oemv1gl1::CalculateBlockCRC32( unsigned long ulCount, unsigned char *ucBuffer )
{
   unsigned long ulTemp1;
   unsigned long ulTemp2;
   unsigned long ulCRC = 0;

   while ( ulCount-- != 0 ) {
     ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;     
     ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );     
     ulCRC = ulTemp1 ^ ulTemp2;
     }

  return( ulCRC );
}


/*! \fn void novatel_oemv1gl1::Display_message()
 	* \brief Prints all class' attributes
*/
void novatel_oemv1gl1::Display_message()
{
   //printf("Hz= %lf\n",pthread_hz);
  if (versiona.bReceived) {
    printf("\nFRAME #VERSIONA \n");
    printf("  comp=%ld\n", versiona.comp);
    printf("  type=%s\n", versiona.type);
    printf("  model=%s\n", versiona.model);
    printf("  psn=%s\n", versiona.psn);
    printf("  hw_version=%s\n", versiona.hw_version);
    printf("  sw_version=%s\n", versiona.sw_version);
    printf("  boot_version=%s\n", versiona.boot_version);
    printf("  comp_date=%s\n", versiona.comp_date);
    printf("  comp_time=%s\n", versiona.comp_time);
    }

  if (bestxyza.bReceived) {
    printf("\nFRAME #BESTXYZA \n");
    printf("  p_sol_status:%d\n", bestxyza.p_sol_status);
    printf("  pos_type:%d\n", bestxyza.pos_type);
    printf("  p.x=%f p.y=%f p.z=%f\n", bestxyza.p.x, bestxyza.p.y, bestxyza.p.z);
    printf("  p_std.x=%f p_std.y=%f p_std.z=%f\n", bestxyza.p_std.x, bestxyza.p_std.y, bestxyza.p_std.z);
    printf("  v_sol_status:%d\n", bestxyza.v_sol_status);
    printf("  vel_type:%d\n", bestxyza.vel_type);
    printf("  v.x=%f v.y=%f v.z=%f\n", bestxyza.v.x, bestxyza.v.y, bestxyza.v.z);
    printf("  v_std.x=%f v_std.y=%f v_std.z=%f\n", bestxyza.v_std.x, bestxyza.v_std.y, bestxyza.v_std.z);
    printf("  stnID %s\n", bestxyza.stnID );
    printf("  v_latency=%f, diff_age=%f, sol_age=%f\n", bestxyza.v_latency, bestxyza.diff_age, bestxyza.sol_age);
    printf("  SVs=%d\n", (int) bestxyza.SVs);
    printf("  solnSVs=%d\n", (int) bestxyza.solnSVs);
    printf("  ggL1=%d  ggL1L2=%d\n", (int) bestxyza.ggL1, (int) bestxyza.ggL1L2);
    printf("  ext_sol_stat=%d\n", (int) bestxyza.ext_sol_stat);
    printf("  sig_mask=%d\n", (int) bestxyza.sig_mask);
    }

  if (pdpxyza.bReceived) {
    printf("\nFRAME #PDPXYZA \n");
    printf("  p_sol_status:%d\n", pdpxyza.p_sol_status);
    printf("  pos_type:%d\n", pdpxyza.pos_type);
    printf("  p.x=%f p.y=%f p.z=%f\n", pdpxyza.p.x, pdpxyza.p.y, pdpxyza.p.z);
    printf("  p_std.x=%f p_std.y=%f p_std.z=%f\n", pdpxyza.p_std.x, pdpxyza.p_std.y, pdpxyza.p_std.z);
    printf("  v_sol_status:%d\n", pdpxyza.v_sol_status);
    printf("  vel_type:%d\n", pdpxyza.vel_type);
    printf("  v.x=%f v.y=%f v.z=%f\n", pdpxyza.v.x, pdpxyza.v.y, pdpxyza.v.z);
    printf("  v_std.x=%f v_std.y=%f v_std.z=%f\n", pdpxyza.v_std.x, pdpxyza.v_std.y, pdpxyza.v_std.z);
    printf("  stnID %s\n", pdpxyza.stnID );
    printf("  v_latency=%f, diff_age=%f, sol_age=%f\n", pdpxyza.v_latency, pdpxyza.diff_age, pdpxyza.sol_age);
    printf("  SVs=%d\n", (int) pdpxyza.SVs);
    printf("  solnSVs=%d\n", (int) pdpxyza.solnSVs);
    }

  if (bestposa.bReceived) {
    printf("\nFRAME #BESTPOSA \n");
    printf("  p_sol_status:%d\n", bestxyza.p_sol_status);
    printf("  pos_type:%d\n", bestxyza.pos_type);
    printf("  latitude=%f longitude=%f heigth=%f\n", bestposa.lat, bestposa.lon, bestposa.hgt);
    printf("  undulation=%f \n", bestposa.undulation);
    printf("  lat_std.x=%f lon_std.y=%f hgt_std.z=%f\n", bestposa.lat_std, bestposa.lon_std, bestposa.hgt_std);
    printf("  stnID %s\n", bestposa.stnID );
    printf("  diff_age=%f, sol_age=%f\n", bestposa.diff_age, bestposa.sol_age);
    printf("  SVs=%d\n", (int) bestposa.SVs);
    printf("  solnSVs=%d\n", (int) bestposa.solnSVs);
    printf("  ggL1=%d  ggL1L2=%d\n", (int) bestposa.ggL1, (int) bestposa.ggL1L2);
    printf("  ext_sol_stat=%d\n", (int) bestposa.ext_sol_stat);
    printf("  sig_mask=%d\n", (int) bestposa.sig_mask);
    }

  if (ionutca.bReceived) {
    printf("\nFRAME #IONUTCA\n");
    printf("  a0=%f a1=%f a2=%f a3=%f\n", ionutca.a0, ionutca.a1, ionutca.a2, ionutca.a3);
    printf("  b0=%f b1=%f b2=%f b3=%f\n", ionutca.b0, ionutca.b1, ionutca.b2, ionutca.b3);
    printf("  utc_wn=%lu\n", ionutca.utc_wn);
    printf("  tot=%lu\n", ionutca.tot);
    printf("  A0=%f A1=%f\n", ionutca.A0, ionutca.A1);
    printf("  wn_lsf=%lu\n", ionutca.wn_lsf);
    printf("  dn=%lu\n", ionutca.dn);
    printf("  delta_ls=%ld\n", ionutca.deltat_ls);
    printf("  delta_lsf=%ld\n", ionutca.deltat_lsf);
    printf("  delta_utc=%lu\n", ionutca.deltat_utc);
    }
}

/*! \fn bestxyza_t novatel_oemv1gl1::GetBestxyza()
    * \brief Gets the current bestxyza value
*/
bestxyza_t novatel_oemv1gl1::GetBestxyza(){
    return bestxyza;
}

/*! \fn bestposa_t novatel_oemv1gl1::GetBestposa()
    * \brief Gets the current bestposa value
*/
bestposa_t novatel_oemv1gl1::GetBestposa(){
    return bestposa;
}

/*! \fn pdpxyza_t novatel_oemv1gl1::GetPdpxyza()
    * \brief Gets the current pdpxyza value
*/
pdpxyza_t novatel_oemv1gl1::GetPdpxyza(){
    return pdpxyza;
}

/*! \fn ionutca_t novatel_oemv1gl1::GetIonutca()
    * \brief Gets the current Ionutca value
*/
ionutca_t novatel_oemv1gl1::GetIonutca(){
    return ionutca;
}

/*! \fn pdpposa_t novatel_oemv1gl1::GetPdpposa()
    * \brief Gets the current pdpposa value
*/
pdpposa_t novatel_oemv1gl1::GetPdpposa(){
    return pdpposa;
}

/*! \fn void novatel_oemv1gl1::OpenLogFiles()
    * \brief Open all log files
*/
void novatel_oemv1gl1::OpenLogFiles()
{
    // Open KML file to start writing tracks and write header
    OpenKMLFile();

    // Open CSV file
    OpenCSVFile();

    // Open full log file
    string output_file;
    std::string path = ros::package::getPath("novatel_oemv");
    output_file= path + "/logs/rovlog.txt";
    full_ofstream.open(output_file.c_str(),ios::out); //Opening output file for writing
}

/*! \fn void novatel_oemv1gl1::CloseLogFiles()
    * \brief Close all log files
*/
void novatel_oemv1gl1::CloseLogFiles()
{
    // Close KML file to start writing tracks and write header
    CloseKMLFile();

    // Close CSV file
    CloseCSVFile();

    // Close full log txt file
    full_ofstream.close();
}

/*! \fn void novatel_oemv1gl1::OpenKMLFile()
 	* \brief Open KML file to start writing tracks and write header
*/
void novatel_oemv1gl1::OpenKMLFile()
{
	string output_file;
        std::string path = ros::package::getPath("novatel_oemv");
	output_file= path + "/logs/rovlog.kml";

        ROS_INFO("Open KML file in %s", output_file.c_str() );

	kml_ofstream.open(output_file.c_str(),ios::out); //Opening output file for writing

    kml_ofstream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl
        << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << endl
        << "<Document>" << endl
        << "<name>Paths</name>" << endl
        << "<description>Robotnik GUARDIAN Path</description>" << endl
        << "<Style id=\"yellowLineGreenPoly\">" << endl
        << "  <LineStyle>" << endl
        << "    <color>7f00ffff</color>" << endl
        << "    <width>4</width>" << endl
        << "  </LineStyle>" << endl
        << "  <PolyStyle>" << endl
        << "    <color>7f00ff00</color>" << endl
        << "  </PolyStyle>" << endl
        << "</Style>" << endl
        << "<Placemark>" << endl
        << "  <name>GUARDIAN Path</name>" << endl
        << "  <description>Transparent green wall with yellow outlines</description>" << endl
        << "  <styleUrl>#yellowLineGreenPoly</styleUrl>" << endl
        << "  <LineString>" << endl
        << "  <extrude>1</extrude>" << endl
        << "  <tessellate>1</tessellate>" << endl
        << "  <altitudeMode>absolute</altitudeMode>" << endl
        << "  <coordinates>" << endl;
}

/*! \fn void novatel_oemv1gl1::CloseKMLFile()
 	* \brief Write to complete and close KML file
*/
void novatel_oemv1gl1::CloseKMLFile()
{
    kml_ofstream << "  </coordinates>" << endl
        << "  </LineString>" << endl
        << "</Placemark>" << endl
        << "</Document>" << endl
        << "</kml>" << endl;

    kml_ofstream.close();
}

/*! \fn void novatel_oemv1gl1::OpenCSVFile()
 	* \brief Open CSV file to start writing tracks and write header
*/
void novatel_oemv1gl1::OpenCSVFile()
{
	string output_file;
        std::string path = ros::package::getPath("novatel_oemv");
	output_file= path + "/logs/rovlog.csv";

	csv_ofstream.open(output_file.c_str(),ios::out); //Opening output file for writing
        csv_ofstream << "seconds,sequence,p_sol_status,pos_type,x,y,z,std(x),std(y),std(z),"
                 << "v_sol_status,vel_type,vx,vy,vz,std(vx),std(vy),std(vz),"
                 << "stnID,v_latency,diff_age,bestxyza.sol_age,SVs,solnSVs" << endl;

    output_file= path + "/logs/rovlogpdp.csv";
    pdpcsv_ofstream.open(output_file.c_str(),ios::out); //Opening output file for writing
    pdpcsv_ofstream << "seconds,sequence,p_sol_status,pos_type,x,y,z,std(x),std(y),std(z),"
                 << "v_sol_status,vel_type,vx,vy,vz,std(vx),std(vy),std(vz)" << endl;
}

/*! \fn void novatel_oemv1gl1::CloseCSVFile()
 	* \brief Closes CSV file
*/
void novatel_oemv1gl1::CloseCSVFile()
{
	csv_ofstream.close(); //Close output file
	pdpcsv_ofstream.close(); //Close output file
}

