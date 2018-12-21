MODULE LOGGER

!////////////////
!GLOBAL VARIABLES
!////////////////
!PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
PERS string ipController;
PERS num loggerPort;

!Clock Synchronization
PERS bool startLog;
PERS bool startRob;

!Robot configuration	
PERS tooldata currentTool;    
PERS wobjdata currentWobj;
VAR speeddata currentSpeed;
VAR zonedata currentZone;
PERS num loggerWaitTime;

PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	TPWrite "LOGGER: Logger waiting for incoming connections ...";
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
		IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
			TPWrite "LOGGER: Problem serving an incoming connection.";
			TPWrite "LOGGER: Try reconnecting.";
		ENDIF
		 !Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "LOGGER: Connected to IP " + clientIP;
ENDPROC

PROC main()

	!We have a counter which will output values to digital IO lines
	VAR num syncCounter:= 170;
	VAR num syncMax:= 2047;
	
	
	
	VAR string data;
	VAR robtarget position;
	VAR jointtarget joints;
    !VAR fcforcevector forceTorque;
	VAR string sendString;
	VAR bool connected;

	VAR string date;
	VAR string time;
	VAR clock timer;

	
	
	startLog:=FALSE;
	WaitUntil startRob \PollRate:=0.01;
	startLog:=TRUE;
	ClkStart timer;

	date:= CDate();
	time:= CTime();


	
	
	connected:=FALSE;
	ServerCreateAndConnect ipController,loggerPort;	
	connected:=TRUE;
	WHILE TRUE DO
	
	
		!test block, toggles high/low every 2 seconds
		!setGO lSync, 1000;	
		!WaitTime 2;	
		!setGO lSync, 1;	
		!WaitTime 2;		
		!/test block
		
		
		IF syncCounter < syncMax THEN
			syncCounter:= syncCounter + 1;
		ELSE
			syncCounter:=0;
		ENDIF
		
		
		!Flip sync signal value as close (timing) to the read of cartesian position as possible 
		setGO lSync, syncCounter;
		
		!Read out Cartesian Coordinates
		position := CRobT(\Tool:=currentTool \WObj:=currentWObj);

		
		
		!Format output string
		data := "# 0 ";
		data := data + NumToStr(syncCounter,0) + " ";
		data := data + NumToStr(position.trans.x,1) + " ";
		data := data + NumToStr(position.trans.y,1) + " ";
		data := data + NumToStr(position.trans.z,1) + " ";
		data := data + NumToStr(position.rot.q1,3) + " ";
		data := data + NumToStr(position.rot.q2,3) + " ";
		data := data + NumToStr(position.rot.q3,3) + " ";
		data := data + NumToStr(position.rot.q4,3); !End of string

		
		IF connected = TRUE THEN
			SocketSend clientSocket \Str:=data;
			!TPWrite data;
		ENDIF
		WaitTime loggerWaitTime;
	
		!Joint Coordinates
		!joints := CJointT();
		!data := "# 1 ";
		!data := data + date + " " + time + " ";
		!data := data + NumToStr(ClkRead(timer),2) + " ";
		!data := data + NumToStr(joints.robax.rax_1,2) + " ";
		!data := data + NumToStr(joints.robax.rax_2,2) + " ";
		!data := data + NumToStr(joints.robax.rax_3,2) + " ";
		!data := data + NumToStr(joints.robax.rax_4,2) + " ";
		!data := data + NumToStr(joints.robax.rax_5,2) + " ";
		!data := data + NumToStr(joints.robax.rax_6,2); !End of string
		!IF connected = TRUE THEN
		!	SocketSend clientSocket \Str:=data;
		!ENDIF
		!WaitTime loggerWaitTime;
	
		!Force/Torque readings
		!forceTorque:= FCGetForce();
		!data := "# 2 ";
		!data := data + date + " " + time + " ";
		!data := data + NumToStr(ClkRead(timer),2) + " ";
		!data := data + NumToStr(forceTorque.xforce,2) + " ";
		!data := data + NumToStr(forceTorque.yforce,2) + " ";
		!data := data + NumToStr(forceTorque.zforce,2) + " ";
		!data := data + NumToStr(forceTorque.xtorque,2) + " ";
		!data := data + NumToStr(forceTorque.ytorque,2) + " ";
		!data := data + NumToStr(forceTorque.ztorque,2); !End of string

		!IF connected = TRUE THEN
		!	SocketSend clientSocket \Str:=data;
		!ENDIF
		!WaitTime loggerWaitTime;
		
		
	ENDWHILE
ERROR
	IF ERRNO=ERR_SOCK_CLOSED THEN
		TPWrite "LOGGER: Client has closed connection.";
	ELSE
		TPWrite "LOGGER: Connection lost: Unknown problem #:" + NumToStr(ERRNO,0);
	ENDIF
	connected:=FALSE;
	!Closing the server
	SocketClose clientSocket;
	SocketClose serverSocket;
	!Reinitiate the server
	ServerCreateAndConnect ipController,loggerPort;
	connected:= TRUE;
	RETRY;
ENDPROC

ENDMODULE