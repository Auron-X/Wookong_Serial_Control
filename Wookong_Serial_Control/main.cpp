#include <WinSock2.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <string.h>
#include <vector>

#include <opencv2\opencv.hpp>

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "opencv_core249d.lib")
#pragma comment(lib, "opencv_features2d249d.lib")
#pragma comment(lib, "opencv_highgui249d.lib")
#pragma comment(lib, "opencv_imgproc249d.lib")
#pragma comment(lib, "opencv_objdetect249d.lib")
#pragma comment(lib, "opencv_video249d.lib")




using namespace std;
using namespace cv;

HANDLE quadroSerial, imuSerial;
char key;
bool stopSend = false;

LPCTSTR quadroPort = L"COM2"; 
LPCTSTR imuPort = L"COM1";

int lpevtFlag = EV_BREAK | EV_RXCHAR | EV_RXFLAG | EV_TXEMPTY | EV_ERR;

//Monitoring mode
//0 - Transmitter 1-4 channels
//1 - Transmitter Full
//2 - Serial send (Transmitter + Autopilot) 1-4 channels
//3 - Autopilot 1-4 channels
//4 - Bypass
int monitoring_mode = 2;

//FCC parameters
bool bypass_aileron = false;
bool bypass_elevator = false;
bool bypass_throttle = false;
bool bypass_rudder = false;

//Quadro send packet
unsigned char data_send[20];
unsigned char checkSum_send = 0;
UWORD packet_header_serial_send = 0xEFFE;
BYTE packet_size_serial_send = 0x10;
WORD ch1_serial_send = 1500;
WORD ch2_serial_send = 1500;
WORD ch3_serial_send = 1200;
WORD ch4_serial_send = 1500;
WORD ch5_serial_send = 1200;
WORD ch6_serial_send = 1850;
WORD cam_roll_serial_send = 0;
WORD cam_pitch_serial_send = 0;

//Quadro receive packet
unsigned char data_recv[29];
unsigned char checkSum_recv = 0;
UWORD packet_header_serial_recv = 0xEFFE;
BYTE packet_size_serial_recv = 0x19;
BYTE mode_recv = 0;
WORD ch1_serial_recv = 1500;
WORD ch2_serial_recv = 1500;
WORD ch3_serial_recv = 1500;
WORD ch4_serial_recv = 1500;
WORD ch5_serial_recv = 0;
WORD ch6_serial_recv = 0;
WORD roll_recv = 0;
WORD pitch_recv = 0;
WORD yaw_recv = 0;
WORD rateX_recv = 0;
WORD rateY_recv = 0;
WORD rateZ_recv = 0;

//IMU receive packet
unsigned char imu_data_recv[100];
unsigned char imu_checkSum_recv = 0;
BYTE packet_header_imu_recv = 0x2A;
int packet_size_imu_recv = 100;
int imu_pitch;
int imu_roll;
int imu_yaw;

//Camera
Mat frame_in, frame_gray, frame_out;




bool openSerialPorts()
{
	//Open Quadro port
	DCB quadroSerialParams = {0};

	quadroSerial = CreateFile(quadroPort, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (quadroSerial == INVALID_HANDLE_VALUE)
	{
		cout << "Quadro Serial open error !!!" << endl;
		return 0;
	}
		
	quadroSerialParams.DCBlength = sizeof(quadroSerialParams);
	if (!GetCommState(quadroSerial, &quadroSerialParams))		
	{
		cout << "Quadro COM check error !!!" << endl;
		return 0;
	}
	quadroSerialParams.BaudRate = CBR_115200;
	quadroSerialParams.ByteSize = 8;
	quadroSerialParams.StopBits = ONESTOPBIT;
	quadroSerialParams.Parity = NOPARITY;
	if(!SetCommState(quadroSerial, &quadroSerialParams))		
	{
		cout << "Quadro COM check error !!!" << endl;
		return 0;
	}

	//Open IMU port
	DCB imuSerialParams = {0};

	imuSerial = CreateFile(imuPort, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (imuSerial == INVALID_HANDLE_VALUE)
	{
		cout << "IMU Serial open error !!!" << endl;
		return 0;
	}
	
	imuSerialParams.DCBlength = sizeof(imuSerialParams);
	
	if (!GetCommState(imuSerial, &imuSerialParams))		
	{
		cout << "IMU COM check error !!!" << endl;
		return 0;
	}
	imuSerialParams.BaudRate = CBR_115200;
	imuSerialParams.ByteSize = 8;
	imuSerialParams.StopBits = ONESTOPBIT;
	imuSerialParams.Parity = NOPARITY;

	

	if(!SetCommState(imuSerial, &imuSerialParams))		
	{
		cout << "IMU COM check error !!!" << endl;
		return 0;
	}

	Sleep(50);
	return 1;
}
bool closeSerialPorts()
{
	if (quadroSerial != 0)
	{
		//CloseHandle(serial);
		cout << "Serial closed" << endl;
		return 1;
	}
	else 
	{
		cout << "Serial close error !!!" << endl;
		return 0;
	}
}
int charToNumber(char c)
{
	return (int)c - 48;
}
void readDataFromPacket(unsigned char buf[256], int& ch1_serial_send, int& ch2_serial_send, int& ch3_serial_send, int& ch4_serial_send)
{
	int ind = 0;
	ch1_serial_send = ch2_serial_send = ch3_serial_send = ch4_serial_send = 0;
	//Separator is Tabulation (8 chars)
	//Extract integer 1
	
	{


		if (charToNumber(buf[ind]) == 9)
		{
			ch1_serial_send += charToNumber(buf[ind]) * 100;
			ch1_serial_send += charToNumber(buf[ind + 1]) * 10;
			ch1_serial_send += charToNumber(buf[ind + 2]);
			ind += 4;
		}
		else
		{
			ch1_serial_send += charToNumber(buf[ind]) * 1000;
			ch1_serial_send += charToNumber(buf[ind + 1]) * 100;
			ch1_serial_send += charToNumber(buf[ind + 2]) * 10;
			ch1_serial_send += charToNumber(buf[ind + 3]);
			ind += 5;
		}

		//Extract integer 2
		if (charToNumber(buf[ind]) == 9)
		{
			ch2_serial_send += charToNumber(buf[ind]) * 100;
			ch2_serial_send += charToNumber(buf[ind + 1]) * 10;
			ch2_serial_send += charToNumber(buf[ind + 2]);
			ind += 4;
		}
		else
		{
			ch2_serial_send += charToNumber(buf[ind]) * 1000;
			ch2_serial_send += charToNumber(buf[ind + 1]) * 100;
			ch2_serial_send += charToNumber(buf[ind + 2]) * 10;
			ch2_serial_send += charToNumber(buf[ind + 3]);
			ind +=5;
		}


		//Extract integer 3
		if (charToNumber(buf[ind]) == 9)
		{
			ch3_serial_send += charToNumber(buf[ind]) * 100;
			ch3_serial_send += charToNumber(buf[ind + 1]) * 10;
			ch3_serial_send += charToNumber(buf[ind + 2]);
			ind += 4;
		}
		else
		{
			ch3_serial_send += charToNumber(buf[ind]) * 1000;
			ch3_serial_send += charToNumber(buf[ind + 1]) * 100;
			ch3_serial_send += charToNumber(buf[ind + 2]) * 10;
			ch3_serial_send += charToNumber(buf[ind + 3]);
			ind += 5;
		}

		//Extract integer 4
		if (charToNumber(buf[ind]) == 9)
		{
			ch4_serial_send += charToNumber(buf[ind]) * 100;
			ch4_serial_send += charToNumber(buf[ind + 1]) * 10;
			ch4_serial_send += charToNumber(buf[ind + 2]);
			ind += 4;
		}
		else
		{
			ch4_serial_send += charToNumber(buf[ind]) * 1000;
			ch4_serial_send += charToNumber(buf[ind + 1]) * 100;
			ch4_serial_send += charToNumber(buf[ind + 2]) * 10;
			ch4_serial_send += charToNumber(buf[ind + 3]);
			ind += 5;
		}
	}
}
void mouseGetColorEvent(int evt, int x, int y, int flags, void* param)
{
	if(evt==CV_EVENT_LBUTTONDOWN)
	{
        Mat hsv;
		cvtColor(frame_in, hsv,CV_BGR2HSV);
		Vec3b pixel = hsv.at<Vec3b>(y, x);
		cout << (int)pixel.val[0] << " " << (int)pixel.val[1] << " " << (int)pixel.val[2] << endl;
    }
}

//Tread functions
void scanKeysThread()
{
	//Scan control keys

	while (!stopSend)
	{
		//Monitoring mode switch
		if (GetAsyncKeyState(VK_TAB))
			monitoring_mode = (monitoring_mode+1)%5;

		//Bypass switch: 1 - Aileron, 2 - Elevator, 3 - Throttle, 4 - Rudder
		if (GetAsyncKeyState(0x31))
			bypass_aileron = !bypass_aileron;
		if (GetAsyncKeyState(0x32))
			bypass_elevator = !bypass_elevator;
		if (GetAsyncKeyState(0x33))
			bypass_throttle = !bypass_throttle;
		if (GetAsyncKeyState(0x34))
			bypass_rudder = !bypass_rudder;

		//Throttle Channel-3
		//if (GetAsyncKeyState(VK_UP) && ch3_serial_send>905)		ch3_serial_send-=10;
		//if (GetAsyncKeyState(VK_DOWN) && ch3_serial_send<2095)	ch3_serial_send+=10; 
		if (GetAsyncKeyState(VK_UP))	ch3_serial_send=1100;
		if (GetAsyncKeyState(VK_DOWN))	ch3_serial_send=1700; 
		//Aileron Channel-1
		//if (GetAsyncKeyState(VK_RIGHT) && ch1_serial_send<2095)	ch1_serial_send+=5;
		//if (GetAsyncKeyState(VK_LEFT) && ch1_serial_send>905)	ch1_serial_send-=5;
		if (GetAsyncKeyState(VK_RIGHT))	ch1_serial_send=1900; 
		if (GetAsyncKeyState(VK_LEFT))	ch1_serial_send=1200; 
		

		//Elevator Channel-2
		if (GetAsyncKeyState(VK_NUMPAD8) && ch2_serial_send<2095)	ch2_serial_send+=5; 
		if (GetAsyncKeyState(VK_NUMPAD5) && ch2_serial_send>905)	ch2_serial_send-=5; 

		//Rudder Channel-4
		if (GetAsyncKeyState(VK_NUMPAD6) && ch4_serial_send<2095)	ch4_serial_send+=5;
		if (GetAsyncKeyState(VK_NUMPAD4) && ch4_serial_send>905)	ch4_serial_send-=5;

		//Exit ESC
		if (GetAsyncKeyState(VK_ESCAPE) && (!stopSend)) 
		{
			stopSend = true;
			closeSerialPorts();
		}

		Sleep(30);
	}
	ExitThread(0);
}
void printDataThread()
{
	while (!stopSend)
	{
		switch (monitoring_mode)
		{
		case 0:
			cout << "PWM: " << ch1_serial_recv << '\t' << ch2_serial_recv << '\t' << ch3_serial_recv << '\t' << ch4_serial_recv << endl;
			break;
		case 1:
			cout << "Mode: " << (int) mode_recv << endl;
			cout << "PWM: " <<  ch1_serial_recv << '\t' << ch2_serial_recv << '\t' << ch3_serial_recv << '\t' << ch4_serial_recv << endl;
			cout << "IMU: " <<roll_recv << '\t' << pitch_recv << '\t' << yaw_recv << '\t' << rateX_recv << '\t' << rateY_recv << '\t' << rateZ_recv << endl;
			break;
		case 2:
			cout << "Send PWM: ";
			if (bypass_aileron == true)
				cout << ch1_serial_recv << '\t';
			else cout << ch1_serial_send << '\t';
			if (bypass_elevator == true)
				cout << ch2_serial_recv << '\t';
			else cout << ch2_serial_send << '\t';
			if (bypass_throttle == true)
				cout << ch3_serial_recv << '\t';
			else cout << ch3_serial_send << '\t';
			if (bypass_rudder == true)
				cout << ch4_serial_recv << '\t';
			else cout << ch4_serial_send << endl;
			break;
		case 3:
			cout << "PC-PWM: " << ch1_serial_send << '\t' << ch2_serial_send << '\t' << ch3_serial_send << '\t' << ch4_serial_send << endl;
			break;
		case 4:
			cout << "BYPASS      Aileron: " << bypass_aileron << '\t' << "Elevator: " << bypass_elevator << '\t' 
					<< "Throttle: " << bypass_throttle << '\t' << "Rudder: " << bypass_rudder << '\t' << endl;
			break;
		}
		Sleep(1000);
	}
	ExitThread(0);
}
void quadroSendSerialThread()
{
	while (!stopSend)
	{
		//Header
		data_send[0] = (unsigned char)((packet_header_serial_send & 0xFF00) >> 8);
		data_send[1] = (unsigned char)(packet_header_serial_send & 0x00FF);
		//Packet size
		data_send[2] = (unsigned char)(packet_size_serial_send);
		//PWM Channels 1-6
		if (bypass_aileron == true) 
		{
			data_send[4] = (unsigned char)((ch1_serial_recv & 0xFF00) >> 8); 
			data_send[3] = (unsigned char)(ch1_serial_recv & 0x00FF);
		}
		else
		{
			data_send[4] = (unsigned char)((ch1_serial_send & 0xFF00) >> 8); 
			data_send[3] = (unsigned char)(ch1_serial_send & 0x00FF);
		}
		if (bypass_elevator == true)
		{
			data_send[6] = (unsigned char)((ch2_serial_recv & 0xFF00) >> 8); 
			data_send[5] = (unsigned char)(ch2_serial_recv & 0x00FF);
		}
		else
		{
			data_send[6] = (unsigned char)((ch2_serial_send & 0xFF00) >> 8); 
			data_send[5] = (unsigned char)(ch2_serial_send & 0x00FF);
		}
		if (bypass_throttle == true)
		{
			data_send[8] = (unsigned char)((ch3_serial_recv & 0xFF00) >> 8); 
			data_send[7] = (unsigned char)(ch3_serial_recv & 0x00FF);
		}
		else
		{
			data_send[8] = (unsigned char)((ch3_serial_send & 0xFF00) >> 8); 
			data_send[7] = (unsigned char)(ch3_serial_send & 0x00FF);
		}
		if (bypass_rudder == true)
		{
			data_send[10] = (unsigned char)((ch4_serial_recv & 0xFF00) >> 8); 
			data_send[9] = (unsigned char)(ch4_serial_recv & 0x00FF);
		}
		else
		{
			data_send[10] = (unsigned char)((ch4_serial_send & 0xFF00) >> 8); 
			data_send[9] = (unsigned char)(ch4_serial_send & 0x00FF);
		}
		data_send[12] =  (unsigned char)((ch5_serial_send & 0xFF00) >> 8); 
		data_send[11] = (unsigned char)(ch5_serial_send & 0x00FF);
		data_send[14] =  (unsigned char)((ch6_serial_send & 0xFF00) >> 8); 
		data_send[13] = (unsigned char)(ch6_serial_send & 0x00FF);
		//PWM for Camera Pitch/Roll
		data_send[15] =  (unsigned char)((cam_pitch_serial_send & 0xFF00) >> 8); 
		data_send[16] = (unsigned char)(cam_pitch_serial_send & 0x00FF);
		data_send[17] =  (unsigned char)((cam_roll_serial_send & 0xFF00) >> 8); 
		data_send[18] = (unsigned char)(cam_roll_serial_send & 0x00FF);
		//Check Sum
		checkSum_send = 0;
		for (int i=2; i<19; i++)
			checkSum_send += data_send[i];
		checkSum_send = 0 - checkSum_send; 
		data_send[19] = checkSum_send;

		DWORD dwBytesToWrite = sizeof(data_send);
		
		DWORD dwBytesWritten;
		bool statusFlg = WriteFile (quadroSerial, data_send, dwBytesToWrite, &dwBytesWritten, NULL);
		if (statusFlg == false)
			cout << "Error: Failed serial send" << endl;
		else if (dwBytesWritten != dwBytesToWrite)
            cout << ("Error: Not all data has been sent") << endl;
				
		//Wait
		Sleep(20);
	}
	
}
void quadroRecvSerialThread()
{
	DWORD dwBytesToRead;
	DWORD dwBytesRead;

	while(!stopSend)
	{
		//Read 1 byte - First byte of header
		dwBytesToRead = 1;
		bool statusFlg = ReadFile (quadroSerial, &data_recv[0], dwBytesToRead, &dwBytesRead,NULL);  
		if (statusFlg == false)
			cout << "Error Quadro: Failed serial read" << endl;
		else if (dwBytesRead != dwBytesToRead)
            cout << ("Error Quadro: Not all data has been read ") << endl;

		if (data_recv[0] == (unsigned char)((packet_header_serial_recv & 0xFF00) >> 8))
		{
			//Read 1 byte - Second byte of header
			statusFlg = ReadFile (quadroSerial, &data_recv[1], dwBytesToRead, &dwBytesRead,NULL);  
			if (statusFlg == false)
				cout << "Error Quadro: Failed serial read" << endl;
			else if (dwBytesRead != dwBytesToRead)
				cout << ("Error Quadro: Not all data has been read") << endl;
			if (data_recv[1] == (unsigned char)(packet_header_serial_recv & 0x00FF))
			{
				//Read 1 byte - Packet size
				statusFlg = ReadFile (quadroSerial, &data_recv[2], dwBytesToRead, &dwBytesRead,NULL);  
				if (statusFlg == false)
					cout << "Error Quadro: Failed serial read" << endl;
				else if (dwBytesRead != dwBytesToRead)
					cout << ("Error Quadro: Not all data has been read") << endl;
				if (data_recv[2] != (unsigned char)(packet_size_serial_recv))
				{
					cout << "Error Quadro: Wrong packet size with 0XEFFE header" << endl;
					continue;
				}

				//Read 26 bytes - Packet data + Checksum
				dwBytesToRead = packet_size_serial_recv+1;
				statusFlg = ReadFile (quadroSerial, &data_recv[3], dwBytesToRead, &dwBytesRead,NULL);  
				if (statusFlg == false)
					cout << "Error Quadro:  Failed serial read" << endl;
				else if (dwBytesRead != dwBytesToRead)
					cout << ("Error Quadro: Not all data has been read") << endl;

				//Check Sum
				BYTE checkSum = 0;
				for (int i=2; i<28; i++)
					checkSum += data_recv[i];
				checkSum = 0 - checkSum; 
				checkSum_recv = data_recv[28];
				if (checkSum == checkSum_recv) 
				{
					//Process data
					mode_recv = data_recv[3];
					roll_recv = (data_recv[5] << 8)+data_recv[4];
					pitch_recv = (data_recv[7] << 8)+data_recv[6];
					yaw_recv = (data_recv[9] << 8)+data_recv[8];
					rateX_recv = (data_recv[11] << 8)+data_recv[10];
					rateY_recv = (data_recv[13] << 8)+data_recv[12];
					rateZ_recv = (data_recv[15] << 8)+data_recv[14];
					ch1_serial_recv = (data_recv[17] << 8)+data_recv[16];
					ch2_serial_recv = (data_recv[19] << 8)+data_recv[18];
					ch3_serial_recv = (data_recv[21] << 8)+data_recv[20];
					ch4_serial_recv = (data_recv[23] << 8)+data_recv[22];
					ch5_serial_recv = (data_recv[25] << 8)+data_recv[24];
					ch6_serial_recv = (data_recv[27] << 8)+data_recv[26];
				}
				else cout << "Error Quadro: Received packet with wrong checksum" << endl;
				

			}



		}
        
	}
}
void imuRecvSerialThread()
{
	DWORD dwBytesToRead;
	DWORD dwBytesRead;

	while(!stopSend)
	{
		//Read 1 byte - Header
		dwBytesToRead = 1;
		bool statusFlg = ReadFile (imuSerial, &imu_data_recv[0], dwBytesToRead, &dwBytesRead,NULL);  
		if (statusFlg == false)
			cout << "Error IMU: Failed serial read" << endl;
		else if (dwBytesRead != dwBytesToRead)
            cout << ("Error IMU: Not all data has been read ") << endl;

		if (imu_data_recv[0] == (unsigned char)(packet_header_imu_recv))
		{
			
		}
			
	}
}
void targetHoveringThread() 
{
	Mat	frame_HSV, frame_binary;
	double fps;
	bool auto_focus = false, flg;
	int id = -1;
	char pressed_key;

	vector <vector <Point>> candidates;
	Point2f target_center(-1,-1);
	Moments target_moments;

	//Camera Initialization
	VideoCapture camera_capture("video.avi");
	if (!camera_capture.isOpened())
		{
		cout << "Camera Open Error" << endl;
		cin >> id;
		return;
		}

	//Read from
	camera_capture.set(CV_CAP_PROP_POS_MSEC, 200000);

	fps = camera_capture.get(CV_CAP_PROP_FPS);
	cout << "Camera Initialized...\n" << "Resolution:  " << camera_capture.get(CV_CAP_PROP_FRAME_WIDTH) << "x" << camera_capture.get(CV_CAP_PROP_FRAME_HEIGHT) << "\nFPS:  " << fps << endl;
	cvNamedWindow("Original");
	cvSetMouseCallback("Original", mouseGetColorEvent, 0);

	//Main Loop
	while(1)
	{
		
		camera_capture.read(frame_in);
		frame_in.copyTo(frame_out);
		

		//Delete Noise
		Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size(10,10),
                                       Point(5,5));
		
		
		//Red Object detection
		id = -1;
		//Threshholding
		cvtColor(frame_out, frame_HSV, CV_BGR2HSV);
		inRange (frame_HSV, Scalar(0, 150, 70), Scalar(30,250,110), frame_binary);

		//erode(frame_binary,frame_binary, element);
		//dilate(frame_binary,frame_binary, element);
		
		imshow("Binary1", frame_binary);

		findContours(frame_binary, candidates, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		flg = false;
		for (int i=0; i<candidates.size(); i++)
		{
			if ((id == -1 && contourArea(candidates[i]) > 300) || 
				(id != -1 && contourArea(candidates[i]) >= contourArea(candidates[id])))
			{				
				id = i;
				flg = true;
			}
			if (flg) break;
		}	
		
		if (id != -1)
		{
			target_moments = moments(candidates[id]);
			target_center.x = target_moments.m10/target_moments.m00;
			target_center.y = target_moments.m01/target_moments.m00;	
			line(frame_out, Point(target_center.x+7, target_center.y), Point(target_center.x-7, target_center.y),Scalar(0,0,150), 3);
			line(frame_out, Point(target_center.x, target_center.y+7), Point(target_center.x, target_center.y-7),Scalar(0,0,150), 3);
			drawContours(frame_out, candidates, id, Scalar(0,100,0), 2);
			//cout << "Area: " << contourArea(candidates[id]) << "   X: " << target_center.x << "   Y: "  << target_center.y << endl;
		}
		

		//cout << candidates.size() << "   " << id << endl;

		//Blue Object detection
		id = -1;
		//Threshholding
		cvtColor(frame_out, frame_HSV, CV_BGR2HSV);
		inRange (frame_HSV, Scalar(80, 10, 0), Scalar(120,40,255), frame_binary);

		//erode(frame_binary,frame_binary, element);
		//dilate(frame_binary,frame_binary, element);
		imshow("Binary2", frame_binary);

		findContours(frame_binary, candidates, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		flg = false;
		for (int i=0; i<candidates.size(); i++)
		{
			if ((id == -1 && contourArea(candidates[i]) > 300) || 
				(id != -1 && contourArea(candidates[i]) >= contourArea(candidates[id])))
			{				
				id = i;
				flg = true;
			}
			if (flg) break;
		}	
		
		if (id != -1)
		{
			target_moments = moments(candidates[id]);
			target_center.x = target_moments.m10/target_moments.m00;
			target_center.y = target_moments.m01/target_moments.m00;	
			line(frame_out, Point(target_center.x+7, target_center.y), Point(target_center.x-7, target_center.y),Scalar(0,0,150), 3);
			line(frame_out, Point(target_center.x, target_center.y+7), Point(target_center.x, target_center.y-7),Scalar(0,0,150), 3);
			drawContours(frame_out, candidates, id, Scalar(0,100,0), 2);
			//cout << "Area: " << contourArea(candidates[id]) << "   X: " << target_center.x << "   Y: "  << target_center.y << endl;
		}
		

		//cout << candidates.size() << "   " << id << endl;
		
		imshow("Original", frame_in);
		imshow("Out", frame_out);
		
		
		pressed_key = waitKey(20);
		//if (pressed_key == 25) break;
	}
}

void serverUDPThread()
{
	WSADATA WsaData;
	WSAStartup( MAKEWORD(2,2), &WsaData);
	int servSocket = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
	if ( servSocket <= 0 )
    {
        printf( "Failed to create socket\n" );
        return;
    }
	sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( 5600 );
	::bind( servSocket, (const sockaddr*) &address, sizeof(sockaddr_in) );
	
	while (!stopSend)
    {
        unsigned char packet_data[256];
        unsigned int maximum_packet_size = sizeof( packet_data );
        typedef int socklen_t;

        sockaddr_in from;
        socklen_t fromLength = sizeof( from );

        int received_bytes = recvfrom( servSocket, (char*)packet_data, maximum_packet_size,
                                   0, (sockaddr*)&from, &fromLength );
		if ( received_bytes <= 0 )
            break;

        unsigned int from_address = ntohl( from.sin_addr.s_addr );
        unsigned int from_port = ntohs( from.sin_port );

        // Process received packet
		int ch1_serial_send_serial_recv = 0, ch2_serial_send_serial_recv = 0, ch3_serial_send_serial_recv = 0, ch4_serial_send_serial_recv = 0;
		readDataFromPacket(packet_data, ch1_serial_send_serial_recv, ch2_serial_send_serial_recv, ch3_serial_send_serial_recv, ch4_serial_send_serial_recv);
			cout << "Received:  " << ch1_serial_send_serial_recv  << " " << ch2_serial_send_serial_recv  << " " << ch3_serial_send_serial_recv  << " " << ch4_serial_send_serial_recv << endl;
		ch1_serial_send = ch1_serial_send_serial_recv;
		ch2_serial_send = ch2_serial_send_serial_recv;
		ch3_serial_send = ch3_serial_send_serial_recv;
		ch4_serial_send = ch4_serial_send_serial_recv;
		
      
    }
	ExitThread(0);
}


void main()
{
	if (openSerialPorts())
		{
		thread scanKeys(scanKeysThread);
		thread printData(printDataThread);
		thread imuRecvSerial(imuRecvSerialThread);
		thread quadroSendSerial(quadroSendSerialThread);
		thread quadroRecvSerial(quadroRecvSerialThread);
		//thread serverUDP(serverUDPThread);
		//thread targetHovering(targetHoveringThread);

		scanKeys.join();
		printData.join();
		imuRecvSerial.join();
		quadroSendSerial.join();
		quadroRecvSerial.join();
		//serverUDP.join();
		//targetHovering.join();
		}
	else getchar();
	
}