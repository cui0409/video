#include "stdafx.h"
#include "opencv\opencv\build\include\opencv2\opencv.hpp"
#include "opencv\opencv\build\include\opencv2/video/video.hpp"
#include "opencv\opencv\build\include\opencv2\core\core.hpp"
#include "opencv\opencv\build\include\opencv2\highgui\highgui.hpp"
#include "opencv\opencv\build\include\opencv2\imgproc\imgproc.hpp"
#include "opencv\opencv\build\include\opencv2/imgproc\imgproc_c.h"
#include "opencv\opencv\build\include\opencv2\ml.hpp"

#include <windows.h>
#include <iostream>

#pragma comment(lib, "winmm.lib") 

#include <string>

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>

#include<thread>

//����ʾ����̨
//#pragma comment( linker, "/subsystem:windows /entry:mainCRTStartup" )

using namespace cv;
using namespace std;

HWAVEIN hWaveIn;  //�����豸
WAVEFORMATEX waveform; //�ɼ���Ƶ�ĸ�ʽ���ṹ��
BYTE *pBuffer1;//�ɼ���Ƶʱ�����ݻ���
WAVEHDR wHdr1; //�ɼ���Ƶʱ�������ݻ���Ľṹ��
FILE *pf;

double getPSNR(const Mat& I1, const Mat& I2);
Scalar getMSSIM(const Mat& I1, const Mat& I2);

double getPSNR(const Mat& I1, const Mat& I2)
{
	Mat s1;
	absdiff(I1, I2, s1);
	s1.convertTo(s1, CV_32F);
	s1 = s1.mul(s1);
	Scalar s = sum(s1);
	double sse = s.val[0] + s.val[1] + s.val[2];
	if (sse <= 1e-10)
		return 0;
	else
	{
		double mse = sse / (double)(I1.channels() * I1.total());
		double psnr = 10.0 * log10((255 * 255) / mse);
		return psnr;
	}
}
Scalar getMSSIM(const Mat& i1, const Mat& i2)
{
	const double C1 = 6.5025, C2 = 58.5225;

	int d = CV_32F;
	Mat I1, I2;
	i1.convertTo(I1, d);
	i2.convertTo(I2, d);
	Mat I2_2 = I2.mul(I2);
	Mat I1_2 = I1.mul(I1);
	Mat I1_I2 = I1.mul(I2);

	Mat mu1, mu2;
	GaussianBlur(I1, mu1, Size(11, 11), 1.5);
	GaussianBlur(I2, mu2, Size(11, 11), 1.5);
	Mat mu1_2 = mu1.mul(mu1);
	Mat mu2_2 = mu2.mul(mu2);
	Mat mu1_mu2 = mu1.mul(mu2);
	Mat sigma1_2, sigma2_2, sigma12;
	GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
	sigma1_2 -= mu1_2;
	GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
	sigma2_2 -= mu2_2;
	GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
	sigma12 -= mu1_mu2;
	Mat t1, t2, t3;
	t1 = 2 * mu1_mu2 + C1;
	t2 = 2 * sigma12 + C2;
	t3 = t1.mul(t2);
	t1 = mu1_2 + mu2_2 + C1;
	t2 = sigma1_2 + sigma2_2 + C2;
	t1 = t1.mul(t2);
	Mat ssim_map;
	divide(t3, t1, ssim_map);
	Scalar mssim = mean(ssim_map);
	return mssim;
}

string get_time()
{
	SYSTEMTIME  st, lt;
	//GetSystemTime(&lt);
	GetLocalTime(&lt);

	char szResult[30] = "\0";

	sprintf_s(szResult, 30, "%d-%d-%d-%d-%d-%d-%d", lt.wYear, lt.wMonth, lt.wDay, lt.wHour, lt.wMinute, lt.wSecond, lt.wMilliseconds);

	return szResult;
}


//ģ����⣬���ԭͼ����ģ��ͼ�񣬷���0�����򷵻�1
//10������Խ�������һ��Ϊ5
int VideoBlurDetect(const cv::Mat &srcimg)
{
	cv::Mat img;
	cv::cvtColor(srcimg, img, CV_BGR2GRAY); // �������ͼƬתΪ�Ҷ�ͼ��ʹ�ûҶ�ͼ���ģ����

											//ͼƬÿ���ֽ�������  
	int width = img.cols;
	int height = img.rows;
	ushort* sobelTable = new ushort[width*height];
	memset(sobelTable, 0, width*height * sizeof(ushort));

	int i, j, mul;
	//ָ��ͼ���׵�ַ  
	uchar* udata = img.data;
	for (i = 1, mul = i*width; i < height - 1; i++, mul += width)
		for (j = 1; j < width - 1; j++)

			sobelTable[mul + j] = abs(udata[mul + j - width - 1] + 2 * udata[mul + j - 1] + udata[mul + j - 1 + width] - \
				udata[mul + j + 1 - width] - 2 * udata[mul + j + 1] - udata[mul + j + width + 1]);

	for (i = 1, mul = i*width; i < height - 1; i++, mul += width)
		for (j = 1; j < width - 1; j++)
			if (sobelTable[mul + j] < 50 || sobelTable[mul + j] <= sobelTable[mul + j - 1] || \
				sobelTable[mul + j] <= sobelTable[mul + j + 1]) sobelTable[mul + j] = 0;

	int totLen = 0;
	int totCount = 1;

	uchar suddenThre = 50;
	uchar sameThre = 3;
	//����ͼƬ  
	for (i = 1, mul = i*width; i < height - 1; i++, mul += width)
	{
		for (j = 1; j < width - 1; j++)
		{
			if (sobelTable[mul + j])
			{
				int   count = 0;
				uchar tmpThre = 5;
				uchar max = udata[mul + j] > udata[mul + j - 1] ? 0 : 1;

				for (int t = j; t > 0; t--)
				{
					count++;
					if (abs(udata[mul + t] - udata[mul + t - 1]) > suddenThre)
						break;

					if (max && udata[mul + t] > udata[mul + t - 1])
						break;

					if (!max && udata[mul + t] < udata[mul + t - 1])
						break;

					int tmp = 0;
					for (int s = t; s > 0; s--)
					{
						if (abs(udata[mul + t] - udata[mul + s]) < sameThre)
						{
							tmp++;
							if (tmp > tmpThre) break;
						}
						else break;
					}

					if (tmp > tmpThre) break;
				}

				max = udata[mul + j] > udata[mul + j + 1] ? 0 : 1;

				for (int t = j; t < width; t++)
				{
					count++;
					if (abs(udata[mul + t] - udata[mul + t + 1]) > suddenThre)
						break;

					if (max && udata[mul + t] > udata[mul + t + 1])
						break;

					if (!max && udata[mul + t] < udata[mul + t + 1])
						break;

					int tmp = 0;
					for (int s = t; s < width; s++)
					{
						if (abs(udata[mul + t] - udata[mul + s]) < sameThre)
						{
							tmp++;
							if (tmp > tmpThre) break;
						}
						else break;
					}

					if (tmp > tmpThre) break;
				}
				count--;

				totCount++;
				totLen += count;
			}
		}
	}
	//ģ����
	float result = (float)totLen / totCount;
	delete[] sobelTable;
	sobelTable = NULL;

	return result;
}


//ģ����⣬���ԭͼ����ģ��ͼ�񣬷���true�����򷵻�false 
bool blurDetect(Mat srcImage)
{
	Mat gray1;

	if (srcImage.channels() != 1)
	{
		cvtColor(srcImage, gray1, CV_RGB2GRAY);
	}
	else
	{
		gray1 = srcImage.clone();
	}
	Mat tmp_m1, tmp_sd1;    //�����洢��ֵ�ͷ���  
	double m1 = 0, sd1 = 0;

	m1 = mean(gray1)[0];
	//ʹ��3x3��Laplacian���Ӿ����˲�  
	Mat dst, abs_dst;
	Laplacian(gray1, dst, CV_16S, 3);
	//�鵽0~255  
	convertScaleAbs(dst, abs_dst);
	//�����ֵ�ͷ���  
	meanStdDev(abs_dst, tmp_m1, tmp_sd1);//
	m1 = tmp_m1.at<double>(0, 0);     //��ֵ  //��Ӧ��ͼ�������
	sd1 = tmp_sd1.at<double>(0, 0);       //��׼��   //��Ӧ������ֵ���ֵ����ɢ�̶�

	double mul = sd1 * sd1;
	if (mul < 4960 || mul > 5100)
	{
		return true;
	}
	else
	{
		return false;
	}

}

//����ƽ���ݶ�
double cal_mean_gradient(Mat src)
{
	Mat img;
	cvtColor(src, img, CV_RGB2GRAY); // ת��Ϊ�Ҷ�ͼ
	img.convertTo(img, CV_64FC1);
	double tmp = 0;
	int rows = img.rows - 1;
	int cols = img.cols - 1;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			double dx = img.at<double>(i, j + 1) - img.at<double>(i, j);
			double dy = img.at<double>(i + 1, j) - img.at<double>(i, j);
			double ds = std::sqrt((dx*dx + dy*dy) / 2);
			tmp += ds;
		}
	}
	double imageAvG = tmp / (rows*cols);
	return imageAvG;
}

void  video_test()
{
	char video_name[100];

	string time;
	time = get_time();
	sprintf_s(video_name, "%s%s%s", "F:\\video_audio\\video", time.c_str(), ".mpeg");

	VideoCapture capture(1);
	//if (!capture.isOpened())
	//{
	//	cout << "open video error";
	//	MessageBox(NULL, TEXT("�ɼ���Ƶ����"), TEXT("���"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
	//	return -1;
	//}

	double rate = capture.get(CAP_PROP_FPS);//��ȡ��Ƶ֡��

	int width = capture.get(CAP_PROP_FRAME_WIDTH);
	int height = capture.get(CAP_PROP_FRAME_HEIGHT);
	Size videoSize(width, height);

	VideoWriter writer;
	writer.open(video_name, CAP_OPENCV_MJPEG, rate, videoSize);

	//���������ʼ֡
	long frameToStart = 0;
	capture.set(CAP_PROP_POS_FRAMES, frameToStart);

	//�����������֡  //�ɼ�5��
	int frameToStop = 300;

	Mat frame;
	//ָ��ÿ�ζ���ȡ�̶�֡������Ƶ
	while (frameToStart < frameToStop)
	{
		capture >> frame;
		writer << frame;

		//imshow("video", frame);
		//namedWindow("video", WINDOW_AUTOSIZE);

		frameToStart++;
		//waitKey(1);
	}
	writer.release();
	//.... �����ǲɼ���Ƶ

	cout << "wait video test..." << endl;
	//system("pause");


	stringstream conv;
	//const string src_video = "F:\\testvideo\\src_video.mpeg";

	//const string test_video = "F:\\testvideo\\testvideo2019-7-11-13-55-59-775.mpeg";
	//const string test_video = video_name;
	const string test_video = "F:\\zhengchang.mpeg";
	//const string test_video = "F:\\kadun.mpeg";
	//const string test_video = "F:\\heiping.mpeg";
	//const string test_video = "F:\\huaping.mpeg";

	int psnrTriggerValue, delay = 30;
	conv >> psnrTriggerValue >> delay;

	VideoCapture /*capture_src(src_video),*/ capture_test(test_video);
	//if (!capture_src.isOpened())
	//{
	//	cout << "can not open src video " << src_video << endl;
	//	return -1;
	//}
	if (!capture_test.isOpened())
	{
		cout << "can not open test video " << test_video << endl;
		return ;
	}

	//double rate_src = capture_src.get(CAP_PROP_FPS);
	double rate_test = capture_test.get(CAP_PROP_FPS);

	//int num_src = capture_src.get(CAP_PROP_FRAME_COUNT);//src��֡��
	int num_test = capture_test.get(CAP_PROP_FRAME_COUNT);//test��֡��


	//��������Ƶ�������������ʱ�䣬<3s
	int i = 0;
	Mat frame_show;
	while (i < num_test)
	{
		capture_test >> frame_show;

		imshow("video", frame_show);
		namedWindow("video", WINDOW_AUTOSIZE);

		i++;
		waitKey(1);
	}

	//Size refS = Size((int)capture_src.get(CAP_PROP_FRAME_WIDTH), (int)capture_src.get(CAP_PROP_FRAME_HEIGHT));
	Size uTSi = Size((int)capture_test.get(CAP_PROP_FRAME_WIDTH), (int)capture_test.get(CAP_PROP_FRAME_HEIGHT));

	//if (refS != uTSi)
	//{
	//	cout << "Inputs have different size!!! Closing." << endl;
	//	return -1;
	//}
	const char* win_test = "Test video";
	const char* win_src = "Src video";
	// Windows
	//namedWindow(win_src, WINDOW_AUTOSIZE);
	//namedWindow(win_test, WINDOW_AUTOSIZE);
	//moveWindow(win_src, 400, 0);
	//moveWindow(win_test, refS.width, 0);

	Mat frame_src, frame_test;
	double psnrV;
	vector<double> vec_psnrv;

	Mat mat_test, mat_src;



	int index_src = 0; //src��ǰ֡
	int totalNum = 0;//ͳ�Ʋ������ĸ���
	double psnrv_mat;
	double psnrv_mat2;

	//for (size_t i = 0; i < num_src / rate_src / 6; ++i)                      
	//{
	//	capture_src.set(CAP_PROP_POS_FRAMES, i * rate_src);//0��60��120��...
	//	capture_src >> mat_src;

	//	for (size_t j = 0; j < num_test / rate_test; ++j)                
	//	{
	//		capture_test.set(CAP_PROP_POS_FRAMES, j * rate_test);//0��60��120��...
	//		capture_test >> mat_test;


	//		psnrv_mat = getPSNR(mat_src, mat_test);//ֵԽ��ʧ��ԽС��20��40��
	//		vec_psnrv.push_back(psnrv_mat);

	//		//Scalar sc = getMSSIM(mat_src, mat_test);//ֵԽ��ʧ��ԽС����0��1��

	//		//if(sc[0] > 0.7 && sc[1] > 0.7 && sc[2] > 0.7)//����Ƚ�
	//		//	totalNum++;
	//	}

	//}
	////����Ԫ�أ�����Ԫ�ز�ֵ����ֵ����10��Ϊ������
	//for (size_t k = 0; k < vec_psnrv.size() - 1 ; ++k)
	//{
	//	if(abs(vec_psnrv[k + 1] - vec_psnrv[k]) > 10 || vec_psnrv[k] < 10)
	//		totalNum++;
	//}

	//���ȡtest��Ƶ3֡,ȡ��test��ǰ��֡
	Mat first_test;
	capture_test.set(CAP_PROP_POS_FRAMES, rate_test);
	capture_test >> first_test;

	Mat second_test;
	capture_test.set(CAP_PROP_POS_FRAMES, rate_test * 2);
	capture_test >> second_test;

	Mat third_test;
	capture_test.set(CAP_PROP_POS_FRAMES, rate_test * 3);
	capture_test >> third_test;


	//���ȼ�  1.����ʾ������ ����ȫ������ֵ��Ϊ0��Mat�������ֵΪ0��
	double minv1 = 0.0, maxv1 = 0.0;
	double* minp1 = &minv1;
	double* maxp1 = &maxv1;
	minMaxIdx(first_test, minp1, maxp1);

	double minv2 = 0.0, maxv2 = 0.0;
	double* minp2 = &minv2;
	double* maxp2 = &maxv2;
	minMaxIdx(second_test, minp2, maxp2);

	double minv3 = 0.0, maxv3 = 0.0;
	double* minp3 = &minv3;
	double* maxp3 = &maxv3;
	minMaxIdx(third_test, minp3, maxp3);

	if (maxv1 == 0.0 /*&& maxv2 == 0.0*/)
	{
		MessageBox(NULL, TEXT("������������"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
		return ;
	}

	//2.���٣�1.һֱ����һ�����棬���⼸֡����С��ע����Խ��滹�����ڱ仯 2.�ü��뿨����ѭ����

	psnrv_mat = getPSNR(first_test, third_test);//����һ�����棬��ǰ�����2֮֡�����ƶȺܸ�
	if (psnrv_mat > 25 && psnrv_mat < 60 || psnrv_mat == 0)
	{
		MessageBox(NULL, TEXT("���٣�������"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
		return ;
	}
	//else if(psnrv_mat2 > 25 && psnrv_mat2 < 40 || psnrv_mat2 == 0)
	//	MessageBox(NULL, TEXT("���٣�������"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);


	//�������ؼ�ĻҶ��������ݶȲ�
	//Mat imageGrey;
	//cvtColor(first_test, imageGrey, CV_RGB2GRAY);
	//Mat imageSobel;
	//Sobel(imageGrey, imageSobel, CV_16U, 1, 1);//��ƽ���Ҷ�ֵ
	//double meanValue = 0.0;
	//meanValue = mean(imageSobel)[0];


	//����ƽ���ݶ�
	double average_gradient = cal_mean_gradient(first_test);
	double average_gradient2 = cal_mean_gradient(second_test);
	double average_gradient3 = cal_mean_gradient(third_test);
	if ((average_gradient > 7.2) && (average_gradient2 > 7.2) && (average_gradient3 > 7.2))
	{
		MessageBox(NULL, TEXT("ģ����������"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
		return ;
	}

	//3.ѩ�����߻���(˺�ѻ��λ)         
	//����ͼ�����صĻҶ�ֵ�仯һ�㶼ƽ���������С����ѩ���ġ���˸�㡱���ػҶ�ֵ���ұ仯��
	//�Ҷ�ֵ��Ծ�Դ󣬼��㷽��Ҳƫ�󡣼��ѩ����˼·��С���ڷ����

	// �������3֡  ��Ƶģ��
	//bool mohu = blurDetect(first_test);
	//bool mohu2 = blurDetect(second_test);
	//bool mohu3 = blurDetect(third_test);

	//if (mohu && mohu2 && mohu3)
	////if((mohu && mohu2) || (mohu && mohu3) || (mohu2 && mohu3))
	//{
	//	MessageBox(NULL, TEXT("ģ����������"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
	//	return -1;
	//}

	//����Ϊ����
	MessageBox(NULL, TEXT("����"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
}

int getPcmDB(char *pcmdata, size_t size) {

	int db = 0;
	short int value = 0;
	double sum = 0;
	for (int i = 0; i < size; i += 2)
	{
		memcpy(&value, pcmdata + i, 2); //��ȡ2���ֽڵĴ�С��ֵ��  
		sum += abs(value); //����ֵ���  
	}
	sum = sum / (size / 2); //��ƽ��ֵ��2���ֽڱ�ʾһ������������������Ϊ��size/2����  
	if (sum > 0)
	{
		db = (int)(20.0*log10(sum));
	}
	return db;
}

void audio_test()
{
	char audio_name[100];
	string time;
	time = get_time();
	sprintf_s(audio_name, "%s%s%s", "F:\\video_audio\\audio", time.c_str(), ".pcm");


	HANDLE     wait;
	waveform.wFormatTag = WAVE_FORMAT_PCM;//������ʽΪPCM
	waveform.nSamplesPerSec = 8000;//�����ʣ�16000��/��
	waveform.wBitsPerSample = 16;//�������أ�16bits/��
	waveform.nChannels = 1;//������������2���� // HDMIֻ����1
	waveform.nAvgBytesPerSec = 16000;//ÿ��������ʣ�����ÿ���ܲɼ������ֽڵ�����
	waveform.nBlockAlign = 2;//һ����Ĵ�С������bit���ֽ�������������
	waveform.cbSize = 0;//һ��Ϊ0

	wait = CreateEvent(NULL, 0, 0, NULL);

	//���������豸����Ŀ
	int n = waveInGetNumDevs();

	//ʹ��waveInOpen����������Ƶ�ɼ���	��WAVE_MAPPERΪ-1��ϵͳ���Զ�ѡ��һ����Ҫ����豸
	//-1ΪĬ�ϣ�0Ϊ�Դ���˷磬1Ϊ�ɼ�����˷�	//ע�⣺�������豸�ž͵ý����Դ�������˷磬ʹ�òɼ�����˷磺Live Streaming Audio Device
	waveInOpen(&hWaveIn, 1, &waveform, (DWORD_PTR)wait, 0L, CALLBACK_EVENT);

	//HDMI��������AV˫����
	//�����������飨������Խ���������飩����������Ƶ����
	DWORD bufsize = 1024 * 100;//ÿ�ο���10k�Ļ���洢¼������
	int i = 5;
	fopen_s(&pf, audio_name, "wb");
	while (i--)//¼��5�����������������Ƶ��������紫������޸�Ϊʵʱ¼�����ŵĻ�����ʵ�ֶԽ�����
	{
		pBuffer1 = new BYTE[bufsize];
		wHdr1.lpData = (LPSTR)pBuffer1;
		wHdr1.dwBufferLength = bufsize;
		wHdr1.dwBytesRecorded = 0;
		wHdr1.dwUser = 0;
		wHdr1.dwFlags = 0;
		wHdr1.dwLoops = 1;
		waveInPrepareHeader(hWaveIn, &wHdr1, sizeof(WAVEHDR));//׼��һ���������ݿ�ͷ����¼��
		waveInAddBuffer(hWaveIn, &wHdr1, sizeof(WAVEHDR));//ָ���������ݿ�Ϊ¼�����뻺��
		waveInStart(hWaveIn);//��ʼ¼��
		Sleep(1000);//�ȴ�����¼��1s
		waveInReset(hWaveIn);//ֹͣ¼��
		fwrite(pBuffer1, 1, wHdr1.dwBytesRecorded, pf);
		delete pBuffer1;
		printf("%ds  ", i);
	}


	fclose(pf);

	waveInClose(hWaveIn);


	//�ȼ���Ƿ�����Ƶ
	ifstream inFile(audio_name, ios::in | ios::binary | ios::ate);
	long nFileSizeInBytes = inFile.tellg();

	if (nFileSizeInBytes == 0)
	{
		MessageBox(NULL, TEXT("δ��⵽��Ƶ"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
		return;
	}


	//��Ƶ����
	//��ȡ�������֮ƽ��ֵ ����db(������ֵ 2 ^ 16 - 1 = 65535 ���ֵ�� 96.32db)
	char* pcmdata = audio_name;
	size_t size = sizeof(WAVEHDR);
	int db = getPcmDB(audio_name, size);
	if (db > 0 && db < 86)
		MessageBox(NULL, TEXT("����"), TEXT("��Ƶ�����"), MB_DEFBUTTON1 | MB_DEFBUTTON2);
}

int main()
{
	//��Ƶ���
	thread video(video_test);
	//��Ƶ���
	thread audio(audio_test);

	video.join();
	audio.join();

	return 0;
}