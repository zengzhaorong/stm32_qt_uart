#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include "mainwindow.h"
#include "uart_comm.h"
#include "config.h"

/* C++ include C */
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif


#define DEV_TTY_USB		"/dev/ttyUSB"

static MainWindow *mainwindow;


/* QT: main window layout - �����沼�� */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
	QTextCodec *codec = QTextCodec::codecForName("GBK");
	char dev_name[32] = {};
	QImage image;

	/* set window title - ���ô��ڱ��� */
	setWindowTitle(codec->toUnicode(DEFAULT_WINDOW_TITLE));

	/* set window size - ���ô��ڴ�С */
	resize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);

	/* set central widget - �������ؼ� */
	mainWindow = new QWidget;
	setCentralWidget(mainWindow);

	/* set clock widget - ����ʱ�ӿؼ� */
	clockLabel = new QLabel(mainWindow);
	clockLabel->setWordWrap(true);	// adapt to text, can show multi row
	clockLabel->setGeometry(10, 10, 200, 20);	// set geometry - ���ü��δ�С����
	clockLabel->show();	// display widget - ��ʾ����

	/* create timer to show main window - ������ʱ������ʾ������ */
	timer = new QTimer(this);
	/* if timer timeout it will call slot function - �����ʱ����ʱʱ�䵽�˾ͻ���òۺ���showMainwindow() */
	connect(timer, SIGNAL(timeout()), this, SLOT(showMainwindow()));
	/* set timeout time and start timer - ���ó�ʱʱ�䲢������ʱ����TIMER_INTERV_MS=1ms */
	timer->start(TIMER_INTERV_MS);

	/* choice uart port - ѡ��uart�˿ڣ�/dev/ttyUSBx */
	uartListBox = new QComboBox(mainWindow);
	uartListBox->setGeometry(10, 100, 150, 30);
	uartListBox->setEditable(true);
	/* set course */
	for(int i=0; i<6; i++)
	{
		memset(dev_name, 0, sizeof(dev_name));
		sprintf(dev_name, "%s%d", DEV_TTY_USB, i);
		uartListBox->addItem(dev_name);
	}

	connectBtn = new QPushButton(mainWindow);	// connect/disconnect button - ����/�Ͽ���ť
	connectBtn->setText(codec->toUnicode(TEXT_CONNECT));
	connectBtn->setGeometry(55, 140, 45, 25);
	/* button slot function - �����Ĳۺ������ź���ۣ��������˰���ʱ������õ��ۺ��� */
	connect(connectBtn, SIGNAL(clicked()), this, SLOT(connect_device()));
	connectBtn->setFocus();

	/* user information image - �û���ϢͼƬ */
	image.load(USER_INFO_IMG);
	userInfo = new QLabel(mainWindow);
	userInfo->setPixmap(QPixmap::fromImage(image));
	userInfo->setGeometry(0, DEFAULT_WINDOW_HEIGHT/2, image.width(), image.height());
	userInfo->show();

	connect_state = 0;
}


MainWindow::~MainWindow(void)
{
	
}

/* timer timeout slot function, to show main window - ��ʱ����ʱʱ�䵽�Ĳۺ�������ʾ������ */
void MainWindow::showMainwindow(void)
{

	timer->stop();

	/* update clock - ����ʱ����ʾ */
	QDateTime time = QDateTime::currentDateTime();
	QString str = time.toString("yyyy-MM-dd hh:mm:ss dddd");
	clockLabel->setText(str);


	/* start timer again - �ٴ�������ʱ�� */
	timer->start(TIMER_INTERV_MS);

}

/* ����uart�豸 */
void MainWindow::connect_device(void)
{
	QTextCodec *codec = QTextCodec::codecForName("GBK");
	QString input_qstr;
	QByteArray ba;
	char uart_dev[32];
	int ret;
	
	if(connect_state == 0)	// disconnected, connect it
	{
		/* get input content - ��ȡ��������� */
		input_qstr = uartListBox->currentText();
		if(input_qstr.length() <= 0)
		{
			cout << "input error!\n" << endl;
			return ;
		}

		qDebug() << input_qstr;
		
		ba = input_qstr.toLatin1();
		memset(uart_dev, 0, sizeof(uart_dev));
		strncpy((char *)uart_dev, ba.data(), strlen(ba.data()));

		/* start uart communicate - ����uartͨ�� */
		ret = uart_comm_start(uart_dev);
		if(ret == 0)
		{
			uartListBox->setEnabled(false);	// disable input - ��ֹ����
			connectBtn->setText(codec->toUnicode(TEXT_DISCONNECT));	// set button "disconnect" - ���ð�ť��ʾ���Ͽ���
			connect_state = 1;
		}
	}
	else	// connected, disconnect it
	{
		disconnect_device();
	}

}

/* �Ͽ�uart�豸 */
void MainWindow::disconnect_device(void)
{
	QTextCodec *codec = QTextCodec::codecForName("GBK");
	cout << "disconnect device" << endl;

	uartListBox->setEnabled(true);
	connectBtn->setText(codec->toUnicode(TEXT_CONNECT));	// set button "connect" - ���ð�ť�����ӡ�
	
	uart_comm_stop();	// ֹͣuart�豸

	connect_state = 0;
}

/* �Ͽ�uart�豸 */
void mainwin_set_disconnect(void)
{
	mainwindow->disconnect_device();
}

/* main window initial - �������ʼ�� */
int mainwindow_init(void)
{
	mainwindow = new MainWindow;

	mainwindow->show();
	
	return 0;
}

