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


/* QT: main window layout - 主界面布局 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
	QTextCodec *codec = QTextCodec::codecForName("GBK");
	char dev_name[32] = {};
	QImage image;

	/* set window title - 设置窗口标题 */
	setWindowTitle(codec->toUnicode(DEFAULT_WINDOW_TITLE));

	/* set window size - 设置窗口大小 */
	resize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);

	/* set central widget - 设置主控件 */
	mainWindow = new QWidget;
	setCentralWidget(mainWindow);

	/* set clock widget - 设置时钟控件 */
	clockLabel = new QLabel(mainWindow);
	clockLabel->setWordWrap(true);	// adapt to text, can show multi row
	clockLabel->setGeometry(10, 10, 200, 20);	// set geometry - 设置几何大小坐标
	clockLabel->show();	// display widget - 显示控制

	/* create timer to show main window - 创建定时器来显示主界面 */
	timer = new QTimer(this);
	/* if timer timeout it will call slot function - 如果定时器定时时间到了就会调用槽函数showMainwindow() */
	connect(timer, SIGNAL(timeout()), this, SLOT(showMainwindow()));
	/* set timeout time and start timer - 设置超时时间并启动定时器，TIMER_INTERV_MS=1ms */
	timer->start(TIMER_INTERV_MS);

	/* choice uart port - 选择uart端口，/dev/ttyUSBx */
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

	connectBtn = new QPushButton(mainWindow);	// connect/disconnect button - 连接/断开按钮
	connectBtn->setText(codec->toUnicode(TEXT_CONNECT));
	connectBtn->setGeometry(55, 140, 45, 25);
	/* button slot function - 按键的槽函数，信号与槽：当触发了按键时，会调用到槽函数 */
	connect(connectBtn, SIGNAL(clicked()), this, SLOT(connect_device()));
	connectBtn->setFocus();

	/* user information image - 用户信息图片 */
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

/* timer timeout slot function, to show main window - 定时器定时时间到的槽函数，显示主界面 */
void MainWindow::showMainwindow(void)
{

	timer->stop();

	/* update clock - 更新时间显示 */
	QDateTime time = QDateTime::currentDateTime();
	QString str = time.toString("yyyy-MM-dd hh:mm:ss dddd");
	clockLabel->setText(str);


	/* start timer again - 再次启动定时器 */
	timer->start(TIMER_INTERV_MS);

}

/* 连接uart设备 */
void MainWindow::connect_device(void)
{
	QTextCodec *codec = QTextCodec::codecForName("GBK");
	QString input_qstr;
	QByteArray ba;
	char uart_dev[32];
	int ret;
	
	if(connect_state == 0)	// disconnected, connect it
	{
		/* get input content - 获取输入的内容 */
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

		/* start uart communicate - 启动uart通信 */
		ret = uart_comm_start(uart_dev);
		if(ret == 0)
		{
			uartListBox->setEnabled(false);	// disable input - 禁止输入
			connectBtn->setText(codec->toUnicode(TEXT_DISCONNECT));	// set button "disconnect" - 设置按钮显示“断开”
			connect_state = 1;
		}
	}
	else	// connected, disconnect it
	{
		disconnect_device();
	}

}

/* 断开uart设备 */
void MainWindow::disconnect_device(void)
{
	QTextCodec *codec = QTextCodec::codecForName("GBK");
	cout << "disconnect device" << endl;

	uartListBox->setEnabled(true);
	connectBtn->setText(codec->toUnicode(TEXT_CONNECT));	// set button "connect" - 设置按钮“连接”
	
	uart_comm_stop();	// 停止uart设备

	connect_state = 0;
}

/* 断开uart设备 */
void mainwin_set_disconnect(void)
{
	mainwindow->disconnect_device();
}

/* main window initial - 主界面初始化 */
int mainwindow_init(void)
{
	mainwindow = new MainWindow;

	mainwindow->show();
	
	return 0;
}

