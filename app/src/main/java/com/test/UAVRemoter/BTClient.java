/*
  微型四轴源码版权归西电航协研发部门团队所有，未经本团队同意，请勿随意在网上传播本源码。
	与本软件相关参考资料西电航协微型四轴开发指南，内容对本套包含的所有软件以及硬件相关都做了详细的讲解
  如果有同学做了各种有意义的改进或有任何建议，请随时与我们保持联系。
	作者：PhillWeston
	联系邮箱：2436559745@qq.com
*/

package com.test.UAVRemoter;

import android.app.Activity;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.graphics.Color;
import android.graphics.Point;
import android.graphics.PorterDuff;
import android.nfc.Tag;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupMenu;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;
import android.content.Intent;

import java.util.ListIterator;
import java.util.Vector;
import java.util.jar.JarEntry;


public class BTClient extends Activity {

    private final static String TAG = BTClient.class.getSimpleName();

    public static final String EXTRAS_DEVICE_NAME = "DEVICE_NAME";
    public static final String EXTRAS_DEVICE_ADDRESS = "DEVICE_ADDRESS";



    private String mDeviceName;
    private String mDeviceAddress;
    private short rssi;
    private BluetoothLeService mBluetoothLeService; //BLE收发服务
    private boolean mConnected = false;
    public Vector<StringBuilder> mlogData=new Vector<StringBuilder>();
    private logData temp;
    private Help mhelp;
    private License mlicense;
    private logData mlogdata;
    //
    private AuxiliaryBar mAuxiliaryBar;


	private final static int REQUEST_CONNECT_DEVICE = 1; // 宏定义查询设备句柄

	//向BLE发送数据的周期，现在是两类数据，一是摇杆的4通道值，二是请求IMU数据跟新命令
    //BLE模块本身传输速率有限，尽量减少数据发送量
	private final static int WRITE_DATA_PERIOD=40;
    private static int IMU_CNT = 0; //update IMU period，跟新IMU数据周期，40*10ms
    Handler timeHandler = new Handler();    //定时器周期，用于跟新IMU数据等
	 
	private TextView throttleText,yawText,pitchText,rollText;
	private TextView pitchAngText,rollAngText,yawAngText,altText,distanceText,voltageText;
	private Button connectButton,armButton,launchLandButton,headFreeButton,altHoldButton;
	private Switch switchTool;
	private ImageView settingsView;
    //
	private BatteryView batteryView;
    private MySurfaceView rockerViewLeft;
    private MySurfaceView rockerViewRight;



    //pattern
    private final static int JAPAN_PATTERN=0;
    private final static int AMERICA_PATTERN=1;
    int pattern=JAPAN_PATTERN;
    private WindowManager wm;
    // Code to manage Service lifecycle.
    // 管理BLE数据收发服务整个生命周期
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            mBluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            mBluetoothLeService = null;
        }
    };

    //rssi
    private void rssiState(short rssi)
    {
        if(rssi<-70){
            Toast.makeText(BTClient.this,"蓝牙信号不佳",Toast.LENGTH_SHORT);
        }
    }

    // Handles various events fired by the Service.
    // ACTION_GATT_CONNECTED: connected to a GATT server.
    // ACTION_GATT_DISCONNECTED: disconnected from a GATT server.
    // ACTION_GATT_SERVICES_DISCOVERED: discovered GATT services.
    // ACTION_DATA_AVAILABLE: received data from the device.  This can be a result of read
    //                        or notification operations.
    // 定义处理BLE收发服务的各类事件接收机mGattUpdateReceiver，主要包括下面几种：
    // ACTION_GATT_CONNECTED: 连接到GATT
    // ACTION_GATT_DISCONNECTED: 断开GATT
    // ACTION_GATT_SERVICES_DISCOVERED: 发现GATT下的服务
    // ACTION_DATA_AVAILABLE: BLE收到数据
    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            int reCmd=-2;
//            rssi=intent.getExtras().getShort(BluetoothDevice.EXTRA_RSSI);//获取额外rssi值
//            rssiState(rssi);
            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                Log.d(TAG,"GATT is connected");
                resetButtonValue(R.string.Disconnect);
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                Log.d(TAG,"GATT is disconnected");
                resetButtonValue(R.string.Connect);
            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {

                // Show all the supported services and characteristics on the user interface.
                // 获得所有的GATT服务，对于UAVRemoter的BLE透传模块，包括GAP（General Access Profile），
                // GATT（General Attribute Profile），还有Unknown（用于数据读取）
                //可以用于日志loading
                /**
                 *
                 **/
                mBluetoothLeService.getSupportedGattServices();

            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {

                final byte[] data = intent.getByteArrayExtra(BluetoothLeService.EXTRA_DATA);

                if (data != null && data.length > 0) {
                    final StringBuilder stringBuilder = new StringBuilder(data.length);
                    for(byte byteChar : data)
                        stringBuilder.append(String.format("%02X ", byteChar));
                    mlogData.addElement(stringBuilder);
//                    temp.SetData(mlogData);
//                    Log.i(TAG, "RX Data:"+mlogData);
                }


                //解析得到的数据，获得MSP命令编号
                reCmd=Protocol.processDataIn( data,data.length);
                updateLogData(1);   //跟新IMU数据，update the IMU data
            }
        }
    };


    Runnable runnable = new Runnable() {
        @Override
        public void run() {
            try {
                timeHandler.postDelayed(this, WRITE_DATA_PERIOD);

                if(IMU_CNT >= 10){
                    IMU_CNT = 0;
                    //request for IMU data update，请求IMU跟新
                    btSendBytes(Protocol.getSendData(Protocol.FLY_STATE,
                            Protocol.getCommandData(Protocol.FLY_STATE)));
                }
                IMU_CNT++;


                // process stick movement，处理摇杆数据
                //rockerViewLeft.touchReadyToSend和rockerViewLeft.touchReadyToSend其中一个有变化就传输变化
                //会出现定高的时候 某个值传输总是1500；
                //40ms
                if(rockerViewLeft.touchReadyToSend==true ||rockerViewRight.touchReadyToSend){
                    btSendBytes(Protocol.getSendData(Protocol.SET_4CON,
                            Protocol.getCommandData(Protocol.SET_4CON)));

                    Log.i(TAG,mConnected+"Throttle: " +Protocol.throttle +",yaw: " +Protocol.yaw+ ",roll: "
                            + Protocol.roll +",pitch: "+ Protocol.pitch);

                    rockerViewLeft.touchReadyToSend=false;
                    rockerViewRight.touchReadyToSend=false;
                }

                //跟新显示摇杆数据，update the joystick data
                //delete update the data
                //data update is put in onCreate

            } catch (Exception e) {

            }
        }
    };


    //设置按键显示为默认初始化值
    //在连接成功或者断开的时候，都需要把button的值复位
    private void resetButtonValue(final int connectBtnId) {
        connectButton.setText(connectBtnId);
        armButton.setText(R.string.Unarm);
        launchLandButton.setText(R.string.Launch);
        headFreeButton.setTextColor(Color.WHITE);
        altHoldButton.setTextColor(Color.WHITE);
    }


    private void rockerPattern()
    {
        rockerViewLeft=(MySurfaceView)findViewById(R.id.rockerView_left);
        rockerViewRight=(MySurfaceView)findViewById(R.id.rockerView_right);
        if(pattern==JAPAN_PATTERN)
        {
            rockerViewLeft.tag=0;
            rockerViewRight.tag=1;
        }
        else{
            rockerViewRight.tag=0;
            rockerViewLeft.tag=1;
        }
        if (rockerViewLeft != null) {
            rockerViewLeft.setCallBackMode(MySurfaceView.CallBackMode.CALL_BACK_MODE_STATE_CHANGE);
            rockerViewLeft.setOnPointChangeListener(new MySurfaceView.OnPointChangeListener() {
                @Override
                public void onStart() {
                    switch(pattern){
                        case JAPAN_PATTERN:
                            pitchText.setText("Pitch:1500");
                            rollText.setText("Roll:1500");
                            break;
                        case AMERICA_PATTERN:
                            throttleText.setText("Throttle:1500");
                            yawText.setText("Yaw:1500");
                            break;
                    }
                }

                @Override
                public void point(int x,int y) {
                    switch(pattern){
                        case JAPAN_PATTERN:
                            Protocol.pitch=y;
                            Protocol.roll=x;
//                            pitchText.setText("Pitch:"+Integer.toString(Protocol.pitch));
//                            rollText.setText("Roll:"+Integer.toString(Protocol.roll));
                            break;
                        case AMERICA_PATTERN:
                            Protocol.yaw=x;
                            Protocol.throttle=y;
//                            throttleText.setText("Throttle:"+Integer.toString(Protocol.throttle));
//                            yawText.setText("Yaw:"+Integer.toString(Protocol.yaw));
                            break;
                    }
                }
                @Override
                public void onFinish() {
                    switch(pattern){
                        case JAPAN_PATTERN:
                            pitchText.setText("Pitch:1500");
                            rollText.setText("Roll:1500");
                            break;
                        case AMERICA_PATTERN:
                            throttleText.setText("Throttle:1500");
                            yawText.setText("Yaw:1500");
                            break;
                    }
                }
            });
        }
        //Right
        if (rockerViewRight!= null) {
            rockerViewRight.setCallBackMode(MySurfaceView.CallBackMode.CALL_BACK_MODE_STATE_CHANGE);
            rockerViewRight.setOnPointChangeListener(new MySurfaceView.OnPointChangeListener() {
                @Override
                public void onStart() {
                    switch(pattern){
                        case AMERICA_PATTERN:
                            pitchText.setText("Pitch:1500");
                            rollText.setText("Roll:1500");
                            break;
                        case JAPAN_PATTERN:
                            throttleText.setText("Throttle:1500");
                            yawText.setText("Yaw:1500");
                            break;
                    }
                }

                @Override
                public void point(int x,int y) {
                    switch(pattern){
                        case AMERICA_PATTERN:
                            Protocol.pitch=y;
                            Protocol.roll=x;
//                            pitchText.setText("Pitch:"+Integer.toString(Protocol.pitch));
//                            rollText.setText("Roll:"+Integer.toString(Protocol.roll));
                            break;
                        case JAPAN_PATTERN:
                            Protocol.yaw=x;
                            Protocol.throttle=y;
//                            throttleText.setText("Throttle:"+Integer.toString(Protocol.throttle));
//                            yawText.setText("Yaw:"+Integer.toString(Protocol.yaw));
                            break;
                    }
                }
                @Override
                public void onFinish() {
                    switch(pattern){
                        case AMERICA_PATTERN:
                            pitchText.setText("Pitch:1500");
                            rollText.setText("Roll:1500");
                            break;
                        case JAPAN_PATTERN:
                            throttleText.setText("Throttle:1500");
                            yawText.setText("Yaw:1500");
                            break;
                    }
                }
            });
        }
    }


	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main); // 设置画面为主画面 main.xml 
		//显示 text
		throttleText = (TextView)findViewById(R.id.throttleText); // 
		yawText = (TextView)findViewById(R.id.yawText);
		pitchText = (TextView)findViewById(R.id.pitchText);
		rollText = (TextView)findViewById(R.id.rollText);
		//pitchAngText,rollAngText,yawAngText,altText,voltageText
		pitchAngText = (TextView)findViewById(R.id.pitchAngText);
		rollAngText = (TextView)findViewById(R.id.rollAngText);
		yawAngText = (TextView)findViewById(R.id.yawAngText);
		altText = (TextView)findViewById(R.id.altText);
		voltageText = (TextView)findViewById(R.id.voltageText);
		distanceText= (TextView)findViewById(R.id.distanceText);
		switchTool=(Switch)findViewById(R.id.tool_switch);
		settingsView=(ImageView)findViewById(R.id.settings);

		 

		//按钮
        connectButton=(Button)findViewById(R.id.connectButton);
		armButton=(Button)findViewById(R.id.armButton);
		launchLandButton=(Button)findViewById(R.id.lauchLandButton);
		headFreeButton=(Button)findViewById(R.id.headFreeButton);
		altHoldButton=(Button)findViewById(R.id.altHoldButton);

        wm= (WindowManager) this
                .getSystemService(Context.WINDOW_SERVICE);

        //绑定BLE收发服务mServiceConnection
        Intent gattServiceIntent = new Intent(this, BluetoothLeService.class);
        bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);

        //开启IMU数据跟新定时器
        timeHandler.postDelayed(runnable, WRITE_DATA_PERIOD); //每隔1s执行
        //在连接之前摇杆是可以用的

        //设置未联网就开启battery 线程
        //BatteryView.BatteryThread mbt = batteryView.new BatteryThread();
        //mbt.start();
        //

        rockerPattern();



        switchTool.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(isChecked){
                    pattern=AMERICA_PATTERN;
                }else {
                    pattern=JAPAN_PATTERN;
                }
            }
        });

        //settings
        settingsView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showPopupMenu(settingsView);
            }
        });
	}


    @Override
    protected void onResume() {
        super.onResume();

        //注册BLE收发服务接收机mGattUpdateReceiver
        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        if (mBluetoothLeService != null) {
            Log.d(TAG, "mBluetoothLeService NOT null");
        }

    }

	@Override
	public void onPause()
	{
		super.onPause();
        //注销BLE收发服务接收机mGattUpdateReceiver
        unregisterReceiver(mGattUpdateReceiver);
	}
	@Override
	public void onStop()
	{
		super.onStop();
	}

    @Override
    protected void onDestroy() {
        super.onDestroy();

        //解绑BLE收发服务mServiceConnection
        unbindService(mServiceConnection);
        mBluetoothLeService = null;
    }



    // 连接按键响应函数
    public void onConnectButtonClicked(View v) {
        if (!mConnected) {
            //进入扫描页面
            Intent serverIntent = new Intent(this, DeviceScanActivity.class); // 跳转程序设置
            startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE); // 设置返回宏定义

        } else {
            //断开连接
            mBluetoothLeService.disconnect();
        }
    }

    public void onSendArmButtonClicked(View v)
	{
        String arm = getResources().getString(R.string.Arm);
        String unarm = getResources().getString(R.string.Unarm);
        String disconnectToast = getResources().getString(R.string.DisconnectToast);

		if(mConnected){
			if(armButton.getText() != arm)	{
				btSendBytes(Protocol.getSendData(Protocol.DISARM_IT, Protocol.getCommandData(Protocol.DISARM_IT)));
				armButton.setText(arm);
			}else{
				btSendBytes(Protocol.getSendData(Protocol.ARM_IT, Protocol.getCommandData(Protocol.ARM_IT)));
				armButton.setText(unarm);
			}
		}else {
            Toast.makeText(this, disconnectToast, Toast.LENGTH_SHORT).show();
        }
	}
	
	//Take off , land down
	public void onlauchLandButtonClicked(View v)
	{
        String launch  = getResources().getString(R.string.Launch);
        String land = getResources().getString(R.string.Land);
        String disconnectToast = getResources().getString(R.string.DisconnectToast);

        if(mConnected){
            if(launchLandButton.getText() != land){
                btSendBytes(Protocol.getSendData(Protocol.LAUNCH, Protocol.getCommandData(Protocol.LAUNCH)));
                launchLandButton.setText(land);
                //没有设置摇杆不受控制，因此LAUNCH_THROTTLE会因摇杆接触而失效
                Protocol.throttle=Protocol.LAUNCH_THROTTLE;
                //这里因为只要一个变化就会传输数据，因此随意定义一个控件为true
                rockerViewRight.touchReadyToSend=true;

            }else{
                btSendBytes(Protocol.getSendData(Protocol.LAND_DOWN, Protocol.getCommandData(Protocol.LAND_DOWN)));
                launchLandButton.setText(launch);
                Protocol.throttle=Protocol.LAND_THROTTLE;

                rockerViewRight.touchReadyToSend=true;
            }
        }else {
            Toast.makeText(this, disconnectToast, Toast.LENGTH_SHORT).show();
        }
	}

	//无头模式键
	public void onheadFreeButtonClicked(View v)
	{
        String disconnectToast = getResources().getString(R.string.DisconnectToast);

		if(mConnected){
			if(headFreeButton.getCurrentTextColor()!=Color.GREEN)
			{	btSendBytes(Protocol.getSendData(Protocol.HEAD_FREE, Protocol.getCommandData(Protocol.HEAD_FREE)));
				headFreeButton.setTextColor(Color.GREEN);
			}else{
				btSendBytes(Protocol.getSendData(Protocol.STOP_HEAD_FREE, Protocol.getCommandData(Protocol.STOP_HEAD_FREE)));
				headFreeButton.setTextColor(Color.WHITE);
			}
		}else {
            Toast.makeText(this, disconnectToast, Toast.LENGTH_SHORT).show();
        }
		
	}

	//定高键
	public void onaltHoldButtonClicked(View v)
	{
        String disconnectToast = getResources().getString(R.string.DisconnectToast);

		if(mConnected){
			if( altHoldButton.getCurrentTextColor()!=Color.GREEN )
			{	//定高定点都开
				btSendBytes(Protocol.getSendData(Protocol.HOLD_ALT, Protocol.getCommandData(Protocol.HOLD_ALT))); 
				altHoldButton.setTextColor(Color.GREEN);
				rockerViewLeft.altCtrlMode=1;
				rockerViewRight.altCtrlMode=1;
			}else{
				btSendBytes(Protocol.getSendData(Protocol.STOP_HOLD_ALT, Protocol.getCommandData(Protocol.STOP_HOLD_ALT)));
				altHoldButton.setTextColor(Color.WHITE);
				rockerViewRight.altCtrlMode=0;
				rockerViewLeft.altCtrlMode=0;
			}
		}else {
            Toast.makeText(this, disconnectToast, Toast.LENGTH_SHORT).show();
        }
	}
	
	//校准
	public void onAccCaliButtonClicked(View v)
	{
        String disconnectToast = getResources().getString(R.string.DisconnectToast);

		if(mConnected){
			btSendBytes(Protocol.getSendData(Protocol.MSP_ACC_CALIBRATION, Protocol.getCommandData(Protocol.MSP_ACC_CALIBRATION)));
		}else {
            Toast.makeText(this, disconnectToast, Toast.LENGTH_SHORT).show();
        }
	}

	public void btSendBytes(byte[] data)
	{
        //当已经连接上时才发送
		if(mConnected){
            mBluetoothLeService.writeCharacteristic(data);
		}
	}
	
	// 接收扫描结果，响应startActivityForResult()
	public void onActivityResult(int requestCode, int resultCode, Intent data) {
		switch (requestCode) {
		case REQUEST_CONNECT_DEVICE:
			if (resultCode == Activity.RESULT_OK){
                mDeviceName = data.getExtras().getString(EXTRAS_DEVICE_NAME);
                mDeviceAddress = data.getExtras().getString(EXTRAS_DEVICE_ADDRESS);
                Log.i(TAG, "mDeviceName:"+mDeviceName+",mDeviceAddress:"+mDeviceAddress);
                //连接该BLE UAVRemoter模块
                if (mBluetoothLeService != null) {
                    final boolean result = mBluetoothLeService.connect(mDeviceAddress);
                    Log.d(TAG, "Connect request result=" + result);
                }
            }
			break;
		default:
			break;
		}
	}

    //跟新Log相关的数据，主要是飞控传过来的IMU数据和摇杆值数据
    //update log,included the IMU data from FC and joysticks data
    //msg 0 -> joystick data
    //msg 1 -> IMU data
    private void updateLogData(int msg){
        //delete msg=0
        if(1 == msg)
        {
            pitchAngText.setText("Pitch Ang: "+Protocol.pitchAng);
            rollAngText.setText("Roll Ang: "+Protocol.rollAng);
            yawAngText.setText("Yaw Ang: "+Protocol.yawAng);
            altText.setText("Alt:"+Protocol.alt + "m");
            voltageText.setText("Voltage:"+Protocol.voltage + " V");
            distanceText.setText("Distance:"+Protocol.speedZ + "m");
            batteryView.setPower(Protocol.voltage);
            yawText.setText("Yaw:"+Protocol.yawFly);
            throttleText.setText("Throttle:"+Protocol.throttleFly);
            pitchText.setText("Pitch:"+Protocol.pitchFly);
            rollText.setText("Roll:"+Protocol.rollFly);
        }
    }


    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        return intentFilter;
    }

    private void showPopupMenu(View view) {
        // View当前PopupMenu显示的相对View的位置
        PopupMenu popupMenu = new PopupMenu(this, view);
        // menu布局
        popupMenu.getMenuInflater().inflate(R.menu.menu, popupMenu.getMenu());
        // menu的item点击事件
        popupMenu.setOnMenuItemClickListener(new PopupMenu.OnMenuItemClickListener() {
            @Override
            public boolean onMenuItemClick(MenuItem item) {
                int height=getScreenPara().y/4*3;
                int width =getScreenPara().x/4*3;
                switch (item.getItemId()) {
                    case R.id.action_help:
                        mhelp=new Help(BTClient.this,R.style.MyDialog);
                        mhelp.show();
                        mhelp.getWindow().setLayout(width, height);
                        Log.i(TAG,width+"and"+height);
                        mhelp.getWindow().setBackgroundDrawableResource(android.R.color.transparent);
                        mhelp.setCancelable(false);
                        break;
                    case R.id.action_license:
                        mlicense=new License(BTClient.this,R.style.MyDialog);
                        mlicense.show();
                        mlicense.getWindow().setLayout(width,height);
                        Log.i(TAG,width+"and"+height);
                        mlicense.getWindow().setBackgroundDrawableResource(android.R.color.transparent);
                        mlicense.setCancelable(false);
//                        Toast.makeText(BTClient.this, "action_share", Toast.LENGTH_LONG).show();
                        break;
                    case R.id.action_log:
                        mlogdata=new logData(BTClient.this,R.style.MyDialog);
                        mlogdata.show();
                        mlogdata.getWindow().setLayout(width,height);
                        Log.i(TAG,width+"and"+height);
                        mlogdata.getWindow().setBackgroundDrawableResource(android.R.color.transparent);
                        mlogdata.setCancelable(false);
                        mlogdata.SetData(mlogData);
                        break;
                    case R.id.action_bar:
                        mAuxiliaryBar=new AuxiliaryBar(BTClient.this,R.style.MyDialog,Protocol.progressBar1,
                                Protocol.progressBar2,Protocol.progressBar3,Protocol.progressBar4);
                        mAuxiliaryBar.show();
                        mAuxiliaryBar.getWindow().setLayout(width, height);
                        Log.i(TAG,width+"and"+height);
                        mAuxiliaryBar.getWindow().setBackgroundDrawableResource(android.R.color.transparent);
                        mAuxiliaryBar.setCancelable(false);
                        mAuxiliaryBar.setOnProgressGetListener(new AuxiliaryBar.OnProgressGetListener() {
                            @Override
                            public void getProgress(int progress1, int progress2, int progress3, int progress4) {
                                Protocol.progressBar1=progress1;
                                Protocol.progressBar2=progress2;
                                Protocol.progressBar3=progress3;
                                Protocol.progressBar4=progress4;
                                Log.d(TAG,"Yaw Auxi:"+Protocol.progressBar1+"Throttle Auxi:"+Protocol.progressBar2+"Pitch Auxi:"
                                        +Protocol.progressBar3+"Yaw Auxi:"+Protocol.progressBar4);
                                btSendBytes(Protocol.getSendData(Protocol.HEAD_FREE, Protocol.getCommandData(Protocol.SET_4CON)));
                                Log.d(TAG,"btsendbytes_Auxi");
                            }
                        });
                        break;
                    default:
                        break;
                }
                return true;
            }
        });
        // PopupMenu关闭事件
        popupMenu.setOnDismissListener(new PopupMenu.OnDismissListener() {
            @Override
            public void onDismiss(PopupMenu menu) {
//                Toast.makeText(getApplicationContext(), "关闭PopupMenu", Toast.LENGTH_SHORT).show();
            }
        });
        popupMenu.show();
    }

    //get screen width
    private Point getScreenPara()
    {
        Point point=new Point();
        wm.getDefaultDisplay().getSize(point);
        return point;
    }



}
