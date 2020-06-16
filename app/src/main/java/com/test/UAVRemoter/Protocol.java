/*
  微型四轴源码版权归西电航协研发部门团队所有，未经本团队同意，请勿随意在网上传播本源码。
	与本软件相关参考资料西电航协微型四轴开发指南，内容对本套包含的所有软件以及硬件相关都做了详细的讲解
  如果有同学做了各种有意义的改进或有任何建议，请随时与我们保持联系。
	作者：PhillWeston
	联系邮箱：2436559745@qq.com
*/
//发送部分写有些麻烦，直接定义一个发送缓冲Byte变长数组，写writeInt8 writeInt16 ，把数据到里头，再统一发，发完清空清长。
//UAVRemoter APP和飞控之间通信协议使用了MWC飞控协议（MSP，Multiwii Serial Protocol），
//MSP协议格式详见http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

package com.test.UAVRemoter;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.nfc.Tag;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

import java.util.LinkedList;
import java.util.List;

public class Protocol {
	public static final int MAX = 250;
	public static final int MIN = -250;
	public static int throttle,yaw,pitch,roll;
	public static int progressBar2 = (MAX-MIN)/2+MIN;
	public static int progressBar1 = (MAX-MIN)/2+MIN;
	public static int progressBar3 = (MAX-MIN)/2+MIN;
	public static int progressBar4 = (MAX-MIN)/2+MIN;
	public static float pitchAng,rollAng,yawAng,alt,voltage,speedZ;
	public static int throttleFly,yawFly,pitchFly,rollFly;
	public static byte[] outputData;
	public static final int 
	SET_THROTTLE=0x01,
	SET_YAW=0x02,
	SET_PITCH=0x03,
	SET_ROLL=0x04,
	ARM_IT=5,
	DISARM_IT=6,
	SET_4CON=7,
	LAUNCH=8,
	LAND_DOWN=9,
	HOLD_ALT=10,
	STOP_HOLD_ALT=11,
	HEAD_FREE=12,
	STOP_HEAD_FREE=13,
//	POS_HOLD=14,
//	STOP_POS_HOLD=15,
	FLY_STATE=16,//pitch、roll、yaw、Altitude、GPS_FIX?、Sat num、Voltage
	MSP_SET_1WP=17,
	SET_MOTOR=214,
	MSP_ACC_CALIBRATION=205;
	
	public static final int LAUNCH_THROTTLE=1455;
	public static final int LAND_THROTTLE=1340;

	
	
	private static final String MSP_HEADER = "$M<";
	
	static byte[] getCommandData(int cmd)
	{
		 
		List<Byte> cmdData=new LinkedList<Byte>();
		switch(cmd)
		{
//		case SET_THROTTLE:
//			cmdData.add((byte)((throttle )&0xff));
//			cmdData.add((byte)((throttle>>8)&0xff));
//			break;
//		case SET_YAW:
//			cmdData.add((byte)((yaw )&0xff));
//			cmdData.add((byte)((yaw>>8)&0xff));
//			break;
//		case SET_PITCH:
//			cmdData.add((byte)((pitch )&0xff));
//			cmdData.add((byte)((pitch>>8)&0xff));
//			break;
//		case SET_ROLL:
//			cmdData.add((byte)((roll )&0xff));
//			cmdData.add((byte)((roll>>8)&0xff));
//			break;
//		case SET_MOTOR:
//			cmdData.add((byte)((throttle )&0xff));
//			cmdData.add((byte)((throttle>>8)&0xff));
//			break;
		case SET_4CON:
			cmdData.add((byte)((roll>>8)&0xff));
			cmdData.add((byte)((pitch )&0xff));
			cmdData.add((byte)((pitch>>8)&0xff));
			cmdData.add((byte)((yaw )&0xff));
			cmdData.add((byte)((yaw>>8)&0xff));
			cmdData.add((byte)((throttle)&0xff));
			cmdData.add((byte)((throttle>>8)&0xff));
			cmdData.add((byte)((progressBar4)&0xff));
			cmdData.add((byte)((progressBar4>>8)&0xff)); //roll
			cmdData.add((byte)((progressBar3 )&0xff));
			cmdData.add((byte)((progressBar3>>8)&0xff)); //pitch
			cmdData.add((byte)((progressBar1 )&0xff));
			cmdData.add((byte)((progressBar1>>8)&0xff)); //yaw
			cmdData.add((byte)((progressBar2)&0xff));
			cmdData.add((byte)((progressBar2>>8)&0xff)); //throttle
			break;
		case ARM_IT:
			return null;
			//break;
		case DISARM_IT:
			return null;

			//break;
		case LAUNCH:

			//throttle
			return null; 
		case LAND_DOWN:

			return null;
		case HOLD_ALT: 
			return null;
		case STOP_HOLD_ALT:
			return null;
		case STOP_HEAD_FREE:
			return null;
		case HEAD_FREE:
			return null;
//		case POS_HOLD:
//			return null;
//		case STOP_POS_HOLD:
//			return null;
		case FLY_STATE://请求IMU跟新
			return null;
		case MSP_ACC_CALIBRATION:
			return null;
		default: break;
		}
		
		byte[] commandData = new byte[cmdData.size()];
		 int i = 0;
		  for (byte b: cmdData) {
			  commandData[i++] = b;
		  }
		  Log.d("dataLength", ":"+i);
		return commandData;
	}
	// get the whole cmd data to send to MUV
	static byte[] getSendData(int cmd,byte[] data)
	{ 
		if(cmd < 0) 
			return null;
		List<Byte> bf = new LinkedList<Byte>();
		  for (byte c : MSP_HEADER.getBytes()) {
		    bf.add( c );
		  }  
		  byte checksum=0;
		 // byte pl_size = (byte)((payload != null ? PApplet.parseInt(payload.length) : 0)&0xFF);
		  byte dataSize=(byte)((data!=null)?(data.length):0);
		  bf.add(dataSize);
		  checksum ^= (dataSize&0xFF);
		  
		  bf.add((byte)(cmd & 0xFF));
		  checksum ^= (cmd&0xFF);
		  
		  if (data != null) {
		    for (byte c :data){
		      bf.add((byte)(c&0xFF));
		      checksum ^= (c&0xFF);
		    }
		  }
		  bf.add(checksum);
		  
		  byte[] sendData = new byte[bf.size()];
		  int i = 0;
		  for (byte b: bf) {
			  sendData[i++] = b;
		  }
		  
		  return (sendData);
		   
	}
	public static final int
	  IDLE = 0,
	  HEADER_START = 1,
	  HEADER_M = 2,
	  HEADER_ARROW = 3,
	  HEADER_SIZE = 4,
	  HEADER_CMD = 5,
	  HEADER_ERR = 6;
	static boolean frameEnd=false;
	static int c_state = IDLE;
	static boolean err_rcvd;
	static byte checksum=0;
	static byte cmd;
	static int offset=0, dataSize=0;
	static byte[] inBuf = new byte[256];
	static int p;
	public static int read32() {return (inBuf[p++]&0xff) + ((inBuf[p++]&0xff)<<8) + ((inBuf[p++]&0xff)<<16) + ((inBuf[p++]&0xff)<<24); }
	public static int read16() {return (inBuf[p++]&0xff) + ((inBuf[p++])<<8); }
	public static int read8()  {return  inBuf[p++]&0xff;}
	// process to extract the data from initial data frame
	static int processDataIn(byte[] inData,int len)
	{
		 int i=0;
		int c;
	 	System.out.println("dataInLen:"+len+" "+inData[0]);
		for(i=0;i<len;i++)
		{ 
			c = inData[i];
			if (c_state == IDLE) {//$
		        c_state = (c=='$') ? HEADER_START : IDLE;
		      } 
			else if (c_state == HEADER_START) {//M
		        c_state = (c=='M') ? HEADER_M : IDLE;
		      } 
			else if (c_state == HEADER_M) {
		        if (c == '>') {	//right
		          c_state = HEADER_ARROW;
		        } else if (c == '!') {
		          c_state = HEADER_ERR;	//wrong
		        } else {
		          c_state = IDLE;
		        }
		      } 
			else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
		        // is this an error message?  
		        err_rcvd = (c_state == HEADER_ERR);        // now we are expecting the payload size 
		        dataSize = (c&0xFF);
		        // reset index variables  
		        p = 0;
		        offset = 0;
		        checksum = 0;
		        checksum ^= (c&0xFF);
		        // the command is to follow  
		        c_state = HEADER_SIZE;
		      } 
			else if (c_state == HEADER_SIZE) {
		        cmd = (byte)(c&0xFF);  //code
		        checksum ^= (c&0xFF);
		        c_state = HEADER_CMD;
		      } 
			else if (c_state == HEADER_CMD && offset < dataSize) {
		          checksum ^= (c&0xFF);
		          inBuf[offset++] = (byte)(c&0xFF);
		      } 
			else if (c_state == HEADER_CMD && offset >= dataSize) {
				frameEnd=true;
		        // compare calculated and transferred checksum  
		        if ((checksum&0xFF) == (c&0xFF)) {//校验对比
		          if (err_rcvd) {
		            //System.err.println("Copter did not understand request type "+c);
		          } 
		          else {
		            // we got a valid response packet, evaluate it   
		        	  evaluateCommand(cmd,dataSize);
		        	  System.out.println("cmd="+cmd); 
		          }
		        } 
		        else //校验出错
		        { 
		          System.out.println("invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF));
		          System.out.print("<"+(cmd&0xFF)+" "+(dataSize&0xFF)+"> {");
		          for (i=0; i<dataSize; i++) {
		            if (i!=0) { System.err.print(' '); }
		            System.out.print((inBuf[i] & 0xFF));
		          }
		          System.out.println("} ["+c+"]");
		          System.out.println(new String(inBuf, 0, dataSize)); 
		        }
		        c_state = IDLE;
		      //  return -1;
		      }
		} 
		if(!frameEnd)
			return -2;
		else
		{
			frameEnd=false;
			if(err_rcvd)
				return -1;
			else
				return cmd; 
		} 
	}
	
	public static void evaluateCommand(byte cmd, int dataSize) {
		  int i;
		  int icmd = (int)(cmd&0xFF);
		  switch(icmd) {
		    case FLY_STATE:
		    	rollAng = read16()/10;
		    	pitchAng = read16()/10;
		    	yawAng = read16()/10;
		        alt = read32()/100.0f;		//cm
				voltage=read16()/100.0f;
		        speedZ=read16()/1000.0f;
				yawFly = read16();
				throttleFly= read16();
				pitchFly = read16();
				rollFly = read16();
		        System.out.println("pitch:"+pitchAng);
		       break;
		  }
	}

}