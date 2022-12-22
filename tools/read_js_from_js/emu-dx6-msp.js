'use strict';

const THREE = require('three');
const net = require('net');
const ROSLIB = require('roslib');
const ioctl = require('ioctl');

const DEBUG_INFO = {
    DEBUG_SILENT:  0,
    DEBUG_MINOR:   1,
    DEBUG_ALL:     2
}

let DEBUG_LEVEL = DEBUG_INFO.DEBUG_SILENT
var FREQ = 200    // in Hz
var ip = '127.0.0.1'
var port = 5762 // MSP port on UART2
// via MSP:
const MSP = require('./node-msp') // https://github.com/cs8425/node-msp
var msp = new MSP()
var mspBOXID = []
var mspBOXNAME = []
var mspStateMode = 0

var RC_MAP = '';
var IMU_acc = [0,0,0];
var IMU_gyro = [0,0,0];
var channelMSPIndexes = {
    A: 3, E: 2, R: 1, T: 0,
    '1': 4, '2': 5, '3': 6, '4': 7,
};
var THROTTLE_PRINT = 0
var Attitude_Quat = new THREE.Quaternion();
var Attitude_Setpoint_Euler = new THREE.Euler();
var Throttle_Setpoint = 0;
var stateArm = false, JoystickAvailable = false;
var TLeft = 0, TRight = 0, deltaLeft = 0, deltaRight = 0;

/***************************************
           Useful function
***************************************/
var SaturateChannel = function(value){
    if(value > 2000) value = 2000;
    if(value < 1000) value = 1000;
    return value;
}


/***************************************
           Setting up ROS
***************************************/
var ros = new ROSLIB.Ros({
    url : 'ws://0.0.0.0:9090'
});
// ROS Connection
ros.on('connection', function(){
    console.log('Connected to ROS websocket server.');
});
// ROS Error
ros.on('error', function(error) {
    throw new Error("Error connecting to websocket server");
});
// ROS Close
ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// Defining topics
var imuTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/imu_data',
    messageType : 'sensor_msgs/Imu'
});

var pwmTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/pwm_data',
    messageType : 'std_msgs/Float32MultiArray'
});

var attitudeSetpointTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/attitude_setpoint',
    messageType : 'mavros_msgs/AttitudeTarget'
}).subscribe(function(message) {
    if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_MINOR){
	console.log('Receiving attitude setpoint')
    }
    var setpoint_message = message
    Attitude_Setpoint_Euler.setFromQuaternion(new THREE.Quaternion(setpoint_message.orientation.x, 
    	                                                           setpoint_message.orientation.y,
    	                                                           setpoint_message.orientation.z,
    	                                                           setpoint_message.orientation.w), 'ZYX');
    channelValues[channelMSPIndexes['A']] = SaturateChannel(Attitude_Setpoint_Euler.X + 1500)
    channelValues[channelMSPIndexes['E']] = SaturateChannel(Attitude_Setpoint_Euler.Y + 1500)
    channelValues[channelMSPIndexes['R']] = SaturateChannel(Attitude_Setpoint_Euler.Z + 1500)
    Throttle_Setpoint = setpoint_message.thrust
    channelValues[channelMSPIndexes['T']] = Throttle_Setpoint
});

var armingTopic = new ROSLIB.Topic({
    ros  : ros,
    name : '/commandARM',
    messageType : 'std_msgs/Bool'
}).subscribe(function(message){
    if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_MINOR){
	console.log('Receiving arming command');
    }
    if(message.data == true){
	channelValues[channelMSPIndexes[1]] = 1900;
	channelValues[channelMSPIndexes['T']] = 1000;
    }else{
	channelValues[channelMSPIndexes[1]] = 1100;
    }
});

/**********************************
             MSP usage
**********************************/
msp.on('frame', function(err, frame){
    if(err) return
    //	console.log((new Date()).getTime(), 'frame', JSON.stringify(frame))

    //	var obj = msp.parseFrame(frame)
    //	console.log((new Date()).getTime(), 'data', obj)
})
msp.on('data', function(obj){
    //console.log((new Date()).getTime(), 'data', obj.code, obj)
    switch(obj.code){
    case msp.Codes.MSP_RX_MAP:{
	RC_MAP = obj.RC_MAP;
	for(var i=0; i<RC_MAP.length; i++){
	    //			console.log(RC_MAP[i], i)
	    channelMSPIndexes[RC_MAP[i]] = i
	}
	if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_ALL){
	    console.log('RC_MAP', RC_MAP)
	    console.log('channelMSPIndexes', channelMSPIndexes)
	}
	if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_MINOR && THROTTLE_PRINT++ % FREQ == 0){
	    console.log('RC_MAP data')
	}
	break;
    }
    case msp.Codes.MSP_RAW_IMU:{
	IMU_acc = obj.accelerometer;
	IMU_gyro = obj.gyroscope;
	if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_MINOR && THROTTLE_PRINT++ % FREQ == 0){
	    console.log('IMU data')
	}
	break;
    }
    case msp.Codes.MSP_ATTITUDE:{
	Attitude_Quat.setFromEuler(new THREE.Euler(obj.z, obj.y, obj.x,'ZYX'));
	if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_MINOR && THROTTLE_PRINT++ % FREQ == 0){
	    console.log('ATTITUDE data')
	}
	break;
    }
    case msp.Codes.MSP_RC:{
	//console.log(obj.channels)
	break;
    }
    case msp.Codes.MSP_STATUS_EX:{
	//console.log(obj.mode)
	if(mspBOXID.length == 0){
	    msp.send_message(msp.Codes.MSP_BOXIDS)
	    msp.send_message(msp.Codes.MSP_BOXNAMES)
	}else{
	    if(mspStateMode == obj.mode){
		break;
	    }else{
		mspStateMode = obj.mode;
	    }
	    if(stateArm == false && ((obj.mode >> 0) & 0b01) == true){
		stateArm = true
		console.log("FC: Arm successful!")
	    }
	    if(stateArm == true && ((obj.mode >> 0) & 0b01) == false){
		stateArm = false
		console.log('FC: Disarm successful!')
	    }
	    var len = Math.ceil(Math.log(obj.mode)/Math.log(2))
	    if(len > 1){
		console.error('FC Alert:')
		for(var i = 0; i < len; i++){
		    if((obj.mode >> i) & 0b01 == true){
			console.log(mspBOXNAME[i])
		    }
		}
	    }
	}
	break;
    }
    case msp.Codes.MSP_BOXIDS:{
	mspBOXID = obj.AUX_CONFIG_IDS
	break;
    }
    case msp.Codes.MSP_BOXNAMES:{
	mspBOXNAME = obj.AUX_CONFIG
	break;
    }
    case msp.Codes.MSP_SERVO:{
	//console.log(obj.servo)
	deltaLeft = obj.servo[4];
	deltaRight = obj.servo[5];
	break;
    }
    }
})

var client = net.connect(port, ip, function(){
    console.log('connected to FC!')

    msp.setSender(function(data){
	//console.log('_write', data)
	client.write(data)
    })

    //	msp.pull_FC_info()

    msp.send_message(msp.Codes.MSP_RX_MAP, false, false);
})
client.on('error', function(err){
    console.log('FC err', err)
    js.close()
})
client.on('data', function(data){
    //console.log(data)
    msp.readbytes(data)
})
client.on('end', function(){
    console.log('disconnected from server')
})

////////////////////////
// from: https://gist.github.com/creationix/1695870
///////////////////////
var FS = require('fs');
var EventEmitter = require('events').EventEmitter;

// http://www.mjmwired.net/kernel/Documentation/input/joystick-api.txt
function parse(buffer) {
    var event = {
	time: buffer.readUInt32LE(0),
	number: buffer[7],
	value: buffer.readInt16LE(4)
    }
    if (buffer[6] & 0x80) event.init = true;
    if (buffer[6] & 0x01) event.type = "button";
    if (buffer[6] & 0x02) event.type = "axis";
    return event;
}

// Expose as a nice JavaScript API
function Joystick(id) {
    this.onOpen = this.onOpen.bind(this);
    this.onRead = this.onRead.bind(this);
    this.buffer = new Buffer.alloc(8);
    try{
	FS.open("/dev/input/js" + id, "r", this.onOpen);
    } catch (error){
	console.error('Error reading joystick!')
    }
}
Joystick.prototype = Object.create(EventEmitter.prototype, {
    constructor: {value: Joystick}
});

Joystick.prototype.onOpen = function (err, fd) {
    if (err) {
	JoystickAvailable = false;
	console.error('\n\n Error connecting Joystick \n Auto mode activated by default \n\n')
	return true;//this.emit("error", err);
    }
    this.fd = fd;
    this.startRead();
    //var number_of_axes = 0
    //console.log(fd)
    //ioctl(fd, 0x80016a11, number_of_axes);
    //console.log(number_of_axes)
};

Joystick.prototype.startRead = function () {
    FS.read(this.fd, this.buffer, 0, 8, null, this.onRead);
};

Joystick.prototype.onRead = function (err, bytesRead) {
    if (err) return this.emit("error", err);
    var event = parse(this.buffer);
    this.emit(event.type, event);
    if (this.fd) this.startRead();
};

Joystick.prototype.close = function (callback) {
    if(JoystickAvailable){
	FS.close(this.fd, callback);
	this.fd = undefined;
    }
};
///////////////////

// define joystick channel
var joy2RC = {
    0: 'A',
    1: 'E',
    2: 'R',
    3: 'T',
    4: '1',
    5: '2',
}
var joyBtn2RC = {
    0: '1',
    1: '4',
    2: '3',
    3: '3',
    4: '3',
    5: '3',
}
var joy2RCDirection = {
    0: -1,
    1: -1,
    2: -1,
    3:  -2,
    4:  1,
    5:  1,
}
var joy2RCOffset = {
    0: 0,
    1: 0,
    2: 0,
    3: -500,
    4:  0,
    5: 0,
}
var channelValues = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500];
var js = new Joystick(0);
js.on('button', function(data){
    var ch = data.number
    var val = data.value
    val = 800 * val + 1100

    channelValues[channelMSPIndexes[joyBtn2RC[ch]]] = val;
});
js.on('axis', function(data){
    var ch = data.number
    var val = data.value
    val = (1000 * val / 65535) * joy2RCDirection[ch] + 1500 + joy2RCOffset[ch]
    channelValues[channelMSPIndexes[joy2RC[ch]]] = val;
    //console.log(ch)
    //Print out output
    //console.log(channelValues)
    //	msp.send_message(msp.Codes.MSP_SET_RAW_RC, channelValues);
});

function publishROS() {
    // IMU message
    var imuMessage = new ROSLIB.Message({
	header : {
	    frame_id : "world"
	},
	orientation : {
	    x : Attitude_Quat.x,
	    y : Attitude_Quat.y,
	    z : Attitude_Quat.z,
	    w : Attitude_Quat.w
	},
	orientation_covariance : [0,0,0,0,0,0,0,0,0],
	angular_velocity : {
	    x : IMU_gyro[0],
	    y : IMU_gyro[1],
	    z : IMU_gyro[2],
	},
	angular_velocity_covariance  : [0,0,0,0,0,0,0,0,0],
	linear_acceleration : {
	    x : IMU_acc[0],
	    y : IMU_acc[1],
	    z : IMU_acc[2],
	},
	linear_acceleration_covariance  : [0,0,0,0,0,0,0,0,0],
    });
    imuTopic.publish(imuMessage);
    if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_ALL){
	console.log('Publishing IMU data');
    }
    // PWM info
    var pwmMessage = new ROSLIB.Message({
	layout : {
	    dim : [{
		label : "PWM data",
		size : 4,
		stride: 1,
	    }],
	    data_offset : 0,
	},
	data : [TLeft, TRight, deltaLeft, deltaRight],
    });
    pwmTopic.publish(pwmMessage);
}


function communicateMSP() {
    if(DEBUG_LEVEL >= DEBUG_INFO.DEBUG_ALL){
	console.log('Acquiring MSP');
    }
    var buf = Buffer.alloc(channelValues.length*2)
    for(var i=0; i<channelValues.length; i++){
	buf.writeUInt16LE(channelValues[i] ,2*i)
    }
    msp.send_message(msp.Codes.MSP_SET_RAW_RC, buf)
    //msp.send_message(msp.Codes.MSP_RAW_IMU);
    msp.send_message(msp.Codes.MSP_STATUS_EX)
    //msp.send_message(msp.Codes.MSP_ATTITUDE);
    //msp.send_message(msp.Codes.MSP_RC);
    msp.send_message(msp.Codes.MSP_SERVO);
    publishROS();
    var t = setTimeout(communicateMSP, 1000.0/FREQ);
}
communicateMSP()

