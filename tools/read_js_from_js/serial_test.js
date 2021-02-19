var SerialPort = require("serialport");

const Readline = SerialPort.parsers.Readline
const port = new SerialPort('/dev/ttyACM0', {baudRate: 115200})
const parser = new Readline()
port.pipe(parser)

port.on('open', function(){
    console.log('Connected to FC!');
    msp.setSender(function(data){
	port.write(data);
    });
    msp.send_message(msp.Codes.MSP_RX_MAP, false, false);    
});

port.on('end', function(){
    console.log('disconnected from server');
});

port.on('error', function(err){
    console.log('FC err', err);
    js.close();
});

parser.on('data', function(){
    msp.readbytes(data);
});

