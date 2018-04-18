const io = require("socket.io-client");
const ROSLIB = require("roslib");
var rpio = require('rpio');

//// RPI GPIO SET UP 
var pin1 = 12;           /* P12/GPIO18 */
var pin2 = 32;           /* P32/GPIO12 */
var range = 2000;     
var max = 200;          /*   the bottom 8th of a larger scale */
var clockdiv = 128;       /* Clock divider (PWM refresh rate), 8 == 2.4MHz */
rpio.init({gpiomem: false});
rpio.open(pin1, rpio.PWM);
rpio.open(pin2, rpio.PWM);
rpio.pwmSetClockDivider(clockdiv);
rpio.pwmSetRange(pin1, range); 
rpio.pwmSetRange(pin2, range); 

//// ROS SET UP
var ros = new ROSLIB.Ros(); 
ros.connect('ws://localhost:9090');
var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/image_raw/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});

//// WEBSOCKET SET UP 
const socket = io.connect("http://10.12.87.46:3000/video", { secure: true, reconnect: true });
socket.on('connect', function(){
    console.log('connected to server');
    socket.emit('connection', namespace='/video', {data: 'ROS client is connected!'});

    listener.subscribe(function(message) {
        var imagedata = "data:image/jpg;base64," + message.data;
        socket.emit('streaming', namespace='/video', {data: message.data});
        // listener.unsubscribe();
      });
});
socket.on('throttleup', function() {
    console.log('moving foward');
    rpio.pwmSetData(pin1, max);
    rpio.pwmSetData(pin2, max); 
});

socket.on('throttleleft', function(){
    console.log('moving left');
    rpio.pwmSetData(pin1, max);
    rpio.pwmSetData(pin2, 0);
});
socket.on('throttleright', function(){
    console.log('moving right');
    rpio.pwmSetData(pin1, 0);
    rpio.pwmSetData(pin2, max);
});
socket.on('disconnect', function() {
    rpio.open(pin1, rpio.INPUT);
    rpio.open(pin2, rpio.INPUT);
    console.log('disconnected from server');
});
