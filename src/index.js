const io = require("socket.io-client");
const ROSLIB = require("roslib");

var ros = new ROSLIB.Ros(); 
ros.connect('ws://localhost:9090');
var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/image_raw/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});

const socket = io.connect("http://localhost:5000/video", { secure: true, reconnect: true });
socket.on('connect', function(){
    console.log('connected to server');
    socket.emit('connection', namespace='/video', {data: 'ROS client is connected!'});

    listener.subscribe(function(message) {
        var imagedata = "data:image/jpg;base64," + message.data;
        socket.emit('streaming', namespace='/video', {data: message.data});
        // listener.unsubscribe();
      });

});
socket.on('disconnect', function(){
    console.log('disconnected from server');
});