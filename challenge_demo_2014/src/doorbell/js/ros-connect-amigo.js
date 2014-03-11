/**
* Maintains the connection to the Rosbridge server
*
* this package:
* - connect to ros
* - poll for lost connections
* - show dialog on connection problems
**/

/*global $:false */

// configuration
var rosUrl = 'ws://' + window.location.hostname + ':9090';

// global variables
var ros;

// code wrapper
(function () {
"use strict";

// initialize the connection to rosbridge
function init() {

  // get the html elements
  var buttonReconnect = $('#reconnect');
  var modalReconnect  = $('#modalConnectionLost');

  buttonReconnect.click(function(e) {
    ros.connect(rosUrl);
  });

  // Connecting to ROS.
  ros = new ROSLIB.Ros({
    url : rosUrl
  });

  ros.addListener('connection', function(e) {
    console.log('rosbridge connection made');
    modalReconnect.modal('hide');
  });

  ros.addListener('close', function(e) {
    console.log('rosbridge connection closed');
    modalReconnect.modal('show');
  });

  ros.addListener('error', function(e) {
    console.log('rosbridge connection error');
  });

  console.log('ros connect loaded');
}

// when the dom is ready, start the code
$(document).ready(init);

// end wrapper
}());