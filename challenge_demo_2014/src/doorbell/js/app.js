$( document ).ready(function() {

    function sendTrigger(msg) {
        console.log('sendTrigger: ', msg);

        var trigger = new ROSLIB.Topic({
            ros : ros,
            name : '/trigger',
            messageType : 'std_msgs/String'
        });

        var triggerMessage = new ROSLIB.Message({
            data: msg
        });
        trigger.publish(triggerMessage);
    }

    $('#doorbell').click(function(e) {
        e.preventDefault();

        sendTrigger('doorbell');
    });

    var MjpegViewer = new MJPEGCANVAS.Viewer({
        divID : 'webcam-image',
        host : window.location.hostname,
        width : $('#webcam-image').width(),
        height : $('#webcam-image').width()/4*3,
        topic : '/amigo/top_kinect/rgb/image_color'
    });
});