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
        sendTrigger('doorbell');
        $(this).fadeOut(100).fadeIn(100);
    });
});