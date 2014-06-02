var color_map = {};

$( document ).ready(function() {

    var source = $('#objects-template').html();
    var template = Handlebars.compile(source);

    var obj_list = $('#objects-list');

    var trigger = new ROSLIB.Topic({
        ros : ros,
        name : '/detected_objects',
        messageType : 'std_msgs/String'
    });

    trigger.subscribe(object_callback);

    function object_callback(msg) {
        var objs = msg.data.split('|');
        var data = objs.map(function (o) {
            var c;
            if (color_map[o]) {
                c = color_map[o];
            } else {
                c = color_map[o] =  random_colour();
            }
            return {
                color: c, name: o
            };
        });
        //background-color
        obj_list.html(template(data));
    }

    obj_list.on('click', 'button', function (e) {
        var name = $(e.currentTarget).html();
        console.log('click', name);
    });

    function random_colour(){
        return '#'+Math.floor(Math.random()*16777215).toString(16);
    }
});