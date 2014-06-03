var color_map = {};

$( document ).ready(function() {

    var source = $('#objects-template').html();
    var template = Handlebars.compile(source);

    var obj_list = $('#objects-list');

    var detector = new ROSLIB.Topic({
        ros : ros,
        name : '/detected_objects',
        messageType : 'std_msgs/String'
    });
    detector.subscribe(object_callback);

    var trigger = new ROSLIB.Topic({
        ros : ros,
        name : '/trigger',
        messageType : 'std_msgs/String'
    });

    function object_callback(msg) {
        var objs = msg.data;
        if (!objs) {
            objs = [];
        } else {
            objs = objs.split('|');
        }

        var data = objs.map(function (o) {
            var c;
            if (color_map[o]) {
                c = color_map[o];
            } else {
                c = color_map[o] =  random_color();
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
        trigger.publish({data:name});
    });

    function random_colour(){
        return '#'+Math.floor(Math.random()*16777215).toString(16);
    }

    var golden_ratio_conjugate = 0.618033988749895;
    var h = Math.random(); // use random start value

    function random_color(){
        h += golden_ratio_conjugate;
        h %= 1;
        return get_color(h, 0.7, 0.8);
    }

    function get_color(h, s, l) {
        h = Math.floor(h*360);
        s = Math.floor(s*100);
        l = Math.floor(l*100);
        return 'hsl(' + h + ',' + s + '%,' + l + '%)';
    }
});